// Copyright 2026 Jacob Lambert
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Localization supervisor: owns exactly one SLAM (GLIM) child process; never restarts the ROS
// stack or other host processes. See SupervisorCore for the contracts.
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <astrobot_interfaces/msg/localization_status.hpp>
#include <astrobot_interfaces/srv/reset_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include "astrobot_localization/child_config.hpp"
#include "astrobot_localization/supervisor_core.hpp"

extern char ** environ;

namespace astrobot_localization
{
namespace fs = std::filesystem;
using LocalizationStatus = astrobot_interfaces::msg::LocalizationStatus;

namespace
{

double seconds(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}
StampKey key(const builtin_interfaces::msg::Time & t) {return {t.sec, t.nanosec};}

}  // namespace

class LocalizationSupervisor : public rclcpp::Node, public SupervisorEffects {
public:
  explicit LocalizationSupervisor(const rclcpp::NodeOptions & options)
  : Node("localization_supervisor", options), core_(*this, 2)
  {
    config_path_ = declare_parameter<std::string>("config_path", "");
    const auto map_topic = declare_parameter<std::string>("map_topic", "terrain/local_costmap");
    char run_template[] = "/tmp/astrobot-localization-XXXXXX";
    if (!mkdtemp(run_template)) {
      throw std::runtime_error("cannot create localization run directory");
    }
    run_directory_ = run_template;

    status_pub_ = create_publisher<LocalizationStatus>("localization/status", 10);
    fix_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("localization/absolute_pose",
        20);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("slam/pose_corrected", 10);
    lidar_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("slam/lidar_pose_corrected", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("slam/aligned_points_corrected",
        rclcpp::SensorDataQoS());

    fix_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/rtk/pose", 20,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        on_fix(msg);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu & m) {
          core_.imu(seconds(m.header.stamp),
          {m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z});
    });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 20,
        [this](const nav_msgs::msg::Odometry & m) {
          const auto & t = m.twist.twist;
          core_.odom(seconds(m.header.stamp),
          {t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z});
    });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "slam_raw/pose_corrected", 20,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        on_pose(msg);
      });
    lidar_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "slam_raw/lidar_pose_corrected", 20,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        core_.lidar(seconds(msg->header.stamp), key(msg->header.stamp), [this, msg] {
          lidar_pub_->publish(*msg);
      },
        now_s());
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "slam_raw/aligned_points_corrected", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        core_.cloud(seconds(msg->header.stamp), key(msg->header.stamp), [this, msg] {
          cloud_pub_->publish(*msg);
      },
        now_s());
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 2,
        [this](const nav_msgs::msg::OccupancyGrid & m) {
          SupervisorCore::MapView view;
          view.stamp = seconds(m.header.stamp);
          view.frame_id = m.header.frame_id;
          view.resolution = m.info.resolution;
          view.origin_x = m.info.origin.position.x;
          view.origin_y = m.info.origin.position.y;
          view.width = m.info.width;
          view.height = m.info.height;
          view.data = m.data.data();
          view.size = m.data.size();
          core_.map(view, now_s());
    });
    integrity_sub_ = create_subscription<std_msgs::msg::Bool>(
      "slam/map_integrity", 10,
      [this](const std_msgs::msg::Bool & m) {
        core_.map_integrity(m.data);
      });
    nav_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "navigate_to_pose/_action/status", 10,
      [this](const action_msgs::msg::GoalStatusArray & m) {
        bool active = false;
        for (const auto & s : m.status_list) {
          active |= s.status == 1 || s.status == 2 || s.status == 3;
        }
        core_.navigation_active(active);
      });
    reset_client_ = create_client<astrobot_interfaces::srv::ResetMap>("slam/reset_map");
    cancel_client_ =
      create_client<action_msgs::srv::CancelGoal>("navigate_to_pose/_action/cancel_goal");
    for (const char * name : {"local_costmap/clear_entirely_local_costmap",
        "global_costmap/clear_entirely_global_costmap"})
    {
      clear_clients_.push_back(create_client<nav2_msgs::srv::ClearEntireCostmap>(name));
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
          core_.tick(now_s(), wall_s());
          });
  }

  ~LocalizationSupervisor() override {stop_child();}

  // SupervisorEffects
  void publish_status(
    State state, uint64_t epoch, const std::string & reason, bool motion_permitted,
    const std::optional<StampKey> & last_pose) override
  {
    LocalizationStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.state = static_cast<uint8_t>(state);
    msg.epoch = epoch;
    msg.reason = reason;
    msg.motion_permitted = motion_permitted;
    if (last_pose) {
      msg.last_pose_stamp.sec = last_pose->first;
      msg.last_pose_stamp.nanosec = last_pose->second;
    }
    status_pub_->publish(msg);
  }
  void publish_fix(const Fix &) override {fix_pub_->publish(*current_fix_);}
  void publish_pose(const PoseSample &) override {pose_pub_->publish(*current_pose_);}
  bool reset_ready() override {return reset_client_->service_is_ready();}
  void request_reset(uint64_t id, uint64_t epoch) override
  {
    auto request = std::make_shared<astrobot_interfaces::srv::ResetMap::Request>();
    request->epoch = epoch;
    request->minimum_stamp = now();
    reset_client_->async_send_request(request,
      [this, id](rclcpp::Client<astrobot_interfaces::srv::ResetMap>::SharedFuture f) {
        const auto response = f.get();
        core_.reset_result(id, response->success, response->epoch);
    });
  }
  bool clear_ready() override
  {
    for (const auto & c : clear_clients_) {
      if (!c->service_is_ready()) {return false;}
    }
    return true;
  }
  void request_clear(uint64_t id) override
  {
    for (const auto & c : clear_clients_) {
      c->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>(),
        [this, id](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture f) {
          bool ok = true;
          try {
            f.get();
          } catch (const std::exception &) {
            ok = false;
          }
          core_.clear_result(id, ok);
                            });
    }
  }
  void cancel_navigation() override
  {
    if (cancel_client_->service_is_ready()) {
      cancel_client_->async_send_request(std::make_shared<action_msgs::srv::CancelGoal::Request>());
    }
  }
  Launch launch_child(uint64_t epoch, const std::array<double, 7> & reference) override
  {
    switch (check_revision_config(config_path_)) {
      case ConfigCheck::REVISION_CONSUMER_REQUIRED:
        return Launch::REVISION_CONSUMER_REQUIRED;
      case ConfigCheck::INVALID:
        return Launch::INVALID_CONFIG;
      case ConfigCheck::OK:
        break;
    }
    std::string ns = get_namespace();
    while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
    const fs::path directory =
      prepare_child_config(config_path_, run_directory_, epoch, reference,
        ns + "/localization/absolute_pose");
    const std::string executable = ament_index_cpp::get_package_prefix("glim_ros") +
      "/lib/glim_ros/glim_rosnode";
    std::vector<std::string> args = {executable, "--ros-args", "-r", "__node:=slam_raw", "-r",
      "__ns:=" + std::string(get_namespace()), "-p", "use_sim_time:=true",
      "-p", "config_path:=" + directory.string()};
    std::vector<char *> argv;
    for (auto & a : args) {argv.push_back(a.data());}
    argv.push_back(nullptr);
    pid_t pid = -1;
    if (const int error = posix_spawn(&pid, executable.c_str(), nullptr, nullptr, argv.data(),
        environ); error != 0)
    {
      throw std::runtime_error("spawn failed: " + std::string(std::strerror(error)));
    }
    child_ = pid;
    return Launch::STARTED;
  }
  void stop_child() override
  {
    if (child_ <= 0) {return;}
    int status = 0;
    if (waitpid(child_, &status, WNOHANG) == 0) {
      kill(child_, SIGTERM);
      if (!wait_for(child_, std::chrono::seconds(1))) {
        kill(child_, SIGKILL);
        wait_for(child_, std::chrono::seconds(1));
      }
    }
    child_ = -1;
  }
  bool child_exited() override
  {
    if (child_ <= 0) {return false;}
    int status = 0;
    return waitpid(child_, &status, WNOHANG) == child_ ? (child_ = -1, true) : false;
  }
  void log_error(const std::string & message) override
  {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

private:
  static bool wait_for(pid_t pid, std::chrono::milliseconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    int status = 0;
    while (std::chrono::steady_clock::now() < end) {
      if (waitpid(pid, &status, WNOHANG) != 0) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  void on_fix(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    Fix fix;
    fix.stamp = seconds(msg->header.stamp);
    fix.frame_id = msg->header.frame_id;
    const auto & p = msg->pose.pose;
    fix.position = {p.position.x, p.position.y, p.position.z};
    fix.quaternion = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    std::copy(msg->pose.covariance.begin(), msg->pose.covariance.end(), fix.covariance.begin());
    current_fix_ = msg;
    core_.fix(fix, now_s());
  }

  void on_pose(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    PoseSample sample;
    sample.stamp = seconds(msg->header.stamp);
    sample.key = key(msg->header.stamp);
    sample.frame_id = msg->header.frame_id;
    const auto & p = msg->pose;
    sample.position = {p.position.x, p.position.y, p.position.z};
    sample.quaternion = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    current_pose_ = msg;
    core_.pose(sample, now_s());
  }

  double now_s() {return now().nanoseconds() * 1e-9;}
  double wall_s() {return steady_.now().nanoseconds() * 1e-9;}

  SupervisorCore core_;
  std::string config_path_;
  fs::path run_directory_;
  pid_t child_{-1};
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr current_fix_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr integrity_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_sub_;
  rclcpp::Client<astrobot_interfaces::srv::ResetMap>::SharedPtr reset_client_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_client_;
  std::vector<rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clear_clients_;
  rclcpp::TimerBase::SharedPtr timer_;
};

rclcpp::Node::SharedPtr make_localization_supervisor(const rclcpp::NodeOptions & options)
{
  return std::make_shared<LocalizationSupervisor>(options);
}

}  // namespace astrobot_localization

RCLCPP_COMPONENTS_REGISTER_NODE(astrobot_localization::LocalizationSupervisor)
