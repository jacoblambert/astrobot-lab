#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "slam_mapping/pose_history_se3.hpp"
#include "astrobot_interfaces/msg/localization_status.hpp"

namespace slam_mapping
{
class SlamMapToOdomTfNode : public rclcpp::Node
{
public:
  SlamMapToOdomTfNode()
  : Node("slam_map_to_odom_tf")
  {
    slam_pose_topic_ = declare_parameter<std::string>("slam_pose_topic", "slam/pose_corrected");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    base_offset_x_ = declare_parameter<double>("base_offset_x", 0.264);
    base_offset_y_ = declare_parameter<double>("base_offset_y", 0.017);
    base_yaw_offset_ = declare_parameter<double>("base_yaw_offset", M_PI);
    base_offset_z_ = declare_parameter<double>("base_offset_z", 0.262);
    auto imu_from_base = declare_parameter<std::vector<double>>("imu_from_base_quaternion", std::vector<double>{});
    // An empty quaternion explicitly selects the legacy yaw parameter. Any
    // supplied quaternion wins, including the default Husky pi-yaw orientation.
    if (!std::isfinite(base_offset_x_) || !std::isfinite(base_offset_y_) ||
        !std::isfinite(base_offset_z_) || !std::isfinite(base_yaw_offset_))
      throw std::invalid_argument("nonfinite IMU/base extrinsic");
    if (imu_from_base.empty()) imu_from_base = {0.,0.,std::sin(base_yaw_offset_/2),std::cos(base_yaw_offset_/2)};
    if (imu_from_base.size()!=4) throw std::invalid_argument("imu_from_base_quaternion must be xyzw");
    Eigen::Quaterniond q(imu_from_base[3],imu_from_base[0],imu_from_base[1],imu_from_base[2]);
    if (!q.coeffs().allFinite() || std::abs(q.squaredNorm()-1)>0.02) throw std::invalid_argument("invalid IMU/base quaternion");
    imu_from_base_ = Eigen::Isometry3d::Identity();
    imu_from_base_.linear() = q.normalized().toRotationMatrix();
    // Existing XY offsets are base origin expressed in IMU. Z is negative for
    // the IMU mounted above base_link; do not rotate the lever arm with yaw only.
    imu_from_base_.translation() << base_offset_x_,base_offset_y_,-base_offset_z_;
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    transform_tolerance_sec_ = declare_parameter<double>("transform_tolerance_sec", 0.1);
    require_localization_ = declare_parameter<bool>("require_localization", true);
    health_sub_ = create_subscription<astrobot_interfaces::msg::LocalizationStatus>(
      "localization/status", 10, [this](const astrobot_interfaces::msg::LocalizationStatus & msg) {
        using Status = astrobot_interfaces::msg::LocalizationStatus;
        health_wall_ = std::chrono::steady_clock::now();
        health_valid_ = msg.state == Status::REBUILDING || msg.state == Status::HEALTHY || msg.state == Status::DEGRADED;
        if (msg.epoch != epoch_ || !health_valid_) {
          have_slam_pose_ = false;
          correction_stamp_ = -1.0;
          history_.clear();
          minimum_stamp_ = rclcpp::Time(msg.header.stamp).seconds();
        }
        epoch_ = msg.epoch;
      });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      slam_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamMapToOdomTfNode::slam_pose_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamMapToOdomTfNode::odom_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)),
      std::bind(&SlamMapToOdomTfNode::publish_tf, this));

    RCLCPP_INFO(
      get_logger(),
      "SLAM map->odom TF: slam_pose=%s odom=%s frame=%s->%s base=%s imu_to_base=[%.3f, %.3f, yaw %.3f]",
      slam_pose_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), base_offset_x_, base_offset_y_, base_yaw_offset_);
  }

private:
  static Eigen::Isometry3d pose_from_msg(const geometry_msgs::msg::Pose & pose)
  {
    auto out=Eigen::Isometry3d::Identity();
    const auto& q=pose.orientation;
    out.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
    out.translation()<<pose.position.x,pose.position.y,pose.position.z;
    return out;
  }

  void slam_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!valid_pose(msg->pose) || msg->header.frame_id != map_frame_ ||
        (require_localization_ && (!health_valid_ || rclcpp::Time(msg->header.stamp).seconds() < minimum_stamp_))) return;
    latest_slam_pose_ = *msg;
    have_slam_pose_ = true;
    update_correction();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!valid_pose(msg->pose.pose) || msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) return;
    latest_odom_ = *msg;
    have_odom_ = true;
    history_.insert(rclcpp::Time(msg->header.stamp).seconds(), pose_from_msg(msg->pose.pose));
    update_correction();
  }

  static bool valid_pose(const geometry_msgs::msg::Pose & p) {
    const auto & q = p.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
      std::isfinite(norm) && std::abs(norm - 1.0) <= 0.02;
  }

  void update_correction() {
    if (!have_slam_pose_) return;
    const double stamp = rclcpp::Time(latest_slam_pose_.header.stamp).seconds();
    if (stamp <= correction_stamp_) return;
    const auto odom = history_.at(stamp);
    if (!odom) return;
    const Eigen::Isometry3d map_base = pose_from_msg(latest_slam_pose_.pose) * imu_from_base_;
    correction_ = map_base * odom->inverse();
    correction_stamp_ = stamp;
  }

  void publish_tf()
  {
    if (require_localization_ && (!health_valid_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - health_wall_).count() > 0.1)) return;
    if (!have_slam_pose_ || !have_odom_) {
      return;
    }

    const double now = this->now().seconds();
    if (now < last_clock_) {
      history_.clear();
      have_slam_pose_ = have_odom_ = false;
      correction_stamp_ = -1.0;
    }
    last_clock_ = now;
    const double age = now - correction_stamp_;
    if (correction_stamp_ < 0.0 || age < -0.05 || age > 0.5 ||
        now - rclcpp::Time(latest_odom_.header.stamp).seconds() > 0.5) return;
    const auto& map_odom = correction_;

    geometry_msgs::msg::TransformStamped transform;
    // Hold a same-measurement-time correction; current odometry propagates motion.
    // Publication time never substitutes for the freshness check above.
    transform.header.stamp = this->now() + rclcpp::Duration::from_seconds(transform_tolerance_sec_);
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_odom.translation().x();
    transform.transform.translation.y = map_odom.translation().y();
    transform.transform.translation.z = map_odom.translation().z();
    const Eigen::Quaterniond q(map_odom.linear());
    transform.transform.rotation.x=q.x(); transform.transform.rotation.y=q.y();
    transform.transform.rotation.z=q.z(); transform.transform.rotation.w=q.w();
    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "publishing %s->%s x=%.3f y=%.3f yaw=%.3f",
      map_frame_.c_str(), odom_frame_.c_str(), map_odom.translation().x(), map_odom.translation().y(),
      std::atan2(map_odom.linear()(1,0),map_odom.linear()(0,0)));
  }

  std::string slam_pose_topic_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double base_offset_x_;
  double base_offset_y_;
  double base_yaw_offset_;
  double base_offset_z_;
  Eigen::Isometry3d imu_from_base_;
  double publish_rate_hz_;
  double transform_tolerance_sec_;
  bool have_slam_pose_{false};
  bool require_localization_{true};
  bool health_valid_{false};
  uint64_t epoch_{0};
  double minimum_stamp_{0.0};
  std::chrono::steady_clock::time_point health_wall_;
  rclcpp::Subscription<astrobot_interfaces::msg::LocalizationStatus>::SharedPtr health_sub_;
  PoseHistorySE3 history_;
  Eigen::Isometry3d correction_{Eigen::Isometry3d::Identity()};
  double correction_stamp_{-1.0};
  double last_clock_{0.0};
  bool have_odom_{false};
  geometry_msgs::msg::PoseStamped latest_slam_pose_;
  nav_msgs::msg::Odometry latest_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace slam_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_mapping::SlamMapToOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
