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
// Terrain for navigation: native LiDAR chunks + SLAM pose revisions -> Nav2 local window,
// global cost mosaic and exploration class mosaic. Fails closed (unknown) on any contract
// violation; only the localization supervisor's map reset (slam/reset_map) recovers.
#include <sys/resource.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>

#include <astrobot_interfaces/srv/reset_map.hpp>

#include "astrobot_terrain/ros_conversions.hpp"

namespace astrobot_terrain
{

class TerrainNode : public rclcpp::Node {
public:
  explicit TerrainNode(const rclcpp::NodeOptions & options)
  : Node("terrain", options)
  {
    PipelineConfig config;
    auto & robot = config.robot;
    const auto base_from_imu = declare_parameter<std::vector<double>>("base_from_imu",
        std::vector<double>{});
    if (base_from_imu.size() != 16) {
      throw std::invalid_argument("base_from_imu: 16 row-major values required");
    }
    for (int i = 0; i < 16; ++i) {robot.base_from_imu(i / 4, i % 4) = base_from_imu[i];}
    const auto lidar = declare_parameter<std::vector<double>>("base_from_lidar_translation",
        {0.0, 0.0, 0.0});
    const auto box = declare_parameter<std::vector<double>>("chassis_aabb",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (lidar.size() != 3 || box.size() != 6) {
      throw std::invalid_argument("lidar translation (3) / chassis box (6)");
    }
    std::copy(lidar.begin(), lidar.end(), robot.base_from_lidar_translation.begin());
    std::copy(box.begin(), box.end(), robot.chassis_aabb.begin());
    // Padded Nav2 footprint half extents and padding (the unpadded body is cleared).
    const double half_length = declare_parameter("footprint_half_length", 0.595);
    const double half_width = declare_parameter("footprint_half_width", 0.435);
    const double padding = declare_parameter("footprint_padding", 0.10);
    robot.body = Body{half_length - padding, half_width - padding};
    config.local.window_s = static_cast<int>(declare_parameter("window_s", 5));
    config.local.evidence_lifetime_s = static_cast<int>(declare_parameter("evidence_lifetime_s",
        60));
    config.local.max_bookkeeping_frames =
      static_cast<size_t>(declare_parameter("max_bookkeeping_frames", 32768));
    config.local.tolerate_stream_gaps = declare_parameter("tolerate_stream_gaps", true);
    config.gap.cost = static_cast<int>(declare_parameter("gap_cost", 70));
    config.gap.hazard_clearance_m = declare_parameter("gap_hazard_clearance_m", 0.30);
    config.gap.max_support_gap_m = declare_parameter("gap_max_support_m", 0.5);
    config.gap.occupied_shadow_lethal_m = declare_parameter("occupied_shadow_lethal_m", 0.20);
    config.mosaic_half_extent_m = declare_parameter("mosaic_half_extent_m", 30.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    heartbeat_s_ = declare_parameter("clock_heartbeat_s", 2.0);
    duplicate_grace_s_ = declare_parameter("duplicate_authority_grace_s", 10.0);
    chunk_topic_ = declare_parameter<std::string>("chunk_topic", "lidar_selected/chunks");
    revision_topic_ = declare_parameter<std::string>("revision_topic", "slam_raw/revisions");
    pipeline_ = std::make_unique<TerrainPipeline>(config);

    local_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("local_costmap_topic", "terrain/local_costmap"), 1);
    global_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("global_costmap_topic", "terrain/global_costmap"), 1);
    explore_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("exploration_map_topic", "terrain/exploration_map"), 1);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("status_topic", "terrain/status"), 10);
    using ResetMap = astrobot_interfaces::srv::ResetMap;
    auto on_reset = [this](const std::shared_ptr<ResetMap::Request> request,
      std::shared_ptr<ResetMap::Response> response) {reset(*request, *response);};
    reset_srv_ = create_service<ResetMap>(
      declare_parameter<std::string>("reset_service", "slam/reset_map"), on_reset);
    const auto revisions = rclcpp::QoS(100).reliable().transient_local();
    revision_sub_ = create_subscription<astrobot_interfaces::msg::SlamRevision>(
      revision_topic_, revisions,
      [this](const astrobot_interfaces::msg::SlamRevision & msg) {
        on_revision(msg);
      });
    // 64 chunks = ~1 sim-s of callback stall before the reader drops.
    chunk_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        chunk_topic_, 64, [this](const sensor_msgs::msg::PointCloud2 & msg) {on_chunk(msg);});
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 100, [this](const rosgraph_msgs::msg::Clock & msg) {on_clock(msg);});
    last_clock_wall_ = steady_.now();
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] {watchdog();});
    mosaic_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_mosaics();});
    publish_unknown();
  }

private:
  void on_revision(const astrobot_interfaces::msg::SlamRevision & msg)
  {
    guarded([&] {pipeline_->revision(revision_from_msg(msg));});
  }

  void on_chunk(const sensor_msgs::msg::PointCloud2 & msg)
  {
    guarded([&] {pipeline_->chunk(decode_native(msg));});
  }

  void on_clock(const rosgraph_msgs::msg::Clock & msg)
  {
    last_clock_wall_ = steady_.now();
    if (pipeline_->failed()) {return;}
    std::optional<PlannerWindow> window;
    guarded([&] {window = pipeline_->clock(stamp_ns(msg.clock));});
    if (window) {
      local_pub_->publish(grid_message(window->costs, window->map_from_grid, resolution(),
          window->stamp_ns, frame_id_));
      publish_status();
    }
  }

  template<typename F>
  void guarded(F && function)
  {
    if (pipeline_->failed()) {return;}
    try {
      function();
    } catch (const std::exception & e) {
      pipeline_->fault(e.what());
    }
    if (pipeline_->failed() && !fault_reported_) {
      fault_reported_ = true;
      RCLCPP_ERROR(get_logger(), "terrain faulted: %s", pipeline_->core().status().failure.c_str());
      publish_unknown();
    }
  }

  void reset(
    const astrobot_interfaces::srv::ResetMap::Request & request,
    astrobot_interfaces::srv::ResetMap::Response & response)
  {
    response.epoch = request.epoch;
    try {
      std::string reason;
      response.success = pipeline_->reset(request.epoch, stamp_ns(request.minimum_stamp), &reason);
      response.reason = reason;
      if (response.success) {
        fault_reported_ = false;
        RCLCPP_INFO(get_logger(), "map reset for epoch %s", std::to_string(request.epoch).c_str());
        publish_unknown();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "map reset failed: %s", e.what());
      response.success = false;
      response.reason = e.what();
    }
  }

  void watchdog()
  {
    if (pipeline_->failed()) {
      publish_unknown();
      return;
    }
    guarded([&] {
        if (!pipeline_->core().capture().empty() &&
        (steady_.now() - last_clock_wall_).seconds() > heartbeat_s_)
        {
          throw std::runtime_error("simulation wall heartbeat lost");
        }
      // A supervisor SLAM restart briefly shows the old and new revision publishers in discovery;
      // only the reset epoch is accepted, so fault only if a second authority persists.
        if (count_publishers(chunk_topic_) > 1 || count_publishers(revision_topic_) > 1) {
          if (!duplicate_since_) {duplicate_since_ = steady_.now();}
          if ((steady_.now() - *duplicate_since_).seconds() > duplicate_grace_s_) {
            throw std::runtime_error("multiple terrain input authorities");
          }
        } else {
          duplicate_since_.reset();
        }
    });
    publish_status();
  }

  void publish_unknown()
  {
    const int cells = pipeline_->config().local.cells;
    local_pub_->publish(grid_message(ClassGrid(cells, cells, -1), pipeline_->last_map_from_grid(),
        resolution(),
                                     std::max<int64_t>(0, pipeline_->core().watermark_ns()),
        frame_id_));
    publish_status();
  }

  void publish_mosaics()
  {
    const auto * cost = pipeline_->cost_mosaic();
    const auto * explore = pipeline_->explore_mosaic();
    if (!cost || !explore) {return;}
    const int64_t stamp = std::max<int64_t>(0, pipeline_->core().watermark_ns());
    for (const auto & [publisher, mosaic] : {std::pair{global_pub_, cost},
        std::pair{explore_pub_, explore}})
    {
      Pose origin = Pose::Identity();
      origin(0, 3) = mosaic->origin_x();
      origin(1, 3) = mosaic->origin_y();
      publisher->publish(grid_message(mosaic->grid(), origin, mosaic->resolution(), stamp,
          frame_id_));
    }
  }

  void publish_status()
  {
    const auto s = pipeline_->core().status();
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    nlohmann::json status = {
      {"policy", "astrobot_terrain"},
      {"valid", s.valid},
      {"failure", s.failure.empty() ? nlohmann::json(nullptr) : nlohmann::json(s.failure)},
      {"capture", s.capture.empty() ? nlohmann::json(nullptr) : nlohmann::json(s.capture)},
      {"epoch", pipeline_->epoch() ? nlohmann::json(*pipeline_->epoch()) : nlohmann::json(nullptr)},
      {"received", s.received},
      {"projected", s.projected},
      {"integrated", s.integrated_sequence + 1},
      {"skipped_sequences", s.skipped_sequences},
      {"late_chunks", s.late_chunks},
      {"retained_bytes", s.retained_bytes},
      {"bookkeeping_accepted", s.bookkeeping_accepted},
      {"evidence_ns", s.evidence_ns},
      {"peak_process_rss_mib", usage.ru_maxrss / 1024.0},
    };
    std_msgs::msg::String msg;
    msg.data = status.dump();
    status_pub_->publish(msg);
  }

  double resolution() const {return pipeline_->config().local.resolution;}

  std::unique_ptr<TerrainPipeline> pipeline_;
  std::string frame_id_, chunk_topic_, revision_topic_;
  double heartbeat_s_{2.0}, duplicate_grace_s_{10.0};
  bool fault_reported_{false};
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  rclcpp::Time last_clock_wall_;
  std::optional<rclcpp::Time> duplicate_since_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_pub_, global_pub_, explore_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<astrobot_interfaces::srv::ResetMap>::SharedPtr reset_srv_;
  rclcpp::Subscription<astrobot_interfaces::msg::SlamRevision>::SharedPtr revision_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr chunk_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_, mosaic_timer_;
};

rclcpp::Node::SharedPtr make_terrain_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<TerrainNode>(options);
}

}  // namespace astrobot_terrain

RCLCPP_COMPONENTS_REGISTER_NODE(astrobot_terrain::TerrainNode)
