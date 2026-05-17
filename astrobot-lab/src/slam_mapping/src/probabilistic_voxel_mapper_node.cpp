#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "slam_mapping/probabilistic_voxel_map.hpp"

namespace slam_mapping
{
namespace
{
uint32_t field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return field.offset;
    }
  }
  throw std::runtime_error("PointCloud2 is missing float32 field '" + name + "'");
}

float read_float32(const uint8_t * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

void write_float32(std::vector<uint8_t> & data, const float value)
{
  const auto * bytes = reinterpret_cast<const uint8_t *>(&value);
  data.insert(data.end(), bytes, bytes + sizeof(float));
}
}  // namespace

class ProbabilisticVoxelMapperNode : public rclcpp::Node
{
public:
  ProbabilisticVoxelMapperNode()
  : Node("probabilistic_voxel_mapper"),
    map_(read_occupancy_config()),
    bev_config_(read_bev_config())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/glim_rosnode/aligned_points_corrected");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/glim_rosnode/pose_corrected");
    voxel_output_topic_ = declare_parameter<std::string>("voxel_output_topic", "/slam/prob_voxel_map");
    costmap_output_topic_ = declare_parameter<std::string>("costmap_output_topic", "/slam/bev_costmap");
    frame_id_ = declare_parameter<std::string>("frame_id", "glim_map");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    publish_voxel_map_ = declare_parameter<bool>("publish_voxel_map", true);
    clear_on_frame_change_ = declare_parameter<bool>("clear_on_frame_change", false);

    const auto qos = rclcpp::SensorDataQoS();
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&ProbabilisticVoxelMapperNode::cloudCallback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 20, std::bind(&ProbabilisticVoxelMapperNode::poseCallback, this, std::placeholders::_1));
    voxel_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_output_topic_, 1);
    costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_output_topic_, 1);
    publish_timer_ = create_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 0.1)),
      std::bind(&ProbabilisticVoxelMapperNode::publishMaps, this));

    RCLCPP_INFO(
      get_logger(),
      "Probabilistic voxel mapper: cloud=%s pose=%s voxel_out=%s costmap_out=%s voxel=%.3f bev=%.3f",
      input_topic_.c_str(), pose_topic_.c_str(), voxel_output_topic_.c_str(), costmap_output_topic_.c_str(),
      map_.config().voxel_size, bev_config_.resolution);
  }

private:
  OccupancyConfig read_occupancy_config()
  {
    OccupancyConfig config;
    config.voxel_size = declare_parameter<double>("voxel_size", config.voxel_size);
    config.hit_log_odds = declare_parameter<double>("hit_log_odds", config.hit_log_odds);
    config.miss_log_odds = declare_parameter<double>("miss_log_odds", config.miss_log_odds);
    config.min_log_odds = declare_parameter<double>("min_log_odds", config.min_log_odds);
    config.max_log_odds = declare_parameter<double>("max_log_odds", config.max_log_odds);
    config.occupied_threshold = declare_parameter<double>("occupied_threshold", config.occupied_threshold);
    config.free_threshold = declare_parameter<double>("free_threshold", config.free_threshold);
    config.max_range = declare_parameter<double>("max_range", config.max_range);
    config.point_stride = declare_parameter<int>("point_stride", config.point_stride);
    config.ray_stride = declare_parameter<int>("ray_stride", config.ray_stride);
    config.use_bounds = declare_parameter<bool>("use_bounds", config.use_bounds);
    config.min_bounds.x = declare_parameter<double>("min_x", config.min_bounds.x);
    config.min_bounds.y = declare_parameter<double>("min_y", config.min_bounds.y);
    config.min_bounds.z = declare_parameter<double>("min_z", config.min_bounds.z);
    config.max_bounds.x = declare_parameter<double>("max_x", config.max_bounds.x);
    config.max_bounds.y = declare_parameter<double>("max_y", config.max_bounds.y);
    config.max_bounds.z = declare_parameter<double>("max_z", config.max_bounds.z);
    config.max_voxels = declare_parameter<int>("max_voxels", config.max_voxels);
    return config;
  }

  BevProjectionConfig read_bev_config()
  {
    BevProjectionConfig config;
    config.resolution = declare_parameter<double>("bev_resolution", config.resolution);
    config.width_m = declare_parameter<double>("width_m", config.width_m);
    config.height_m = declare_parameter<double>("height_m", config.height_m);
    config.origin_x = declare_parameter<double>("origin_x", config.origin_x);
    config.origin_y = declare_parameter<double>("origin_y", config.origin_y);
    config.low_z_percentile = declare_parameter<double>("low_z_percentile", config.low_z_percentile);
    config.high_z_percentile = declare_parameter<double>("high_z_percentile", config.high_z_percentile);
    config.ground_window_radius = declare_parameter<int>("ground_window_radius", config.ground_window_radius);
    config.ground_percentile = declare_parameter<double>("ground_percentile", config.ground_percentile);
    config.min_occupied_voxels_per_cell =
      declare_parameter<int>("min_occupied_voxels_per_cell", config.min_occupied_voxels_per_cell);
    config.obstacle_height = declare_parameter<double>("obstacle_height", config.obstacle_height);
    config.lethal_obstacle_height = declare_parameter<double>("lethal_obstacle_height", config.lethal_obstacle_height);
    config.obstacle_cost_min = declare_parameter<int>("obstacle_cost_min", config.obstacle_cost_min);
    config.obstacle_cost_max = declare_parameter<int>("obstacle_cost_max", config.obstacle_cost_max);
    config.terrain_risk_height = declare_parameter<double>("terrain_risk_height", config.terrain_risk_height);
    config.terrain_risk_cost_max = declare_parameter<int>("terrain_risk_cost_max", config.terrain_risk_cost_max);
    config.occupied_threshold = declare_parameter<double>("bev_occupied_threshold", config.occupied_threshold);
    config.free_threshold = declare_parameter<double>("bev_free_threshold", config.free_threshold);
    config.obstacle_inflation_radius =
      declare_parameter<double>("obstacle_inflation_radius", config.obstacle_inflation_radius);
    config.obstacle_inflation_cost = declare_parameter<int>("obstacle_inflation_cost", config.obstacle_inflation_cost);
    config.obstacle_inflation_min_cost =
      declare_parameter<int>("obstacle_inflation_min_cost", config.obstacle_inflation_min_cost);
    config.obstacle_inflation_exponent =
      declare_parameter<double>("obstacle_inflation_exponent", config.obstacle_inflation_exponent);
    return config;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    sensor_origin_ = Point3{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    have_pose_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for pose on %s", pose_topic_.c_str());
      return;
    }
    if (msg->is_bigendian) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported");
      return;
    }

    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    try {
      x_offset = field_offset(*msg, "x");
      y_offset = field_offset(*msg, "y");
      z_offset = field_offset(*msg, "z");
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      return;
    }

    if (!last_cloud_frame_.empty() && msg->header.frame_id != last_cloud_frame_ && clear_on_frame_change_) {
      map_.clear();
      RCLCPP_WARN(get_logger(), "Input frame changed from '%s' to '%s'; cleared map", last_cloud_frame_.c_str(),
        msg->header.frame_id.c_str());
    }
    last_cloud_frame_ = msg->header.frame_id;
    last_stamp_ = msg->header.stamp;

    std::vector<Point3> points;
    points.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height));
    for (uint32_t row = 0; row < msg->height; ++row) {
      const std::size_t row_start = static_cast<std::size_t>(row) * msg->row_step;
      for (uint32_t col = 0; col < msg->width; ++col) {
        const std::size_t start = row_start + static_cast<std::size_t>(col) * msg->point_step;
        if (start + msg->point_step > msg->data.size()) {
          continue;
        }
        const auto * raw = msg->data.data() + start;
        points.push_back(Point3{read_float32(raw + x_offset), read_float32(raw + y_offset), read_float32(raw + z_offset)});
      }
    }

    const auto start = std::chrono::steady_clock::now();
    const auto stats = map_.integratePointCloud(sensor_origin_, points);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(elapsed).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "prob voxel integrate points=%zu integrated=%zu hits=%zu invalid=%zu bounds=%zu voxels=%zu time_ms=%.1f",
      stats.input_points, stats.integrated_points, stats.hit_updates, stats.skipped_invalid, stats.skipped_bounds,
      stats.voxel_count, elapsed_ms);
  }

  void publishMaps()
  {
    if (publish_voxel_map_) {
      publishVoxelMap();
    }
    publishCostmap();
  }

  void publishVoxelMap()
  {
    const auto voxels = map_.occupiedVoxels();
    if (voxels.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp = last_stamp_;
    output.header.frame_id = frame_id_;
    output.height = 1;
    output.width = static_cast<uint32_t>(voxels.size());
    output.is_bigendian = false;
    output.is_dense = true;
    output.point_step = 16;
    output.row_step = output.width * output.point_step;
    output.fields.resize(4);
    output.fields[0].name = "x";
    output.fields[0].offset = 0;
    output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[0].count = 1;
    output.fields[1].name = "y";
    output.fields[1].offset = 4;
    output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[1].count = 1;
    output.fields[2].name = "z";
    output.fields[2].offset = 8;
    output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[2].count = 1;
    output.fields[3].name = "log_odds";
    output.fields[3].offset = 12;
    output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[3].count = 1;
    output.data.reserve(static_cast<std::size_t>(output.width) * output.point_step);

    for (const auto & voxel : voxels) {
      write_float32(output.data, static_cast<float>(voxel.center.x));
      write_float32(output.data, static_cast<float>(voxel.center.y));
      write_float32(output.data, static_cast<float>(voxel.center.z));
      write_float32(output.data, voxel.log_odds);
    }
    voxel_publisher_->publish(output);
  }

  void publishCostmap()
  {
    const auto grid = map_.projectToBev(bev_config_);
    nav_msgs::msg::OccupancyGrid output;
    output.header.stamp = last_stamp_;
    output.header.frame_id = frame_id_;
    output.info.resolution = static_cast<float>(grid.resolution);
    output.info.width = static_cast<uint32_t>(grid.width);
    output.info.height = static_cast<uint32_t>(grid.height);
    output.info.origin.position.x = grid.origin_x;
    output.info.origin.position.y = grid.origin_y;
    output.info.origin.orientation.w = 1.0;
    output.data = grid.data;
    costmap_publisher_->publish(output);
  }

  ProbabilisticVoxelMap map_;
  BevProjectionConfig bev_config_;
  std::string input_topic_;
  std::string pose_topic_;
  std::string voxel_output_topic_;
  std::string costmap_output_topic_;
  std::string frame_id_;
  std::string last_cloud_frame_;
  double publish_rate_hz_;
  bool publish_voxel_map_;
  bool clear_on_frame_change_;
  bool have_pose_{false};
  Point3 sensor_origin_;
  builtin_interfaces::msg::Time last_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
}  // namespace slam_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_mapping::ProbabilisticVoxelMapperNode>());
  rclcpp::shutdown();
  return 0;
}
