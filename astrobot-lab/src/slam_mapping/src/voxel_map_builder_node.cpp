#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace slam_mapping
{
struct VoxelKey
{
  int32_t x;
  int32_t y;
  int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & key) const
  {
    const auto x = static_cast<uint32_t>(key.x);
    const auto y = static_cast<uint32_t>(key.y);
    const auto z = static_cast<uint32_t>(key.z);
    size_t seed = 0;
    seed ^= static_cast<size_t>(x) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= static_cast<size_t>(y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= static_cast<size_t>(z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }
};

class VoxelMapBuilderNode : public rclcpp::Node
{
public:
  VoxelMapBuilderNode()
  : Node("voxel_map_builder")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/glim_rosnode/aligned_points_corrected");
    output_topic_ = declare_parameter<std::string>("output_topic", "/slam/voxel_map");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    max_voxels_ = declare_parameter<int>("max_voxels", 2000000);
    use_bounds_ = declare_parameter<bool>("use_bounds", false);
    min_x_ = declare_parameter<double>("min_x", -std::numeric_limits<double>::infinity());
    max_x_ = declare_parameter<double>("max_x", std::numeric_limits<double>::infinity());
    min_y_ = declare_parameter<double>("min_y", -std::numeric_limits<double>::infinity());
    max_y_ = declare_parameter<double>("max_y", std::numeric_limits<double>::infinity());
    min_z_ = declare_parameter<double>("min_z", -std::numeric_limits<double>::infinity());
    max_z_ = declare_parameter<double>("max_z", std::numeric_limits<double>::infinity());
    clear_on_frame_change_ = declare_parameter<bool>("clear_on_frame_change", true);

    if (voxel_size_ <= 0.0) {
      throw std::runtime_error("voxel_size must be positive");
    }

    map_.reserve(200000);
    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&VoxelMapBuilderNode::cloud_callback, this, std::placeholders::_1));
    publish_timer_ = create_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 0.1)),
      std::bind(&VoxelMapBuilderNode::publish_map, this));

    RCLCPP_INFO(
      get_logger(), "Voxel map builder: %s -> %s voxel_size=%.3f max_voxels=%d",
      input_topic_.c_str(), output_topic_.c_str(), voxel_size_, max_voxels_);
  }

private:
  static uint32_t field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    for (const auto & field : msg.fields) {
      if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        return field.offset;
      }
    }
    throw std::runtime_error("PointCloud2 is missing float32 field '" + name + "'");
  }

  static float read_float32(const uint8_t * data)
  {
    float value = 0.0F;
    std::memcpy(&value, data, sizeof(float));
    return value;
  }

  static void write_float32(std::vector<uint8_t> & data, const float value)
  {
    const auto * bytes = reinterpret_cast<const uint8_t *>(&value);
    data.insert(data.end(), bytes, bytes + sizeof(float));
  }

  VoxelKey point_to_key(const float x, const float y, const float z) const
  {
    return VoxelKey{
      static_cast<int32_t>(std::floor(static_cast<double>(x) / voxel_size_)),
      static_cast<int32_t>(std::floor(static_cast<double>(y) / voxel_size_)),
      static_cast<int32_t>(std::floor(static_cast<double>(z) / voxel_size_))};
  }

  bool within_bounds(const float x, const float y, const float z) const
  {
    if (!use_bounds_) {
      return true;
    }
    return min_x_ <= x && x <= max_x_ && min_y_ <= y && y <= max_y_ && min_z_ <= z && z <= max_z_;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
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

    if (msg->is_bigendian) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported");
      return;
    }

    if (frame_id_.empty()) {
      frame_id_ = msg->header.frame_id;
    } else if (msg->header.frame_id != frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Input frame changed from '%s' to '%s'",
        frame_id_.c_str(), msg->header.frame_id.c_str());
      if (clear_on_frame_change_) {
        map_.clear();
        frame_id_ = msg->header.frame_id;
      }
    }
    last_stamp_ = msg->header.stamp;

    size_t inserted = 0;
    size_t skipped_capacity = 0;
    for (uint32_t row = 0; row < msg->height; ++row) {
      const size_t row_start = static_cast<size_t>(row) * msg->row_step;
      for (uint32_t col = 0; col < msg->width; ++col) {
        const size_t start = row_start + static_cast<size_t>(col) * msg->point_step;
        if (start + msg->point_step > msg->data.size()) {
          continue;
        }
        const auto * point = msg->data.data() + start;
        const float x = read_float32(point + x_offset);
        const float y = read_float32(point + y_offset);
        const float z = read_float32(point + z_offset);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !within_bounds(x, y, z)) {
          continue;
        }

        if (max_voxels_ > 0 && map_.size() >= static_cast<size_t>(max_voxels_)) {
          const auto key = point_to_key(x, y, z);
          if (map_.find(key) == map_.end()) {
            ++skipped_capacity;
            continue;
          }
        }

        const auto result = map_.insert(point_to_key(x, y, z));
        if (result.second) {
          ++inserted;
        }
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "voxel map voxels=%zu inserted=%zu skipped_capacity=%zu",
      map_.size(), inserted, skipped_capacity);
  }

  void publish_map()
  {
    if (map_.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp = last_stamp_;
    output.header.frame_id = frame_id_.empty() ? "map" : frame_id_;
    output.height = 1;
    output.width = static_cast<uint32_t>(map_.size());
    output.is_bigendian = false;
    output.is_dense = true;
    output.point_step = 12;
    output.row_step = output.width * output.point_step;
    output.fields.resize(3);
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
    output.data.reserve(static_cast<size_t>(output.width) * output.point_step);

    for (const auto & key : map_) {
      write_float32(output.data, static_cast<float>((static_cast<double>(key.x) + 0.5) * voxel_size_));
      write_float32(output.data, static_cast<float>((static_cast<double>(key.y) + 0.5) * voxel_size_));
      write_float32(output.data, static_cast<float>((static_cast<double>(key.z) + 0.5) * voxel_size_));
    }

    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double voxel_size_;
  double publish_rate_hz_;
  int max_voxels_;
  bool use_bounds_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  bool clear_on_frame_change_;
  std::string frame_id_;
  builtin_interfaces::msg::Time last_stamp_;
  std::unordered_set<VoxelKey, VoxelKeyHash> map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
}  // namespace slam_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_mapping::VoxelMapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
