#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace pointcloud_preprocessor
{
class PointCloudCropFilterNode : public rclcpp::Node
{
public:
  PointCloudCropFilterNode()
  : Node("pointcloud_crop_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/pointcloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/pointcloud/filtered");
    min_x_ = declare_parameter<double>("min_x", 0.20);
    max_x_ = declare_parameter<double>("max_x", 0.80);
    min_y_ = declare_parameter<double>("min_y", -0.45);
    max_y_ = declare_parameter<double>("max_y", 0.45);
    min_z_ = declare_parameter<double>("min_z", -0.35);
    max_z_ = declare_parameter<double>("max_z", 0.05);

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&PointCloudCropFilterNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Cropping %s -> %s inside box x=[%.2f, %.2f] y=[%.2f, %.2f] z=[%.2f, %.2f]",
      input_topic_.c_str(), output_topic_.c_str(), min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
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

    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header;
    output.height = 1;
    output.fields = msg->fields;
    output.is_bigendian = msg->is_bigendian;
    output.point_step = msg->point_step;
    output.is_dense = false;
    output.data.reserve(static_cast<size_t>(msg->width) * msg->height * msg->point_step);

    size_t removed = 0;
    size_t total = 0;
    for (uint32_t row = 0; row < msg->height; ++row) {
      const size_t row_start = static_cast<size_t>(row) * msg->row_step;
      for (uint32_t col = 0; col < msg->width; ++col) {
        ++total;
        const size_t start = row_start + static_cast<size_t>(col) * msg->point_step;
        if (start + msg->point_step > msg->data.size()) {
          continue;
        }

        const auto * point = msg->data.data() + start;
        const float x = read_float32(point + x_offset);
        const float y = read_float32(point + y_offset);
        const float z = read_float32(point + z_offset);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const bool inside_crop_box =
          min_x_ <= x && x <= max_x_ &&
          min_y_ <= y && y <= max_y_ &&
          min_z_ <= z && z <= max_z_;
        if (inside_crop_box) {
          ++removed;
          continue;
        }

        output.data.insert(output.data.end(), point, point + msg->point_step);
      }
    }

    output.width = static_cast<uint32_t>(output.data.size() / output.point_step);
    output.row_step = output.width * output.point_step;
    publisher_->publish(output);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "pointcloud crop removed=%zu kept=%u total=%zu", removed, output.width, total);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace pointcloud_preprocessor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_preprocessor::PointCloudCropFilterNode>());
  rclcpp::shutdown();
  return 0;
}
