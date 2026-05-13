#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace slam_mapping
{
class VoxelToBevCostmapNode : public rclcpp::Node
{
public:
  VoxelToBevCostmapNode()
  : Node("voxel_to_bev_costmap")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/slam/voxel_map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/slam/bev_costmap");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_m_ = declare_parameter<double>("width_m", 40.0);
    height_m_ = declare_parameter<double>("height_m", 40.0);
    origin_x_ = declare_parameter<double>("origin_x", -20.0);
    origin_y_ = declare_parameter<double>("origin_y", -20.0);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 1);
    ground_window_radius_ = declare_parameter<int>("ground_window_radius", 2);
    slope_window_radius_ = declare_parameter<int>("slope_window_radius", 1);
    obstacle_height_ = declare_parameter<double>("obstacle_height", 0.02);
    lethal_obstacle_height_ = declare_parameter<double>("lethal_obstacle_height", 0.20);
    use_height_span_evidence_ = declare_parameter<bool>("use_height_span_evidence", true);
    mark_obstacles_lethal_ = declare_parameter<bool>("mark_obstacles_lethal", true);
    obstacle_inflation_seed_height_ = declare_parameter<double>("obstacle_inflation_seed_height", 0.10);
    obstacle_seed_cost_ = declare_parameter<int>("obstacle_seed_cost", 100);
    obstacle_inflation_radius_ = declare_parameter<double>("obstacle_inflation_radius", 0.25);
    obstacle_inflation_cost_ = declare_parameter<int>("obstacle_inflation_cost", 60);
    slope_warn_ = declare_parameter<double>("slope_warn", 0.45);
    slope_lethal_ = declare_parameter<double>("slope_lethal", 1.00);
    free_fill_radius_ = declare_parameter<double>("free_fill_radius", 0.20);
    min_cost_ = declare_parameter<int>("min_cost", 1);
    unknown_value_ = declare_parameter<int>("unknown_value", -1);
    publish_debug_ = declare_parameter<bool>("publish_debug", false);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be positive");
    }
    width_ = std::max(1, static_cast<int>(std::ceil(width_m_ / resolution_)));
    height_ = std::max(1, static_cast<int>(std::ceil(height_m_ / resolution_)));

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VoxelToBevCostmapNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Voxel BEV costmap: %s -> %s resolution=%.2f size=%dx%d obstacle_warn=%.2f lethal_obstacle=%.2f inflation_seed=%.2f seed_cost=%d inflation=%.2f inflation_cost=%d free_fill=%.2f height_span=%s slope=[%.2f, %.2f]",
      input_topic_.c_str(), output_topic_.c_str(), resolution_, width_, height_, obstacle_height_,
      lethal_obstacle_height_, obstacle_inflation_seed_height_, obstacle_seed_cost_, obstacle_inflation_radius_,
      obstacle_inflation_cost_, free_fill_radius_, use_height_span_evidence_ ? "true" : "false", slope_warn_,
      slope_lethal_);
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

  int index(const int row, const int col) const { return row * width_ + col; }

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

    const int cells = width_ * height_;
    std::vector<int> counts(cells, 0);
    std::vector<double> min_z(cells, std::numeric_limits<double>::infinity());
    std::vector<double> max_z(cells, -std::numeric_limits<double>::infinity());

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
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        const int c = static_cast<int>(std::floor((static_cast<double>(x) - origin_x_) / resolution_));
        const int r = static_cast<int>(std::floor((static_cast<double>(y) - origin_y_) / resolution_));
        if (r < 0 || r >= height_ || c < 0 || c >= width_) {
          continue;
        }
        const int i = index(r, c);
        counts[i] += 1;
        min_z[i] = std::min(min_z[i], static_cast<double>(z));
        max_z[i] = std::max(max_z[i], static_cast<double>(z));
      }
    }

    std::vector<uint8_t> observed(cells, 0);
    for (int i = 0; i < cells; ++i) {
      observed[i] = counts[i] >= min_points_per_cell_ ? 1 : 0;
    }

    // Local terrain estimate: minimum of nearby low voxels. This follows slope
    // slowly but ignores rock tops inside the same cell.
    std::vector<double> ground(cells, std::numeric_limits<double>::quiet_NaN());
    for (int r = 0; r < height_; ++r) {
      for (int c = 0; c < width_; ++c) {
        double local_min = std::numeric_limits<double>::infinity();
        for (int dr = -ground_window_radius_; dr <= ground_window_radius_; ++dr) {
          for (int dc = -ground_window_radius_; dc <= ground_window_radius_; ++dc) {
            const int rr = r + dr;
            const int cc = c + dc;
            if (rr < 0 || rr >= height_ || cc < 0 || cc >= width_) {
              continue;
            }
            const int j = index(rr, cc);
            if (observed[j]) {
              local_min = std::min(local_min, min_z[j]);
            }
          }
        }
        if (std::isfinite(local_min)) {
          ground[index(r, c)] = local_min;
        }
      }
    }

    nav_msgs::msg::OccupancyGrid output;
    output.header = msg->header;
    output.info = nav_msgs::msg::MapMetaData();
    output.info.map_load_time = msg->header.stamp;
    output.info.resolution = resolution_;
    output.info.width = static_cast<uint32_t>(width_);
    output.info.height = static_cast<uint32_t>(height_);
    output.info.origin.position.x = origin_x_;
    output.info.origin.position.y = origin_y_;
    output.info.origin.orientation.w = 1.0;
    output.data.assign(cells, static_cast<int8_t>(unknown_value_));

    int observed_cells = 0;
    int lethal_cells = 0;
    std::vector<uint8_t> obstacle_seed(cells, 0);
    for (int r = 0; r < height_; ++r) {
      for (int c = 0; c < width_; ++c) {
        const int i = index(r, c);
        if (!observed[i] || !std::isfinite(ground[i])) {
          continue;
        }
        ++observed_cells;

        const double obstacle_residual = std::max(0.0, max_z[i] - ground[i]);
        const double height_span = std::max(0.0, max_z[i] - min_z[i]);
        const double obstacle_evidence =
          use_height_span_evidence_ ? std::max(obstacle_residual, height_span) : obstacle_residual;
        const double obstacle_cost = mark_obstacles_lethal_ && obstacle_evidence >= lethal_obstacle_height_
          ? 100.0
          : scaled_cost(obstacle_evidence, obstacle_height_, lethal_obstacle_height_);

        double slope = 0.0;
        if (slope_window_radius_ > 0) {
          const int left = nearest_observed_ground(r, c, 0, -slope_window_radius_, ground, observed);
          const int right = nearest_observed_ground(r, c, 0, slope_window_radius_, ground, observed);
          const int down = nearest_observed_ground(r, c, -slope_window_radius_, 0, ground, observed);
          const int up = nearest_observed_ground(r, c, slope_window_radius_, 0, ground, observed);
          if (left >= 0 && right >= 0) {
            slope = std::max(slope, std::abs(ground[right] - ground[left]) / (2.0 * slope_window_radius_ * resolution_));
          }
          if (down >= 0 && up >= 0) {
            slope = std::max(slope, std::abs(ground[up] - ground[down]) / (2.0 * slope_window_radius_ * resolution_));
          }
        }
        const double slope_cost = scaled_cost(slope, slope_warn_, slope_lethal_);
        int cost = std::clamp(static_cast<int>(std::round(std::max(obstacle_cost, slope_cost))), min_cost_, 100);
        if (obstacle_evidence >= obstacle_inflation_seed_height_) {
          cost = std::max(cost, std::clamp(obstacle_seed_cost_, 1, 100));
        }
        output.data[i] = static_cast<int8_t>(cost);
        obstacle_seed[i] = obstacle_evidence >= obstacle_inflation_seed_height_ ? 1 : 0;
      }
    }

    const int inflation_cells = std::max(0, static_cast<int>(std::ceil(obstacle_inflation_radius_ / resolution_)));
    if (inflation_cells > 0 && obstacle_inflation_cost_ > 0) {
      const int inflated_cost = std::clamp(obstacle_inflation_cost_, 1, 100);
      for (int r = 0; r < height_; ++r) {
        for (int c = 0; c < width_; ++c) {
          const int seed_index = index(r, c);
          if (!obstacle_seed[seed_index]) {
            continue;
          }
          for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
            for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
              const int rr = r + dr;
              const int cc = c + dc;
              if (rr < 0 || rr >= height_ || cc < 0 || cc >= width_) {
                continue;
              }
              const double distance = std::hypot(static_cast<double>(dr), static_cast<double>(dc)) * resolution_;
              if (distance > obstacle_inflation_radius_) {
                continue;
              }
              const int i = index(rr, cc);
              output.data[i] = static_cast<int8_t>(std::max(static_cast<int>(output.data[i]), inflated_cost));
            }
          }
        }
      }
    }

    const int fill_cells = std::max(0, static_cast<int>(std::ceil(free_fill_radius_ / resolution_)));
    if (fill_cells > 0) {
      for (int r = 0; r < height_; ++r) {
        for (int c = 0; c < width_; ++c) {
          const int source_index = index(r, c);
          if (!observed[source_index] || output.data[source_index] >= obstacle_inflation_cost_) {
            continue;
          }
          for (int dr = -fill_cells; dr <= fill_cells; ++dr) {
            for (int dc = -fill_cells; dc <= fill_cells; ++dc) {
              const int rr = r + dr;
              const int cc = c + dc;
              if (rr < 0 || rr >= height_ || cc < 0 || cc >= width_) {
                continue;
              }
              const double distance = std::hypot(static_cast<double>(dr), static_cast<double>(dc)) * resolution_;
              if (distance > free_fill_radius_) {
                continue;
              }
              const int i = index(rr, cc);
              if (output.data[i] == unknown_value_) {
                output.data[i] = static_cast<int8_t>(min_cost_);
              }
            }
          }
        }
      }
    }

    for (const auto cost : output.data) {
      if (cost >= 100) {
        ++lethal_cells;
      }
    }

    costmap_pub_->publish(output);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "BEV costmap observed=%d lethal=%d input_points=%u",
      observed_cells, lethal_cells, msg->width * msg->height);
  }

  int nearest_observed_ground(
    const int row, const int col, const int dr, const int dc, const std::vector<double> & ground,
    const std::vector<uint8_t> & observed) const
  {
    const int rr = row + dr;
    const int cc = col + dc;
    if (rr < 0 || rr >= height_ || cc < 0 || cc >= width_) {
      return -1;
    }
    const int i = index(rr, cc);
    return observed[i] && std::isfinite(ground[i]) ? i : -1;
  }

  static double scaled_cost(const double value, const double warn, const double lethal)
  {
    if (value >= lethal) {
      return 100.0;
    }
    if (value <= warn) {
      return 0.0;
    }
    const double t = (value - warn) / std::max(lethal - warn, 1e-6);
    return 1.0 + 98.0 * std::clamp(t, 0.0, 1.0);
  }

  std::string input_topic_;
  std::string output_topic_;
  double resolution_;
  double width_m_;
  double height_m_;
  double origin_x_;
  double origin_y_;
  int width_;
  int height_;
  int min_points_per_cell_;
  int ground_window_radius_;
  int slope_window_radius_;
  double obstacle_height_;
  double lethal_obstacle_height_;
  bool use_height_span_evidence_;
  bool mark_obstacles_lethal_;
  double obstacle_inflation_seed_height_;
  int obstacle_seed_cost_;
  double obstacle_inflation_radius_;
  int obstacle_inflation_cost_;
  double slope_warn_;
  double slope_lethal_;
  double free_fill_radius_;
  int min_cost_;
  int unknown_value_;
  bool publish_debug_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace slam_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_mapping::VoxelToBevCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
