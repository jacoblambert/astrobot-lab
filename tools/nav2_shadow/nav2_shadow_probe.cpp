// Offline installed-Nav2 inflation / footprint probe. No map or cmd publishers.
// stdin: width height resolution origin_x origin_y, row-major terrain costs,
// query_count, then x y yaw; optional motion_count and x0 y0 yaw0 x1 y1 yaw1.
// stdout: JSON of inflated grid, query costs and offline swept-area vetoes.
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "footprint_area.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    int width, height;
    double resolution, ox, oy;
    if (!(std::cin >> width >> height >> resolution >> ox >> oy) ||
        width < 1 || height < 1 || width > 200 || height > 200 ||
        resolution != .1 || !std::isfinite(ox) || !std::isfinite(oy)) {
      throw std::runtime_error("invalid bounded 10cm grid");
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("inflation.inflation_radius", .75),
      rclcpp::Parameter("inflation.cost_scaling_factor", 2.),
      rclcpp::Parameter("inflation.inflate_unknown", false),
      rclcpp::Parameter("inflation.inflate_around_unknown", false)});
    auto node = std::make_shared<nav2_util::LifecycleNode>("terrain_shadow_probe", "", options);
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf2_ros::Buffer tf(node->get_clock());
    nav2_costmap_2d::LayeredCostmap layered("map", false, true);
    layered.resizeMap(width, height, resolution, ox, oy);
    auto inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
    layered.addPlugin(inflation);
    inflation->initialize(&layered, "inflation", &tf, node, group);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    const double xs[] = {.495, .495, -.495, -.495};
    const double ys[] = {.335, -.335, -.335, .335};
    for (size_t i = 0; i < 4; ++i) {footprint[i].x = xs[i]; footprint[i].y = ys[i];}
    nav2_costmap_2d::padFootprint(footprint, .10);
    layered.setFootprint(footprint);
    auto * grid = layered.getCostmap();
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      int value;
      if (!(std::cin >> value) || (value != -1 && value != 0 && value != 20 && value != 100)) {
        throw std::runtime_error("invalid terrain cost");
      }
      // Nav2 non-trinary StaticLayer conversion with lethal threshold 100.
      const unsigned char cost = value == -1 ? nav2_costmap_2d::NO_INFORMATION :
        value == 100 ? nav2_costmap_2d::LETHAL_OBSTACLE :
        static_cast<unsigned char>((value / 100.) * nav2_costmap_2d::LETHAL_OBSTACLE);
      grid->setCost(x, y, cost);
    }
    inflation->updateCosts(*grid, 0, 0, width, height);
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(grid);
    std::cout << "{\"inflated\":[";
    for (int i = 0; i < width * height; ++i) {
      if (i) std::cout << ',';
      std::cout << static_cast<unsigned>(grid->getCharMap()[i]);
    }
    int count;
    if (!(std::cin >> count) || count < 0 || count > 50000) throw std::runtime_error("invalid query budget");
    std::cout << "],\"queries\":[";
    for (int i = 0; i < count; ++i) {
      double x, y, yaw;
      if (!(std::cin >> x >> y >> yaw) || !std::isfinite(x+y+yaw)) throw std::runtime_error("invalid query");
      unsigned mx, my;
      const int center = grid->worldToMap(x, y, mx, my) ? grid->getCost(mx, my) : 255;
      const double edge = checker.footprintCostAtPose(x, y, yaw, footprint);
      // This mirrors only the cost/footprint primitives, not a planner/controller.
      const bool accepted = center < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
        edge >= 0 && edge < nav2_costmap_2d::LETHAL_OBSTACLE;
      const bool area_accepted=accepted && footprint_area_clear(*grid,x,y,yaw,footprint);
      if (i) std::cout << ',';
      std::cout << "{\"center\":" << center << ",\"perimeter\":" << edge <<
        ",\"accepted\":" << (accepted ? "true" : "false") <<
        ",\"area_accepted\":" << (area_accepted ? "true" : "false") << '}';
    }
    std::cout << "],\"motions\":[";
    int motion_count=0;
    std::cin >> std::ws;
    if (std::cin.peek()!=std::char_traits<char>::eof() &&
        (!(std::cin >> motion_count) || motion_count<0 || motion_count>1000))
      throw std::runtime_error("invalid motion budget");
    for (int i=0;i<motion_count;++i) {
      double x0,y0,yaw0,x1,y1,yaw1;
      if (!(std::cin >> x0 >> y0 >> yaw0 >> x1 >> y1 >> yaw1)) throw std::runtime_error("invalid motion");
      const bool clear=footprint_swept_area_clear(*grid,x0,y0,yaw0,x1,y1,yaw1,footprint);
      if (i) std::cout << ',';
      std::cout << "{\"swept_area_clear\":" << (clear ? "true" : "false") << '}';
    }
    std::cout << "]}\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
