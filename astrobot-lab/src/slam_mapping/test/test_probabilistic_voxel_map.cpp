#include "slam_mapping/probabilistic_voxel_map.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using slam_mapping::BevProjectionConfig;
using slam_mapping::OccupancyConfig;
using slam_mapping::Point3;
using slam_mapping::ProbabilisticVoxelMap;

void require(const bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int8_t cell_value(const slam_mapping::BevGrid & grid, const double x, const double y)
{
  const int col = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
  const int row = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
  require(0 <= col && col < grid.width, "BEV x coordinate outside grid");
  require(0 <= row && row < grid.height, "BEV y coordinate outside grid");
  return grid.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
    static_cast<std::size_t>(col)];
}

void test_key_round_trip()
{
  OccupancyConfig config;
  config.voxel_size = 0.10;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);
  const Point3 point{0.24, -0.06, 1.01};
  const auto key = map.worldToKey(point);
  require(key.x == 2, "x key mismatch");
  require(key.y == -1, "y key mismatch");
  require(key.z == 10, "z key mismatch");
  const auto center = map.keyCenter(key);
  require(std::abs(center.x - 0.25) < 1.0e-9, "x center mismatch");
  require(std::abs(center.y + 0.05) < 1.0e-9, "y center mismatch");
  require(std::abs(center.z - 1.05) < 1.0e-9, "z center mismatch");
}

void test_endpoint_not_cleared()
{
  OccupancyConfig config;
  config.voxel_size = 1.0;
  config.hit_log_odds = 1.0;
  config.miss_log_odds = -0.25;
  config.occupied_threshold = 0.5;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  for (int i = 0; i < 4; ++i) {
    map.integratePointCloud(Point3{0.1, 0.1, 0.1}, {Point3{3.2, 0.1, 0.1}});
  }
  require(map.isOccupied(map.worldToKey(Point3{3.2, 0.1, 0.1})), "endpoint should be occupied");
  require(map.isFree(map.worldToKey(Point3{1.2, 0.1, 0.1})), "ray body should be free");
}

void test_high_ray_does_not_clear_low_obstacle()
{
  OccupancyConfig config;
  config.voxel_size = 0.25;
  config.hit_log_odds = 1.0;
  config.miss_log_odds = -0.20;
  config.occupied_threshold = 0.5;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, 0.0, 0.25};
  const Point3 low_rock{2.0, 0.0, 0.25};
  for (int i = 0; i < 2; ++i) {
    map.integratePointCloud(origin, {low_rock});
  }
  const auto low_key = map.worldToKey(low_rock);
  require(map.isOccupied(low_key), "low rock should be initially occupied");

  const Point3 high_hit{4.0, 0.0, 1.25};
  for (int i = 0; i < 10; ++i) {
    map.integratePointCloud(origin, {high_hit});
  }
  require(map.isOccupied(low_key), "high ray should not clear low rock voxel");
}

void test_probability_is_gradual_and_clamped()
{
  OccupancyConfig config;
  config.voxel_size = 1.0;
  config.hit_log_odds = 0.7;
  config.miss_log_odds = -0.1;
  config.max_log_odds = 1.0;
  config.min_log_odds = -0.5;
  config.occupied_threshold = 0.6;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.1, 0.1, 0.1};
  const Point3 hit{2.1, 0.1, 0.1};
  for (int i = 0; i < 10; ++i) {
    map.integratePointCloud(origin, {hit});
  }
  const auto hit_key = map.worldToKey(hit);
  require(map.logOdds(hit_key) <= 1.0F, "log odds should clamp");
  require(map.isOccupied(hit_key), "repeated hit should be occupied");
}

void test_dynamic_clearing_is_progressive()
{
  OccupancyConfig config;
  config.voxel_size = 0.25;
  config.hit_log_odds = 1.0;
  config.miss_log_odds = -0.20;
  config.occupied_threshold = 0.5;
  config.free_threshold = -0.5;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, 0.0, 0.0};
  const Point3 obstacle{2.0, 0.0, 0.0};
  const Point3 clear_return_behind_obstacle{4.0, 0.0, 0.0};
  const auto obstacle_key = map.worldToKey(obstacle);

  for (int i = 0; i < 3; ++i) {
    map.integratePointCloud(origin, {obstacle});
  }
  require(map.isOccupied(obstacle_key), "obstacle should be occupied after repeated hits");

  map.integratePointCloud(origin, {clear_return_behind_obstacle});
  require(map.isOccupied(obstacle_key), "single clearing scan should not remove established obstacle");

  for (int i = 0; i < 20; ++i) {
    map.integratePointCloud(origin, {clear_return_behind_obstacle});
  }
  require(!map.isOccupied(obstacle_key), "many clearing scans should remove stale obstacle");
  require(map.isFree(obstacle_key), "many clearing scans should mark stale obstacle voxel free");
}

void test_bev_projection_marks_obstacle_column()
{
  OccupancyConfig config;
  config.voxel_size = 0.10;
  config.hit_log_odds = 1.0;
  config.occupied_threshold = 0.5;
  config.point_stride = 1;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, 0.0, 0.2};
  std::vector<Point3> points;
  for (int i = 0; i < 4; ++i) {
    points.push_back(Point3{1.0, 0.0, 0.0});
    points.push_back(Point3{1.0, 0.0, 0.35});
  }
  map.integratePointCloud(origin, points);

  BevProjectionConfig bev;
  bev.resolution = 0.20;
  bev.width_m = 4.0;
  bev.height_m = 4.0;
  bev.origin_x = -2.0;
  bev.origin_y = -2.0;
  bev.obstacle_height = 0.12;
  bev.lethal_obstacle_height = 0.30;
  bev.obstacle_inflation_radius = 0.0;
  const auto grid = map.projectToBev(bev);
  const int x = static_cast<int>(std::floor((1.0 - bev.origin_x) / bev.resolution));
  const int y = static_cast<int>(std::floor((0.0 - bev.origin_y) / bev.resolution));
  const auto value = grid.data[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
    static_cast<std::size_t>(x)];
  require(value >= 65, "BEV projection should mark obstacle column");
}

void test_known_synthetic_scene_bev()
{
  OccupancyConfig config;
  config.voxel_size = 0.10;
  config.hit_log_odds = 1.0;
  config.miss_log_odds = -0.15;
  config.occupied_threshold = 0.5;
  config.point_stride = 1;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, 0.0, 0.5};
  std::vector<Point3> scene_points;
  for (int scan = 0; scan < 3; ++scan) {
    for (double x = 0.5; x <= 2.0; x += 0.25) {
      for (double y = -0.75; y <= 0.75; y += 0.25) {
        scene_points.push_back(Point3{x, y, 0.0});
      }
    }
    scene_points.push_back(Point3{1.5, 0.25, 0.0});
    scene_points.push_back(Point3{1.5, 0.25, 0.20});
    scene_points.push_back(Point3{1.5, 0.25, 0.40});
    map.integratePointCloud(origin, scene_points);
    scene_points.clear();
  }

  BevProjectionConfig bev;
  bev.resolution = 0.10;
  bev.width_m = 5.0;
  bev.height_m = 5.0;
  bev.origin_x = -2.5;
  bev.origin_y = -2.5;
  bev.obstacle_height = 0.12;
  bev.lethal_obstacle_height = 0.35;
  bev.obstacle_inflation_radius = 0.0;
  const auto grid = map.projectToBev(bev);

  require(cell_value(grid, 1.5, 0.25) >= 65, "known box obstacle should project occupied");
  require(cell_value(grid, 1.0, -0.50) == 0, "known flat ground should project free");
}

void test_bev_projection_adds_traversable_terrain_risk()
{
  OccupancyConfig config;
  config.voxel_size = 0.05;
  config.hit_log_odds = 1.0;
  config.occupied_threshold = 0.5;
  config.point_stride = 1;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, 0.0, 0.5};
  std::vector<Point3> points;
  for (int scan = 0; scan < 3; ++scan) {
    points.push_back(Point3{1.0, 0.0, 0.0});
    points.push_back(Point3{1.0, 0.0, 0.08});
    map.integratePointCloud(origin, points);
    points.clear();
  }

  BevProjectionConfig bev;
  bev.resolution = 0.10;
  bev.width_m = 4.0;
  bev.height_m = 4.0;
  bev.origin_x = -2.0;
  bev.origin_y = -2.0;
  bev.obstacle_height = 0.12;
  bev.terrain_risk_height = 0.03;
  bev.terrain_risk_cost_max = 20;
  bev.obstacle_inflation_radius = 0.0;
  const auto grid = map.projectToBev(bev);

  const auto risk = cell_value(grid, 1.0, 0.0);
  require(risk > 0, "sub-obstacle rough terrain should get nonzero traversability risk");
  require(risk < 65, "sub-obstacle rough terrain should remain traversable");
}

void test_bev_projection_inflation_tapers_with_distance()
{
  OccupancyConfig config;
  config.voxel_size = 0.10;
  config.hit_log_odds = 1.0;
  config.occupied_threshold = 0.5;
  config.point_stride = 1;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, 0.0, 0.5};
  std::vector<Point3> points;
  for (int scan = 0; scan < 3; ++scan) {
    points.push_back(Point3{0.0, 0.0, 0.0});
    points.push_back(Point3{0.0, 0.0, 0.40});
    points.push_back(Point3{0.2, 0.0, 0.0});
    points.push_back(Point3{0.7, 0.0, 0.0});
    points.push_back(Point3{1.0, 0.0, 0.0});
    map.integratePointCloud(origin, points);
    points.clear();
  }

  BevProjectionConfig bev;
  bev.resolution = 0.10;
  bev.width_m = 4.0;
  bev.height_m = 4.0;
  bev.origin_x = -2.0;
  bev.origin_y = -2.0;
  bev.lethal_obstacle_height = 0.35;
  bev.obstacle_inflation_radius = 0.80;
  bev.obstacle_inflation_cost = 60;
  bev.obstacle_inflation_min_cost = 8;
  bev.obstacle_inflation_exponent = 1.0;
  const auto grid = map.projectToBev(bev);

  const auto near_cost = cell_value(grid, 0.25, 0.0);
  const auto far_cost = cell_value(grid, 0.65, 0.0);
  const auto outside_cost = cell_value(grid, 1.05, 0.0);
  require(near_cost > far_cost, "inflation cost should decay with distance");
  require(far_cost > outside_cost, "inflation should taper before dropping outside the radius");
}

void test_bev_projection_uses_low_ground_percentile_for_rock_core()
{
  OccupancyConfig config;
  config.voxel_size = 0.10;
  config.hit_log_odds = 1.0;
  config.occupied_threshold = 0.5;
  config.point_stride = 1;
  config.use_bounds = false;
  ProbabilisticVoxelMap map(config);

  const Point3 origin{0.0, -2.0, 0.8};
  for (int scan = 0; scan < 3; ++scan) {
    std::vector<Point3> points;
    for (int ix = -3; ix <= 3; ++ix) {
      for (int iy = -3; iy <= 3; ++iy) {
        const double x = static_cast<double>(ix) * 0.10;
        const double y = static_cast<double>(iy) * 0.10;
        if (-2 <= ix && ix <= 2 && -2 <= iy && iy <= 2) {
          points.push_back(Point3{x, y, 0.30});
          points.push_back(Point3{x, y, 0.45});
        } else {
          points.push_back(Point3{x, y, 0.0});
        }
      }
    }
    map.integratePointCloud(origin, points);
  }

  BevProjectionConfig bev;
  bev.resolution = 0.10;
  bev.width_m = 4.0;
  bev.height_m = 4.0;
  bev.origin_x = -2.0;
  bev.origin_y = -2.0;
  bev.ground_window_radius = 3;
  bev.ground_percentile = 0.10;
  bev.obstacle_height = 0.12;
  bev.lethal_obstacle_height = 0.35;
  bev.obstacle_inflation_radius = 0.0;
  const auto grid = map.projectToBev(bev);

  require(cell_value(grid, 0.0, 0.0) >= 88, "rock core should stay high-cost with contaminated local ground samples");
}

}  // namespace

int main()
{
  test_key_round_trip();
  test_endpoint_not_cleared();
  test_high_ray_does_not_clear_low_obstacle();
  test_probability_is_gradual_and_clamped();
  test_dynamic_clearing_is_progressive();
  test_bev_projection_marks_obstacle_column();
  test_known_synthetic_scene_bev();
  test_bev_projection_adds_traversable_terrain_risk();
  test_bev_projection_inflation_tapers_with_distance();
  test_bev_projection_uses_low_ground_percentile_for_rock_core();
  std::cout << "probabilistic voxel map tests passed" << std::endl;
  return 0;
}
