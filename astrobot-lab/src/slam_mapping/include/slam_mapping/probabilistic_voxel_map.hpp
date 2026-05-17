#ifndef SLAM_MAPPING__PROBABILISTIC_VOXEL_MAP_HPP_
#define SLAM_MAPPING__PROBABILISTIC_VOXEL_MAP_HPP_

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace slam_mapping
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct VoxelKey
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const;
};

struct OccupancyConfig
{
  double voxel_size{0.05};
  double hit_log_odds{0.85};
  double miss_log_odds{-0.20};
  double min_log_odds{-3.0};
  double max_log_odds{4.0};
  double occupied_threshold{1.0};
  double free_threshold{-0.8};
  double max_range{35.0};
  int point_stride{2};
  int ray_stride{1};
  bool use_bounds{true};
  Point3 min_bounds{-25.0, -25.0, -2.0};
  Point3 max_bounds{25.0, 25.0, 4.0};
  int max_voxels{4000000};
};

struct BevProjectionConfig
{
  double resolution{0.10};
  double width_m{40.0};
  double height_m{40.0};
  double origin_x{-20.0};
  double origin_y{-20.0};
  double low_z_percentile{0.10};
  double high_z_percentile{0.90};
  int ground_window_radius{2};
  double ground_percentile{0.10};
  int min_occupied_voxels_per_cell{1};
  double obstacle_height{0.12};
  double lethal_obstacle_height{0.45};
  int obstacle_cost_min{88};
  int obstacle_cost_max{100};
  double terrain_risk_height{0.03};
  int terrain_risk_cost_max{20};
  double occupied_threshold{1.0};
  double free_threshold{-0.8};
  double obstacle_inflation_radius{0.30};
  int obstacle_inflation_cost{55};
  int obstacle_inflation_min_cost{8};
  double obstacle_inflation_exponent{1.0};
};

struct IntegrationStats
{
  std::size_t input_points{0};
  std::size_t integrated_points{0};
  std::size_t hit_updates{0};
  std::size_t miss_updates{0};
  std::size_t skipped_invalid{0};
  std::size_t skipped_bounds{0};
  std::size_t skipped_capacity{0};
  std::size_t voxel_count{0};
};

struct BevGrid
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;
};

struct VoxelSnapshot
{
  VoxelKey key;
  Point3 center;
  float log_odds{0.0F};
};

class ProbabilisticVoxelMap
{
public:
  explicit ProbabilisticVoxelMap(OccupancyConfig config = OccupancyConfig{});

  void clear();
  IntegrationStats integratePointCloud(const Point3 & sensor_origin, const std::vector<Point3> & points);
  BevGrid projectToBev(const BevProjectionConfig & config) const;
  std::vector<VoxelSnapshot> occupiedVoxels() const;

  VoxelKey worldToKey(const Point3 & point) const;
  Point3 keyCenter(const VoxelKey & key) const;
  float logOdds(const VoxelKey & key) const;
  bool isOccupied(const VoxelKey & key) const;
  bool isFree(const VoxelKey & key) const;
  std::size_t voxelCount() const;
  const OccupancyConfig & config() const;

private:
  void updateVoxel(const VoxelKey & key, double delta);
  bool withinBounds(const Point3 & point) const;
  void integrateRay(const Point3 & origin, const Point3 & endpoint, bool add_hit);

  OccupancyConfig config_;
  std::unordered_map<VoxelKey, float, VoxelKeyHash> voxels_;
};

}  // namespace slam_mapping

#endif  // SLAM_MAPPING__PROBABILISTIC_VOXEL_MAP_HPP_
