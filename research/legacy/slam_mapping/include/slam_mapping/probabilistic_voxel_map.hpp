#ifndef SLAM_MAPPING__PROBABILISTIC_VOXEL_MAP_HPP_
#define SLAM_MAPPING__PROBABILISTIC_VOXEL_MAP_HPP_

#include <cstdint>
#include <array>
#include <unordered_set>
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
  // Temporal forgetting: each integration cycle every voxel's log-odds is multiplied
  // by decay_factor (1.0 = disabled), and voxels whose |log_odds| falls below
  // decay_min_abs_log_odds are pruned. Re-observed voxels (hit every cycle) stay
  // saturated; un-reobserved voxels (e.g. yaw-drift smear ghosts) fade to unknown
  // without needing a clearing ray to traverse them.
  double decay_factor{1.0};
  double decay_min_abs_log_odds{0.05};

  // Near-field obstacle persistence (blind-cone latch). Within this XY radius of the sensor origin,
  // an already-occupied voxel is NOT miss-cleared or decayed. Up close, LiDAR beams pass OVER low
  // rocks to the ground beyond and cast misses through the rock voxel -> it clears -> the rover drives
  // into the now-"free" rock and wedges (confirmed live). Latching keeps a rock seen at range marked as
  // the rover closes in. Self-correcting: once the rover backs >radius away, normal clearing resumes
  // (so a phantom is re-observed and cleared from afar). 0 = disabled.
  double near_field_persist_radius{0.0};
  // New evidence model is opt-in for reproducible comparison with archived maps.
  bool use_surface_evidence{false};
  double surface_max_age_sec{30.0};
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
  // Ground reference for the height bands. false = neighborhood-minimum percentile (downhill
  // floor over the window): on a slope this reference sits downhill, so height-above-ground is
  // inflated by the slope rise across the window and a smooth ramp (~>22 deg with the default
  // 0.5 m window) falsely reads as an obstacle. true = least-squares PLANE fit over the window's
  // low_z (detrends the linear slope): height becomes residual-above-local-plane, so a rock on a
  // slope reads as the rock's height and bare sloped regolith reads ~0. The plane gradient also
  // yields the TRUE local slope (used for the slope cost instead of the gradient-of-minimum).
  bool use_plane_fit{false};
  // Trim threshold (m) for the robust plane fit: window cells whose low_z sits more than this
  // above the first-pass plane are excluded from the ground refit (they're obstacle tops), so the
  // plane stays on true ground and a compact rock filling the window isn't absorbed into ground.
  double plane_fit_trim{0.05};
  // R-GPF (Region-wise Ground Plane Fitting, Patchwork; arXiv 2108.05560). The plane_fit_trim above does
  // fit-ALL-then-trim-ONCE: the first pass includes rock tops so the plane is pulled UP, and trimming only
  // drops points already above that inflated plane -> on a rock that fills/dominates the window the refit
  // still RIDES UP (proven: a 0.34 m rock read 0.06 m -> missed). R-GPF instead SEEDS the fit from the
  // LOWEST cells (start on true ground), then iterates a ONE-SIDED inlier band (keep only cells at/below
  // the plane) so rock tops are rejected every pass and the plane converges onto the ground floor. Same
  // 0.5 m window + same fit() LSQ -> keeps the proven curvature immunity; just seeded + iterated. When
  // enabled, replaces the single trim pass.
  bool use_gpf_ground{false};
  double gpf_seed_fraction{0.30};   // fraction of window low_z cells (lowest by z) used as the initial seed
  double gpf_inlier_band{0.05};     // one-sided residual band (m): cells with z-plane <= band are ground
  int gpf_iterations{3};            // seed + refit passes
  // Robust ground FLOOR for the obstacle-height check (2026-06-17). The plane-fit ground above rides
  // UP onto a rock that fills the (small) window -> a 0.34 m rock measured ~0.06 m above its own
  // rock-contaminated ground -> missed (proven: 0/14 footprint cells marked; LiDAR saw it richly).
  // Enlarging the plane window does NOT help (plane rides up at every radius). Fix: also measure height
  // above a robust LOW-PERCENTILE FLOOR of low_z over a WIDER window (immune to a rock filling it), take
  // the MAX with the plane-based height. To keep the plane-fit's slope-FP suppression, detrend the floor
  // term by the TRUE terrain slope (fit to the floor/ground cells only) across the floor window extent.
  bool use_ground_floor{true};
  int ground_floor_window_radius{6};       // ~1.3 m window: wide enough to find ground around a rock
  double ground_floor_percentile{0.10};    // robust ground level = this percentile of window low_z
  double ground_floor_slope_band{0.15};    // cells within this of the floor are "ground" -> true-slope fit
  int min_occupied_voxels_per_cell{1};
  // A cell only becomes a cost-100 obstacle SEED (lethal + inflated + poisons exploration
  // reachability) if its peak voxel log-odds >= this. Default = occupied_threshold (no-op:
  // any occupied voxel qualifies); raise it (e.g. 2.0) to require multi-frame confirmation so
  // lowering min_occupied_voxels_per_cell recovers distant rocks without near-field noise
  // flooding false seeds at startup.
  double seed_confidence_log_odds{1.0};
  // If true, a cell admitted at exactly min_occupied_voxels_per_cell must show an intrinsic
  // vertical span >= obstacle_height (a real surface) to be classified -- rejects flat noise pairs.
  bool single_cell_height_gate{false};
  double obstacle_height{0.12};
  double lethal_obstacle_height{0.45};
  int obstacle_cost_min{88};
  int obstacle_cost_max{100};
  double terrain_risk_height{0.03};
  int terrain_risk_cost_max{20};
  // Slope (crater-rim) costing from the local ground-height gradient. Smooth slopes barely exceed
  // terrain_risk_height (they don't "stick up" above local ground), so the height bands read them as
  // free; this adds a soft "avoid unless necessary" cost proportional to local ground slope. Keep
  // below the exploration precheck high-cost threshold so craters stay traversable, just dispreferred.
  double slope_cost_max{0.0};   // 0 = disabled; soft cost ceiling for the steepest slopes
  double slope_min{0.12};       // rise/run where slope cost starts (~7 deg)
  double slope_max{0.5};        // rise/run where slope cost saturates (~27 deg)
  int slope_window_cells{3};    // half-window (cells) for the ground-gradient estimate
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
  // Measurement times supporting obstacle height, never publication times.
  std::vector<double> obstacle_observed_at;
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
  IntegrationStats integratePointCloud(const Point3 & sensor_origin, const std::vector<Point3> & points,
                                      double stamp = std::numeric_limits<double>::quiet_NaN());
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
  struct Surface {
    std::array<float, 3> low{};
    std::array<float, 3> high{};
    std::array<uint64_t, 3> scans{};
    std::array<double, 3> stamps{};
    double last_hit{0};
    bool confirmed{false};
  };
  std::unordered_map<VoxelKey, Surface, VoxelKeyHash> surfaces_;
  std::unordered_set<VoxelKey, VoxelKeyHash> frame_misses_;
  uint64_t scan_id_{0};
  double stamp_{-1};
};

}  // namespace slam_mapping

#endif  // SLAM_MAPPING__PROBABILISTIC_VOXEL_MAP_HPP_
