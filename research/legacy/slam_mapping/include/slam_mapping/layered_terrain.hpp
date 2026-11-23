#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#ifdef TERRAIN_OBSERVATIONS
#include "slam_mapping/observation_moments.hpp"
#endif
#ifdef TERRAIN_DIAGNOSTICS
#include "slam_mapping/terrain_diagnostics.hpp"
#endif

namespace slam_mapping {
// Experimental geometric terrain evidence. No GT labels, occupancy rays or
// graph corrections enter this class. The input must already be time-aligned.
struct TerrainConfig {
  int width{200}, height{200};
  double resolution{.1}, origin_x{-10}, origin_y{-10};
  double lifetime{30}, layer_band{.025}, ground_band{.03};
  double obstacle_height{.08}, max_slope{.35}, max_uncertainty{.02};
  double interpolation_gap{.30};
  bool interpolate{true};
};
enum TerrainReason : uint8_t {
  NO_RETURNS=0, MEASURED_GROUND=1, INFERRED_GROUND=2, OBSTACLE=3,
  NO_GROUND_MODEL=4, DISCONNECTED_SURFACE=5, LAYER_CAPACITY=6, UNCERTAIN_RAISED=7,
  NEGATIVE_EDGE=8, INSUFFICIENT_AREA=9, CLEARANCE_OCCLUDED=10, POSITIVE_EDGE=11,
  UNSUPPORTED_EXTENT=12
};
struct TerrainOutput {
  std::vector<int8_t> cost;
  std::vector<uint8_t> reason;
  std::vector<float> ground, uncertainty, obstacle_height, age;
};
class LayeredTerrain {
 public:
  explicit LayeredTerrain(TerrainConfig config);
  // Returns false on nonmonotonic/nonfinite stamp or invalid sensor pose.
  bool integrate(const double* xyz, size_t count, double stamp, const Eigen::Vector3d& sensor, double yaw);
  TerrainOutput project(double now);
#ifdef TERRAIN_ROLLING_GRID
  // Experimental lattice translation: preserve world geometry and immutable
  // stamps. Entering cells are unknown; no backfill or pose reintegration.
  void shift_origin(double x,double y);
#endif
#ifdef TERRAIN_DIAGNOSTICS
  const TerrainDiagnostics& diagnostics() const { return diagnostics_; }
#endif
  const TerrainConfig& config() const { return cfg_; }
  uint64_t received{0}, invalid{0}, outside{0};
 private:
  struct Scatter {
    // Covariance is symmetric. Preserve the lower triangle used by Eigen's
    // self-adjoint solver, in double precision; do not quantize geometry.
    std::array<double,6> values{};
    Eigen::Matrix3d matrix() const {
      Eigen::Matrix3d result;
      result << values[0],values[1],values[3], values[1],values[2],values[4], values[3],values[4],values[5];
      return result;
    }
    Scatter& operator+=(const Eigen::Matrix3d& value) {
      size_t k=0;
      for (int row=0;row<3;++row) for (int col=0;col<=row;++col) values[k++]+=value(row,col);
      return *this;
    }
  };
  struct Slice {
    Eigen::Vector3d mean{Eigen::Vector3d::Zero()}, peak{Eigen::Vector3d::Zero()}, trough{Eigen::Vector3d::Zero()};
    Scatter scatter;
    // Immutable start of a <=1 second aggregation bucket. Expiry may discard
    // less than one second of recent evidence, never rejuvenate old extrema.
    double first_stamp{-1};
    uint32_t count{0}, view{0};
    uint8_t views{0}, support_count{0}, corner_support{0};
#ifdef TERRAIN_TWO_PATH
    NativeSurfaceEvidence native;
#endif
    // 5x5 observed XY subregions, not occupied voxels or invented returns.
    // Fits existing alignment padding; expiry follows the same bucket.
    uint32_t xy_support{0};
    // Once a third separated point is seen, retain the certificate, not a
    // redundant third position. support[0] keeps its exact acquisition origin
    // through first_stamp and the bounded shared origin table below.
    std::array<Eigen::Vector3d,2> support{{Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()}};
#if defined(TERRAIN_OBSERVATIONS) && defined(TERRAIN_DIAGNOSTICS)
    std::array<uint32_t,4> observation_classes{};
    Scatter within_scatter;
    std::array<uint32_t,4> patch_observation_classes{};
#endif
  };
  struct Layer: Slice {
    std::vector<Slice> buckets;
  };
  // Fourth layer retains overflow evidence for conservative obstacle decisions;
  // it is never eligible to define ground.
  struct Cell { std::array<Layer,4> layers; double overflow_until{-1}; };
  struct Plane { Eigen::Vector3d abc; double sigma; bool valid{false}; };
  int index(double x, double y) const;
  Plane fit(int x, int y, const std::vector<int>& lowest,
            const std::vector<Eigen::Vector3d>& lowest_points) const;
  void expire(double now);
  static void add_support(Slice& slice,const Eigen::Vector3d& point);
  static void add(Slice& slice,const Eigen::Vector3d& point,double stamp,uint32_t view,uint32_t xy_support,uint8_t corners);
  void add(Layer& layer,const Eigen::Vector3d& point,double stamp,size_t observation_index);
  TerrainConfig cfg_;
#ifdef TERRAIN_DIAGNOSTICS
  TerrainDiagnostics diagnostics_;
#endif
  std::vector<Cell> cells_;
#ifdef TERRAIN_OBSERVATIONS
  // Native-input work state is separate from frequently scanned retained cells.
  // Projection/expiry never need to pull these matrices through their caches.
  struct NativeObservation {
    ObservationMoments moments;
    double stamp{-1};
#ifdef TERRAIN_POINTWISE_BOUNDARY
    Eigen::Vector3d boundary_direction{Eigen::Vector3d::Zero()};
    bool check_boundary{false}, boundary{false};
#endif
  };
  std::vector<NativeObservation> observations_;
  // At most four entries per cell. Reset after each accepted rigid input.
  std::vector<size_t> observed_layers_;
#ifdef TERRAIN_POINTWISE_BOUNDARY
  // Current native input only; associates every original return with the
  // exact layer selected during integration. No retained raw-cloud history.
  std::vector<size_t> native_point_layers_;
#endif
#ifdef TERRAIN_DEFER_NATIVE
  // Geometry integration stays ordered. Native votes depend only on this
  // input's immutable moments, so compute across inputs before projection.
  struct NativeRow { size_t index; ObservationMoments moments; unsigned kind{0}; bool boundary{false}; };
  struct NativeSnapshot { double stamp; std::vector<NativeRow> rows; };
  std::vector<NativeSnapshot> pending_native_;
  size_t pending_native_bytes_{0};
  void retain_native(double stamp);
  void classify_native(NativeSnapshot& snapshot,int workers) const;
  void flush_native();
#endif
#endif
  // Native 30Hz x 30s uses ~900 entries, shared across all cell buckets. A
  // metadata-capacity eviction can only remove a historical-clearance bypass;
  // it never refreshes evidence or invents an observation.
  std::map<double,Eigen::Vector3d> origins_;
  // Earliest immutable bucket start, not a rounded expiry deadline. Comparing
  // now-start preserves the original floating-point boundary exactly.
  double oldest_bucket_stamp_{std::numeric_limits<double>::infinity()};
  double last_stamp_{-1}, evidence_time_{-1}, view_yaw_{0};
  uint32_t view_{0};
  Eigen::Vector3d sensor_{Eigen::Vector3d::Zero()}, view_sensor_{Eigen::Vector3d::Zero()};
};
}
