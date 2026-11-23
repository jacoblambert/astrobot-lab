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

#include "astrobot_terrain/layered_terrain.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#ifdef _OPENMP
#include <omp.h>
#endif

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <queue>
#include <stdexcept>

#include "astrobot_terrain/angular_support.hpp"

namespace astrobot_terrain
{
LayeredTerrain::LayeredTerrain(TerrainConfig config)
: cfg_(config)
{
  for (double value : {cfg_.layer_band, cfg_.ground_band, cfg_.obstacle_height,
      cfg_.max_slope, cfg_.max_uncertainty, cfg_.interpolation_gap})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("nonfinite terrain configuration");}}
  if (cfg_.width < 1 || cfg_.height < 1 || int64_t(cfg_.width) * cfg_.height > 1000000 ||
    !std::isfinite(cfg_.resolution) || cfg_.resolution <= 0 || !std::isfinite(cfg_.lifetime) ||
    cfg_.lifetime <= 0 || cfg_.lifetime > 300 ||
    !std::isfinite(cfg_.origin_x) || !std::isfinite(cfg_.origin_y) || cfg_.layer_band <= 0 ||
    cfg_.max_uncertainty <= 0 || cfg_.ground_band <= 0 ||
    cfg_.obstacle_height <= cfg_.ground_band ||
    cfg_.max_slope <= 0 || cfg_.interpolation_gap < 0)
  {
    throw std::invalid_argument("invalid terrain geometry/configuration");
  }
  cells_.resize(cfg_.width * cfg_.height);
  observations_.resize(4 * cells_.size());
}
void LayeredTerrain::shift_origin(double x, double y)
{
  const double dx = (x - cfg_.origin_x) / cfg_.resolution,
    dy = (y - cfg_.origin_y) / cfg_.resolution;
  if (!std::isfinite(dx) || !std::isfinite(dy) ||
    std::abs(dx - std::round(dx)) > 1e-8 || std::abs(dy - std::round(dy)) > 1e-8 ||
    std::abs(dx) > 1000000 || std::abs(dy) > 1000000)
  {
    throw std::invalid_argument("rolling origin must translate by bounded integer cells");
  }
  const int sx = static_cast<int>(std::round(dx)), sy = static_cast<int>(std::round(dy));
  if (!sx && !sy) {return;}
  // Pending votes address the OLD grid. Commit them before moving cells.
  flush_native();
  // Allocate before moving state. Re-entering a region cannot resurrect its
  // evicted free space: newly exposed cells contain no observations.
  std::vector<Cell> shifted(cells_.size());
  for (int yy = 0; yy < cfg_.height; ++yy) {
    for (int xx = 0; xx < cfg_.width; ++xx) {
      const int old_x = xx + sx, old_y = yy + sy;
      if (old_x >= 0 && old_y >= 0 && old_x < cfg_.width && old_y < cfg_.height) {
        shifted[yy * cfg_.width + xx] = std::move(cells_[old_y * cfg_.width + old_x]);
      }
    }
  }
  cells_.swap(shifted);
  cfg_.origin_x = x; cfg_.origin_y = y;
  std::fill(observations_.begin(), observations_.end(), NativeObservation{});
  observed_layers_.clear();
  // The retained oldest-stamp lower bound can cause extra expiry work, never
  // delayed expiry. Sensor/origin history and all bucket stamps stay unchanged.
}
void LayeredTerrain::add_support(Slice & layer, const Eigen::Vector3d & point)
{
  if (layer.support_count >= 3) {return;}
  for (uint8_t k = 0; k < layer.support_count; ++k) {
    if ((point - layer.support[k]).norm() < .003) {return;}}
  if (layer.support_count < 2) {layer.support[layer.support_count] = point;}
  ++layer.support_count;
}
void LayeredTerrain::add(
  Slice & layer, const Eigen::Vector3d & point, double stamp, uint32_t view,
  uint32_t xy_support, uint8_t corners)
{
  layer.xy_support |= xy_support;
  layer.corner_support |= corners;
  if (!layer.count) {
    layer.mean = layer.peak = layer.trough = point; layer.first_stamp = stamp;
    layer.count = 1; layer.view = view; layer.views = 1;
    layer.support[0] = point; layer.support_count = 1;
    return;
  }
  const double weight = 1. / (layer.count + 1.);
  const Eigen::Vector3d delta = point - layer.mean;
  layer.mean += (point - layer.mean) * weight;
  layer.scatter += delta * (point - layer.mean).transpose();
  if (point.z() > layer.peak.z()) {layer.peak = point;}
  if (point.z() < layer.trough.z()) {layer.trough = point;}
  ++layer.count;
  if (layer.view != view) {layer.view = view; layer.views = 2;}
  add_support(layer, point);
}
void LayeredTerrain::add(
  Layer & layer, const Eigen::Vector3d & point, double stamp,
  [[maybe_unused]] size_t observation_index)
{
  auto & observation = observations_[observation_index];
  if (observation.stamp != stamp) {
    observation.moments = {}; observation.stamp = stamp;
    observed_layers_.push_back(observation_index);
  }
  observation.moments.add(point);
  if (layer.buckets.empty() || std::floor(layer.buckets.back().first_stamp) != std::floor(stamp)) {
    if (layer.buckets.size() == layer.buckets.capacity()) {
      const size_t limit = size_t(std::ceil(cfg_.lifetime)) + 1;
      if (layer.buckets.size() >= limit) {
        throw std::logic_error("immutable bucket bound exceeded");
      }
      layer.buckets.reserve(std::min(limit, std::max(size_t(1), 2 * layer.buckets.capacity())));
    }
    layer.buckets.emplace_back();
    oldest_bucket_stamp_ = std::min(oldest_bucket_stamp_, stamp);
  }
  const double gx = (point.x() - cfg_.origin_x) / cfg_.resolution,
    gy = (point.y() - cfg_.origin_y) / cfg_.resolution;
  const int sx = std::clamp(static_cast<int>((gx - std::floor(gx)) * 5), 0, 4),
    sy = std::clamp(static_cast<int>((gy - std::floor(gy)) * 5), 0, 4);
  const uint32_t xy_support = 1u << (sy * 5 + sx);
  uint8_t corners = 0;
  for (int y = 0; y <= 1; ++y) {
    for (int x = 0; x <= 1; ++x) {
      const double dx = (gx - std::floor(gx) - x) * cfg_.resolution,
        dy = (gy - std::floor(gy) - y) * cfg_.resolution;
      if (std::abs(dx) > 1e-6 && std::abs(dy) > 1e-6 &&
        std::abs(std::abs(dx) - std::abs(dy)) > 1e-6)
      {
        corners |= 1u << (2 * (2 * y + x) + (std::abs(dy) > std::abs(dx)));
      }
    }
  }
  add(layer.buckets.back(), point, stamp, view_, xy_support, corners);
  add(static_cast<Slice &>(layer), point, stamp, view_, xy_support, corners);
}
void LayeredTerrain::expire(double now)
{
  while (!origins_.empty() && now - origins_.begin()->first > cfg_.lifetime) {
    origins_.erase(origins_.begin());
  }
  // No bucket can expire before the earliest one. In particular, projection
  // at the integration stamp need not scan every cell a second time. New
  // buckets always update this bound and can never already be expired.
  if (now - oldest_bucket_stamp_ <= cfg_.lifetime) {return;}
  // Votes must reach their original buckets BEFORE any bucket is expired.
  flush_native();
  double oldest = std::numeric_limits<double>::infinity();
#ifdef _OPENMP
  const int workers = std::min(4, omp_get_max_threads());
  const bool many_cells = cells_.size() >= 4096;
#pragma omp parallel for schedule(static) num_threads(workers) reduction(min:oldest) if(many_cells)
#endif
  for (size_t cell_index = 0; cell_index < cells_.size(); ++cell_index) {
    for (auto & layer : cells_[cell_index].layers) {
      auto & buckets = layer.buckets;
      if (buckets.empty()) {continue;}
      if (now - buckets.front().first_stamp <= cfg_.lifetime) {
        oldest = std::min(oldest, buckets.front().first_stamp);
        continue;
      }
      buckets.erase(buckets.begin(),
          std::find_if(buckets.begin(), buckets.end(), [&](const auto & s){
          return now - s.first_stamp <= cfg_.lifetime;
    }));
      if (!buckets.empty()) {oldest = std::min(oldest, buckets.front().first_stamp);}
      Slice aggregate;
      for (const auto & s : buckets) {
        if (!aggregate.count) {aggregate = s; continue;}
        const Eigen::Vector3d delta = s.mean - aggregate.mean;
        aggregate.scatter += s.scatter.matrix() + delta * delta.transpose() *
          (static_cast<double>(s.count) * aggregate.count / (aggregate.count + s.count));
        aggregate.mean += delta * (static_cast<double>(s.count) / (aggregate.count + s.count));
        aggregate.count += s.count;
        aggregate.xy_support |= s.xy_support;
        aggregate.corner_support |= s.corner_support;
        aggregate.native.merge(s.native);
        if (s.peak.z() > aggregate.peak.z()) {aggregate.peak = s.peak;}
        if (s.trough.z() < aggregate.trough.z()) {aggregate.trough = s.trough;}
        if (s.views >= 2 || s.view != aggregate.view) {aggregate.views = 2;}
        aggregate.view = s.view;
      // A live bucket with three distinct points independently certifies
      // spatial support, irrespective of the aggregate's greedy pair choice.
        if (s.support_count >= 3) {aggregate.support_count = 3;} else {
          for (uint8_t k = 0; k < s.support_count; ++k) {
            add_support(aggregate, s.support[k]);
          }
        }
      }
      static_cast<Slice &>(layer) = aggregate;
    }
  }
  oldest_bucket_stamp_ = oldest;
}
int LayeredTerrain::index(double x, double y) const
{
  const double col = std::floor((x - cfg_.origin_x) / cfg_.resolution),
    row = std::floor((y - cfg_.origin_y) / cfg_.resolution);
  if (col < 0 || row < 0 || col >= cfg_.width || row >= cfg_.height) {return -1;}
  return static_cast<int>(row) * cfg_.width + static_cast<int>(col);
}
bool LayeredTerrain::integrate(
  const double * xyz, size_t count, double stamp,
  const Eigen::Vector3d & sensor, double yaw)
{
  if ((count && !xyz) || !std::isfinite(stamp) || stamp < 0 || stamp <= last_stamp_ ||
    stamp < evidence_time_ || !sensor.allFinite() || !std::isfinite(yaw)) {return false;}
  expire(stamp);
  evidence_time_ = stamp;
  if (!view_ || (sensor - view_sensor_).norm() >= .1 || std::abs(std::remainder(yaw - view_yaw_,
      2 * M_PI)) >= 5 * M_PI / 180)
  {
    ++view_; view_sensor_ = sensor; view_yaw_ = yaw;
  }
  last_stamp_ = stamp; sensor_ = sensor; received += count;
  observed_layers_.clear();
  if (count > 64 * 1024 * 1024 / sizeof(size_t)) {
    throw std::length_error("native point association exceeds 64MiB bound");
  }
  native_point_layers_.assign(count, std::numeric_limits<size_t>::max());
  if (count) {
    origins_.emplace(stamp, sensor);
    if (origins_.size() > 4096) {origins_.erase(origins_.begin());}
  }
  for (size_t i = 0; i < count; ++i) {
    const Eigen::Vector3d point(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    if (!point.allFinite()) {++invalid; continue;}
    const int id = index(point.x(), point.y());
    if (id < 0) {++outside; continue;}
    auto & cell = cells_[id];
    int match = -1, empty = -1; double best = cfg_.layer_band;
    for (int j = 0; j < 3; ++j) {
      auto & layer = cell.layers[j];
      if (!layer.count) {empty = j; continue;}
      const double dz = std::abs(point.z() - layer.mean.z());
      if (dz <= best) {best = dz; match = j;}
    }
    if (match < 0) {
      if (empty < 0) {
        cell.overflow_until = stamp + cfg_.lifetime; add(cell.layers[3], point, stamp, 4 * id + 3);
        native_point_layers_[i] = 4 * id + 3;
        continue;
      }
      add(cell.layers[empty], point, stamp, 4 * id + empty);
      native_point_layers_[i] = 4 * id + empty;
    } else {
      add(cell.layers[match], point, stamp, 4 * id + match);
      native_point_layers_[i] = 4 * id + match;
    }
  }
  // sqrt(sum residual^2) is an upper bound, not an observed displacement.
  // Exceeding it does NOT demonstrate a boundary: sample density alone can
  // cross that bound on millimetre roughness. Keep its cheap clear certificate,
  // but require an actual >5mm return when the bound is inconclusive. No RMS,
  // percentile, discarded outlier or larger geometric threshold is used.
#ifdef _OPENMP
  const int boundary_workers = std::min(4, omp_get_max_threads());
  const bool parallel_boundary = observed_layers_.size() >= 1024;
#pragma omp parallel for schedule(static) num_threads(boundary_workers) if(parallel_boundary)
#endif
  for (size_t entry = 0; entry < observed_layers_.size(); ++entry) {
    auto & observation = observations_[observed_layers_[entry]];
    const auto & moments = observation.moments;
    observation.boundary_direction = moments.bounded_gradient_direction(cfg_.max_slope);
    observation.boundary = !observation.boundary_direction.allFinite();
    const double squared = observation.boundary_direction.dot(moments.scatter *
        observation.boundary_direction);
    observation.check_boundary = !observation.boundary && squared > .005 * .005;
  }
  for (size_t i = 0; i < count; ++i) {
    const size_t at = native_point_layers_[i];
    if (at == std::numeric_limits<size_t>::max()) {continue;}
    auto & observation = observations_[at];
    if (!observation.check_boundary || observation.boundary) {continue;}
    const Eigen::Vector3d point(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    if (std::abs((point - observation.moments.mean).dot(observation.boundary_direction)) > .005) {
      observation.boundary = true;
    }
  }
  // Large native inputs already meet the existing intra-scan worker threshold.
  // Keep their eager path; batching them only adds copying and commit overhead.
  if (observed_layers_.size() < 1024) {retain_native(stamp); return true;}
  flush_native();
  // Lazily reuse the EXACT ordered neighborhood merge for layers in one cell.
  // Never regroup/reorder point moments or reuse a patch across native inputs.
  struct NativePatch
  {
    ObservationMoments moments;
    bool computed{false};
    ObservationMoments::GroundNormal normal;
  };
  std::vector<NativePatch> patches(cells_.size());
  std::vector<int> patch_centers;
  for (size_t at : observed_layers_) {
    const auto & observation = observations_[at].moments;
    const int center = index(observation.mean.x(), observation.mean.y());
    if (center >= 0 && !patches[center].computed) {
      patches[center].computed = true;
      patch_centers.push_back(center);
    }
  }
  // A strictly increasing input stamp means the touched layers are exactly
  // the current observation. Sort their indices to preserve dy/dx/layer merge
  // order while skipping empty/stale neighbors, without changing arithmetic.
  auto active_layers = observed_layers_;
  std::sort(active_layers.begin(), active_layers.end());
#ifdef _OPENMP
  const int native_workers = std::min(4, omp_get_max_threads());
  const bool parallel_patches = patch_centers.size() >= 1024;
#pragma omp parallel for schedule(static) num_threads(native_workers) if(parallel_patches)
#endif
  for (size_t p = 0; p < patch_centers.size(); ++p) {
    const int center = patch_centers[p];
    auto & cached = patches[center];
    const int cx = center % cfg_.width, cy = center / cfg_.width;
    const int left = std::max(0, cx - 4), right = std::min(cfg_.width - 1, cx + 4);
    for (int y = std::max(0, cy - 4); y <= std::min(cfg_.height - 1, cy + 4); ++y) {
      const size_t first = 4 * (y * cfg_.width + left), end = 4 * (y * cfg_.width + right + 1);
      auto at = std::lower_bound(active_layers.begin(), active_layers.end(), first);
      for (; at != active_layers.end() && *at < end; ++at) {
        cached.moments.merge(observations_[*at].moments);
      }
    }
    cached.normal = cached.moments.ground_normal(cfg_.max_slope);
  }
  // All moments/patches are now immutable. Each unique touched layer owns its
  // own aggregate and final bucket; no worker updates another layer's votes.
#ifdef _OPENMP
  const bool parallel_layers = observed_layers_.size() >= 1024;
#pragma omp parallel for schedule(static) num_threads(native_workers) if(parallel_layers)
#endif
  for (size_t entry = 0; entry < observed_layers_.size(); ++entry) {
    const size_t observation_index = observed_layers_[entry];
    auto * layer = &cells_[observation_index / 4].layers[observation_index % 4];
    const auto & observation = observations_[observation_index].moments;
    const unsigned kind = observation.normal_class(cfg_.max_slope);
    unsigned patch_kind = kind;
    // Independent native residual evidence. A ground-normal vote must never
    // clear a previously observed face; this flag expires with its bucket.
    bool native_boundary = false;
    {
      const int center = index(observation.mean.x(), observation.mean.y());
      ObservationMoments empty;
      const auto & patch = center >= 0 ? patches[center].moments : empty;
      // Same fixed 9x9 stencil as the ground fit, but only this rigid input;
      // 5mm is the existing fit's minimum sigma, not a widened ground band.
      const bool supported = center >= 0 && observation.supported_by_cached(patch,
          patches[center].normal, .005);
      // Separate local discontinuity from an offset relative to a larger,
      // possibly curved patch. No normal/ground certificate follows from this
      // residual alone; contextual support still controls the other path.
      native_boundary = observations_[observation_index].boundary;
      if (kind == 0 || kind == 2) {patch_kind = supported ? 2 : 0;}
    }
    layer->native.add(patch_kind, native_boundary);
    layer->buckets.back().native.add(patch_kind, native_boundary);
  }
  return true;
}
void LayeredTerrain::retain_native(double stamp)
{
  if (observed_layers_.empty()) {return;}
  const size_t bytes = observed_layers_.size() * sizeof(NativeRow);
  constexpr size_t limit = 64 * 1024 * 1024;
  if (bytes > limit) {throw std::length_error("single native snapshot exceeds 64MiB work bound");}
  if (pending_native_bytes_ + bytes > limit || pending_native_.size() >= 2048) {flush_native();}
  NativeSnapshot snapshot{stamp, {}};
  snapshot.rows.reserve(observed_layers_.size());
  for (size_t at : observed_layers_) {
    snapshot.rows.push_back({at, observations_[at].moments});
    snapshot.rows.back().boundary = observations_[at].boundary;
  }
  std::sort(snapshot.rows.begin(), snapshot.rows.end(),
    [](const auto & a, const auto & b){
      return a.index < b.index;
    });
  pending_native_.push_back(std::move(snapshot));
  pending_native_bytes_ += bytes;
}
void LayeredTerrain::classify_native(NativeSnapshot & snapshot, [[maybe_unused]] int workers) const
{
  struct Patch { ObservationMoments moments; ObservationMoments::GroundNormal normal;
    bool computed{false}; };
  std::vector<Patch> patches(cells_.size());
  std::vector<int> centers;
  for (const auto & row : snapshot.rows) {
    const int center = index(row.moments.mean.x(), row.moments.mean.y());
    if (center >= 0 && !patches[center].computed) {
      patches[center].computed = true; centers.push_back(center);
    }
  }
#ifdef _OPENMP
  const bool parallel_centers = workers > 1 && centers.size() >= 1024;
#pragma omp parallel for schedule(static) num_threads(workers) if(parallel_centers)
#endif
  for (size_t i = 0; i < centers.size(); ++i) {
    const int center = centers[i], cx = center % cfg_.width, cy = center / cfg_.width;
    auto & patch = patches[center];
    const int left = std::max(0, cx - 4), right = std::min(cfg_.width - 1, cx + 4);
    for (int y = std::max(0, cy - 4); y <= std::min(cfg_.height - 1, cy + 4); ++y) {
      const size_t first = 4 * (y * cfg_.width + left), end = 4 * (y * cfg_.width + right + 1);
      auto at = std::lower_bound(snapshot.rows.begin(), snapshot.rows.end(), first,
          [](const auto & row, size_t key){return row.index < key;});
      for (; at != snapshot.rows.end() && at->index < end; ++at) {
        patch.moments.merge(at->moments);
      }
    }
    patch.normal = patch.moments.ground_normal(cfg_.max_slope);
  }
#ifdef _OPENMP
  const bool parallel_rows = workers > 1 && snapshot.rows.size() >= 1024;
#pragma omp parallel for schedule(static) num_threads(workers) if(parallel_rows)
#endif
  for (size_t i = 0; i < snapshot.rows.size(); ++i) {
    auto & row = snapshot.rows[i];
    const auto & observation = row.moments;
    row.kind = observation.normal_class(cfg_.max_slope);
    const int center = index(observation.mean.x(), observation.mean.y());
    const bool supported = center >= 0 && observation.supported_by_cached(patches[center].moments,
        patches[center].normal, .005);
    if (row.kind == 0 || row.kind == 2) {row.kind = supported ? 2 : 0;}
  }
}
void LayeredTerrain::flush_native()
{
  if (pending_native_.empty()) {return;}
  int workers = 1;
#ifdef _OPENMP
  workers = std::min(4, omp_get_max_threads());
#endif
  const bool across_inputs = pending_native_.size() >= 4;
  std::vector<std::exception_ptr> failures(pending_native_.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers) if(across_inputs && workers > 1)
#endif
  for (size_t i = 0; i < pending_native_.size(); ++i) {
    try {
      classify_native(pending_native_[i], across_inputs ? 1 : workers);
    } catch (...) {
      failures[i] = std::current_exception();
    }
  }
  for (const auto & failure : failures) {
    if (failure) {
      std::rethrow_exception(failure);
    }
  }
  // Only native byte-sized votes are deferred: addition saturates at two and
  // flags OR together, so row ordering has no floating-point side effects.
  // Commit to ORIGINAL immutable buckets, not the last/newest bucket.
  for (const auto & snapshot : pending_native_) {
    for (const auto & row : snapshot.rows) {
      auto & layer = cells_[row.index / 4].layers[row.index % 4];
      auto bucket = std::find_if(layer.buckets.begin(), layer.buckets.end(), [&](const auto & b){
            return std::floor(b.first_stamp) == std::floor(snapshot.stamp);
    });
      if (bucket == layer.buckets.end()) {
        throw std::logic_error("deferred native bucket lost before vote commit");
      }
      layer.native.add(row.kind, row.boundary);
      bucket->native.add(row.kind, row.boundary);
    }
  }
  pending_native_.clear(); pending_native_bytes_ = 0;
}
LayeredTerrain::Plane LayeredTerrain::fit(
  int x, int y, const std::vector<int> & lowest,
  [[maybe_unused]] const std::vector<Eigen::Vector3d> & lowest_points) const
{
  struct Sample { Eigen::Vector3d row; double z; };
  // The neighborhood is fixed at 9x9. Avoid two heap allocations per fitted
  // cell per projection; retain the same ordering, fit and inlier decisions.
  std::array<Sample, 81> samples;
  size_t sample_count = 0;
  const double cx = cfg_.origin_x + (x + .5) * cfg_.resolution,
    cy = cfg_.origin_y + (y + .5) * cfg_.resolution;
  for (int dy = -4; dy <= 4; ++dy) {
    for (int dx = -4; dx <= 4; ++dx) {
      int col = x + dx, row = y + dy;
      if (col < 0 || row < 0 || col >= cfg_.width || row >= cfg_.height) {continue;}
      int id = row * cfg_.width + col;
      if (lowest[id] < 0) {continue;}
      const auto & p = lowest_points[id];
      samples[sample_count++] = {{p.x() - cx, p.y() - cy, 1}, p.z()};
    }
  }
  if (sample_count < 6) {return Plane{};}
  std::sort(samples.begin(), samples.begin() + sample_count,
    [](const auto & a, const auto & b){
      return a.z < b.z;
    });
  std::array<bool, 81> use{};
  for (size_t i = 0; i < std::max(size_t(6), sample_count / 3); ++i) {
    use[i] = true;
  }
  Eigen::Vector3d abc = Eigen::Vector3d::Zero();
  double sigma = .005;
  for (int iteration = 0; iteration < 4; ++iteration) {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero(); Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    int n = 0;
    for (size_t i = 0; i < sample_count; ++i) {
      if (use[i]) {
        normal += samples[i].row * samples[i].row.transpose(); rhs += samples[i].row * samples[i].z;
        ++n;
      }
    }
    if (n < 6 || normal.determinant() < 1e-7) {return Plane{};}
    abc = normal.ldlt().solve(rhs);
    if (!abc.allFinite() || abc.head<2>().norm() > cfg_.max_slope) {return Plane{};}
    double squared = 0; int inliers = 0; bool unchanged = true;
    for (size_t i = 0; i < sample_count; ++i) {
      const double residual = samples[i].z - samples[i].row.dot(abc);
      const bool selected = std::abs(residual) <= cfg_.ground_band;
      unchanged &= use[i] == selected;
      use[i] = selected;
      if (use[i]) {squared += residual * residual; ++inliers;}
    }
    if (inliers < 6) {return Plane{};}
    sigma = std::max(.005, std::sqrt(squared / inliers));
    // The next iteration would accumulate exactly the same samples in exactly
    // the same order. Its fit, residuals and sigma would be identical. This is
    // exact convergence, not a looser residual/iteration tolerance.
    if (unchanged) {break;}
  }
  if (sigma > cfg_.max_uncertainty) {return Plane{};}
  // Convert the centered plane to world XYZ; residuals use actual return XY.
  abc.z() -= abc.x() * cx + abc.y() * cy;
  return {abc, sigma, true};
}
TerrainOutput LayeredTerrain::project(double now)
{
  if (!std::isfinite(now) || now < 0 || now < evidence_time_) {
    throw std::invalid_argument("invalid terrain projection time");
  }
  flush_native();
  expire(now);
  evidence_time_ = now;
  const size_t size = cells_.size(); const float nan = std::numeric_limits<float>::quiet_NaN();
  TerrainOutput out;
  out.cost.assign(size, -1); out.reason.assign(size, NO_RETURNS);
  out.ground.assign(size, nan); out.uncertainty.assign(size, nan);
  out.obstacle_height.assign(size, nan); out.age.assign(size, nan);
  std::vector<int> lowest(size, -1); std::vector<Plane> planes(size);
  // Each fit reads up to 81 neighbors. Keep an exact compact snapshot instead
  // of repeatedly touching the large retained cell/layer/bucket representation.
  // Only the opt-in candidate uses this cache; selection and merge order do not
  // change. Unobserved entries are never read (lowest remains the validity mask).
  std::vector<Eigen::Vector3d> lowest_points;
  lowest_points.resize(size);
  std::vector<uint8_t> negative_edges(size, 0);
  std::vector<uint8_t> native_edges(size, 0);
  std::vector<uint8_t> ground_patches(size, 0);
  for (size_t i = 0; i < size; ++i) {
    for (const auto & layer : cells_[i].layers) {
      if (layer.count && layer.native.boundary()) {native_edges[i] = 1;}}
    if (cells_[i].overflow_until >= now) {out.reason[i] = LAYER_CAPACITY;}
    for (int j = 0; j < 3; ++j) {
      const auto & l = cells_[i].layers[j];
      if (l.count && now - l.first_stamp <= cfg_.lifetime &&
        (lowest[i] < 0 || l.mean.z() < cells_[i].layers[lowest[i]].mean.z())) {lowest[i] = j;}
    }
    if (lowest[i] >= 0) {lowest_points[i] = cells_[i].layers[lowest[i]].mean;}
  }
#ifdef _OPENMP
  const int workers = std::min(4, omp_get_max_threads());
#pragma omp parallel for schedule(static) num_threads(workers) if(size >= 4096)
#endif
  for (int y = 0; y < cfg_.height; ++y) {
    for (int x = 0; x < cfg_.width; ++x) {
      const int id = y * cfg_.width + x;
      if (lowest[id] < 0) {continue;}
      planes[id] = fit(x, y, lowest, lowest_points);
      const auto & plane = planes[id];
      if (!plane.valid) {out.reason[id] = NO_GROUND_MODEL; continue;}
      const double cx = cfg_.origin_x + (x + .5) * cfg_.resolution,
        cy = cfg_.origin_y + (y + .5) * cfg_.resolution;
      out.ground[id] = plane.abc.dot(Eigen::Vector3d(cx, cy, 1)); out.uncertainty[id] = plane.sigma;
      double maximum = -1e6, age = 1e6;
      bool supported = false, area_supported = false, raised_support = false;
      bool unresolved_raised = false;
      for (const auto & layer : cells_[id].layers) {
        if (!layer.count || now - layer.first_stamp > cfg_.lifetime) {continue;}
        const double h = layer.peak.z() - plane.abc.dot(Eigen::Vector3d(layer.peak.x(),
            layer.peak.y(), 1));
        const double low = layer.mean.z() - plane.abc.dot(Eigen::Vector3d(layer.mean.x(),
            layer.mean.y(), 1));
        supported |= std::abs(low) <= cfg_.ground_band;
        if (std::abs(low) <= cfg_.ground_band && layer.count >= 3) {
          const Eigen::Matrix3d covariance = layer.scatter.matrix() /
            static_cast<double>(layer.count);
          const double a = covariance(0, 0), b = covariance(0, 1), d = covariance(1, 1);
        // A line of rim hits (or repeated identical returns) does not measure
        // a ground patch. Such cells may only use the bounded inference path.
          area_supported |= .5 * (a + d - std::hypot(a - d, 2 * b)) >= 1e-6;
        }
        maximum = std::max(maximum, h); age = std::min(age, now - layer.first_stamp);
        raised_support |= h + 2 * plane.sigma >= cfg_.obstacle_height &&
          (layer.support_count >= 3 || layer.views >= 2);
      // The highest return on a partially observed face is a LOWER bound on
      // obstacle height, not a measured top. Ground in the same XY cell must
      // not turn a 6cm slice of an 8cm wall into drivable roughness. A separate
      // sub-threshold raised layer needs a resolved, traversable surface normal;
      // otherwise retain unknown without lowering the 8cm obstacle threshold.
        if (!raised_support && low > cfg_.ground_band) {
          bool resolved_top = false;
          if (layer.count >= 3) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(layer.scatter.matrix() /
              static_cast<double>(layer.count));
            resolved_top = eig.info() == Eigen::Success && eig.eigenvalues()[1] > 1e-6 &&
              eig.eigenvalues()[0] <= cfg_.max_uncertainty * cfg_.max_uncertainty &&
              std::abs(eig.eigenvectors()(2,
                0)) >= 1. / std::sqrt(1. + cfg_.max_slope * cfg_.max_slope);
          }
          unresolved_raised |= !resolved_top;
        }
      // Downward wall returns are not support merely because they share an XY
      // cell with the rim. Keep nearby cells unknown: a finite cell can straddle
      // the discontinuity. This is geometric evidence, not a GT hole mask.
        const double bottom = layer.trough.z() - plane.abc.dot(Eigen::Vector3d(layer.trough.x(),
            layer.trough.y(), 1));
        bool downward_wall = false;
        if (std::abs(low) <= cfg_.ground_band && layer.support_count >= 3) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(layer.scatter.matrix() /
            static_cast<double>(layer.count));
        // At least two resolved spatial dimensions are required. A single
        // straight scan-ring segment cannot establish a surface normal.
          downward_wall = eig.info() == Eigen::Success && eig.eigenvalues()[1] > 1e-6 &&
            std::abs(eig.eigenvectors()(2,
              0)) < 1. / std::sqrt(1. + cfg_.max_slope * cfg_.max_slope);
        if (!layer.native.boundary() && layer.native.only_ground()) {
              downward_wall = false;
}
        }
        if ((bottom < -cfg_.ground_band && (layer.support_count >= 3 || layer.views >= 2)) ||
          downward_wall)
        {
          negative_edges[id] = 1;
        }
      }
      if (!raised_support && maximum > cfg_.ground_band) {
      // Two scan rings on a box corner can each look like a sloping horizontal
      // patch after height clustering. Test their joint 3D geometry as well:
      // splitting rings into layers must not erase the vertical wall normal.
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        double count = 0.;
        for (const auto & layer : cells_[id].layers) {
          if (!layer.count) {continue;}
          const Eigen::Vector3d delta = layer.mean - mean;
          scatter += layer.scatter.matrix() + delta * delta.transpose() *
            (count * layer.count / (count + layer.count));
          mean += delta * (layer.count / (count + layer.count)); count += layer.count;
        }
        if (count >= 6) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter / count);
          unresolved_raised |= eig.info() == Eigen::Success && eig.eigenvalues()[1] > 1e-6 &&
            std::abs(eig.eigenvectors()(2,
              0)) < 1. / std::sqrt(1. + cfg_.max_slope * cfg_.max_slope);
        }
      }
    // Point support may be used around a neighboring corner even when these
    // points alone have insufficient area to classify their own whole cell.
      ground_patches[id] = supported && !unresolved_raised && !negative_edges[id];
      ground_patches[id] &= !native_edges[id];
      out.obstacle_height[id] = maximum; out.age[id] = age;
      if (raised_support) {
        out.cost[id] = 100; out.reason[id] = OBSTACLE;
      } else if (cells_[id].overflow_until >= now) {
        out.reason[id] = LAYER_CAPACITY;
      } else if (maximum + 2 * plane.sigma >= cfg_.obstacle_height || !supported ||
        unresolved_raised) {out.reason[id] = UNCERTAIN_RAISED;} else if (!area_supported) {
        out.reason[id] = INSUFFICIENT_AREA;
      } else {out.cost[id] = maximum > .04 ? 20 : 0; out.reason[id] = MEASURED_GROUND;}
    }
  }
  for (int edge = 0; edge < static_cast<int>(size); ++edge) {
    if (negative_edges[edge]) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int x = edge % cfg_.width + dx, y = edge / cfg_.width + dy;
          if (x < 0 || y < 0 || x >= cfg_.width || y >= cfg_.height) {continue;}
          const int id = y * cfg_.width + x;
          if (out.reason[id] != OBSTACLE) {out.cost[id] = -1; out.reason[id] = NEGATIVE_EDGE;}
        }
      }
    }
  }
  // An unresolved raised face has unresolved lateral extent too. Ground hits
  // in the adjacent part of a coarse cell cannot certify that the whole cell
  // is clear of that face. Withhold one-cell boundary support, never mark flat
  // space occupied and never propagate the guard recursively. Recompute from
  // live evidence so a resolved low top or expired observation clears it.
  std::vector<int> positive_edges;
  for (int id = 0; id < static_cast<int>(size); ++id) {
    if (out.reason[id] == UNCERTAIN_RAISED) {positive_edges.push_back(id);}}
  for (int edge : positive_edges) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int x = edge % cfg_.width + dx, y = edge / cfg_.width + dy;
        if (x < 0 || y < 0 || x >= cfg_.width || y >= cfg_.height) {continue;}
        const int id = y * cfg_.width + x;
        if (out.reason[id] == MEASURED_GROUND || out.reason[id] == NO_RETURNS ||
          out.reason[id] == INSUFFICIENT_AREA)
        {
          out.cost[id] = -1; out.reason[id] = POSITIVE_EDGE;
        }
      }
    }
  }
  // Native ambiguity supplements, never erases, the existing obstacle/partial-
  // face reason. Apply once before connectivity/interpolation; never recursively.
  for (int edge = 0; edge < static_cast<int>(size); ++edge) {
    if (native_edges[edge] &&
      out.reason[edge] != OBSTACLE)
    {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int x = edge % cfg_.width + dx, y = edge / cfg_.width + dy;
          if (x < 0 || y < 0 || x >= cfg_.width || y >= cfg_.height) {continue;}
          const int id = y * cfg_.width + x;
          if ((out.cost[id] >= 0 && out.cost[id] < 100) || out.reason[id] == NO_RETURNS ||
            out.reason[id] == INSUFFICIENT_AREA)
          {
            out.cost[id] = -1; out.reason[id] = NEGATIVE_EDGE;
          }
        }
      }
    }
  }
  // Reject isolated plateau/floor components. The nearest observed local
  // surface seeds connectivity, not the lowest (which could be a pit bottom).
  // This remains an experimental anchor, not proof of wheel support.
  int seed = -1; double best = std::numeric_limits<double>::infinity();
  for (int y = 0; y < cfg_.height; ++y) {
    for (int x = 0; x < cfg_.width; ++x) {
      int id = y * cfg_.width + x;
      const double cx = cfg_.origin_x + (x + .5) * cfg_.resolution,
        cy = cfg_.origin_y + (y + .5) * cfg_.resolution;
      double distance = std::hypot(cx - sensor_.x(), cy - sensor_.y());
      if (out.reason[id] == MEASURED_GROUND && distance >= .5 && distance <= 2.5 &&
        distance < best)
      {
        best = distance; seed = id;
      }
    }
  }
  std::vector<bool> connected(size, false); std::queue<int> queue;
  if (seed >= 0) {connected[seed] = true; queue.push(seed);}
  const int reach = std::max(1,
      static_cast<int>(std::floor(cfg_.interpolation_gap / cfg_.resolution + 1e-6)));
  // The stencil is fixed for the whole projection. Preserve the original
  // dy/dx order and exact distance/rounding rules, but do not recompute them
  // for every visited ground cell.
  struct Link { int dx, dy; std::vector<int> intermediate; };
  std::vector<Link> links;
  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const double distance = std::hypot(dx, dy) * cfg_.resolution;
      if (!distance || distance > cfg_.interpolation_gap + 1e-6) {continue;}
      Link link{dx, dy, {}};
      const int steps = std::max(std::abs(dx), std::abs(dy));
      for (int k = 1; k < steps; ++k) {
        const int row = static_cast<int>(std::round(static_cast<double>(dy) * k / steps));
        const int col = static_cast<int>(std::round(static_cast<double>(dx) * k / steps));
        link.intermediate.push_back(row * cfg_.width + col);
      }
      links.push_back(std::move(link));
    }
  }
  while (!queue.empty()) {
    int current = queue.front(); queue.pop();
    int x = current % cfg_.width, y = current / cfg_.width;
    for (const auto & link : links) {
      const int dx = link.dx, dy = link.dy;
      const int xx = x + dx, yy = y + dy;
      if (xx < 0 || yy < 0 || xx >= cfg_.width || yy >= cfg_.height) {continue;}
      int next = yy * cfg_.width + xx;
      if (connected[next] || out.reason[next] != MEASURED_GROUND) {continue;}
      const auto & p = planes[current];
      double expected = p.abc.x() * dx * cfg_.resolution + p.abc.y() * dy * cfg_.resolution;
      if (std::abs(out.ground[next] - out.ground[current] - expected) > .04) {continue;}
      bool blocked = false;
      for (int offset : link.intermediate) {
        const int along = current + offset;
        blocked |= out.reason[along] != NO_RETURNS && out.reason[along] != INSUFFICIENT_AREA &&
          out.reason[along] != MEASURED_GROUND;
      }
      if (!blocked) {connected[next] = true; queue.push(next);}
    }
  }
  for (size_t i = 0; i < size; ++i) {
    if (out.reason[i] == MEASURED_GROUND && !connected[i]) {
      out.cost[i] = -1; out.reason[i] = DISCONNECTED_SURFACE;
    }
  }
  if (cfg_.interpolate) {
    for (int y = 0; y < cfg_.height; ++y) {
      for (int x = 0; x < cfg_.width; ++x) {
        const int id = y * cfg_.width + x;
        if (out.reason[id] != NO_RETURNS && out.reason[id] != INSUFFICIENT_AREA) {continue;}
        int brackets = 0;
        double inferred = 0., uncertainty = 0., age = 0.;
        bool conflict = false;
        for (auto direction : std::array<std::array<int, 2>,
          4>{{{{1, 0}}, {{0, 1}}, {{1, 1}}, {{1, -1}}}})
        {
          int left = -1, right = -1, ld = 0, rd = 0;
          for (int sign : {-1, 1}) {
            for (int step = 1; step <= reach; ++step) {
              int xx = x + sign * step * direction[0], yy = y + sign * step * direction[1];
              if (xx < 0 || yy < 0 || xx >= cfg_.width || yy >= cfg_.height) {break;}
              int near = yy * cfg_.width + xx;
              if (out.reason[near] == MEASURED_GROUND) {
                if (sign < 0) {left = near; ld = step;} else {right = near; rd = step;} break;
              }
              if (out.reason[near] != NO_RETURNS && out.reason[near] != INSUFFICIENT_AREA &&
                out.reason[near] != INFERRED_GROUND) {break;}
            }
          }
          if (left < 0 || right < 0 || (ld + rd) * cfg_.resolution * std::hypot(direction[0],
              direction[1]) > cfg_.interpolation_gap + 1e-6) {continue;}
          const Eigen::Vector3d at(cfg_.origin_x + (x + .5) * cfg_.resolution,
            cfg_.origin_y + (y + .5) * cfg_.resolution, 1);
          double a = planes[left].abc.dot(at), b = planes[right].abc.dot(at);
          if (std::abs(a - b) > .02) {continue;}
          const double value = .5 * (a + b);
          if (brackets && std::abs(value - inferred / brackets) > .02) {conflict = true;}
          ++brackets; inferred += value;
          uncertainty = std::max(uncertainty,
              static_cast<double>(std::max(out.uncertainty[left], out.uncertainty[right])));
          age = std::max(age, static_cast<double>(std::max(out.age[left], out.age[right])));
        }
    // One chord can cut across a hole corner. Require independent bracketing
    // in two nonparallel directions, without using inferred endpoints.
        if (brackets >= 2 && !conflict) {
          out.cost[id] = 20; out.reason[id] = INFERRED_GROUND; out.ground[id] = inferred / brackets;
          out.uncertainty[id] = uncertainty; out.age[id] = age;
        }
      }
    }
  }
  // A small floor patch somewhere in a cell does not establish support over
  // that cell's extent. This matters even without interpolation: a cell can
  // straddle a hole rim. Independently certify each output corner using actual
  // retained ground returns in all eight 45-degree XY sectors, whose pairwise spans
  // stay within the existing interpolation diameter. No inferred endpoint, Gaussian tail, invented
  // viewpoint, GT boundary or recursively eroded support is used here.
  if (std::any_of(out.cost.begin(), out.cost.end(), [](int8_t cost){
      return cost >= 0 && cost < 100;
      }))
  {
    const int vertex_width = cfg_.width + 1;
    std::vector<int> corner_anchor(size_t(vertex_width) * (cfg_.height + 1), -1);
    std::vector<uint8_t> needed_corner(corner_anchor.size(), 0);
    for (int y = 0; y < cfg_.height; ++y) {
      for (int x = 0; x < cfg_.width; ++x) {
        const int id = y * cfg_.width + x;
        if (out.cost[id] < 0 || out.cost[id] == 100) {continue;}
        for (int dy = 0; dy <= 1; ++dy) {
          for (int dx = 0; dx <= 1; ++dx) {
            needed_corner[(y + dy) * vertex_width + x + dx] = 1;
          }
        }
      }
    }
    std::vector<uint8_t> cached_corners(size, 0);
    std::vector<double> cached_residual(size, 0.);
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers) if(size >= 4096)
#endif
    for (int id = 0; id < static_cast<int>(size); ++id) {
      if (!ground_patches[id]) {continue;}
      for (const auto & layer : cells_[id].layers) {
        for (const auto & bucket : layer.buckets) {
          if (!(bucket.corner_support & ~cached_corners[id])) {continue;}
          const auto residual = [&](const Eigen::Vector3d & p) {
              return std::abs(p.z() - planes[id].abc.dot(Eigen::Vector3d(p.x(), p.y(), 1)));
            };
          const double maximum = std::max({residual(bucket.mean), residual(bucket.peak),
                residual(bucket.trough)});
          if (maximum <= .02) {
            cached_corners[id] |= bucket.corner_support;
            cached_residual[id] = std::max(cached_residual[id], maximum);
          }
        // The cache needs witnesses for eight bits, not every duplicate
        // observation. Its residual bound covers the retained witnesses; the
        // original per-bucket test still handles any failed anchor bound.
          if (cached_corners[id] == 255) {break;}
        }
        if (cached_corners[id] == 255) {break;}
      }
    }
    const double support_radius = cfg_.interpolation_gap;
    const int support_reach = static_cast<int>(std::ceil(support_radius / cfg_.resolution));
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers) if(size >= 4096)
#endif
    for (int vy = 0; vy <= cfg_.height; ++vy) {
      for (int vx = 0; vx <= cfg_.width; ++vx) {
        if (!needed_corner[vy * vertex_width + vx]) {continue;}
        int anchor = -1;
        for (int dy = -1; dy <= 0 && anchor < 0; ++dy) {
          for (int dx = -1; dx <= 0; ++dx) {
            const int x = vx + dx, y = vy + dy;
            if (x >= 0 && y >= 0 && x < cfg_.width && y < cfg_.height &&
              ground_patches[y * cfg_.width + x])
            {
              anchor = y * cfg_.width + x; break;
            }
          }
        }
        if (anchor < 0) {
          continue;
        }
        const double px = cfg_.origin_x + vx * cfg_.resolution,
          py = cfg_.origin_y + vy * cfg_.resolution;
        const auto ground_bucket = [&](const Slice & bucket) {
            const auto residual = [&](const Eigen::Vector3d & p) {
                return std::abs(p.z() - planes[anchor].abc.dot(Eigen::Vector3d(p.x(), p.y(), 1)));
              };
            return residual(bucket.mean) <= .02 && residual(bucket.peak) <= .02 &&
                   residual(bucket.trough) <= .02;
          };
    // Exact angular certificates from the four adjoining cells are enough on
    // dense ground. Every possible pair is within a 2-cell square's diagonal;
    // no representative-point sampling or quantized position is needed.
        unsigned adjacent = 0;
        if (2 * std::sqrt(2.) * cfg_.resolution <= cfg_.interpolation_gap + 1e-9) {
          const int sector_pairs[4][2] = {{0, 1}, {3, 2}, {7, 6}, {4, 5}};
          for (int cy = 0; cy <= 1; ++cy) {
            for (int cx = 0; cx <= 1; ++cx) {
              const int x = vx - cx, y = vy - cy;
              if (x < 0 || y < 0 || x >= cfg_.width || y >= cfg_.height ||
                !ground_patches[y * cfg_.width + x]) {continue;}
              unsigned local = 0;
              const int at = y * cfg_.width + x;
              const Eigen::Vector3d delta = planes[at].abc - planes[anchor].abc;
              const Eigen::Vector3d center(cfg_.origin_x + (x + .5) * cfg_.resolution,
                cfg_.origin_y + (y + .5) * cfg_.resolution, 1.);
              const double bound = std::abs(delta.dot(center)) + .5 * cfg_.resolution *
                (std::abs(delta.x()) + std::abs(delta.y()));
        // This bound covers every position in the candidate cell. Reusing a
        // local certificate is therefore sufficient for the original per-
        // bucket anchor test; otherwise take that unchanged exact slow path.
              if (cached_residual[at] + bound <= .02) {
                local = (cached_corners[at] >> (2 * (2 * cy + cx))) & 3u;
              }
              if (local != 3) {
                for (const auto & layer : cells_[at].layers) {
                  for (const auto & bucket : layer.buckets) {
                    if (ground_bucket(bucket)) {
                      local |= (bucket.corner_support >> (2 * (2 * cy + cx))) & 3u;
                    }
                    if (local == 3) {break;}
                  }
                  if (local == 3) {break;}
                }
              }
              for (int bit = 0; bit < 2; ++bit) {
                if (local & (1u << bit)) {
                  adjacent |= 1u << sector_pairs[2 * cy + cx][bit];
                }
              }
            }
          }
        }
        if (adjacent == 255) {
          corner_anchor[vy * vertex_width + vx] = anchor; continue;
        }
        unsigned sectors = 0; bool certified = false;
        std::array<Eigen::Vector2d, 8> supports;
        std::array<double, 8> distances;
        std::array<double, 8> radii{};
        distances.fill(std::numeric_limits<double>::infinity());
        const auto sector_of = [](double dx, double dy) {
            return dy > 0 ? (dx > 0 ? (dy < dx ? 0 : 1) : (dy > -dx ? 2 : 3)) :
                   (dx < 0 ? (-dy < -dx ? 4 : 5) : (-dy > dx ? 6 : 7));
          };
        const auto consider = [&](double dx, double dy, double radius) {
      // A subregion must lie wholly inside one sector. Its center alone is
      // never treated as a measured endpoint. Inflate pairwise spans by the
      // region's circumradius to bound every possible actual return position.
            const double half = radius / std::sqrt(2.);
            if (std::abs(dx) <= half + 1e-6 || std::abs(dy) <= half + 1e-6 ||
              std::abs(std::abs(dx) - std::abs(dy)) <= 2 * half + 1e-6 || std::hypot(dx,
              dy) + radius > support_radius + 1e-9) {return;}
            const unsigned sector = sector_of(dx, dy);
            const double distance = std::hypot(dx, dy) + radius;
            if (distance >= distances[sector]) {return;}
            supports[sector] = {dx, dy}; distances[sector] = distance; radii[sector] = radius;
            sectors |= 1u << sector;
            if (sectors == 255) {
              certified = true;
              for (int a = 0; a < 8; ++a) {
                for (int b = a + 1; b < 8; ++b) {
                  certified &= (supports[a] - supports[b]).norm() + radii[a] + radii[b] <=
                    cfg_.interpolation_gap + 1e-9;
                }
              }
            }
      // An empty fixed sector is not necessarily an unsupported angular gap.
      // For example, six evenly spaced actual returns have 60° gaps but can
      // occupy only six of eight sectors. Test actual bearings conservatively.
      // Keep the original all-sector fast certificate and its region bounds.
      // Adjacent missing 45-degree sectors already prove a >=90-degree gap.
      // With all eight sectors, a failed original pairwise-span certificate
      // also cannot pass the identical span check inside angular_support.
      // Skip only proven failures; keep witness ordering and all thresholds.
            if (!certified && sectors != 255 && sector_gaps_can_support(sectors)) {
              certified = angular_support(supports, radii, sectors, cfg_.interpolation_gap);
            }
          };
    // Search nearby cells first, then farther cells only if needed. Point
    // positions, not candidate-cell labels/centers, certify the actual extent.
        for (int iy = 0; iy < 2 * support_reach && !certified; ++iy) {
          for (int ix = 0; ix < 2 * support_reach && !certified; ++ix) {
            const int y = vy + (iy % 2 ? -(iy + 1) / 2 : iy / 2),
              x = vx + (ix % 2 ? -(ix + 1) / 2 : ix / 2);
            if (x < 0 || y < 0 || x >= cfg_.width || y >= cfg_.height) {continue;}
            const int at = y * cfg_.width + x;
            if (!ground_patches[at]) {continue;}
            uint32_t visited_xy = 0;
            for (const auto & layer : cells_[at].layers) {
              for (const auto & bucket : layer.buckets) {
                for (int k = 0; k < std::min(2, static_cast < int > (bucket.support_count)); ++k) {
                  const auto & point = bucket.support[k];
                  const double dx = point.x() - px, dy = point.y() - py;
                  if (std::abs(point.z() - planes[anchor].abc.dot(Eigen::Vector3d(point.x(),
                      point.y(), 1))) > .02)
                  {
                    continue;
                  }
                  consider(dx, dy, 0.);
                  if (certified) {break;}
                }
                const uint32_t fresh_xy = bucket.xy_support & ~visited_xy;
                if (!certified && fresh_xy) {
                  if (ground_bucket(bucket)) {
                // Repeated buckets can only repeat the same bounded XY
                // region. Its representative distance can never improve after
                // the first visit, because selected distances only decrease.
                // Preserve first-visit ordering while avoiding redundant work.
                    visited_xy |= fresh_xy;
                    const double sub = cfg_.resolution / 5.;
                    for (int bit = 0; bit < 25 && !certified; ++bit) {
                      if (fresh_xy & (1u << bit)) {
                        const double dx = cfg_.origin_x + x * cfg_.resolution + (bit % 5 + .5) *
                          sub - px;
                        const double dy = cfg_.origin_y + y * cfg_.resolution + (bit / 5 + .5) *
                          sub - py;
                        consider(dx, dy, sub / std::sqrt(2.));
                      }
                    }
                  }
                }
                if (certified) {break;}
              }
              if (certified) {break;}
            }
          }
        }
        if (certified) {corner_anchor[vy * vertex_width + vx] = anchor;}
      }
    }
    for (int y = 0; y < cfg_.height; ++y) {
      for (int x = 0; x < cfg_.width; ++x) {
        const int id = y * cfg_.width + x;
        if (out.cost[id] < 0 || out.cost[id] == 100) {continue;}
        bool supported = true;
        for (int dy = 0; dy <= 1; ++dy) {
          for (int dx = 0; dx <= 1; ++dx) {
            const int anchor = corner_anchor[(y + dy) * vertex_width + x + dx];
            if (anchor < 0) {
              supported = false; continue;
            }
            const Eigen::Vector3d at(cfg_.origin_x + (x + dx) * cfg_.resolution,
              cfg_.origin_y + (y + dy) * cfg_.resolution, 1.);
      // Inferred cells have an inferred height rather than their own valid fit.
            if (planes[id].valid) {
              supported &= std::abs((planes[id].abc - planes[anchor].abc).dot(at)) <= .02;
            } else {
              const Eigen::Vector3d center(cfg_.origin_x + (x + .5) * cfg_.resolution,
                cfg_.origin_y + (y + .5) * cfg_.resolution, 1.);
              supported &= std::abs(out.ground[id] - planes[anchor].abc.dot(center)) <= .02;
            }
          }
        }
        if (!supported) {out.cost[id] = -1; out.reason[id] = UNSUPPORTED_EXTENT;}
      }
    }
  }
  // A measured floor is not proof of body clearance. In particular a ray can
  // pass underneath a suspended obstacle whose front edge is already known.
  // Conservatively withhold drivable status behind an occupied column whenever
  // the ground sightline does not pass above its observed top. This is an
  // unknown-space guard, not occupied inflation, ray-derived ground, or proof
  // of clearance where no obstacle was observed. Low-column shadows may use
  // an actual retained ground observation from another viewpoint. Geometry at
  // or above the sensor remains conservative: a floor ray can pass underneath
  // it, so historical floor visibility cannot establish overhead clearance.
  // Origins expire with their immutable evidence buckets, never independently.
  const double infinity = std::numeric_limits<double>::infinity();
  // Workers only read this immutable occupancy snapshot. Reading out.cost while
  // other workers change free to unknown would still be a C++ data race even
  // though that change cannot create/remove an occupied cell.
  std::vector<uint8_t> occupied(size);
  for (size_t id = 0; id < size; ++id) {
    occupied[id] = out.cost[id] == 100;
  }
  // With no occupied columns and the sensor inside this convex rectangular
  // grid, every sensor-to-cell-center segment stays inside it and has no
  // blocker. This is exactly the DDA result, not ray-derived ground support.
  // An outside sensor must retain the original fail-closed clearance behavior.
  if (index(sensor_.x(), sensor_.y()) >= 0 &&
    std::none_of(occupied.begin(), occupied.end(), [](uint8_t value){return value != 0;}))
  {
    return out;
  }
  const auto & origins = origins_;
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers) if(size >= 4096)
#endif
  for (int id = 0; id < static_cast<int>(size); ++id) {
    if (out.cost[id] < 0 || out.cost[id] == 100) {continue;}
    const auto shadow_top = [&](const Eigen::Vector3d & origin, const Eigen::Vector3d & target) {
        const int sensor_id = index(origin.x(), origin.y());
        if (sensor_id < 0) {return infinity;}
        int x = sensor_id % cfg_.width, y = sensor_id / cfg_.width;
        const double dx = target.x() - origin.x(), dy = target.y() - origin.y();
        const int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
        const double step_x = dx == 0 ? infinity : cfg_.resolution / std::abs(dx);
        const double step_y = dy == 0 ? infinity : cfg_.resolution / std::abs(dy);
        double next_x = dx ==
          0 ? infinity : (cfg_.origin_x + (x + (sx > 0)) * cfg_.resolution - origin.x()) / dx;
        double next_y = dy ==
          0 ? infinity : (cfg_.origin_y + (y + (sy > 0)) * cfg_.resolution - origin.y()) / dy;
        double entry = 0., highest = -infinity;
        const auto blocked = [&](int col, int row, double start, double end) {
            if (col < 0 || row < 0 || col >= cfg_.width || row >= cfg_.height) {return infinity;}
            const int at = row * cfg_.width + col;
            if (at == id || !occupied[at]) {return -infinity;}
            const double z0 = origin.z() + start * (target.z() - origin.z());
            const double z1 = origin.z() + end * (target.z() - origin.z());
            const double top = out.ground[at] + out.obstacle_height[at] + 2 * out.uncertainty[at] +
              planes[at].abc.head<2>().norm() * cfg_.resolution * std::sqrt(.5);
            return top >= std::min(z0, z1) ? top : -infinity;
          };
        for (int steps = 0; steps <= cfg_.width + cfg_.height; ++steps) {
          const double leave = std::min({next_x, next_y, 1.});
          highest = std::max(highest, blocked(x, y, entry, leave));
          if (leave >= 1.) {break;}
          if (std::abs(next_x - next_y) < 1e-12) {
          // Include both side cells at an exact grid-corner crossing.
            highest = std::max({highest, blocked(x + sx, y, leave, leave),
                  blocked(x, y + sy, leave, leave)});
            x += sx; y += sy; next_x += step_x; next_y += step_y;
          } else if (next_x < next_y) {x += sx; next_x += step_x;} else {y += sy; next_y += step_y;}
          entry = leave;
        }
        return highest;
      };
    const Eigen::Vector3d center(cfg_.origin_x + (id % cfg_.width + .5) * cfg_.resolution,
      cfg_.origin_y + (id / cfg_.width + .5) * cfg_.resolution, out.ground[id]);
    const double current_shadow = shadow_top(sensor_, center);
    bool occluded = current_shadow != -infinity;
    if (occluded && current_shadow < sensor_.z() && out.reason[id] == MEASURED_GROUND) {
      for (const auto & layer : cells_[id].layers) {
        for (const auto & bucket : layer.buckets) {
          const auto origin = origins.find(bucket.first_stamp);
          if (!bucket.support_count || origin == origins.end() ||
            origin->second.z() <= current_shadow) {continue;}
          const auto & point = bucket.support[0];
          const double residual = point.z() - planes[id].abc.dot(Eigen::Vector3d(point.x(),
              point.y(), 1));
          if (std::abs(residual) > cfg_.ground_band) {continue;}
          // Both the actual recorded endpoint and the output cell center must
          // avoid every currently known occupied column. No invented viewpoints,
          // inferred endpoints, independent lifetime, or stale cached verdicts.
          if (shadow_top(origin->second, point) == -infinity && shadow_top(origin->second,
              center) == -infinity)
          {
            occluded = false; break;
          }
        }
        if (!occluded) {break;}
      }
    }
    if (occluded) {out.cost[id] = -1; out.reason[id] = CLEARANCE_OCCLUDED;}
  }
  return out;
}
}  // namespace astrobot_terrain
