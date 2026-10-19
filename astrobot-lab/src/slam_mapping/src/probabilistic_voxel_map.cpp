#include "slam_mapping/probabilistic_voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace slam_mapping
{
namespace
{
double percentile(std::vector<float> values, const double q)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double clamped = std::clamp(q, 0.0, 1.0);
  const auto index = static_cast<std::size_t>(std::round(clamped * static_cast<double>(values.size() - 1)));
  return values[index];
}
}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const
{
  const auto x = static_cast<uint32_t>(key.x);
  const auto y = static_cast<uint32_t>(key.y);
  const auto z = static_cast<uint32_t>(key.z);
  std::size_t seed = 0;
  seed ^= static_cast<std::size_t>(x) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  seed ^= static_cast<std::size_t>(y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  seed ^= static_cast<std::size_t>(z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

ProbabilisticVoxelMap::ProbabilisticVoxelMap(OccupancyConfig config)
: config_(config)
{
  if (!std::isfinite(config_.voxel_size) || config_.voxel_size <= 0.0) {
    throw std::runtime_error("voxel_size must be positive");
  }
  if (config_.point_stride < 1) {
    config_.point_stride = 1;
  }
  if (config_.ray_stride < 1) {
    config_.ray_stride = 1;
  }
}

void ProbabilisticVoxelMap::clear()
{
  voxels_.clear();
  surfaces_.clear();
  frame_misses_.clear();
  scan_id_ = 0;
  stamp_ = -1;
}

VoxelKey ProbabilisticVoxelMap::worldToKey(const Point3 & point) const
{
  return VoxelKey{
    static_cast<int32_t>(std::floor(point.x / config_.voxel_size)),
    static_cast<int32_t>(std::floor(point.y / config_.voxel_size)),
    static_cast<int32_t>(std::floor(point.z / config_.voxel_size))};
}

Point3 ProbabilisticVoxelMap::keyCenter(const VoxelKey & key) const
{
  return Point3{
    (static_cast<double>(key.x) + 0.5) * config_.voxel_size,
    (static_cast<double>(key.y) + 0.5) * config_.voxel_size,
    (static_cast<double>(key.z) + 0.5) * config_.voxel_size};
}

float ProbabilisticVoxelMap::logOdds(const VoxelKey & key) const
{
  const auto iter = voxels_.find(key);
  return iter == voxels_.end() ? 0.0F : iter->second;
}

bool ProbabilisticVoxelMap::isOccupied(const VoxelKey & key) const
{
  return logOdds(key) >= config_.occupied_threshold;
}

bool ProbabilisticVoxelMap::isFree(const VoxelKey & key) const
{
  return logOdds(key) <= config_.free_threshold;
}

std::size_t ProbabilisticVoxelMap::voxelCount() const
{
  return voxels_.size();
}

const OccupancyConfig & ProbabilisticVoxelMap::config() const
{
  return config_;
}

bool ProbabilisticVoxelMap::withinBounds(const Point3 & point) const
{
  if (!config_.use_bounds) {
    return true;
  }
  return config_.min_bounds.x <= point.x && point.x <= config_.max_bounds.x &&
         config_.min_bounds.y <= point.y && point.y <= config_.max_bounds.y &&
         config_.min_bounds.z <= point.z && point.z <= config_.max_bounds.z;
}

void ProbabilisticVoxelMap::updateVoxel(const VoxelKey & key, const double delta)
{
  auto iter = voxels_.find(key);
  if (iter == voxels_.end()) {
    if (config_.max_voxels > 0 && voxels_.size() >= static_cast<std::size_t>(config_.max_voxels)) {
      return;
    }
    iter = voxels_.emplace(key, 0.0F).first;
  }
  iter->second = static_cast<float>(
    std::clamp(static_cast<double>(iter->second) + delta, config_.min_log_odds, config_.max_log_odds));
}

void ProbabilisticVoxelMap::integrateRay(const Point3 & origin, const Point3 & endpoint, const bool add_hit)
{
  const auto start = worldToKey(origin);
  const auto end = worldToKey(endpoint);
  const int dx = std::abs(end.x - start.x);
  const int dy = std::abs(end.y - start.y);
  const int dz = std::abs(end.z - start.z);
  const int steps = std::max({dx, dy, dz, 1});

  VoxelKey last_key{
    std::numeric_limits<int32_t>::min(),
    std::numeric_limits<int32_t>::min(),
    std::numeric_limits<int32_t>::min()};

  for (int i = 0; i < steps; ++i) {
    if (i % config_.ray_stride != 0) {
      continue;
    }
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Point3 point{
      origin.x + (endpoint.x - origin.x) * t,
      origin.y + (endpoint.y - origin.y) * t,
      origin.z + (endpoint.z - origin.z) * t};
    const auto key = worldToKey(point);
    if (key == last_key || key == end) {
      continue;
    }
    // Near-field obstacle persistence: do NOT miss-clear an already-occupied voxel within the blind-cone
    // radius of the sensor. Up close, beams pass OVER low rocks to the ground beyond, casting misses
    // through the rock voxel -> it clears -> the rover drives into the now-"free" rock and wedges.
    if (!config_.use_surface_evidence && config_.near_field_persist_radius > 0.0) {
      const double pdx = point.x - origin.x;
      const double pdy = point.y - origin.y;
      if (pdx * pdx + pdy * pdy < config_.near_field_persist_radius * config_.near_field_persist_radius &&
          logOdds(key) >= config_.occupied_threshold) {
        last_key = key;
        continue;
      }
    }
    frame_misses_.insert(key);
    last_key = key;
  }

  if (add_hit) {
    updateVoxel(end, config_.hit_log_odds);
  }
}

IntegrationStats ProbabilisticVoxelMap::integratePointCloud(const Point3 & sensor_origin, const std::vector<Point3> & points,
                                                         double stamp)
{
  IntegrationStats stats;
  stats.input_points = points.size();
  // Legacy callers advance a nominal 20 Hz clock. ROS/replay must supply measurement time.
  if (std::isnan(stamp)) stamp = stamp_ < 0 ? 0 : stamp_ + .05;
  if (!std::isfinite(stamp) || !std::isfinite(sensor_origin.x) || !std::isfinite(sensor_origin.y) ||
      !std::isfinite(sensor_origin.z) || (stamp_ >= 0 && stamp <= stamp_)) {
    stats.skipped_invalid = points.size();
    return stats;
  }
  const double dt = stamp_ < 0 ? 0 : stamp - stamp_;
  stamp_ = stamp;
  ++scan_id_;
  frame_misses_.clear();

  // Temporal forgetting: decay all voxels toward 0 once per integration cycle and
  // prune near-zero voxels. Applied before integrating this cloud so the current
  // observation keeps full weight. Re-observed voxels recover via hit_log_odds;
  // un-reobserved smear ghosts fade and are erased (no clearing ray required).
  if (config_.decay_factor < 1.0) {
    const double nf2 = config_.use_surface_evidence ? 0.0 :
      config_.near_field_persist_radius * config_.near_field_persist_radius;
    const double decay = std::pow(config_.decay_factor, dt * 20.0);
    for (auto it = voxels_.begin(); it != voxels_.end();) {
      // Near-field persistence (see integrateRay): don't decay an occupied voxel within the blind-cone
      // radius of the sensor -- the rover can't reliably re-observe low obstacles up close.
      if (nf2 > 0.0 && static_cast<double>(it->second) >= config_.occupied_threshold) {
        const Point3 c = keyCenter(it->first);
        const double wx = c.x - sensor_origin.x;
        const double wy = c.y - sensor_origin.y;
        if (wx * wx + wy * wy < nf2) {
          ++it;
          continue;
        }
      }
      const double decayed = static_cast<double>(it->second) * decay;
      if (std::abs(decayed) < config_.decay_min_abs_log_odds) {
        surfaces_.erase(it->first);
        it = voxels_.erase(it);
      } else {
        it->second = static_cast<float>(decayed);
        ++it;
      }
    }
  }

  if (!withinBounds(sensor_origin)) {
    stats.skipped_bounds = points.size();
    stats.voxel_count = voxels_.size();
    return stats;
  }

  // Per-frame hit cap: a voxel receives hit_log_odds AT MOST ONCE per integration cycle, regardless
  // of how many points land in it this frame. Without this, a single dense return (e.g. a grazing
  // LiDAR ring arc on the ground at the lowered mount) deposits many coincident points into one voxel
  // -> it crosses occupied_threshold in ONE frame -> instant false-obstacle "ridges following the
  // LiDAR lines" that appear for a single update. Capping makes log-odds reflect HOW MANY FRAMES a
  // voxel was seen (temporal persistence), so a one-frame artifact stays below occupied_threshold and
  // only genuinely re-observed surfaces (real rocks the rover approaches) accumulate to occupied.
  // Misses are still cast per-ray below (clearing benefits from more evidence; it is not the culprit).
  std::unordered_set<VoxelKey, VoxelKeyHash> frame_hits;
  for (std::size_t i = 0; i < points.size(); i += static_cast<std::size_t>(config_.point_stride)) {
    const auto & raw = points[i];
    if (!std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.z)) {
      ++stats.skipped_invalid;
      continue;
    }
    if (!withinBounds(raw)) {
      ++stats.skipped_bounds;
      continue;
    }

    const double dx = raw.x - sensor_origin.x;
    const double dy = raw.y - sensor_origin.y;
    const double dz = raw.z - sensor_origin.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range <= 1.0e-6) {
      ++stats.skipped_invalid;
      continue;
    }

    bool add_hit = true;
    Point3 endpoint = raw;
    if (config_.max_range > 0.0 && range > config_.max_range) {
      const double scale = config_.max_range / range;
      endpoint = Point3{sensor_origin.x + dx * scale, sensor_origin.y + dy * scale, sensor_origin.z + dz * scale};
      add_hit = false;
    }

    // Cast the ray for misses only; defer the endpoint hit so it is applied once per voxel below.
    integrateRay(sensor_origin, endpoint, false);
    ++stats.integrated_points;
    if (add_hit) {
      frame_hits.insert(worldToKey(endpoint));
      if (config_.use_surface_evidence) {
        const auto key = worldToKey(endpoint);
        if (surfaces_.count(key) || config_.max_voxels <= 0 || surfaces_.size() < static_cast<size_t>(config_.max_voxels)) {
          auto & surface = surfaces_[key];
          if (stamp_ - surface.last_hit > config_.surface_max_age_sec) surface = Surface{};
          const auto slot = scan_id_ % 3;
          surface.stamps[slot] = stamp_;
          if (surface.scans[slot] != scan_id_) {
            surface.scans[slot] = scan_id_;
            surface.low[slot] = surface.high[slot] = static_cast<float>(raw.z);
          } else {
            surface.low[slot] = std::min(surface.low[slot], static_cast<float>(raw.z));
            surface.high[slot] = std::max(surface.high[slot], static_cast<float>(raw.z));
          }
          int support = 0;
          for (auto seen : surface.scans) if (seen > 0 && scan_id_ - seen < 3) ++support;
          surface.confirmed = surface.confirmed || support >= 2;
          surface.last_hit = stamp_;
        }
      }
    }
  }
  // Apply each frame's hits once per voxel (the temporal-persistence cap).
  for (const auto & key : frame_hits) {
    updateVoxel(key, config_.hit_log_odds);
    if (voxels_.count(key)) ++stats.hit_updates;
    else { ++stats.skipped_capacity; surfaces_.erase(key); }
  }
  for (const auto & key : frame_misses_) {
    if (frame_hits.count(key)) continue;
    updateVoxel(key, config_.miss_log_odds);
    if (voxels_.count(key)) ++stats.miss_updates;
    else ++stats.skipped_capacity;
    if (logOdds(key) <= config_.free_threshold) surfaces_.erase(key);
  }

  stats.voxel_count = voxels_.size();
  return stats;
}

std::vector<VoxelSnapshot> ProbabilisticVoxelMap::occupiedVoxels() const
{
  std::vector<VoxelSnapshot> output;
  output.reserve(voxels_.size());
  for (const auto & item : voxels_) {
    if (item.second >= config_.occupied_threshold) {
      output.push_back(VoxelSnapshot{item.first, keyCenter(item.first), item.second});
    }
  }
  return output;
}

BevGrid ProbabilisticVoxelMap::projectToBev(const BevProjectionConfig & config) const
{
  if (!std::isfinite(config.resolution) || config.resolution <= 0 ||
      !std::isfinite(config.width_m) || !std::isfinite(config.height_m) ||
      config.width_m <= 0 || config.height_m <= 0 ||
      config.ground_window_radius < 0 || config.ground_window_radius > 100) {
    throw std::invalid_argument("invalid BEV dimensions or ground window");
  }
  BevGrid grid;
  const double width_cells=std::ceil(config.width_m/config.resolution);
  const double height_cells=std::ceil(config.height_m/config.resolution);
  if (!std::isfinite(config.origin_x) || !std::isfinite(config.origin_y) ||
      width_cells < 1 || height_cells < 1 ||
      !std::isfinite(width_cells*height_cells) || width_cells*height_cells > 4000000) {
    throw std::invalid_argument("BEV origin must be finite and grid must not exceed 4M cells");
  }
  grid.resolution = config.resolution;
  grid.origin_x = config.origin_x;
  grid.origin_y = config.origin_y;
  grid.width = static_cast<int>(width_cells);
  grid.height = static_cast<int>(height_cells);
  grid.data.assign(static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height), -1);
  grid.obstacle_observed_at.assign(grid.data.size(), std::numeric_limits<double>::quiet_NaN());

  const auto index = [&grid](const int x, const int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + static_cast<std::size_t>(x);
  };

  std::vector<std::vector<float>> occupied_z(grid.data.size());
  struct SurfaceObservation { size_t cell; double height; double stamp; };
  std::vector<SurfaceObservation> surface_observations;
  std::vector<uint8_t> observed(grid.data.size(), 0U);
  // Peak voxel log-odds per cell -- a confidence proxy for the obstacle-seed gate below.
  std::vector<double> cell_peak_lo(grid.data.size(), -std::numeric_limits<double>::infinity());
  for (const auto & item : voxels_) {
    const auto center = keyCenter(item.first);
    const int x = static_cast<int>(std::floor((center.x - config.origin_x) / config.resolution));
    const int y = static_cast<int>(std::floor((center.y - config.origin_y) / config.resolution));
    if (x < 0 || grid.width <= x || y < 0 || grid.height <= y) {
      continue;
    }
    const auto i = index(x, y);
    if (config_.use_surface_evidence) {
      const auto found = surfaces_.find(item.first);
      if (found == surfaces_.end()) continue;
      const auto & surface = found->second;
      if (!surface.confirmed || stamp_ - surface.last_hit > config_.surface_max_age_sec) continue;
      std::vector<float> lows, highs;
      for (size_t slot = 0; slot < 3; ++slot) {
        if (surface.scans[slot]) { lows.push_back(surface.low[slot]); highs.push_back(surface.high[slot]); }
      }
      occupied_z[i].push_back(static_cast<float>(percentile(lows, .5)));
      const double high = percentile(highs, .5);
      occupied_z[i].push_back(static_cast<float>(high));
      double observed_at = -1;
      for (size_t slot = 0; slot < 3; ++slot) {
        if (surface.scans[slot] && surface.high[slot] + 1e-6 >= high)
          observed_at = std::max(observed_at, surface.stamps[slot]);
      }
      surface_observations.push_back({i, high, observed_at});
      observed[i] = 1U;
      cell_peak_lo[i] = std::max(cell_peak_lo[i], config.seed_confidence_log_odds);
    } else if (item.second >= config.occupied_threshold) {
      observed[i] = 1U;
      occupied_z[i].push_back(static_cast<float>(center.z));
      if (item.second > cell_peak_lo[i]) {
        cell_peak_lo[i] = item.second;
      }
    }
  }

  std::vector<double> low_z(grid.data.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<double> high_z(grid.data.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < occupied_z.size(); ++i) {
    const int n = static_cast<int>(occupied_z[i].size());
    if (n < (config_.use_surface_evidence ? 1 : config.min_occupied_voxels_per_cell)) {
      continue;
    }
    const double lo = percentile(occupied_z[i], config.low_z_percentile);
    const double hi = percentile(occupied_z[i], config.high_z_percentile);
    // Single-cell height gate: a cell admitted only at the min voxel count must show a real
    // vertical span (a surface), not two coplanar ground/noise specks. Rejects flat noise pairs
    // while keeping genuine small rocks; cells with more voxels are unaffected.
    if (config.single_cell_height_gate && n <= config.min_occupied_voxels_per_cell &&
        (hi - lo) < config.obstacle_height) {
      continue;
    }
    low_z[i] = lo;
    high_z[i] = hi;
  }

  std::vector<double> ground(low_z.size(), std::numeric_limits<double>::quiet_NaN());
  // Plane-fit mode also produces the true local slope (rise/run) at each cell.
  std::vector<double> plane_slope(low_z.size(), std::numeric_limits<double>::quiet_NaN());
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (config.use_plane_fit) {
        // ROBUST least-squares fit z = a*dx + b*dy + c over window low_z (dx,dy in metres,
        // cell-centred). ground = c (plane at cell centre), slope = hypot(a,b). A plain fit rides
        // UP onto a compact obstacle when the obstacle fills much of the window (e.g. a rock at
        // close range), hiding it; so we fit, then REFIT excluding cells that stick up above the
        // first-pass plane by more than plane_fit_trim (obstacle tops), keeping the plane on the
        // true ground. This detrends slope (vs the downhill-minimum reference) without losing rocks.
        const size_t capacity = static_cast<size_t>(2 * config.ground_window_radius + 1) *
                                static_cast<size_t>(2 * config.ground_window_radius + 1);
        std::vector<double> dxv(capacity), dyv(capacity), zv(capacity);
        int n = 0;
        for (int oy = -config.ground_window_radius; oy <= config.ground_window_radius; ++oy) {
          for (int ox = -config.ground_window_radius; ox <= config.ground_window_radius; ++ox) {
            const int nx = x + ox;
            const int ny = y + oy;
            if (nx < 0 || grid.width <= nx || ny < 0 || grid.height <= ny) {
              continue;
            }
            const double z = low_z[index(nx, ny)];
            if (!std::isfinite(z)) {
              continue;
            }
            dxv[n] = ox * config.resolution;
            dyv[n] = oy * config.resolution;
            zv[n] = z;
            ++n;
          }
        }
        if (n < 3) {
          if (config_.use_surface_evidence) continue;
          if (n > 0) {
            double s = 0.0;
            for (int k = 0; k < n; ++k) { s += zv[k]; }
            ground[index(x, y)] = s / n;  // mean fallback (flat assumption)
            plane_slope[index(x, y)] = 0.0;
          }
          continue;
        }
        // Fit plane z=a*dx+b*dy+c over the samples whose 'use' flag is set; returns ok + a,b,c.
        const auto fit = [&](const std::vector<uint8_t> & use, double & a, double & b, double & c) -> bool {
          double Sxx = 0, Sxy = 0, Syy = 0, Sx = 0, Sy = 0, Sxz = 0, Syz = 0, Sz = 0;
          int m = 0;
          for (int k = 0; k < n; ++k) {
            if (!use[k]) { continue; }
            const double dx = dxv[k], dy = dyv[k], z = zv[k];
            Sxx += dx * dx; Sxy += dx * dy; Syy += dy * dy;
            Sx += dx; Sy += dy; Sxz += dx * z; Syz += dy * z; Sz += z; ++m;
          }
          if (m < 3) { return false; }
          const double m22 = static_cast<double>(m);
          const double det = Sxx * (Syy * m22 - Sy * Sy) - Sxy * (Sxy * m22 - Sy * Sx) +
            Sx * (Sxy * Sy - Syy * Sx);
          if (std::abs(det) < 1.0e-9) { return false; }
          const double inv = 1.0 / det;
          a = inv * (Sxz * (Syy * m22 - Sy * Sy) - Sxy * (Syz * m22 - Sy * Sz) +
            Sx * (Syz * Sy - Syy * Sz));
          b = inv * (Sxx * (Syz * m22 - Sy * Sz) - Sxz * (Sxy * m22 - Sy * Sx) +
            Sx * (Sxy * Sz - Syz * Sx));
          c = inv * (Sxx * (Syy * Sz - Syz * Sy) - Sxy * (Sxy * Sz - Syz * Sx) +
            Sxz * (Sxy * Sy - Syy * Sx));
          return true;
        };
        std::vector<uint8_t> use(n, 1U);
        double a = 0, b = 0, c = 0;
        if (config.use_gpf_ground) {
          // R-GPF: SEED the fit from the lowest gpf_seed_fraction of window cells (start on true ground),
          // then iterate a ONE-SIDED inlier band (keep only cells at/below the plane) so rock tops are
          // rejected every pass and the plane converges onto the ground floor -- it never starts high, so
          // a rock dominating the window can't pull it up the way fit-all-then-trim-once does.
          std::vector<int> order(n);
          for (int k = 0; k < n; ++k) { order[k] = k; }
          std::sort(order.begin(), order.end(), [&](int p, int q) { return zv[p] < zv[q]; });
          const int seed_k = std::max(3, static_cast<int>(std::lround(config.gpf_seed_fraction * n)));
          std::fill(use.begin(), use.end(), 0U);
          for (int s = 0; s < seed_k && s < n; ++s) { use[order[s]] = 1U; }
          bool have = false;
          double al = 0, bl = 0, cl = 0;  // last good plane
          for (int iter = 0; iter < std::max(1, config.gpf_iterations); ++iter) {
            if (!fit(use, a, b, c)) { break; }
            al = a; bl = b; cl = c; have = true;
            int inl = 0;
            for (int k = 0; k < n; ++k) {
              use[k] = (zv[k] - (a * dxv[k] + b * dyv[k] + c) <= config.gpf_inlier_band) ? 1U : 0U;
              inl += use[k];
            }
            if (inl < 3) { a = al; b = bl; c = cl; break; }  // keep the last good plane; don't refit on <3
          }
          if (!have) {
            if (config_.use_surface_evidence) continue;
            double s = 0.0;
            for (int k = 0; k < n; ++k) { s += zv[k]; }
            ground[index(x, y)] = s / n;  // mean fallback (matches non-GPF path)
            plane_slope[index(x, y)] = 0.0;
            continue;
          }
          ground[index(x, y)] = c;
          plane_slope[index(x, y)] = std::hypot(a, b);
        } else {
          if (!fit(use, a, b, c)) {
            if (config_.use_surface_evidence) continue;
            double s = 0.0;
            for (int k = 0; k < n; ++k) { s += zv[k]; }
            ground[index(x, y)] = s / n;
            plane_slope[index(x, y)] = 0.0;
            continue;
          }
          // Trim pass: drop samples sticking up above the first-pass plane (obstacle tops), refit.
          int inliers = 0;
          for (int k = 0; k < n; ++k) {
            use[k] = (zv[k] - (a * dxv[k] + b * dyv[k] + c) <= config.plane_fit_trim) ? 1U : 0U;
            inliers += use[k];
          }
          if (inliers >= 3 && inliers < n) {
            double a2 = a, b2 = b, c2 = c;
            if (fit(use, a2, b2, c2)) { a = a2; b = b2; c = c2; }
          }
          ground[index(x, y)] = c;
          plane_slope[index(x, y)] = std::hypot(a, b);
        }
      } else {
        std::vector<float> samples;
        for (int oy = -config.ground_window_radius; oy <= config.ground_window_radius; ++oy) {
          for (int ox = -config.ground_window_radius; ox <= config.ground_window_radius; ++ox) {
            const int nx = x + ox;
            const int ny = y + oy;
            if (nx < 0 || grid.width <= nx || ny < 0 || grid.height <= ny) {
              continue;
            }
            const double z = low_z[index(nx, ny)];
            if (std::isfinite(z)) {
              samples.push_back(static_cast<float>(z));
            }
          }
        }
        if (!samples.empty()) {
          ground[index(x, y)] = percentile(samples, config.ground_percentile);
        }
      }
    }
  }

  // Robust ground FLOOR + true terrain slope over a WIDER window (config.ground_floor_window_radius).
  // The plane-fit ground above rides up onto a rock that fills the small plane window; the floor (a low
  // percentile of low_z over a wide window) finds true ground around the rock instead. The true slope is
  // fit to the GROUND-ONLY cells (low_z within ground_floor_slope_band of the floor), so the floor height
  // term can be slope-detrended without the rock-contaminated plane gradient -> recall AND precision.
  std::vector<double> ground_floor(low_z.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<double> floor_slope(low_z.size(), 0.0);
  if (config.use_ground_floor) {
    const int R = std::max(1, config.ground_floor_window_radius);
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        std::vector<float> samples;
        for (int oy = -R; oy <= R; ++oy) {
          for (int ox = -R; ox <= R; ++ox) {
            const int nx = x + ox, ny = y + oy;
            if (nx < 0 || grid.width <= nx || ny < 0 || grid.height <= ny) { continue; }
            const double z = low_z[index(nx, ny)];
            if (std::isfinite(z)) { samples.push_back(static_cast<float>(z)); }
          }
        }
        if (samples.empty()) { continue; }
        const double floor_z = percentile(samples, config.ground_floor_percentile);
        ground_floor[index(x, y)] = floor_z;
        double Sxx=0,Sxy=0,Syy=0,Sxz=0,Syz=0,Sx=0,Sy=0,Sz=0; int ng=0;
        for (int oy = -R; oy <= R; ++oy) {
          for (int ox = -R; ox <= R; ++ox) {
            const int nx = x + ox, ny = y + oy;
            if (nx < 0 || grid.width <= nx || ny < 0 || grid.height <= ny) { continue; }
            const double z = low_z[index(nx, ny)];
            if (!std::isfinite(z) || z > floor_z + config.ground_floor_slope_band) { continue; }
            const double dx = ox * config.resolution, dy = oy * config.resolution;
            Sxx+=dx*dx; Sxy+=dx*dy; Syy+=dy*dy; Sxz+=dx*z; Syz+=dy*z; Sx+=dx; Sy+=dy; Sz+=z; ++ng;
          }
        }
        if (ng >= 4) {
          const double nn = ng;
          const double det = Sxx*(Syy*nn - Sy*Sy) - Sxy*(Sxy*nn - Sy*Sx) + Sx*(Sxy*Sy - Syy*Sx);
          if (std::abs(det) > 1e-9) {
            const double a = (Sxz*(Syy*nn - Sy*Sy) - Sxy*(Syz*nn - Sy*Sz) + Sx*(Syz*Sy - Syy*Sz)) / det;
            const double b = (Sxx*(Syz*nn - Sy*Sz) - Sxz*(Sxy*nn - Sy*Sx) + Sx*(Sxy*Sz - Syz*Sx)) / det;
            floor_slope[index(x, y)] = std::hypot(a, b);
          }
        }
      }
    }
  }

  std::vector<uint8_t> obstacle_seed(grid.data.size(), 0U);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const auto i = index(x, y);
      if (!observed[i]) {
        continue;
      }
      if (!std::isfinite(low_z[i]) || !std::isfinite(high_z[i]) ||
          (!std::isfinite(ground[i]) && !(config.use_ground_floor && std::isfinite(ground_floor[i])))) {
        continue;
      }
      // Local-slope detrend (2026-06-15): measure obstacle height RELATIVE TO THE FITTED GROUND
      // PLANE, not vertically. On a tilt/slope (a slanted LiDAR at startup, a crater rim, sloped
      // terrain) a pure-ground cell's vertical span is ~slope*cell_size, which the raw
      // (high_z - low_z) term counted as obstacle height -> the false "ridges along the lidar lines
      // when slanted". Subtract the slope-explained span (plane_slope = robust ground gradient, with
      // rock-tops trimmed out): pure sloped ground cancels to ~0, while a real rock protrudes ABOVE
      // the plane so its height survives. plane_slope is NaN in non-plane-fit mode -> no-op there.
      const double slope_g = std::isfinite(plane_slope[i]) ? plane_slope[i] : 0.0;
      const double slope_span = slope_g * config.resolution;
      double height = 0.0;
      if (std::isfinite(ground[i])) {
        height = std::max(height, (high_z[i] - ground[i]) - 0.5 * slope_span);
      }
      height = std::max(height, (high_z[i] - low_z[i]) - slope_span);
      // Robust-floor height term (2026-06-17): catches rocks the plane-fit misses (plane rode up).
      // Detrend by the TRUE terrain slope across the floor window so sloped ground still cancels.
      if (config.use_ground_floor && std::isfinite(ground_floor[i])) {
        const double floor_detrend =
          floor_slope[i] * static_cast<double>(config.ground_floor_window_radius) * config.resolution;
        height = std::max(height, (high_z[i] - ground_floor[i]) - floor_detrend);
      }
      height = std::max(0.0, height);
      // Seed-confidence gate: only let a cell become a cost-100 OBSTACLE SEED (which is lethal
      // AND gets inflated AND poisons exploration reachability over a 0.9 m disc) if its peak
      // voxel log-odds clears seed_confidence_log_odds (multi-frame confirmation). An unconfirmed
      // obstacle-height cell (e.g. a single-frame near-field noise spike) falls through to the
      // soft terrain band below -- not seeded, not inflated -- and gets promoted to a seed only
      // once it has been re-observed enough. This is what makes lowering min_voxels safe.
      const bool seed_confirmed = cell_peak_lo[i] >= config.seed_confidence_log_odds;
      if (height >= config.lethal_obstacle_height && seed_confirmed) {
        grid.data[i] = static_cast<int8_t>(std::clamp(config.obstacle_cost_max, 0, 100));
        obstacle_seed[i] = 1U;
      } else if (height + 1e-6 >= config.obstacle_height && seed_confirmed) {
        const double t = (height - config.obstacle_height) /
          std::max(config.lethal_obstacle_height - config.obstacle_height, 1.0e-3);
        const double cost = static_cast<double>(config.obstacle_cost_min) +
          static_cast<double>(config.obstacle_cost_max - config.obstacle_cost_min) * t;
        grid.data[i] = static_cast<int8_t>(std::clamp(std::round(cost), 0.0, 100.0));
        obstacle_seed[i] = 1U;
      } else {
        const double terrain_span = std::max(config.obstacle_height - config.terrain_risk_height, 1.0e-3);
        const double t = (height - config.terrain_risk_height) / terrain_span;
        double cost = static_cast<double>(config.terrain_risk_cost_max) * std::clamp(t, 0.0, 1.0);
        if (height + 1e-6 >= config.terrain_risk_height && config.terrain_risk_cost_max > 0) {
          cost = std::max(1.0, cost);
        }
        // Slope (crater-rim) cost from the local ground-height gradient. Soft "avoid unless
        // necessary": does NOT seed an obstacle and is NOT inflated -- just a traversal cost so
        // craters/slopes cost more than flat regolith. Smooth slopes don't trip the height bands
        // above, so this is the only thing that makes them dispreferred.
        if (config.slope_cost_max > 0.0) {
          double slope = std::numeric_limits<double>::quiet_NaN();
          if (config.use_plane_fit) {
            // True local slope straight from the fitted plane gradient.
            slope = plane_slope[i];
          } else if (config.slope_window_cells > 0) {
            const int w = config.slope_window_cells;
            const int xl = x - w, xr = x + w, yl = y - w, yr = y + w;
            double gx = 0.0, gy = 0.0;
            bool have = false;
            if (xl >= 0 && xr < grid.width && std::isfinite(ground[index(xl, y)]) && std::isfinite(ground[index(xr, y)])) {
              gx = (ground[index(xr, y)] - ground[index(xl, y)]) / (2.0 * w * config.resolution);
              have = true;
            }
            if (yl >= 0 && yr < grid.height && std::isfinite(ground[index(x, yl)]) && std::isfinite(ground[index(x, yr)])) {
              gy = (ground[index(x, yr)] - ground[index(x, yl)]) / (2.0 * w * config.resolution);
              have = true;
            }
            if (have) {
              slope = std::hypot(gx, gy);
            }
          }
          if (std::isfinite(slope)) {
            const double st = (slope - config.slope_min) / std::max(config.slope_max - config.slope_min, 1.0e-3);
            cost = std::max(cost, static_cast<double>(config.slope_cost_max) * std::clamp(st, 0.0, 1.0));
          }
        }
        grid.data[i] = static_cast<int8_t>(std::clamp(std::round(cost), 0.0, 100.0));
      }
    }
  }

  // Attribute obstacle memory to the actual elevated return. A newer ground
  // return in the same voxel must not renew an old obstacle-height sample.
  for (const auto & observation : surface_observations) {
    const auto i = observation.cell;
    if (!obstacle_seed[i]) continue;
    const double slope = std::isfinite(plane_slope[i]) ? plane_slope[i] : 0;
    double height = observation.height - low_z[i] - slope * config.resolution;
    if (std::isfinite(ground[i])) height = std::max(height, observation.height-ground[i]-.5*slope*config.resolution);
    if (config.use_ground_floor && std::isfinite(ground_floor[i]))
      height = std::max(height, observation.height-ground_floor[i]-
        floor_slope[i]*config.ground_floor_window_radius*config.resolution);
    if (height + 1e-6 >= config.obstacle_height &&
        (!std::isfinite(grid.obstacle_observed_at[i]) || observation.stamp > grid.obstacle_observed_at[i]))
      grid.obstacle_observed_at[i] = observation.stamp;
  }

  const int inflation_cells = static_cast<int>(std::ceil(config.obstacle_inflation_radius / config.resolution));
  if (inflation_cells > 0 && config.obstacle_inflation_cost > 0) {
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        if (!obstacle_seed[index(x, y)]) {
          continue;
        }
        for (int oy = -inflation_cells; oy <= inflation_cells; ++oy) {
          for (int ox = -inflation_cells; ox <= inflation_cells; ++ox) {
            const int nx = x + ox;
            const int ny = y + oy;
            if (nx < 0 || grid.width <= nx || ny < 0 || grid.height <= ny) {
              continue;
            }
            if (std::hypot(static_cast<double>(ox), static_cast<double>(oy)) * config.resolution >
              config.obstacle_inflation_radius)
            {
              continue;
            }
            const auto ni = index(nx, ny);
            if (grid.data[ni] >= 0) {
              const double distance = std::hypot(static_cast<double>(ox), static_cast<double>(oy)) * config.resolution;
              const double normalized = 1.0 - std::clamp(distance / config.obstacle_inflation_radius, 0.0, 1.0);
              const double shaped = std::pow(normalized, std::max(config.obstacle_inflation_exponent, 1.0e-3));
              const double cost = static_cast<double>(config.obstacle_inflation_min_cost) +
                static_cast<double>(config.obstacle_inflation_cost - config.obstacle_inflation_min_cost) * shaped;
              const int decayed_cost = static_cast<int>(std::clamp(std::round(cost), 0.0, 100.0));
              if (grid.data[ni] < decayed_cost) {
                grid.data[ni] = static_cast<int8_t>(decayed_cost);
              }
            }
          }
        }
      }
    }
  }

  return grid;
}

}  // namespace slam_mapping
