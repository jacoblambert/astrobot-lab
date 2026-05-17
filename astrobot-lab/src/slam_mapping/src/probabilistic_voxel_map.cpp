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
  if (config_.voxel_size <= 0.0) {
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
    updateVoxel(key, config_.miss_log_odds);
    last_key = key;
  }

  if (add_hit) {
    updateVoxel(end, config_.hit_log_odds);
  }
}

IntegrationStats ProbabilisticVoxelMap::integratePointCloud(const Point3 & sensor_origin, const std::vector<Point3> & points)
{
  IntegrationStats stats;
  stats.input_points = points.size();
  if (!withinBounds(sensor_origin)) {
    stats.skipped_bounds = points.size();
    stats.voxel_count = voxels_.size();
    return stats;
  }

  const std::size_t before_voxels = voxels_.size();
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

    integrateRay(sensor_origin, endpoint, add_hit);
    ++stats.integrated_points;
    if (add_hit) {
      ++stats.hit_updates;
    }
  }

  if (config_.max_voxels > 0 && voxels_.size() >= static_cast<std::size_t>(config_.max_voxels)) {
    stats.skipped_capacity = voxels_.size() - before_voxels;
  }
  stats.voxel_count = voxels_.size();
  stats.miss_updates = stats.voxel_count > before_voxels ? stats.voxel_count - before_voxels : 0;
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
  BevGrid grid;
  grid.resolution = config.resolution;
  grid.origin_x = config.origin_x;
  grid.origin_y = config.origin_y;
  grid.width = static_cast<int>(std::ceil(config.width_m / config.resolution));
  grid.height = static_cast<int>(std::ceil(config.height_m / config.resolution));
  grid.data.assign(static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height), -1);

  const auto index = [&grid](const int x, const int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + static_cast<std::size_t>(x);
  };

  std::vector<std::vector<float>> occupied_z(grid.data.size());
  std::vector<uint8_t> observed(grid.data.size(), 0U);
  for (const auto & item : voxels_) {
    const auto center = keyCenter(item.first);
    const int x = static_cast<int>(std::floor((center.x - config.origin_x) / config.resolution));
    const int y = static_cast<int>(std::floor((center.y - config.origin_y) / config.resolution));
    if (x < 0 || grid.width <= x || y < 0 || grid.height <= y) {
      continue;
    }
    const auto i = index(x, y);
    observed[i] = 1U;
    if (item.second >= config.occupied_threshold) {
      occupied_z[i].push_back(static_cast<float>(center.z));
    } else if (item.second <= config.free_threshold && grid.data[i] < 0) {
      grid.data[i] = 0;
    }
  }

  std::vector<double> low_z(grid.data.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<double> high_z(grid.data.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < occupied_z.size(); ++i) {
    if (static_cast<int>(occupied_z[i].size()) < config.min_occupied_voxels_per_cell) {
      continue;
    }
    low_z[i] = percentile(occupied_z[i], config.low_z_percentile);
    high_z[i] = percentile(occupied_z[i], config.high_z_percentile);
  }

  std::vector<double> ground(low_z.size(), std::numeric_limits<double>::quiet_NaN());
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
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

  std::vector<uint8_t> obstacle_seed(grid.data.size(), 0U);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const auto i = index(x, y);
      if (!observed[i]) {
        continue;
      }
      if (!std::isfinite(low_z[i]) || !std::isfinite(high_z[i]) || !std::isfinite(ground[i])) {
        if (grid.data[i] < 0) {
          grid.data[i] = 0;
        }
        continue;
      }
      const double height = std::max(high_z[i] - ground[i], high_z[i] - low_z[i]);
      if (height >= config.lethal_obstacle_height) {
        grid.data[i] = static_cast<int8_t>(std::clamp(config.obstacle_cost_max, 0, 100));
        obstacle_seed[i] = 1U;
      } else if (height >= config.obstacle_height) {
        const double t = (height - config.obstacle_height) /
          std::max(config.lethal_obstacle_height - config.obstacle_height, 1.0e-3);
        const double cost = static_cast<double>(config.obstacle_cost_min) +
          static_cast<double>(config.obstacle_cost_max - config.obstacle_cost_min) * t;
        grid.data[i] = static_cast<int8_t>(std::clamp(std::round(cost), 0.0, 100.0));
        obstacle_seed[i] = 1U;
      } else {
        const double terrain_span = std::max(config.obstacle_height - config.terrain_risk_height, 1.0e-3);
        const double t = (height - config.terrain_risk_height) / terrain_span;
        const double cost = static_cast<double>(config.terrain_risk_cost_max) * std::clamp(t, 0.0, 1.0);
        grid.data[i] = static_cast<int8_t>(std::clamp(std::round(cost), 0.0, 100.0));
      }
    }
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
