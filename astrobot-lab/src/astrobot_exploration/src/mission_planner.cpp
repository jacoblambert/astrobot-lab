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

#include "astrobot_exploration/mission_planner.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace astrobot_exploration
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// Exact Euclidean distance (cells) from every cell to the nearest zero cell of `input`
// (scipy.ndimage.distance_transform_edt semantics); infinity if there is none.
std::vector<double> distance_transform(const Mask & input, int width, int height)
{
  std::vector<double> d(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    d[i] = input[i] ? kInf : 0.0;
  }
  auto pass = [](std::vector<double> & f) {
      const int n = static_cast<int>(f.size());
      std::vector<double> out(n);
      std::vector<int> v(n);
      std::vector<double> z(n + 1);
      int k = 0;
      v[0] = 0;
      z[0] = -kInf;
      z[1] = kInf;
      for (int q = 1; q < n; ++q) {
        if (f[q] == kInf) {continue;}
        if (f[v[k]] == kInf) {
          v[k] = q;
          continue;
        }
        double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
        while (s <= z[k]) {
          --k;
          s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = kInf;
      }
      if (f[v[0]] == kInf) {return;}
      k = 0;
      for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) {++k;}
        out[q] = (q - v[k]) * static_cast<double>(q - v[k]) + f[v[k]];
      }
      f.swap(out);
    };
  std::vector<double> line;
  for (int c = 0; c < width; ++c) {
    line.assign(height, 0);
    for (int r = 0; r < height; ++r) {
      line[r] = d[static_cast<size_t>(r) * width + c];
    }
    pass(line);
    for (int r = 0; r < height; ++r) {
      d[static_cast<size_t>(r) * width + c] = line[r];
    }
  }
  for (int r = 0; r < height; ++r) {
    line.assign(d.begin() + static_cast<ptrdiff_t>(r) * width,
          d.begin() + static_cast<ptrdiff_t>(r + 1) * width);
    pass(line);
    std::copy(line.begin(), line.end(), d.begin() + static_cast<ptrdiff_t>(r) * width);
  }
  for (auto & v : d) {
    v = std::sqrt(v);
  }
  return d;
}

double hypot2(double a, double b) {return std::hypot(a, b);}

void sort_by_score(std::vector<Candidate> & c)
{
  std::stable_sort(c.begin(), c.end(),
    [](const Candidate & a, const Candidate & b) {
      return a.total_score > b.total_score;
    });
}

}  // namespace

std::optional<Cell> GridMap::cell(double x, double y) const
{
  const int col = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int row = static_cast<int>(std::floor((y - origin_y) / resolution));
  if (row < 0 || col < 0 || row >= height || col >= width) {return std::nullopt;}
  return Cell{row, col};
}

std::pair<double, double> GridMap::world(int row, int col) const
{
  return {origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution};
}

std::vector<int> clearance_components(
  const Mask & known_free, int width, int height,
  double resolution, double radius)
{
  if (!std::isfinite(resolution) || resolution <= 0 || !std::isfinite(radius) || radius < 0) {
    throw std::invalid_argument("invalid clearance geometry");
  }
  // Pad with unsupported cells so map borders bound the clearance.
  const int pw = width + 2, ph = height + 2;
  Mask padded(static_cast<size_t>(pw) * ph, 0);
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      padded[static_cast<size_t>(r + 1) * pw + c + 1] = known_free[static_cast<size_t>(r) * width +
          c];
    }
  }
  const auto distance = distance_transform(padded, pw, ph);
  std::vector<int> labels(known_free.size(), 0);
  Mask safe(known_free.size(), 0);
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      const size_t i = static_cast<size_t>(r) * width + c;
      safe[i] = known_free[i] &&
        distance[static_cast<size_t>(r + 1) * pw + c + 1] * resolution > radius;
    }
  }
  int next = 0;
  std::deque<Cell> queue;
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      const size_t i = static_cast<size_t>(r) * width + c;
      if (!safe[i] || labels[i]) {continue;}
      labels[i] = ++next;
      queue.push_back({r, c});
      while (!queue.empty()) {
        const auto [row, col] = queue.front();
        queue.pop_front();
        for (const auto & [nr, nc] : {Cell{row + 1, col}, Cell{row - 1, col}, Cell{row, col + 1},
            Cell{row, col - 1}})
        {
          if (nr < 0 || nc < 0 || nr >= height || nc >= width) {continue;}
          const size_t j = static_cast<size_t>(nr) * width + nc;
          if (safe[j] && !labels[j]) {
            labels[j] = next;
            queue.push_back({nr, nc});
          }
        }
      }
    }
  }
  return labels;
}

Mask component_near(
  const std::vector<int> & labels, const Mask & known_free, int width, int height,
  std::optional<Cell> start, int max_steps)
{
  Mask out(labels.size(), 0);
  if (!start || start->first < 0 || start->second < 0 || start->first >= height ||
    start->second >= width) {return out;}
  auto idx = [width](int r, int c) {return static_cast<size_t>(r) * width + c;};
  const int at = labels[idx(start->first, start->second)];
  if (at) {
    for (size_t i = 0; i < labels.size(); ++i) {
      out[i] = labels[i] == at;
    }
    return out;
  }
  if (max_steps <= 0 || !known_free[idx(start->first, start->second)]) {return out;}
  std::set<int> values;
  std::set<Cell> seen{*start};
  std::deque<std::pair<Cell, int>> queue{{*start, 0}};
  while (!queue.empty()) {
    const auto [cell, steps] = queue.front();
    queue.pop_front();
    const auto [row, col] = cell;
    if (labels[idx(row, col)]) {
      values.insert(labels[idx(row, col)]);
      continue;
    }
    if (steps == max_steps) {continue;}
    for (const auto & [nr, nc] : {Cell{row + 1, col}, Cell{row - 1, col}, Cell{row, col + 1},
        Cell{row, col - 1}})
    {
      if (nr >= 0 && nc >= 0 && nr < height && nc < width && known_free[idx(nr,
          nc)] && !seen.count({nr, nc}))
      {
        seen.insert({nr, nc});
        queue.push_back({{nr, nc}, steps + 1});
      }
    }
  }
  for (size_t i = 0; i < labels.size(); ++i) {
    out[i] = labels[i] && values.count(labels[i]);
  }
  return out;
}

void MissionPlanner::set_map(GridMap map)
{
  labels_valid_ = false;
  map_ = std::move(map);
  for (const auto & name : resource_names_) {
    auto & grid = resources_[name];
    if (grid.size() != map_->data.size()) {
      grid.assign(map_->data.size(), std::numeric_limits<float>::quiet_NaN());
    }
  }
}

void MissionPlanner::clear_map()
{
  map_.reset();
  labels_valid_ = false;
}

void MissionPlanner::set_pose(const Pose2D & pose)
{
  pose_ = pose;
  if (!home_) {home_ = pose;}
  mark_visited(pose);
}

void MissionPlanner::set_resources(
  std::vector<std::string> names,
  std::map<std::string, double> weights)
{
  resource_names_ = std::move(names);
  resource_weights_ = std::move(weights);
}

void MissionPlanner::resource_sample(const std::string & name, double value)
{
  if (!pose_ || !map_ || !resources_.count(name)) {return;}
  if (const auto cell = map_->cell(pose_->x, pose_->y)) {
    resources_[name][index(cell->first, cell->second)] = static_cast<float>(value);
  }
}

Mask MissionPlanner::known_free() const
{
  Mask free(map_->data.size());
  for (size_t i = 0; i < free.size(); ++i) {
    free[i] = map_->data[i] >= 0 && map_->data[i] <= p_.free_cost_max;
  }
  return free;
}

Mask MissionPlanner::reachable_free()
{
  const Mask free = known_free();
  if (!labels_valid_) {
    labels_ = clearance_components(free, map_->width, map_->height, map_->resolution,
        p_.goal_clearance_radius_m);
    labels_valid_ = true;
  }
  const int steps = static_cast<int>(std::ceil(p_.reachable_seed_distance_m / map_->resolution));
  Mask reachable =
    component_near(labels_, free, map_->width, map_->height,
      pose_ ? map_->cell(pose_->x, pose_->y) : std::nullopt, steps);
  reachable_cells_ = static_cast<int>(std::count(reachable.begin(), reachable.end(), 1));
  return reachable;
}

int MissionPlanner::clearance_radius_cells() const
{
  return std::max(p_.goal_clearance_cells,
      static_cast<int>(std::ceil(p_.goal_clearance_radius_m / map_->resolution)));
}

bool MissionPlanner::near_unsafe(int row, int col) const
{
  const int r = clearance_radius_cells();
  for (int rr = std::max(0, row - r); rr < std::min(map_->height, row + r + 1); ++rr) {
    for (int cc = std::max(0, col - r); cc < std::min(map_->width, col + r + 1); ++cc) {
      if (map_->at(rr, cc) >= p_.goal_clearance_cost_min) {return true;}
    }
  }
  return false;
}

std::vector<double> MissionPlanner::safety_key(
  int row, int col, int target_row,
  int target_col) const
{
  const int r = clearance_radius_cells();
  double max_cost = -kInf, high = 0, sum = 0;
  int known = 0;
  for (int rr = std::max(0, row - r); rr < std::min(map_->height, row + r + 1); ++rr) {
    for (int cc = std::max(0, col - r); cc < std::min(map_->width, col + r + 1); ++cc) {
      const int16_t v = map_->at(rr, cc);
      if (v < 0) {continue;}
      ++known;
      max_cost = std::max<double>(max_cost, v);
      high += v >= p_.precheck_high_cost;
      sum += v;
    }
  }
  const double dist2 = static_cast<double>((row - target_row) * (row - target_row) +
    (col - target_col) * (col - target_col));
  if (!known) {
    return {1000.0, 1000.0, dist2};
  }
  return {max_cost, high, sum / known, dist2};
}

std::optional<Cell> MissionPlanner::nearest_safe_free_cell(
  int row, int col, const Mask & free,
  int max_radius) const
{
  for (int radius = 0; radius <= max_radius; ++radius) {
    std::optional<Cell> best;
    std::vector<double> best_key;
    for (int rr = std::max(0, row - radius); rr < std::min(map_->height, row + radius + 1); ++rr) {
      for (int cc = std::max(0, col - radius); cc < std::min(map_->width, col + radius + 1); ++cc) {
        if (!free[index(rr, cc)] || near_unsafe(rr, cc)) {continue;}
        auto key = safety_key(rr, cc, row, col);
        if (!best || key < best_key) {
          best = Cell{rr, cc};
          best_key = std::move(key);
        }
      }
    }
    if (best) {return best;}
  }
  return std::nullopt;
}

std::optional<Cell> MissionPlanner::staged_goal_cell(double x, double y, const Mask & free) const
{
  const double dx = x - pose_->x, dy = y - pose_->y;
  const double distance = hypot2(dx, dy);
  if (distance <= 1e-6) {return std::nullopt;}
  const double scale = std::min(1.0, p_.max_goal_distance_m / distance);
  const auto target = map_->cell(pose_->x + dx * scale, pose_->y + dy * scale);
  if (!target) {return std::nullopt;}
  return nearest_safe_free_cell(target->first, target->second, free, 10);
}

void MissionPlanner::prune(double now)
{
  auto drop = [now](std::vector<Memory> & memory, double timeout) {
      if (timeout <= 0) {return;}
      memory.erase(std::remove_if(memory.begin(), memory.end(),
        [&](const Memory & m) {
          return now - m.stamp > timeout;
        }),
                 memory.end());
    };
  drop(blacklist_, p_.blacklist_timeout_sec);
  drop(precheck_rejects_, p_.precheck_reject_timeout_sec);
}

bool MissionPlanner::blacklisted(double x, double y, double now)
{
  prune(now);
  for (const auto & m : blacklist_) {
    if (hypot2(x - m.x, y - m.y) <= p_.blacklist_radius_m) {return true;}
  }
  for (const auto & m : precheck_rejects_) {
    if (hypot2(x - m.x, y - m.y) <= p_.precheck_reject_radius_m) {return true;}
  }
  return false;
}

void MissionPlanner::blacklist(const Pose2D & pose, double now)
{
  const double keep = std::max(p_.blacklist_radius_m * 0.5, 1e-3);
  blacklist_.erase(std::remove_if(blacklist_.begin(), blacklist_.end(),
    [&](const Memory & m) {return !(hypot2(pose.x - m.x, pose.y - m.y) > keep);}),
                   blacklist_.end());
  blacklist_.push_back({pose.x, pose.y, now});
}

void MissionPlanner::precheck_rejected(const Pose2D & pose, double now)
{
  precheck_rejects_.push_back({pose.x, pose.y, now});
}

void MissionPlanner::remember_goal(double x, double y, double now)
{
  recent_goals_.push_back({x, y, now});
}

bool MissionPlanner::worth_sending(double x, double y, double distance, double now)
{
  if (distance < p_.min_goal_distance_m) {return false;}
  recent_goals_.erase(std::remove_if(recent_goals_.begin(), recent_goals_.end(),
    [&](const Memory & m) {return now - m.stamp > p_.recent_goal_timeout_sec;}),
                      recent_goals_.end());
  for (const auto & m : recent_goals_) {
    if (hypot2(x - m.x, y - m.y) <= p_.recent_goal_radius_m) {return false;}
  }
  return true;
}

double MissionPlanner::distance_from_robot(const Pose2D & pose) const
{
  return pose_ ? hypot2(pose.x - pose_->x, pose.y - pose_->y) : 0.0;
}

bool MissionPlanner::home_reached(double tolerance) const
{
  if (!pose_ || !home_) {return false;}
  const double d = hypot2(pose_->x - home_->x, pose_->y - home_->y);
  return std::isfinite(d) && d <= tolerance;
}

Mask MissionPlanner::inside_home_radius() const
{
  Mask inside(map_->data.size(), 1);
  if (!home_) {return inside;}
  for (int r = 0; r < map_->height; ++r) {
    for (int c = 0; c < map_->width; ++c) {
      const auto [x, y] = map_->world(r, c);
      inside[index(r,
          c)] = (x - home_->x) * (x - home_->x) + (y - home_->y) * (y - home_->y) <=
        p_.home_radius_m * p_.home_radius_m;
    }
  }
  return inside;
}

std::vector<std::vector<Cell>> MissionPlanner::clusters(const Mask & mask, int max_clusters) const
{
  Mask visited(mask.size(), 0);
  std::vector<std::vector<Cell>> out;
  for (int r = 0; r < map_->height && static_cast<int>(out.size()) < max_clusters; ++r) {
    for (int c = 0; c < map_->width; ++c) {
      if (!mask[index(r, c)] || visited[index(r, c)]) {continue;}
      std::deque<Cell> queue{{r, c}};
      visited[index(r, c)] = 1;
      std::vector<Cell> cluster;
      while (!queue.empty()) {
        const auto [row, col] = queue.front();
        queue.pop_front();
        cluster.push_back({row, col});
        for (const auto & [nr, nc] : {Cell{row - 1, col}, Cell{row + 1, col}, Cell{row, col - 1},
            Cell{row, col + 1}})
        {
          if (map_->inside(nr, nc) && mask[index(nr, nc)] && !visited[index(nr, nc)]) {
            visited[index(nr, nc)] = 1;
            queue.push_back({nr, nc});
          }
        }
      }
      out.push_back(std::move(cluster));
      if (static_cast<int>(out.size()) >= max_clusters) {break;}
    }
  }
  std::stable_sort(out.begin(), out.end(),
    [](const auto & a, const auto & b) {
      return a.size() > b.size();
    });
  return out;
}

std::optional<Cell> MissionPlanner::nearest_free_goal_cell(
  const std::vector<Cell> & cluster,
  const Mask & free) const
{
  std::set<Cell> candidates;
  double sr = 0, sc = 0;
  for (const auto & [row, col] : cluster) {
    sr += row;
    sc += col;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if ((dr || dc) && map_->inside(row + dr, col + dc) && free[index(row + dr, col + dc)]) {
          candidates.insert({row + dr, col + dc});
        }
      }
    }
  }
  if (candidates.empty()) {return std::nullopt;}
  const double cr = sr / cluster.size(), cc = sc / cluster.size();
  return *std::min_element(candidates.begin(), candidates.end(),
           [&](const Cell & a, const Cell & b) {
             return (a.first - cr) * (a.first - cr) + (a.second - cc) * (a.second - cc) <
                    (b.first - cr) * (b.first - cr) + (b.second - cc) * (b.second - cc);
  });
}

bool MissionPlanner::finalize_goal(
  int & row, int & col, double & x, double & y, double & terrain, double & distance,
  const Mask & free, double now)
{
  std::tie(x, y) = map_->world(row, col);
  if (blacklisted(x, y, now)) {return false;}
  terrain = std::max<double>(0, map_->at(row, col));
  if (terrain >= p_.lethal_cost_min || near_unsafe(row, col)) {return false;}
  distance = hypot2(x - pose_->x, y - pose_->y);
  if (!worth_sending(x, y, distance, now)) {return false;}
  if (p_.max_goal_distance_m > 0.0 && distance > p_.max_goal_distance_m) {
    const auto staged = staged_goal_cell(x, y, free);
    if (!staged) {return false;}
    std::tie(row, col) = *staged;
    std::tie(x, y) = map_->world(row, col);
    if (blacklisted(x, y, now)) {return false;}
    terrain = std::max<double>(0, map_->at(row, col));
    if (terrain >= p_.lethal_cost_min || near_unsafe(row, col)) {return false;}
    distance = hypot2(x - pose_->x, y - pose_->y);
    if (!worth_sending(x, y, distance, now)) {return false;}
  }
  return true;
}

std::vector<Candidate> MissionPlanner::frontier_candidates(
  const Mask & reachable,
  const std::optional<std::pair<double, double>> & gradient,
  bool commit, double now)
{
  const Mask inside = inside_home_radius();
  Mask frontier(reachable.size(), 0);
  for (int r = 0; r < map_->height; ++r) {
    for (int c = 0; c < map_->width; ++c) {
      if (map_->at(r, c) >= 0 || !inside[index(r, c)]) {continue;}
      bool near = false;
      for (int dr = -1; dr <= 1 && !near; ++dr) {
        for (int dc = -1; dc <= 1 && !near; ++dc) {
          near = map_->inside(r + dr, c + dc) && reachable[index(r + dr, c + dc)];
        }
      }
      frontier[index(r, c)] = near;
    }
  }
  std::vector<Candidate> out;
  for (const auto & cluster : clusters(frontier, p_.max_frontiers)) {
    if (static_cast<int>(cluster.size()) < p_.min_frontier_cells) {continue;}
    const auto goal = nearest_free_goal_cell(cluster, reachable);
    if (!goal) {continue;}
    int row = goal->first, col = goal->second;
    double x, y, terrain, distance;
    if (!finalize_goal(row, col, x, y, terrain, distance, reachable, now)) {continue;}
    const double over = std::max(0.0, distance - p_.max_goal_distance_m);
    const double resource = resource_score_near(row, col);
    const double info = std::sqrt(static_cast<double>(cluster.size()));
    double bias = 0.0;
    if (gradient && distance > 1e-3) {
      bias = (x - pose_->x) / distance * gradient->first + (y - pose_->y) / distance *
        gradient->second;
    }
    const double home_distance = home_ ? hypot2(x - home_->x, y - home_->y) : 0.0;
    double total = p_.info_weight * info + p_.resource_weight_scale * resource +
      p_.resource_gradient_weight * bias -
      p_.distance_weight * distance - p_.over_distance_weight * over - p_.terrain_weight * terrain -
      p_.home_penalty_weight * home_distance;
    if (commit && gradient) {
      total += p_.resource_gradient_commit_weight * bias * std::min(distance,
          p_.max_goal_distance_m);
    }
    out.push_back({x, y, info, resource, terrain, distance, total,
        static_cast<int>(cluster.size())});
  }
  return out;
}

std::optional<Cell> MissionPlanner::mission_cell(double x, double y) const
{
  if (!home_ || p_.mission_cell_size_m <= 0.0) {return std::nullopt;}
  const double size = p_.mission_cell_size_m;
  const int width = std::max(1, static_cast<int>(std::ceil(2.0 * p_.home_radius_m / size)));
  const double ox = home_->x - 0.5 * width * size, oy = home_->y - 0.5 * width * size;
  const int col = static_cast<int>(std::floor((x - ox) / size));
  const int row = static_cast<int>(std::floor((y - oy) / size));
  if (row < 0 || col < 0 || row >= width || col >= width) {return std::nullopt;}
  return Cell{row, col};
}

void MissionPlanner::mark_visited(const Pose2D & pose)
{
  const auto cell = mission_cell(pose.x, pose.y);
  const double m = p_.mission_visit_margin_m;
  // A pose within the localization error of a cell edge may lie in the neighbour: count the cell
  // only when the pose is this far inside it, so online coverage cannot read above ground truth.
  if (m > 0.0 && cell &&
    !(mission_cell(pose.x - m, pose.y - m) == cell && cell == mission_cell(pose.x + m, pose.y + m)))
  {
    return;
  }
  if (cell) {visited_.insert(*cell);}
}

void MissionPlanner::update_mission_grid(const Mask & reachable)
{
  if (!map_ || !home_ || p_.mission_cell_size_m <= 0.0) {
    grid_.reset();
    return;
  }
  const double size = p_.mission_cell_size_m, res = map_->resolution;
  MissionGrid grid;
  grid.cell_size = size;
  grid.width = grid.height = std::max(1,
      static_cast<int>(std::ceil(2.0 * p_.home_radius_m / size)));
  grid.origin_x = home_->x - 0.5 * grid.width * size;
  grid.origin_y = home_->y - 0.5 * grid.height * size;
  grid.data.assign(static_cast<size_t>(grid.width) * grid.height, -1);
  MissionStats s;
  s.max_reachable_cells = stats_.max_reachable_cells;
  for (int row = 0; row < grid.height; ++row) {
    const double y0 = grid.origin_y + row * size, y1 = y0 + size, cy = y0 + 0.5 * size;
    for (int col = 0; col < grid.width; ++col) {
      const double x0 = grid.origin_x + col * size, x1 = x0 + size, cx = x0 + 0.5 * size;
      if (hypot2(cx - home_->x, cy - home_->y) > p_.home_radius_m) {continue;}
      ++s.total_cells;
      const int c0 = std::max(0, static_cast<int>(std::floor((x0 - map_->origin_x) / res)));
      const int c1 = std::min(map_->width,
          static_cast<int>(std::ceil((x1 - map_->origin_x) / res)));
      const int r0 = std::max(0, static_cast<int>(std::floor((y0 - map_->origin_y) / res)));
      const int r1 = std::min(map_->height,
          static_cast<int>(std::ceil((y1 - map_->origin_y) / res)));
      if (r0 >= r1 || c0 >= c1) {continue;}
      int known = 0;
      bool any_reachable = false;
      for (int r = r0; r < r1; ++r) {
        for (int c = c0; c < c1; ++c) {
          known += map_->at(r, c) >= 0;
          any_reachable |= reachable[index(r, c)] != 0;
        }
      }
      const bool visited = visited_.count({row, col}) != 0;
      const bool observed = static_cast<double>(known) / ((r1 - r0) * (c1 - c0)) >=
        p_.mission_coverage_min_known_fraction;
      if (!any_reachable && !visited) {continue;}
      ++s.reachable_cells;
      int16_t & value = grid.data[static_cast<size_t>(row) * grid.width + col];
      if (visited) {
        ++s.visited_cells;
        ++s.covered_cells;
        value = 80;
      } else {
        value = observed ? 50 : 0;
      }
      s.observed_cells += observed;
    }
  }
  s.max_reachable_cells = std::max(s.max_reachable_cells, s.reachable_cells);
  s.coverage_fraction = s.total_cells >
    0 ? static_cast<double>(s.covered_cells) / s.total_cells : 0.0;
  s.observed_fraction = s.reachable_cells >
    0 ? static_cast<double>(s.observed_cells) / s.reachable_cells : 0.0;
  stats_ = s;
  grid_ = std::move(grid);
}

std::optional<Cell> MissionPlanner::safe_free_cell_in_mission_cell(
  int mission_row, int mission_col,
  const Mask & reachable) const
{
  const double size = p_.mission_cell_size_m, res = map_->resolution;
  const int width = std::max(1, static_cast<int>(std::ceil(2.0 * p_.home_radius_m / size)));
  const double x0 = home_->x - 0.5 * width * size + mission_col * size;
  const double y0 = home_->y - 0.5 * width * size + mission_row * size;
  const int c0 = std::max(0, static_cast<int>(std::floor((x0 - map_->origin_x) / res)));
  const int c1 = std::min(map_->width,
      static_cast<int>(std::ceil((x0 + size - map_->origin_x) / res)));
  const int r0 = std::max(0, static_cast<int>(std::floor((y0 - map_->origin_y) / res)));
  const int r1 = std::min(map_->height,
      static_cast<int>(std::ceil((y0 + size - map_->origin_y) / res)));
  if (r0 >= r1 || c0 >= c1) {return std::nullopt;}
  const double cx = x0 + 0.5 * size, cy = y0 + 0.5 * size;
  auto target = map_->cell(cx, cy);
  if (!target) {
    target = Cell{(r0 + r1) / 2, (c0 + c1) / 2};
  }
  std::optional<Cell> best;
  std::vector<double> best_key;
  for (int r = r0; r < r1; ++r) {
    for (int c = c0; c < c1; ++c) {
      if (!reachable[index(r, c)] || near_unsafe(r, c)) {continue;}
      const auto [x, y] = map_->world(r, c);
      auto key = safety_key(r, c, target->first, target->second);
      key.resize(3);
      // weak preference for the cell centre
      key.push_back((x - cx) * (x - cx) + (y - cy) * (y - cy));
      if (!best || key < best_key) {
        best = Cell{r, c};
        best_key = std::move(key);
      }
    }
  }
  return best;
}

std::vector<Candidate> MissionPlanner::mission_candidates(
  const Mask & reachable,
  bool nearest_first, double now)
{
  update_mission_grid(reachable);
  std::vector<Candidate> out;
  if (!grid_ || !pose_) {return out;}
  const auto & g = *grid_;
  const int search_radius = std::max(2,
      static_cast<int>(std::ceil(p_.mission_cell_size_m / map_->resolution)));
  for (int row = 0; row < g.height; ++row) {
    for (int col = 0; col < g.width; ++col) {
      const int16_t value = g.data[static_cast<size_t>(row) * g.width + col];
      if (value < 0 || value >= 80) {continue;}
      const double x = g.origin_x + (col + 0.5) * g.cell_size,
        y = g.origin_y + (row + 0.5) * g.cell_size;
      if (blacklisted(x, y, now)) {continue;}
      const auto target = map_->cell(x, y);
      if (!target) {continue;}
      auto nav = safe_free_cell_in_mission_cell(row, col, reachable);
      if (!nav) {
        nav = nearest_safe_free_cell(target->first, target->second, reachable, search_radius);
      }
      if (!nav) {continue;}
      int nav_row = nav->first, nav_col = nav->second;
      const auto [gx, gy] = map_->world(nav_row, nav_col);
      if (blacklisted(gx, gy, now)) {continue;}
      const auto goal_cell = mission_cell(gx, gy);
      if (!goal_cell || visited_.count(*goal_cell)) {continue;}
      double goal_x, goal_y, terrain, distance;
      if (!finalize_goal(nav_row, nav_col, goal_x, goal_y, terrain, distance, reachable, now)) {
        continue;
      }
      const double resource = resource_score_near(nav_row, nav_col);
      const double total = p_.mission_candidate_weight + p_.resource_weight_scale * resource -
        p_.distance_weight * distance -
        p_.terrain_weight * terrain;
      out.push_back({goal_x, goal_y, 1.0, resource, terrain, distance, total, 1});
    }
  }
  if (nearest_first) {
    // Greedy nearest-unvisited sweep (0.5 m distance buckets, score breaks ties): a contiguous
    // outward lawnmower instead of jumping to the highest score anywhere in the radius.
    std::stable_sort(out.begin(), out.end(), [](const Candidate & a, const Candidate & b) {
        const double ba = std::nearbyint(a.distance / 0.5), bb = std::nearbyint(b.distance / 0.5);
        return ba != bb ? ba<bb : a.total_score> b.total_score;
    });
  } else {
    sort_by_score(out);
  }
  if (static_cast<int>(out.size()) > p_.mission_candidate_limit) {
    out.resize(std::max(0, p_.mission_candidate_limit));
  }
  return out;
}

double MissionPlanner::resource_score_near(int row, int col) const
{
  const double res = map_->resolution;
  const int radius = std::max(1, static_cast<int>(std::ceil(p_.resource_influence_radius_m / res)));
  const double sigma = std::max(1.0, p_.resource_decay_radius_m / res);
  double total = 0.0;
  for (const auto & [name, grid] : resources_) {
    const auto w = resource_weights_.find(name);
    const double weight = w == resource_weights_.end() ? 1.0 : w->second;
    bool any = false;
    double best = -kInf;
    for (int r = std::max(0, row - radius); r < std::min(map_->height, row + radius + 1); ++r) {
      for (int c = std::max(0, col - radius); c < std::min(map_->width, col + radius + 1); ++c) {
        const float v = grid[index(r, c)];
        any |= !std::isnan(v);
        const double known = std::isnan(v) ? 0.0 : static_cast<double>(v);
        const double d2 = static_cast<double>((r - row) * (r - row) + (c - col) * (c - col));
        best = std::max(best, known * std::exp(-0.5 * d2 / (sigma * sigma)));
      }
    }
    if (any) {total += weight * best;}
  }
  return total;
}

bool MissionPlanner::has_sensed_resource() const
{
  for (const auto & [name, grid] : resources_) {
    for (float v : grid) {
      if (std::isfinite(v)) {return true;}
    }
  }
  return false;
}

std::optional<std::pair<double, double>> MissionPlanner::resource_gradient() const
{
  if (!map_ || !pose_ || resources_.empty()) {return std::nullopt;}
  const int radius = std::max(2,
      static_cast<int>(std::ceil(p_.resource_gradient_radius_m / map_->resolution)));
  const auto cell = map_->cell(pose_->x, pose_->y);
  if (!cell) {return std::nullopt;}
  std::vector<double> rows, cols, vals;
  for (const auto & [name, grid] : resources_) {
    const auto w = resource_weights_.find(name);
    const double weight = w == resource_weights_.end() ? 1.0 : w->second;
    std::vector<double> rr, cc, vv;
    for (int r = std::max(0, cell->first - radius);
      r < std::min(map_->height, cell->first + radius + 1); ++r)
    {
      for (int c = std::max(0, cell->second - radius);
        c < std::min(map_->width, cell->second + radius + 1); ++c)
      {
        const float v = grid[index(r, c)];
        if (std::isnan(v)) {continue;}
        rr.push_back(r);
        cc.push_back(c);
        vv.push_back(static_cast<double>(v) * weight);
      }
    }
    if (vv.size() < 3) {continue;}
    rows.insert(rows.end(), rr.begin(), rr.end());
    cols.insert(cols.end(), cc.begin(), cc.end());
    vals.insert(vals.end(), vv.begin(), vv.end());
  }
  std::vector<double> dr, dc, dist, v;
  for (size_t i = 0; i < rows.size(); ++i) {
    const double a = rows[i] - cell->first, b = cols[i] - cell->second, d = std::hypot(a, b);
    if (d > 1e-6) {
      dr.push_back(a);
      dc.push_back(b);
      dist.push_back(d);
      v.push_back(vals[i]);
    }
  }
  if (v.size() < 3) {return std::nullopt;}
  const double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  double grow = 0, gcol = 0;
  for (size_t i = 0; i < v.size(); ++i) {
    grow += (v[i] - mean) * dr[i] / dist[i];
    gcol += (v[i] - mean) * dc[i] / dist[i];
  }
  const double norm = std::hypot(gcol, grow);
  if (norm < 1e-6) {return std::nullopt;}
  // world (x, y): columns are x, rows are y
  return std::pair<double, double>{gcol / norm, grow / norm};
}

std::optional<Candidate> MissionPlanner::upgradient_goal(
  const Mask & reachable,
  const std::pair<double, double> & g, double now)
{
  if (!pose_ || !map_) {return std::nullopt;}
  const auto staged =
    staged_goal_cell(pose_->x + g.first * p_.max_goal_distance_m,
      pose_->y + g.second * p_.max_goal_distance_m, reachable);
  if (!staged) {return std::nullopt;}
  const auto [row, col] = *staged;
  const auto [x, y] = map_->world(row, col);
  if (blacklisted(x, y, now) || near_unsafe(row, col)) {return std::nullopt;}
  const double terrain = std::max<double>(0, map_->at(row, col));
  if (terrain >= p_.lethal_cost_min) {return std::nullopt;}
  const double distance = hypot2(x - pose_->x, y - pose_->y);
  if (!worth_sending(x, y, distance, now)) {return std::nullopt;}
  const double resource = resource_score_near(row, col);
  const double total = p_.resource_weight_scale * resource + p_.resource_gradient_weight +
    p_.resource_gradient_commit_weight * std::min(distance, p_.max_goal_distance_m) -
    p_.distance_weight * distance - p_.terrain_weight * terrain;
  return Candidate{x, y, 0.0, resource, terrain, distance, total, 1};
}

std::vector<Candidate> MissionPlanner::resource_exploitation_candidates(
  const Mask & reachable, std::set<Cell> occupied,
  double now)
{
  std::vector<Candidate> out;
  if (!map_ || !pose_ || p_.max_resource_candidates <= 0 || resources_.empty()) {return out;}
  const size_t n = map_->data.size();
  std::vector<double> combined(n, std::nan(""));
  Mask known_any(n, 0);
  for (const auto & [name, grid] : resources_) {
    const auto w = resource_weights_.find(name);
    const double weight = w == resource_weights_.end() ? 1.0 : w->second;
    for (size_t i = 0; i < n; ++i) {
      if (!std::isfinite(grid[i])) {continue;}
      known_any[i] = 1;
      const double value = grid[i] * weight;
      combined[i] = std::isnan(combined[i]) ? value : std::max(combined[i], value);  // np.fmax
    }
  }
  std::vector<size_t> finite;
  for (size_t i = 0; i < n; ++i) {
    if (std::isfinite(combined[i])) {finite.push_back(i);}
  }
  if (finite.empty()) {return out;}
  const size_t top = std::min<size_t>(p_.max_resource_candidates, finite.size());
  std::stable_sort(finite.begin(), finite.end(),
    [&](size_t a, size_t b) {
      return combined[a] > combined[b];
    });
  finite.resize(top);
  const double res = map_->resolution;
  const int radius = std::max(1, static_cast<int>(std::ceil(p_.resource_explore_radius_m / res)));
  const double min_step = std::max(1.0, p_.resource_min_explore_step_m / res);
  const Mask inside = inside_home_radius();
  for (size_t source : finite) {
    const int sr = static_cast<int>(source / map_->width),
      sc = static_cast<int>(source % map_->width);
    std::optional<Candidate> best;
    for (int r = std::max(0, sr - radius); r < std::min(map_->height, sr + radius + 1); ++r) {
      for (int c = std::max(0, sc - radius); c < std::min(map_->width, sc + radius + 1); ++c) {
        if (occupied.count({r, c}) || !reachable[index(r, c)] || !inside[index(r,
            c)] || known_any[index(r, c)]) {continue;}
        if (near_unsafe(r, c)) {continue;}
        const double step = std::hypot(r - sr, c - sc);
        if (step < min_step || step > radius) {continue;}
        const auto [x, y] = map_->world(r, c);
        if (blacklisted(x, y, now)) {continue;}
        const double terrain = std::max<double>(0, map_->at(r, c));
        if (terrain >= p_.lethal_cost_min) {continue;}
        const double distance = hypot2(x - pose_->x, y - pose_->y);
        if (!worth_sending(x, y, distance, now)) {continue;}
        if (p_.max_goal_distance_m > 0.0 && distance > p_.max_goal_distance_m) {continue;}
        const double resource = resource_score_near(r, c);
        if (resource <= 0.0) {continue;}
        const double total = p_.resource_weight_scale * resource - p_.distance_weight * distance -
          p_.terrain_weight * terrain;
        if (!best || total > best->total_score) {
          best = Candidate{x, y, 0.0, resource, terrain, distance, total, 1};
        }
      }
    }
    if (best) {
      if (const auto cell = map_->cell(best->x, best->y)) {occupied.insert(*cell);}
      out.push_back(*best);
    }
  }
  return out;
}

std::string MissionPlanner::goal_quality_error(const Pose2D & goal) const
{
  if (!map_) {return "";}
  const auto cell = map_->cell(goal.x, goal.y);
  if (!cell) {return "goal_outside_costmap";}
  if (map_->at(cell->first, cell->second) < 0) {return "goal_unknown";}
  const int r = clearance_radius_cells();
  int max_cost = std::numeric_limits<int>::min();
  for (int rr = std::max(0, cell->first - r); rr < std::min(map_->height, cell->first + r + 1);
    ++rr)
  {
    for (int cc = std::max(0, cell->second - r); cc < std::min(map_->width, cell->second + r + 1);
      ++cc)
    {
      max_cost = std::max<int>(max_cost, map_->at(rr, cc));
    }
  }
  if (max_cost >= p_.goal_clearance_cost_min) {
    return "goal_near_high_cost_" + std::to_string(max_cost);
  }
  return "";
}

std::string MissionPlanner::path_quality_error(
  const std::vector<std::pair<double,
  double>> & path) const
{
  if (!map_) {return "";}
  if (path.empty()) {return "empty_path";}
  const int radius = std::max(0,
      static_cast<int>(std::ceil(p_.precheck_corridor_radius_m / map_->resolution)));
  std::set<Cell> cells;
  for (const auto & [x, y] : path) {
    const auto cell = map_->cell(x, y);
    if (!cell) {return "path_outside_costmap";}
    for (int dr = -radius; dr <= radius; ++dr) {
      for (int dc = -radius; dc <= radius; ++dc) {
        if (dr * dr + dc * dc > radius * radius) {continue;}
        if (map_->inside(cell->first + dr, cell->second + dc)) {
          cells.insert({cell->first + dr, cell->second + dc});
        }
      }
    }
  }
  if (cells.empty()) {return "path_no_costmap_cells";}
  int unknown = 0, known = 0, high = 0, max_cost = std::numeric_limits<int>::min();
  for (const auto & [r, c] : cells) {
    const int16_t v = map_->at(r, c);
    if (v < 0) {
      ++unknown;
      continue;
    }
    ++known;
    high += v >= p_.precheck_high_cost;
    max_cost = std::max<int>(max_cost, v);
  }
  const double unknown_fraction = static_cast<double>(unknown) / cells.size();
  // Distance-scaled unknown tolerance: exploration must enter unknown, so near goals are
  // permissive (the rover maps as it drives the short hop) and far goals strict.
  const double goal_distance = std::hypot(path.back().first - path.front().first,
      path.back().second - path.front().second);
  const double near_d = p_.precheck_unknown_near_dist_m;
  const double far_d = std::max(p_.precheck_unknown_far_dist_m, near_d + 1e-3);
  double tolerance = p_.precheck_max_unknown_fraction;
  if (goal_distance <= near_d) {
    tolerance = p_.precheck_unknown_near_fraction;
  } else if (goal_distance < far_d) {
    const double t = (goal_distance - near_d) / (far_d - near_d);
    tolerance = p_.precheck_unknown_near_fraction + t *
      (p_.precheck_max_unknown_fraction - p_.precheck_unknown_near_fraction);
  }
  char text[96];
  if (unknown_fraction > tolerance) {
    std::snprintf(text, sizeof(text), "path_unknown_fraction_%.2f_tol%.2f_d%.1f", unknown_fraction,
        tolerance, goal_distance);
    return text;
  }
  if (!known) {return "path_all_unknown";}
  if (max_cost >= p_.precheck_max_cost) {return "path_max_cost_" + std::to_string(max_cost);}
  const double high_fraction = static_cast<double>(high) / known;
  if (high_fraction > p_.precheck_max_high_cost_fraction) {
    std::snprintf(text, sizeof(text), "path_high_cost_fraction_%.2f", high_fraction);
    return text;
  }
  return "";
}

Pose2D MissionPlanner::home_goal(bool * moved) const
{
  if (moved) {*moved = false;}
  const Pose2D home = *home_;
  if (p_.home_approach_radius_m <= 0.0 || !map_) {return home;}
  const auto cell = map_->cell(home.x, home.y);
  if (!cell) {return home;}
  const double res = map_->resolution;
  const int reach = static_cast<int>(std::ceil((p_.home_approach_radius_m +
    p_.home_approach_clearance_m) / res)) + 1;
  const int r0 = std::max(0, cell->first - reach), c0 = std::max(0, cell->second - reach);
  const int r1 = std::min(map_->height, cell->first + reach + 1),
    c1 = std::min(map_->width, cell->second + reach + 1);
  const int w = c1 - c0, h = r1 - r0;
  Mask passable(static_cast<size_t>(w) * h);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      passable[static_cast<size_t>(r) * w + c] = map_->at(r0 + r, c0 + c) < p_.lethal_cost_min;
    }
  }
  const auto clearance = distance_transform(passable, w, h);
  if (clearance[static_cast<size_t>(cell->first - r0) * w + (cell->second - c0)] * res >
    p_.home_approach_clearance_m) {return home;}
  double best_offset = kInf;
  Pose2D best = home;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      const int16_t v = map_->at(r0 + r, c0 + c);
      const auto [x, y] = map_->world(r0 + r, c0 + c);
      const double offset = std::hypot(x - home.x, y - home.y);
      if (v >= 0 && v <= p_.free_cost_max &&
        clearance[static_cast<size_t>(r) * w + c] * res > p_.home_approach_clearance_m &&
        offset <= p_.home_approach_radius_m && offset < best_offset)
      {
        best_offset = offset;
        best = {x, y, home.yaw};
      }
    }
  }
  if (moved) {*moved = std::isfinite(best_offset);}
  return best;
}

std::vector<Candidate> explore_radius_candidates(MissionPlanner & planner, double now)
{
  const Mask reachable = planner.reachable_free();
  if (!planner.reachable_cells()) {return std::vector<Candidate>{};}
  auto mission = planner.mission_candidates(reachable, true, now);
  if (!mission.empty()) {return mission;}
  auto frontier = planner.frontier_candidates(reachable, std::nullopt, false, now);
  sort_by_score(frontier);
  return frontier;
}

std::vector<Candidate> sample_return_candidates(
  MissionPlanner & planner, double now,
  int * resource_candidates)
{
  if (resource_candidates) {*resource_candidates = 0;}
  const Mask reachable = planner.reachable_free();
  if (!planner.reachable_cells()) {return std::vector<Candidate>{};}
  auto mission = planner.mission_candidates(reachable, false, now);
  const auto gradient = planner.params().resource_gradient_weight >
    0.0 ? planner.resource_gradient() : std::nullopt;
  if (gradient) {  // tier 1: up-gradient frontiers plus a committing straight up-gradient goal
    auto tier1 = planner.frontier_candidates(reachable, gradient, true, now);
    if (auto synth = planner.upgradient_goal(reachable, *gradient, now)) {tier1.push_back(*synth);}
    sort_by_score(tier1);
    if (!tier1.empty()) {return tier1;}
  }
  if (planner.has_sensed_resource()) {  // tier 2: expand from the best-known samples
    auto tier2 = planner.resource_exploitation_candidates(reachable, {}, now);
    if (!tier2.empty()) {
      sort_by_score(tier2);
      if (resource_candidates) {*resource_candidates = static_cast<int>(tier2.size());}
      return tier2;
    }
  }
  // tier 3: coverage search outward from home
  if (!mission.empty()) {return mission;}
  auto frontier = planner.frontier_candidates(reachable, gradient, true, now);
  sort_by_score(frontier);
  return frontier;
}

}  // namespace astrobot_exploration
