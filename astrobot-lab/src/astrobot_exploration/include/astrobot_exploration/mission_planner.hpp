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

#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace astrobot_exploration
{

struct Pose2D
{
  double x{0}, y{0}, yaw{0};
};

struct Candidate
{
  double x{0}, y{0};
  double info_gain{0}, resource_score{0}, terrain_cost{0}, distance{0}, total_score{0};
  int cells{0};
};

using Cell = std::pair<int, int>;  // (row, col)

// Exploration map (nav_msgs/OccupancyGrid: -1 unknown, 0..100 cost), row-major.
struct GridMap
{
  int width{0}, height{0};
  double resolution{0.1}, origin_x{0}, origin_y{0};
  std::vector<int16_t> data;
  int16_t at(int row, int col) const {return data[static_cast<size_t>(row) * width + col];}
  bool inside(int row, int col) const {return row >= 0 && col >= 0 && row < height && col < width;}
  std::optional<Cell> cell(double x, double y) const;
  std::pair<double, double> world(int row, int col) const;
};

using Mask = std::vector<uint8_t>;  // one byte per map cell

// Labels of 4-connected cells whose circular footprint (radius) lies on known-free cells
// (map borders unsupported). Conservative for the rectangular rover; Nav2 owns exact paths.
std::vector<int> clearance_components(
  const Mask & known_free, int width, int height,
  double resolution, double radius);
// The component at start, or, when the start cell lacks clearance, the components reachable from
// it through known-free cells within max_steps 4-connected steps (never across obstacles/unknown).
Mask component_near(
  const std::vector<int> & labels, const Mask & known_free, int width, int height,
  std::optional<Cell> start, int max_steps);

struct PlannerParams
{
  // Map semantics
  int free_cost_max{30}, lethal_cost_min{80}, goal_clearance_cost_min{50}, goal_clearance_cells{3};
  double goal_clearance_radius_m{0.75};
  // Frontiers and goal selection
  int min_frontier_cells{4}, max_frontiers{80};
  double max_goal_distance_m{5.0}, min_goal_distance_m{0.75};
  double info_weight{1.0}, distance_weight{0.8}, over_distance_weight{2.0}, terrain_weight{0.025};
  double home_penalty_weight{0.5};
  double home_radius_m{8.0};
  // Mission grid (coverage)
  double mission_cell_size_m{1.0}, mission_coverage_min_known_fraction{0.30},
  mission_candidate_weight{3.0};
  int mission_candidate_limit{80};
  double mission_visit_margin_m{0.15};
  // Reachability seeding (unseen patches beside the rover)
  double reachable_seed_distance_m{1.0};
  // Memory of failed and recent goals
  double blacklist_radius_m{0.75}, blacklist_timeout_sec{300.0};
  double precheck_reject_radius_m{0.35}, precheck_reject_timeout_sec{60.0};
  double recent_goal_radius_m{0.75}, recent_goal_timeout_sec{240.0};
  // Resources (sample_return)
  double resource_weight_scale{5.0}, resource_influence_radius_m{4.0},
  resource_decay_radius_m{1.75};
  double resource_gradient_weight{10.0}, resource_gradient_radius_m{6.0},
  resource_gradient_commit_weight{6.0};
  int max_resource_candidates{20};
  double resource_explore_radius_m{3.0}, resource_min_explore_step_m{0.75};
  // Planner precheck (path quality)
  double precheck_corridor_radius_m{0.35};
  int precheck_max_cost{66}, precheck_high_cost{30};
  double precheck_max_high_cost_fraction{0.05}, precheck_max_unknown_fraction{0.02};
  double precheck_unknown_near_fraction{0.15}, precheck_unknown_near_dist_m{2.0},
  precheck_unknown_far_dist_m{4.0};
  // Home approach when home itself sits beside a mapped obstacle
  double home_approach_radius_m{0.5}, home_approach_clearance_m{0.5};
};

struct MissionStats
{
  int total_cells{0}, reachable_cells{0}, max_reachable_cells{0}, covered_cells{0},
  observed_cells{0}, visited_cells{0};
  double coverage_fraction{0}, observed_fraction{0};
};

// The coverage grid (cells of mission_cell_size_m inside home_radius_m): -1 unreachable,
// 0 reachable, 50 observed, 80 visited.
struct MissionGrid
{
  double origin_x{0}, origin_y{0}, cell_size{1};
  int width{0}, height{0};
  std::vector<int16_t> data;
};

// Map-side state and goal-selection machinery shared by all mission strategies. ROS-free;
// times are mission (simulation) seconds.
class MissionPlanner {
public:
  explicit MissionPlanner(PlannerParams params = {})
  : p_(params) {}
  PlannerParams & params() {return p_;}
  const PlannerParams & params() const {return p_;}

  // Inputs
  void set_map(GridMap map);
  void clear_map();
  bool has_map() const {return map_.has_value();}
  const GridMap & map() const {return *map_;}
  void set_pose(const Pose2D & pose);  // base_link pose in the map frame
  const std::optional<Pose2D> & pose() const {return pose_;}
  const std::optional<Pose2D> & home() const {return home_;}
  void set_resources(std::vector<std::string> names, std::map<std::string, double> weights);
  void resource_sample(const std::string & name, double value);
  void set_resource_cell(const std::string & name, int row, int col, double value)
  {
    if (resources_.count(name)) {resources_[name][index(row, col)] = static_cast<float>(value);}
  }

  // Candidate building blocks
  Mask known_free() const;
  Mask reachable_free();
  int reachable_cells() const {return reachable_cells_;}
  std::vector<Candidate> frontier_candidates(
    const Mask & reachable, const std::optional<std::pair<double, double>> & gradient,
    bool commit, double now);
  std::vector<Candidate> mission_candidates(const Mask & reachable, bool nearest_first, double now);
  std::vector<Candidate> resource_exploitation_candidates(
    const Mask & reachable,
    std::set<Cell> occupied, double now);
  std::optional<Candidate> upgradient_goal(
    const Mask & reachable,
    const std::pair<double, double> & gradient, double now);
  std::optional<std::pair<double, double>> resource_gradient() const;
  bool has_sensed_resource() const;
  double resource_score_near(int row, int col) const;

  // Coverage
  const MissionStats & stats() const {return stats_;}
  const std::optional<MissionGrid> & mission_grid() const {return grid_;}
  double coverage() const {return stats_.coverage_fraction;}
  const std::set<Cell> & visited() const {return visited_;}

  // Goal memory
  void blacklist(const Pose2D & pose, double now);
  void precheck_rejected(const Pose2D & pose, double now);
  void remember_goal(double x, double y, double now);
  bool blacklisted(double x, double y, double now);
  bool worth_sending(double x, double y, double distance, double now);
  size_t blacklist_size() const {return blacklist_.size();}
  size_t precheck_reject_size() const {return precheck_rejects_.size();}
  size_t recent_goal_size() const {return recent_goals_.size();}
  void prune(double now);

  // Goal and path checks (empty string = acceptable)
  std::string goal_quality_error(const Pose2D & goal) const;
  std::string path_quality_error(const std::vector<std::pair<double, double>> & path) const;
  bool near_unsafe(int row, int col) const;
  // Home, or the nearest known-free cell with clearance when a mapped obstacle sits at home.
  Pose2D home_goal(bool * moved = nullptr) const;
  bool home_reached(double tolerance) const;
  double distance_from_robot(const Pose2D & pose) const;

private:
  std::optional<Cell> staged_goal_cell(double x, double y, const Mask & free) const;
  std::optional<Cell> nearest_safe_free_cell(
    int row, int col, const Mask & free,
    int max_radius) const;
  std::vector<double> safety_key(int row, int col, int target_row, int target_col) const;
  std::optional<Cell> safe_free_cell_in_mission_cell(
    int mission_row, int mission_col,
    const Mask & reachable) const;
  std::optional<Cell> mission_cell(double x, double y) const;
  void mark_visited(const Pose2D & pose);
  void update_mission_grid(const Mask & reachable);
  Mask inside_home_radius() const;
  std::vector<std::vector<Cell>> clusters(const Mask & mask, int max_clusters) const;
  std::optional<Cell> nearest_free_goal_cell(
    const std::vector<Cell> & cluster,
    const Mask & free) const;
  int clearance_radius_cells() const;
  size_t index(int row, int col) const {return static_cast<size_t>(row) * map_->width + col;}
  // Candidate geometry shared by frontier tiers: snap far goals to a staged step and run the
  // blacklist / clearance / recency checks. Returns false if the goal is not sendable.
  bool finalize_goal(
    int & row, int & col, double & x, double & y, double & terrain, double & distance,
    const Mask & free,
    double now);

  PlannerParams p_;
  std::optional<GridMap> map_;
  std::optional<Pose2D> pose_, home_;
  std::vector<int> labels_;
  bool labels_valid_{false};
  int reachable_cells_{0};
  std::vector<std::string> resource_names_;
  std::map<std::string, double> resource_weights_;
  std::map<std::string, std::vector<float>> resources_;
  std::set<Cell> visited_;
  MissionStats stats_;
  std::optional<MissionGrid> grid_;
  struct Memory { double x, y, stamp; };
  std::vector<Memory> blacklist_, precheck_rejects_, recent_goals_;
};

// Candidate rankings of the built-in strategies (shared by the plugins).
// explore_radius: nearest unvisited coverage cell, else frontiers.
std::vector<Candidate> explore_radius_candidates(MissionPlanner & planner, double now);
// sample_return: gradient tier, best-sample expansion tier, coverage tier, then frontiers.
std::vector<Candidate> sample_return_candidates(
  MissionPlanner & planner, double now,
  int * resource_candidates = nullptr);

}  // namespace astrobot_exploration
