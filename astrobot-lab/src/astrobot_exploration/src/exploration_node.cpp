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

//
// Single-robot mission manager: drives the rover through Nav2 to the goals a mission strategy
// plugin ranks, prechecks each goal with the global planner, remembers failures, and returns home
// when the strategy says the mission is complete or the time/goal budget runs out. Holds while the
// localization supervisor withholds motion permission.
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <astrobot_interfaces/msg/localization_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "astrobot_exploration/mission_strategy.hpp"

namespace astrobot_exploration
{

using nlohmann::json;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using LocalizationStatus = astrobot_interfaces::msg::LocalizationStatus;

namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalize(double yaw) {return std::atan2(std::sin(yaw), std::cos(yaw));}

Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalize(a.yaw + b.yaw)};
}

double seconds(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

std::string mission_plugin_for(const std::string & mode)
{
  if (mode == "sample_return") {return "astrobot_exploration/SampleReturn";}
  if (mode == "teleop_click") {return "astrobot_exploration/TeleopClick";}
  return "astrobot_exploration/ExploreRadius";
}

json pose_json(const Pose2D & p) {return {{"x", p.x}, {"y", p.y}, {"yaw", p.yaw}};}

json candidate_json(const Candidate & c)
{
  return {{"x", c.x}, {"y", c.y}, {"info_gain", c.info_gain}, {"resource_score", c.resource_score},
    {"terrain_cost", c.terrain_cost}, {"distance", c.distance}, {"total_score", c.total_score},
    {"cells", c.cells}};
}

}  // namespace

class ExplorationManager : public rclcpp::Node {
public:
  explicit ExplorationManager(const rclcpp::NodeOptions & options)
  : Node("exploration_manager", options),
    loader_("astrobot_exploration", "astrobot_exploration::MissionStrategy")
  {
    auto & p = planner_.params();
    mission_mode_ = declare_parameter<std::string>("mission_mode", "explore_radius");
    const auto plugin = declare_parameter<std::string>("mission_plugin",
        mission_plugin_for(mission_mode_));
    const auto map_topic = declare_parameter<std::string>("map_topic", "terrain/exploration_map");
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "slam/pose_corrected");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    base_offset_ = {declare_parameter("base_offset_x", 0.264),
      declare_parameter("base_offset_y", 0.017),
      declare_parameter("base_yaw_offset", M_PI)};
    p.home_radius_m = declare_parameter("home_radius_m", p.home_radius_m);
    home_reached_radius_m_ = declare_parameter("home_reached_radius_m", 0.9);
    p.home_approach_radius_m = declare_parameter("home_approach_radius_m",
        p.home_approach_radius_m);
    p.home_approach_clearance_m = declare_parameter("home_approach_clearance_m",
        p.home_approach_clearance_m);
    return_behavior_tree_ = declare_parameter<std::string>("return_behavior_tree", "");
    p.mission_visit_margin_m = declare_parameter("mission_visit_margin_m",
        p.mission_visit_margin_m);
    max_goals_ = static_cast<int>(declare_parameter("max_goals", 500));
    max_duration_sec_ = declare_parameter("max_duration_sec", 1800.0);
    goal_timeout_sec_ = declare_parameter("goal_timeout_sec", 120.0);
    goal_timeout_per_meter_sec_ = declare_parameter("goal_timeout_per_meter_sec", 18.0);
    goal_timeout_max_sec_ = declare_parameter("goal_timeout_max_sec", 300.0);
    goal_max_runtime_sec_ = declare_parameter("goal_max_runtime_sec", 240.0);
    near_goal_success_radius_m_ = declare_parameter("near_goal_success_radius_m", 0.65);
    p.free_cost_max = static_cast<int>(declare_parameter("free_cost_max", p.free_cost_max));
    p.lethal_cost_min = static_cast<int>(declare_parameter("lethal_cost_min", p.lethal_cost_min));
    p.min_frontier_cells = static_cast<int>(declare_parameter("min_frontier_cells",
        p.min_frontier_cells));
    p.max_frontiers = static_cast<int>(declare_parameter("max_frontiers", p.max_frontiers));
    p.goal_clearance_cells = static_cast<int>(declare_parameter("goal_clearance_cells",
        p.goal_clearance_cells));
    p.goal_clearance_radius_m = declare_parameter("goal_clearance_radius_m",
        p.goal_clearance_radius_m);
    p.goal_clearance_cost_min = static_cast<int>(declare_parameter("goal_clearance_cost_min",
        p.goal_clearance_cost_min));
    reveal_distance_m_ = declare_parameter("reveal_distance_m", 1.5);
    reveal_max_attempts_ = static_cast<int>(declare_parameter("reveal_max_attempts", 4));
    p.reachable_seed_distance_m = declare_parameter("reachable_seed_distance_m",
        p.reachable_seed_distance_m);
    p.max_goal_distance_m = declare_parameter("max_goal_distance_m", p.max_goal_distance_m);
    p.min_goal_distance_m = declare_parameter("min_goal_distance_m", p.min_goal_distance_m);
    p.blacklist_radius_m = declare_parameter("blacklist_radius_m", p.blacklist_radius_m);
    p.blacklist_timeout_sec = declare_parameter("blacklist_timeout_sec", p.blacklist_timeout_sec);
    p.precheck_reject_radius_m = declare_parameter("precheck_reject_radius_m",
        p.precheck_reject_radius_m);
    p.precheck_reject_timeout_sec = declare_parameter("precheck_reject_timeout_sec",
        p.precheck_reject_timeout_sec);
    p.recent_goal_radius_m = declare_parameter("recent_goal_radius_m", p.recent_goal_radius_m);
    p.recent_goal_timeout_sec = declare_parameter("recent_goal_timeout_sec",
        p.recent_goal_timeout_sec);
    p.info_weight = declare_parameter("info_weight", p.info_weight);
    p.distance_weight = declare_parameter("distance_weight", p.distance_weight);
    p.over_distance_weight = declare_parameter("over_distance_weight", p.over_distance_weight);
    p.terrain_weight = declare_parameter("terrain_weight", p.terrain_weight);
    p.home_penalty_weight = declare_parameter("home_penalty_weight", p.home_penalty_weight);
    p.resource_weight_scale = declare_parameter("resource_weight_scale", p.resource_weight_scale);
    p.resource_influence_radius_m = declare_parameter("resource_influence_radius_m",
        p.resource_influence_radius_m);
    p.resource_decay_radius_m = declare_parameter("resource_decay_radius_m",
        p.resource_decay_radius_m);
    p.resource_gradient_weight = declare_parameter("resource_gradient_weight",
        p.resource_gradient_weight);
    p.resource_gradient_radius_m = declare_parameter("resource_gradient_radius_m",
        p.resource_gradient_radius_m);
    p.resource_gradient_commit_weight = declare_parameter("resource_gradient_commit_weight",
        p.resource_gradient_commit_weight);
    p.max_resource_candidates = static_cast<int>(declare_parameter("max_resource_candidates",
        p.max_resource_candidates));
    p.resource_explore_radius_m = declare_parameter("resource_explore_radius_m",
        p.resource_explore_radius_m);
    p.resource_min_explore_step_m = declare_parameter("resource_min_explore_step_m",
        p.resource_min_explore_step_m);
    precheck_enabled_ = declare_parameter("planner_precheck_enabled", true);
    // Operator goals skip the precheck by default (like reveal goals): from a fresh spawn most of
    // the path is still unknown; Nav2 and the live local costmap still avoid observed hazards.
    teleop_precheck_ = declare_parameter("teleop_precheck_enabled", false);
    precheck_timeout_sec_ = declare_parameter("planner_precheck_timeout_sec", 8.0);
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    p.precheck_corridor_radius_m = declare_parameter("planner_precheck_corridor_radius_m",
        p.precheck_corridor_radius_m);
    p.precheck_max_cost = static_cast<int>(declare_parameter("planner_precheck_max_cost",
        p.precheck_max_cost));
    p.precheck_high_cost = static_cast<int>(declare_parameter("planner_precheck_high_cost",
        p.precheck_high_cost));
    p.precheck_max_high_cost_fraction =
      declare_parameter("planner_precheck_max_high_cost_fraction",
        p.precheck_max_high_cost_fraction);
    p.precheck_max_unknown_fraction = declare_parameter("planner_precheck_max_unknown_fraction",
        p.precheck_max_unknown_fraction);
    p.precheck_unknown_near_fraction =
      declare_parameter("planner_precheck_unknown_near_fraction", p.precheck_unknown_near_fraction);
    p.precheck_unknown_near_dist_m = declare_parameter("planner_precheck_unknown_near_dist_m",
        p.precheck_unknown_near_dist_m);
    p.precheck_unknown_far_dist_m = declare_parameter("planner_precheck_unknown_far_dist_m",
        p.precheck_unknown_far_dist_m);
    p.mission_cell_size_m = declare_parameter("mission_cell_size_m", p.mission_cell_size_m);
    p.mission_coverage_min_known_fraction =
      declare_parameter("mission_coverage_min_known_fraction",
        p.mission_coverage_min_known_fraction);
    p.mission_candidate_weight = declare_parameter("mission_candidate_weight",
        p.mission_candidate_weight);
    p.mission_candidate_limit = static_cast<int>(declare_parameter("mission_candidate_limit",
        p.mission_candidate_limit));
    replan_cooldown_sec_ = declare_parameter("replan_cooldown_sec", 12.0);
    replan_min_shift_m_ = declare_parameter("replan_min_shift_m", 2.0);
    replan_margin_ = declare_parameter("replan_margin", 8.0);
    require_localization_ = declare_parameter("require_localization", true);
    const double cycle_period = declare_parameter("cycle_period_sec", 2.0);

    std::vector<std::string> names;
    std::stringstream list(declare_parameter<std::string>("resource_names", "water"));
    for (std::string name; std::getline(list, name, ','); ) {
      name.erase(0, name.find_first_not_of(' '));
      name.erase(name.find_last_not_of(' ') + 1);
      if (!name.empty()) {names.push_back(name);}
    }
    std::map<std::string, double> weights;
    std::stringstream weight_list(declare_parameter<std::string>("resource_weights", "water:1.0"));
    for (std::string item; std::getline(weight_list, item, ','); ) {
      const auto colon = item.find(':');
      auto trim = [](std::string s) {
          s.erase(0, s.find_first_not_of(' '));
          s.erase(s.find_last_not_of(' ') + 1);
          return s;
        };
      if (trim(item).empty()) {continue;}
      if (colon == std::string::npos) {
        weights[trim(item)] = 1.0;
      } else {
        weights[trim(item.substr(0, colon))] = std::stod(item.substr(colon + 1));
      }
    }
    planner_.set_resources(names, weights);

    strategy_ = loader_.createSharedInstance(plugin);
    strategy_->initialize(*this);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 5,
        [this](const nav_msgs::msg::OccupancyGrid & m) {
          on_map(m);
    });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 20,
        [this](const geometry_msgs::msg::PoseStamped & m) {
          planner_.set_pose(compose({m.pose.position.x, m.pose.position.y,
            yaw_of(m.pose.orientation)}, base_offset_));
    });
    teleop_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("teleop_goal_topic", "teleop/goal"), 10,
      [this](const geometry_msgs::msg::PoseStamped & m) {on_teleop_goal(m);});
    for (const auto & name : names) {
      resource_subs_.push_back(create_subscription<std_msgs::msg::Float32>(
          "resource/" + name + "/sample", 10,
          [this, name](const std_msgs::msg::Float32 & m) {
            on_resource(name, m.data);
            }));
    }
    health_sub_ = create_subscription<LocalizationStatus>("localization/status", 10,
        [this](const LocalizationStatus & m) {on_health(m);});
    frontier_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("exploration/frontiers", 10);
    selected_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("exploration/selected_goal", 10);
    mission_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("exploration/mission_grid",
        2);
    status_pub_ = create_publisher<std_msgs::msg::String>("exploration/status", 10);
    events_pub_ = create_publisher<std_msgs::msg::String>("exploration/events", 20);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this,
        declare_parameter<std::string>("action_name", "navigate_to_pose"));
    plan_client_ = rclcpp_action::create_client<ComputePathToPose>(
        this, declare_parameter<std::string>("planner_action_name", "compute_path_to_pose"));
    started_at_ = now_s();
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(cycle_period),
        [this] {cycle();});
    RCLCPP_INFO(get_logger(), "exploration manager mission=%s plugin=%s map=%s",
        mission_mode_.c_str(), plugin.c_str(),
                map_topic.c_str());
  }

private:
  struct Resume { Pose2D pose; std::string kind; std::optional<Candidate> candidate; };

  double now_s() {return now().nanoseconds() * 1e-9;}
  double wall_s() {return steady_.now().nanoseconds() * 1e-9;}
  bool motion_allowed()
  {
    return health_ && health_->motion_permitted && wall_s() - health_wall_ < 0.5;
  }
  bool current(uint64_t generation) const {return generation == generation_;}

  void on_health(const LocalizationStatus & m)
  {
    health_ = m;
    health_wall_ = wall_s();
    if (m.epoch != map_epoch_) {  // a new SLAM epoch rebuilds the map: forget the old one
      map_epoch_ = m.epoch;
      map_minimum_stamp_ = seconds(m.header.stamp);
      planner_.clear_map();
    }
    if (!m.motion_permitted) {
      if (active_goal_) {
        resume_ = Resume{active_goal_pose_, active_goal_kind_, std::nullopt};
        cancel_reason_ = "localization_hold";
        cancel_requested_ = true;
        if (goal_handle_) {nav_client_->async_cancel_goal(goal_handle_);}
        clear_active_goal();
      } else if (pending_check_) {
        resume_ = Resume{pending_pose_, pending_kind_, pending_candidate_};
      }
      pending_check_ = false;
      ++generation_;
    }
    if (m.state == LocalizationStatus::FAULTED) {
      resume_.reset();
      mission_failed("localization_fault");
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid & m)
  {
    if (seconds(m.header.stamp) < map_minimum_stamp_) {return;}
    GridMap map;
    map.width = static_cast<int>(m.info.width);
    map.height = static_cast<int>(m.info.height);
    map.resolution = m.info.resolution;
    map.origin_x = m.info.origin.position.x;
    map.origin_y = m.info.origin.position.y;
    map.data.assign(m.data.begin(), m.data.end());
    planner_.set_map(std::move(map));
  }

  void on_resource(const std::string & name, double value)
  {
    last_resource_sample_[name] = value;
    best_sample_ = std::max(best_sample_, value);
    if (best_sample_ == value) {best_sample_resource_ = name;}
    planner_.resource_sample(name, value);
  }

  void on_teleop_goal(const geometry_msgs::msg::PoseStamped & m)
  {
    Pose2D pose{m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)};
    std::string reason;
    if (strategy_->autonomous()) {
      reason = "wrong_mission_mode";
    } else if (finished_) {
      reason = "mission_finished";
    } else if (pending_check_ || active_goal_ || returning_home_) {
      reason = "busy";
    } else if (!m.header.frame_id.empty() && m.header.frame_id != frame_id_) {
      reason = "unsupported_frame:" + m.header.frame_id;
    }
    if (!reason.empty()) {
      event("teleop_goal_rejected", {{"kind", "teleop_goal"}, {"reason", reason}}, pose);
      return;
    }
    if (planner_.pose() && std::abs(pose.yaw) < 1e-9) {pose.yaw = planner_.pose()->yaw;}
    RCLCPP_INFO(get_logger(), "teleop goal x=%.2f y=%.2f yaw=%.2f", pose.x, pose.y, pose.yaw);
    event("teleop_goal_received", {{"kind", "teleop_goal"}}, pose);
    send_goal(pose, "teleop_goal", std::nullopt);
  }

  void cycle()
  {
    if (finished_) {return publish_status();}
    if (require_localization_ && !motion_allowed()) {return publish_status("localization_hold");}
    MissionContext context{planner_, now_s(), best_sample_};
    if (planner_.home() && !returning_home_ && strategy_->autonomous()) {
      if (now_s() - started_at_ >= max_duration_sec_) {
        pending_return_ = "budget_exhausted";
      } else if (const auto reason = strategy_->urgent_return(context)) {
        pending_return_ = *reason;
      }
      if (pending_return_) {
        resume_.reset();
        if (pending_check_) {
          ++generation_;
          pending_check_ = false;
        }
        if (active_goal_) {
          if (!cancel_requested_) {
            cancel_requested_ = true;
            cancel_reason_ = "mission_return";
            if (goal_handle_) {nav_client_->async_cancel_goal(goal_handle_);}
          }
          return publish_status("canceling_for_return");  // wait for the cancel before sending home
        }
        const auto reason = *pending_return_;
        pending_return_.reset();
        return send_return_home(reason);
      }
    }
    if (resume_ && !active_goal_ && !pending_check_) {
      const auto resume = *resume_;
      resume_.reset();
      return send_goal(resume.pose, resume.kind, resume.candidate);
    }
    if (pending_check_) {
      if (precheck_timeout_sec_ > 0.0 && now_s() - pending_started_at_ >= precheck_timeout_sec_) {
        char reason[32];
        std::snprintf(reason, sizeof(reason), "timeout_%.1fs", now_s() - pending_started_at_);
        reject_pending(reason);
      }
      return publish_status();
    }
    if (active_goal_) {
      if (!cancel_requested_ && goal_handle_ && goal_max_runtime_sec_ > 0 &&
        now_s() - active_goal_started_at_ > goal_max_runtime_sec_)
      {
        cancel_reason_ = "runtime_ceiling";
        cancel_requested_ = true;
        nav_client_->async_cancel_goal(goal_handle_);
      }
      maybe_replan(context);
      return publish_status();
    }
    if (returning_home_) {return publish_status();}
    if (!planner_.has_map() || !planner_.pose() || !planner_.home()) {
      return publish_status("waiting_for_inputs");
    }
    started_ = true;
    if (!strategy_->autonomous()) {
      publish_frontiers({});
      return publish_status("awaiting_command");
    }
    if (now_s() - started_at_ >= max_duration_sec_ || goal_count_ >= max_goals_) {
      return send_return_home("budget_exhausted");
    }
    if (const auto reason = strategy_->return_reason(context)) {return send_return_home(*reason);}
    const auto candidates = strategy_->candidates(context);
    publish_mission_grid();
    publish_frontiers(candidates);
    if (!candidates.empty()) {reveal_attempts_ = 0;}
    if (candidates.empty()) {
      if (send_reveal_goal()) {return;}
      return send_return_home("no_candidates");
    }
    const auto & best = candidates.front();
    send_goal({best.x, best.y, planner_.pose()->yaw}, "explore", best);
  }

  // No candidates (the blind ring around a stationary front-LiDAR rover, or a small seeded
  // pocket): a short goal ahead, left, right, back. Bounded per stuck episode.
  bool send_reveal_goal()
  {
    while (reveal_distance_m_ > 0.0 && reveal_attempts_ < reveal_max_attempts_) {
      const double offsets[] = {0.0, M_PI / 2, -M_PI / 2, M_PI};
      const double yaw = planner_.pose()->yaw + offsets[reveal_attempts_ % 4];
      ++reveal_attempts_;
      const Pose2D pose{planner_.pose()->x + reveal_distance_m_ * std::cos(yaw),
        planner_.pose()->y + reveal_distance_m_ * std::sin(yaw), yaw};
      const auto cell = planner_.map().cell(pose.x, pose.y);
      if (cell && planner_.map().at(cell->first,
          cell->second) >= planner_.params().lethal_cost_min)
      {
        continue;
      }
      event("reveal_goal", {{"kind", "reveal"}, {"attempt", reveal_attempts_}}, pose);
      send_goal(pose, "reveal", std::nullopt);
      return true;
    }
    return false;
  }

  void send_return_home(const std::string & reason)
  {
    if (returning_home_ || !planner_.home()) {return;}
    returning_home_ = true;
    return_reason_ = reason;
    bool moved = false;
    const Pose2D goal = planner_.has_map() ? planner_.home_goal(&moved) : *planner_.home();
    if (moved) {
      RCLCPP_INFO(get_logger(), "return home: obstacle at home; approach x=%.2f y=%.2f", goal.x,
          goal.y);
      event("home_approach", {{"kind", "return_home"}}, goal);
    }
    send_goal(goal, "return_home", std::nullopt);
  }

  void mission_failed(const std::string & reason)
  {
    if (failed_) {return;}
    return_reason_ = reason;
    failed_ = finished_ = true;
    RCLCPP_WARN(get_logger(), "mission failed: %s", reason.c_str());
    event("mission_failed", {{"kind", "return_home"}, {"reason", reason}});
  }

  void send_goal(
    const Pose2D & pose, const std::string & kind,
    const std::optional<Candidate> & candidate)
  {
    if (require_localization_ && !motion_allowed()) {
      resume_ = Resume{pose, kind, candidate};
      return;
    }
    const bool precheck = precheck_enabled_ && kind != "return_home" && kind != "reveal" &&
      (kind != "teleop_goal" || teleop_precheck_);
    if (precheck) {
      start_precheck(pose, kind, candidate);
    } else {
      send_nav_goal(pose, kind, candidate);  // home is a normal Nav2 goal with its bounded recovery
    }
  }

  geometry_msgs::msg::PoseStamped stamped(const Pose2D & pose)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = now();
    out.header.frame_id = frame_id_;
    out.pose.position.x = pose.x;
    out.pose.position.y = pose.y;
    out.pose.orientation.z = std::sin(pose.yaw * 0.5);
    out.pose.orientation.w = std::cos(pose.yaw * 0.5);
    return out;
  }

  double timeout_for(double distance) const
  {
    if (goal_timeout_sec_ <= 0.0) {return 0.0;}
    const double t = goal_timeout_sec_ + goal_timeout_per_meter_sec_ * std::max(0.0, distance);
    return goal_timeout_max_sec_ > 0.0 ? std::min(t, goal_timeout_max_sec_) : t;
  }

  void send_nav_goal(
    const Pose2D & pose, const std::string & kind,
    const std::optional<Candidate> & candidate)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
      if (kind == "return_home") {mission_failed("return_failed");}
      publish_status("action_unavailable");
      event("action_unavailable", {{"kind", kind}}, pose, candidate);
      return;
    }
    const uint64_t generation = ++generation_;
    active_goal_ = true;
    goal_handle_.reset();
    active_goal_started_at_ = now_s();
    active_goal_distance_ = planner_.distance_from_robot(pose);
    active_goal_timeout_ = timeout_for(active_goal_distance_);
    active_goal_pose_ = pose;
    active_goal_kind_ = kind;
    active_goal_score_ = candidate ? candidate->total_score : 0.0;
    cancel_reason_.clear();
    cancel_requested_ = false;
    ++goal_count_;
    if (kind == "explore") {planner_.remember_goal(pose.x, pose.y, now_s());}
    NavigateToPose::Goal goal;
    goal.pose = stamped(pose);
    if (kind == "return_home" && !return_behavior_tree_.empty()) {
      goal.behavior_tree = return_behavior_tree_;
    }
    selected_goal_pub_->publish(goal.pose);
    RCLCPP_INFO(get_logger(), "goal %d: %s x=%.2f y=%.2f score=%.2f distance=%.1fm", goal_count_,
        kind.c_str(), pose.x,
                pose.y, active_goal_score_, active_goal_distance_);
    event("goal_sent",
      {{"kind", kind}, {"goal_index", goal_count_}, {"distance_m", active_goal_distance_},
        {"timeout_sec", active_goal_timeout_}},
          pose, candidate);
    const double sent_at = now_s();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, generation, kind, pose](auto handle) {
        if (!current(generation)) {
          if (handle) {nav_client_->async_cancel_goal(handle);}
          return;
        }
        if (!handle) {
          ++failure_count_;
          planner_.blacklist(pose, now_s());
          event("goal_blacklisted", {{"kind", kind}, {"reason", "action_rejected"}}, pose);
          event("goal_rejected",
            {{"kind", kind}, {"reason", "action_rejected"}, {"failures", failure_count_}}, pose);
          clear_active_goal();
          if (kind == "return_home") {mission_failed("return_failed");}
          return;
        }
        event("goal_accepted", {{"kind", kind}}, pose);
        goal_handle_ = handle;
        if (cancel_requested_) {nav_client_->async_cancel_goal(handle);}
      };
    options.result_callback = [this, generation, kind, pose, sent_at](const auto & result) {
        if (!current(generation) || !active_goal_) {return;}
        goal_result(static_cast<int>(result.code), now_s() - sent_at, kind, pose);
      };
    nav_client_->async_send_goal(goal, options);
  }

  void goal_result(int status, double duration, const std::string & kind, const Pose2D & pose)
  {
    const std::string cancel_reason = cancel_reason_;
    if (status == static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED)) {
      ++success_count_;
      event("goal_succeeded",
        {{"kind", kind}, {"status", status}, {"duration_sec", duration},
          {"successes", success_count_}}, pose);
      if (kind == "return_home") {
        if (planner_.home_reached(home_reached_radius_m_)) {
          finished_ = true;
        } else {
          mission_failed("return_failed");
        }
      }
    } else {
      if (cancel_reason == "replan" || cancel_reason == "localization_hold" ||
        cancel_reason == "mission_return")
      {
        // Deliberate preemption, not a failure: no count, no blacklist; the next cycle reselects.
        event("goal_preempted_done", {{"kind", kind}}, pose);
        clear_active_goal();
        return publish_status("running");
      }
      ++failure_count_;
      planner_.blacklist(pose, now_s());
      event("goal_blacklisted",
        {{"kind", kind},
          {"reason", cancel_reason.empty() ? "status_" + std::to_string(status) : cancel_reason}},
            pose);
      RCLCPP_WARN(get_logger(), "goal failed kind=%s status=%d duration=%.1fs", kind.c_str(),
          status, duration);
      event("goal_failed",
        {{"kind", kind}, {"status", status}, {"duration_sec", duration},
          {"failures", failure_count_}}, pose);
      if (kind == "return_home") {
        // A rock beside home can block the exact pose: within the home tolerance counts as home.
        if (planner_.home_reached(home_reached_radius_m_)) {
          finished_ = true;
          const double d = planner_.distance_from_robot(*planner_.home());
          RCLCPP_INFO(get_logger(), "return home: close enough (%.2f m <= %.2f m)", d,
              home_reached_radius_m_);
          event("return_home_close_enough", {{"kind", "return_home"}, {"home_dist", d}});
        } else {
          mission_failed("return_failed");
        }
      }
    }
    clear_active_goal();
    publish_status(strategy_->autonomous() ? "running" : "awaiting_command");
  }

  void clear_active_goal()
  {
    ++generation_;
    active_goal_ = false;
    goal_handle_.reset();
    active_goal_started_at_ = active_goal_timeout_ = active_goal_distance_ = 0.0;
    active_goal_kind_.clear();
    cancel_reason_.clear();
    cancel_requested_ = false;
  }

  // Mid-flight re-evaluation (sample_return): refresh the overlays and preempt toward a clearly
  // better goal after a cooldown, by a score margin and a minimum spatial shift.
  void maybe_replan(MissionContext & context)
  {
    if (!strategy_->replan_while_active() || active_goal_kind_ != "explore" || cancel_requested_ ||
      !goal_handle_) {return;}
    if (!planner_.has_map() || !planner_.pose()) {return;}
    const auto candidates = strategy_->candidates(context);
    publish_mission_grid();
    publish_frontiers(candidates);
    if (candidates.empty() || now_s() - active_goal_started_at_ < replan_cooldown_sec_) {return;}
    if (planner_.distance_from_robot(active_goal_pose_) <= 2.0 * near_goal_success_radius_m_) {
      return;
    }
    const auto & best = candidates.front();
    const double shift = std::hypot(best.x - active_goal_pose_.x, best.y - active_goal_pose_.y);
    if (shift < replan_min_shift_m_ || best.total_score < active_goal_score_ + replan_margin_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "mid-flight re-plan: score %.1f (active %.1f), shift %.1f m",
        best.total_score,
                active_goal_score_, shift);
    event("goal_preempted",
      {{"kind", "explore"}, {"reason", "replan"}, {"new_score", best.total_score},
        {"old_score", active_goal_score_}, {"shift_m", shift}});
    cancel_reason_ = "replan";
    cancel_requested_ = true;
    nav_client_->async_cancel_goal(goal_handle_);
  }

  void start_precheck(
    const Pose2D & pose, const std::string & kind,
    const std::optional<Candidate> & candidate)
  {
    if (!plan_client_->wait_for_action_server(std::chrono::seconds(1))) {
      event("planner_unavailable", {{"kind", kind}}, pose);
      if (kind == "return_home") {mission_failed("return_failed");}
      return;
    }
    const uint64_t generation = ++generation_;
    pending_check_ = true;
    pending_started_at_ = now_s();
    pending_pose_ = pose;
    pending_kind_ = kind;
    pending_candidate_ = candidate;
    event("planner_precheck_started", {{"kind", kind}}, pose, candidate);
    ComputePathToPose::Goal goal;
    goal.goal = stamped(pose);
    goal.planner_id = planner_id_;
    goal.use_start = false;
    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, generation](auto handle) {
        if (!current(generation)) {
          if (handle) {plan_client_->async_cancel_goal(handle);}
          return;
        }
        if (!handle) {reject_pending("planner_rejected");}
      };
    options.result_callback = [this, generation](const auto & result) {
        if (!current(generation) || !pending_check_) {return;}
        const auto & r = *result.result;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || r.error_code != 0 ||
          r.path.poses.empty())
        {
          return reject_pending(!r.error_msg.empty() ? r.error_msg : "planner_error_" +
            std::to_string(r.error_code));
        }
        if (auto error = planner_.goal_quality_error(pending_pose_); !error.empty()) {
          return reject_pending(error);
        }
        std::vector<std::pair<double, double>> path;
        for (const auto & p : r.path.poses) {
          path.push_back({p.pose.position.x, p.pose.position.y});
        }
        if (auto error = planner_.path_quality_error(path); !error.empty()) {
          return reject_pending(error);
        }
        pending_check_ = false;
        const auto pose = pending_pose_;
        const auto kind = pending_kind_;
        const auto candidate = pending_candidate_;
        send_nav_goal(pose, kind, candidate);
      };
    plan_client_->async_send_goal(goal, options);
  }

  void reject_pending(const std::string & reason)
  {
    planner_.precheck_rejected(pending_pose_, now_s());
    ++precheck_rejections_;
    RCLCPP_WARN(get_logger(), "planner precheck failed x=%.2f y=%.2f: %s", pending_pose_.x,
        pending_pose_.y, reason.c_str());
    event("planner_precheck_rejected",
      {{"kind", pending_kind_}, {"reason", reason}, {"precheck_rejections", precheck_rejections_}},
          pending_pose_, pending_candidate_);
    ++generation_;
    if (pending_kind_ == "return_home") {mission_failed("return_failed");}
    pending_check_ = false;
    pending_kind_.clear();
    pending_candidate_.reset();
  }

  void publish_frontiers(const std::vector<Candidate> & candidates)
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    for (size_t i = 0;
      i < candidates.size() && static_cast<int>(i) < planner_.params().max_frontiers; ++i)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = candidates[i].x;
      pose.position.y = candidates[i].y;
      pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    frontier_pub_->publish(msg);
  }

  void publish_mission_grid()
  {
    const auto & grid = planner_.mission_grid();
    if (!grid) {return;}
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.info.resolution = static_cast<float>(grid->cell_size);
    msg.info.width = static_cast<uint32_t>(grid->width);
    msg.info.height = static_cast<uint32_t>(grid->height);
    msg.info.origin.position.x = grid->origin_x;
    msg.info.origin.position.y = grid->origin_y;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid->data.begin(), grid->data.end());
    mission_grid_pub_->publish(msg);
  }

  json counters()
  {
    planner_.prune(now_s());
    const double coverage = started_ ? planner_.coverage() : 0.0;
    return {{"mission_mode", mission_mode_}, {"goals", goal_count_}, {"successes", success_count_},
      {"failures", failure_count_}, {"precheck_rejections", precheck_rejections_},
      {"coverage", coverage}, {"mission_coverage", planner_.coverage()},
      {"blacklist_size", planner_.blacklist_size()},
      {"precheck_reject_memory_size", planner_.precheck_reject_size()}};
  }

  void event(
    const std::string & name, json fields, const std::optional<Pose2D> & pose = std::nullopt,
    const std::optional<Candidate> & candidate = std::nullopt)
  {
    json record = counters();
    record["event"] = name;
    record["stamp"] = now_s();
    record["elapsed_sec"] = now_s() - started_at_;
    for (auto & [key, value] : fields.items()) {record[key] = value;}
    if (pose) {record["pose"] = pose_json(*pose);}
    if (candidate) {record["candidate"] = candidate_json(*candidate);}
    std_msgs::msg::String msg;
    msg.data = record.dump();
    events_pub_->publish(msg);
  }

  void publish_status(const std::string & state = "running")
  {
    json s = counters();
    const auto & stats = planner_.stats();
    const auto & home = planner_.home();
    s.erase("coverage");
    s.erase("mission_coverage");
    s["state"] = failed_ ? "failed" : (finished_ ? "finished" : state);
    s["returning_home"] = returning_home_;
    s["return_reason"] = return_reason_;
    s["active_goal"] = active_goal_;
    s["active_goal_kind"] = active_goal_kind_;
    s["active_goal_distance"] = active_goal_distance_;
    s["active_goal_timeout_sec"] = active_goal_timeout_;
    s["pending_goal_check"] = pending_check_;
    s["coverage_gain_fraction"] = started_ ? planner_.coverage() : 0.0;
    s["mission_cell_size_m"] = planner_.params().mission_cell_size_m;
    s["home_radius_m"] = planner_.params().home_radius_m;
    s["home_x"] = home ? json(home->x) : json(nullptr);
    s["home_y"] = home ? json(home->y) : json(nullptr);
    s["mission_total_cells"] = stats.total_cells;
    s["mission_reachable_cells"] = stats.reachable_cells;
    s["mission_max_reachable_cells"] = stats.max_reachable_cells;
    s["mission_covered_cells"] = stats.covered_cells;
    s["mission_observed_cells"] = stats.observed_cells;
    s["mission_visited_cells"] = stats.visited_cells;
    s["mission_coverage_fraction"] = stats.coverage_fraction;
    s["mission_observed_fraction"] = stats.observed_fraction;
    s["best_sample"] = best_sample_;
    s["best_sample_resource"] = best_sample_resource_;
    s["last_resource_sample"] = last_resource_sample_;
    s["recent_goal_size"] = planner_.recent_goal_size();
    s["reachable_cells"] = planner_.reachable_cells();
    s["resource_candidates"] = strategy_->resource_candidates();
    std_msgs::msg::String msg;
    msg.data = s.dump();
    status_pub_->publish(msg);
  }

  MissionPlanner planner_;
  pluginlib::ClassLoader<MissionStrategy> loader_;
  std::shared_ptr<MissionStrategy> strategy_;
  std::string mission_mode_, frame_id_, return_behavior_tree_, planner_id_;
  Pose2D base_offset_;
  double home_reached_radius_m_{0.9}, max_duration_sec_{1800}, goal_timeout_sec_{120},
  goal_timeout_per_meter_sec_{18};
  double goal_timeout_max_sec_{300}, goal_max_runtime_sec_{240}, near_goal_success_radius_m_{0.65};
  double reveal_distance_m_{1.5}, precheck_timeout_sec_{8}, replan_cooldown_sec_{12},
  replan_min_shift_m_{2}, replan_margin_{8};
  int max_goals_{500}, reveal_max_attempts_{4}, reveal_attempts_{0};
  bool precheck_enabled_{true}, teleop_precheck_{false}, require_localization_{true};
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  std::optional<LocalizationStatus> health_;
  double health_wall_{0}, map_minimum_stamp_{0}, started_at_{0};
  uint64_t map_epoch_{0}, generation_{0};
  bool started_{false}, finished_{false}, failed_{false}, returning_home_{false};
  std::string return_reason_;
  std::optional<std::string> pending_return_;
  std::optional<Resume> resume_;
  int goal_count_{0}, success_count_{0}, failure_count_{0}, precheck_rejections_{0};
  bool active_goal_{false}, cancel_requested_{false};
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle_;
  double active_goal_started_at_{0}, active_goal_timeout_{0}, active_goal_distance_{0},
  active_goal_score_{0};
  Pose2D active_goal_pose_;
  std::string active_goal_kind_, cancel_reason_;
  bool pending_check_{false};
  double pending_started_at_{0};
  Pose2D pending_pose_;
  std::string pending_kind_;
  std::optional<Candidate> pending_candidate_;
  double best_sample_{0};
  std::string best_sample_resource_;
  std::map<std::string, double> last_resource_sample_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, teleop_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> resource_subs_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr health_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontier_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mission_grid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, events_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr plan_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

rclcpp::Node::SharedPtr make_exploration_manager(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ExplorationManager>(options);
}

}  // namespace astrobot_exploration

RCLCPP_COMPONENTS_REGISTER_NODE(astrobot_exploration::ExplorationManager)
