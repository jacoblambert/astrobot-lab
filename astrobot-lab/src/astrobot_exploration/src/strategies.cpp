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
// Built-in mission strategies: explore_radius (coverage of a disc around home), sample_return
// (find a resource sample above a threshold) and teleop_click (operator goals).
#include <pluginlib/class_list_macros.hpp>

#include "astrobot_exploration/mission_strategy.hpp"

namespace astrobot_exploration
{

// Visit coverage cells of mission_cell_size_m inside home_radius_m until coverage_goal_fraction
// of them are visited. Goals: the nearest unvisited reachable cell (0.5 m distance buckets, score
// breaks ties), so the rover sweeps outward; frontiers when no coverage cell is reachable.
class ExploreRadius : public MissionStrategy {
public:
  void initialize(rclcpp::Node & node) override
  {
    coverage_goal_fraction_ = node.declare_parameter("coverage_goal_fraction", 0.70);
  }
  std::optional<std::string> return_reason(const MissionContext & c) override
  {
    if (c.planner.coverage() >= coverage_goal_fraction_) {return "coverage_goal_met";}
    return std::nullopt;
  }
  std::vector<Candidate> candidates(MissionContext & c) override
  {
    return explore_radius_candidates(c.planner, c.now);
  }

private:
  double coverage_goal_fraction_{0.70};
};

// Point-sensed resource search, three tiers: follow the sensed gradient toward the unseen peak;
// else expand from the best sensed point; else systematic coverage until something is sensed.
// Complete (and return home) at the first sample >= sample_success_threshold.
class SampleReturn : public MissionStrategy {
public:
  void initialize(rclcpp::Node & node) override
  {
    threshold_ = node.declare_parameter("sample_success_threshold", 0.75);
    replan_ = node.declare_parameter("replan_while_active", true);
  }
  std::optional<std::string> urgent_return(const MissionContext & c) override
  {
    if (c.best_sample >= threshold_) {return "resource_found";}
    return std::nullopt;
  }
  std::optional<std::string> return_reason(const MissionContext & c) override
  {
    return urgent_return(c);
  }
  bool replan_while_active() const override {return replan_;}
  int resource_candidates() const override {return resource_candidates_;}
  std::vector<Candidate> candidates(MissionContext & c) override
  {
    return sample_return_candidates(c.planner, c.now, &resource_candidates_);
  }

private:
  double threshold_{0.75};
  bool replan_{true};
  int resource_candidates_{0};
};

// Operator goals only (Ops console click -> teleop/goal); never returns home on its own.
class TeleopClick : public MissionStrategy {
public:
  void initialize(rclcpp::Node &) override {}
  bool autonomous() const override {return false;}
  std::vector<Candidate> candidates(MissionContext &) override {return std::vector<Candidate>{};}
};

}  // namespace astrobot_exploration

PLUGINLIB_EXPORT_CLASS(astrobot_exploration::ExploreRadius, astrobot_exploration::MissionStrategy)
PLUGINLIB_EXPORT_CLASS(astrobot_exploration::SampleReturn, astrobot_exploration::MissionStrategy)
PLUGINLIB_EXPORT_CLASS(astrobot_exploration::TeleopClick, astrobot_exploration::MissionStrategy)
