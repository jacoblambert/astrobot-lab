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
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "astrobot_exploration/mission_planner.hpp"

namespace astrobot_exploration
{

// What a mission strategy sees each cycle.
struct MissionContext
{
  MissionPlanner & planner;
  double now{0};          // mission (simulation) seconds
  double best_sample{0};  // best resource sample so far (any resource)
};

// A mission objective, loaded with pluginlib (plugins.xml). The exploration manager owns goal
// dispatch, the Nav2 precheck, reveal goals, failure memory and the return home; a strategy
// decides which candidates to drive to and when the mission is complete.
//
// To add a mission: subclass this, export it in plugins.xml, and select it with the
// `mission_plugin` parameter (docs/extending.md).
class MissionStrategy {
public:
  virtual ~MissionStrategy() = default;
  // Declare and read the strategy's own parameters.
  virtual void initialize(rclcpp::Node & node) = 0;
  // Goals come from the strategy (true) or from an operator via teleop/goal (false).
  virtual bool autonomous() const {return true;}
  // Mission complete even while a goal is active: the manager cancels it and returns home.
  virtual std::optional<std::string> urgent_return(const MissionContext &) {return std::nullopt;}
  // Mission complete, checked between goals.
  virtual std::optional<std::string> return_reason(const MissionContext &) {return std::nullopt;}
  // Ranked goal candidates (best first). Empty: the manager tries reveal goals, then returns home.
  virtual std::vector<Candidate> candidates(MissionContext & context) = 0;
  // Re-run candidate selection while driving and preempt for a clearly better goal.
  virtual bool replan_while_active() const {return false;}
  // Extra fields for the status JSON (number of resource candidates and the like).
  virtual int resource_candidates() const {return 0;}
};

}  // namespace astrobot_exploration
