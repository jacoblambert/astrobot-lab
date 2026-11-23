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
// Ranked candidates of a built-in strategy for one JSON fixture (parity checks):
//   candidates_tool <fixture.json>   ->   {"candidates": [...], "stats": {...}} on stdout
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "astrobot_exploration/mission_planner.hpp"

using astrobot_exploration::explore_radius_candidates;
using astrobot_exploration::GridMap;
using astrobot_exploration::MissionPlanner;
using astrobot_exploration::PlannerParams;
using astrobot_exploration::sample_return_candidates;
using nlohmann::json;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: candidates_tool <fixture.json>\n";
    return 2;
  }
  const json f = json::parse(std::ifstream(argv[1]));
  PlannerParams p;
  const json & q = f.at("params");
  auto num = [&](const char * key, double & value) {value = q.value(key, value);};
  auto integer = [&](const char * key, int & value) {value = q.value(key, value);};
  integer("free_cost_max", p.free_cost_max);
  integer("lethal_cost_min", p.lethal_cost_min);
  integer("goal_clearance_cost_min", p.goal_clearance_cost_min);
  integer("goal_clearance_cells", p.goal_clearance_cells);
  num("goal_clearance_radius_m", p.goal_clearance_radius_m);
  integer("min_frontier_cells", p.min_frontier_cells);
  integer("max_frontiers", p.max_frontiers);
  num("max_goal_distance_m", p.max_goal_distance_m);
  num("min_goal_distance_m", p.min_goal_distance_m);
  num("info_weight", p.info_weight);
  num("distance_weight", p.distance_weight);
  num("over_distance_weight", p.over_distance_weight);
  num("terrain_weight", p.terrain_weight);
  num("home_penalty_weight", p.home_penalty_weight);
  num("home_radius_m", p.home_radius_m);
  num("mission_cell_size_m", p.mission_cell_size_m);
  num("mission_coverage_min_known_fraction", p.mission_coverage_min_known_fraction);
  num("mission_candidate_weight", p.mission_candidate_weight);
  integer("mission_candidate_limit", p.mission_candidate_limit);
  num("mission_visit_margin_m", p.mission_visit_margin_m);
  num("reachable_seed_distance_m", p.reachable_seed_distance_m);
  num("resource_weight_scale", p.resource_weight_scale);
  num("resource_influence_radius_m", p.resource_influence_radius_m);
  num("resource_decay_radius_m", p.resource_decay_radius_m);
  num("resource_gradient_weight", p.resource_gradient_weight);
  num("resource_gradient_radius_m", p.resource_gradient_radius_m);
  num("resource_gradient_commit_weight", p.resource_gradient_commit_weight);
  integer("max_resource_candidates", p.max_resource_candidates);
  num("resource_explore_radius_m", p.resource_explore_radius_m);
  num("resource_min_explore_step_m", p.resource_min_explore_step_m);
  num("blacklist_radius_m", p.blacklist_radius_m);
  num("recent_goal_radius_m", p.recent_goal_radius_m);
  MissionPlanner planner(p);
  planner.set_resources({"water"}, {{"water", 1.0}});
  const json & m = f.at("map");
  GridMap map;
  map.width = m.at("width");
  map.height = m.at("height");
  map.resolution = m.at("resolution");
  map.origin_x = m.at("origin").at(0);
  map.origin_y = m.at("origin").at(1);
  map.data = m.at("data").get<std::vector<int16_t>>();
  for (const auto & pose : f.at("poses")) {
    planner.set_pose({pose.at(0), pose.at(1), pose.at(2)});
  }
  planner.set_map(map);
  for (const auto & r : f.value("resource_cells", json::array())) {
    planner.set_resource_cell("water", r.at(0), r.at(1), r.at(2));
  }
  const double now = f.at("now");
  for (const auto & b : f.value("blacklist", json::array())) {
    planner.blacklist({b.at(0), b.at(1), 0}, b.at(2));
  }
  int resource = 0;
  const auto candidates = f.at("mode") == "sample_return" ? sample_return_candidates(planner, now,
    &resource) :
    explore_radius_candidates(planner, now);
  json out = {{"candidates", json::array()}};
  for (const auto & c : candidates) {
    out["candidates"].push_back({c.x, c.y, c.info_gain, c.resource_score, c.terrain_cost,
        c.distance, c.total_score, c.cells});
  }
  const auto & s = planner.stats();
  out["stats"] = {{"total", s.total_cells}, {"reachable", s.reachable_cells},
    {"covered", s.covered_cells},
    {"observed", s.observed_cells}, {"coverage", s.coverage_fraction},
    {"reachable_cells", planner.reachable_cells()}};
  std::cout << out.dump() << "\n";
  return 0;
}
