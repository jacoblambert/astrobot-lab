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

// Reachability and goal-selection contracts.
#include <gtest/gtest.h>

#include <cmath>

#include "astrobot_exploration/mission_planner.hpp"

namespace astrobot_exploration
{

namespace
{

Mask full(int w, int h, uint8_t v) {return Mask(static_cast<size_t>(w) * h, v);}
void set(Mask & m, int w, int r, int c, uint8_t v) {m[static_cast<size_t>(r) * w + c] = v;}
bool get(const Mask & m, int w, int r, int c) {return m[static_cast<size_t>(r) * w + c] != 0;}
bool any(const Mask & m)
{
  for (auto v : m) {
    if (v) {return true;}
  }
  return false;
}

GridMap map(int w, int h, int16_t fill)
{
  GridMap g;
  g.width = w;
  g.height = h;
  g.resolution = 0.1;
  g.data.assign(static_cast<size_t>(w) * h, fill);
  return g;
}

}  // namespace

TEST(Reachability, WallCannotBeCrossedOrUsedAsASnappedStart) {
  auto free = full(40, 40, 1);
  for (int r = 0; r < 40; ++r) {
    set(free, 40, r, 20, 0);
}
  const auto labels = clearance_components(free, 40, 40, 0.1, 0.3);
  EXPECT_FALSE(any(component_near(labels, free, 40, 40, Cell{20, 20}, 0)));
  const auto left = component_near(labels, free, 40, 40, Cell{20, 10}, 0);
  EXPECT_TRUE(get(left, 40, 20, 10));
  EXPECT_FALSE(get(left, 40, 20, 30));
}

TEST(Reachability, UnknownAndBoundariesDoNotProveFootprintSupport) {
  auto free = full(40, 40, 1);
  set(free, 40, 20, 20, 0);
  const auto labels = clearance_components(free, 40, 40, 0.1, 0.75);
  EXPECT_FALSE(any(component_near(labels, free, 40, 40, Cell{0, 0}, 0)));
  EXPECT_FALSE(any(component_near(labels, free, 40, 40, Cell{20, 25}, 0)));
  EXPECT_FALSE(any(component_near(labels, free, 40, 40, std::nullopt, 0)));
}

TEST(Reachability, DiagonalCornerDoesNotConnectRegions) {
  Mask eye = full(3, 3, 0);
  for (int i = 0; i < 3; ++i) {
    set(eye, 3, i, i, 1);
}
  const auto labels = clearance_components(eye, 3, 3, 0.1, 0.0);
  const auto at = component_near(labels, eye, 3, 3, Cell{0, 0}, 0);
  EXPECT_EQ(std::count(at.begin(), at.end(), 1), 1);
}

TEST(Reachability, CorridorWidthRespectsConfiguredClearance) {
  auto free = full(50, 50, 0);
  for (int r = 0; r < 50; ++r) {
    for (int c = 17; c < 33; ++c) {
      set(free, 50, r, c, 1);
    }
  }
  EXPECT_TRUE(any(component_near(clearance_components(free, 50, 50, 0.1, 0.75), free, 50, 50,
    Cell{25, 25}, 0)));
  EXPECT_FALSE(any(component_near(clearance_components(free, 50, 50, 0.1, 0.9), free, 50, 50,
    Cell{25, 25}, 0)));
}

TEST(Reachability, StartBesideAnUnseenPatchSeedsFromNearbyClearCellsWithoutCrossingWalls) {
  auto free = full(60, 60, 1);
  for (int r = 0; r < 60; ++r) {
    set(free, 60, r, 40, 0);
}                                                        // wall
  for (int r = 28; r < 31; ++r) {
    for (int c = 22; c < 25; ++c) {
      set(free, 60, r, c, 0);
      // unseen patch beside the rover at (30, 30)
    }
  }
  auto labels = clearance_components(free, 60, 60, 0.1, 0.75);
  EXPECT_FALSE(any(component_near(labels, free, 60, 60, Cell{30, 30}, 0)));
  const auto near = component_near(labels, free, 60, 60, Cell{30, 30}, 10);
  EXPECT_TRUE(get(near, 60, 30, 10));
  for (int r = 0; r < 60; ++r) {
    for (int c = 41; c < 60; ++c) {
      EXPECT_FALSE(get(near, 60, r, c));
    }
  }
  auto blocked = free;
  set(blocked, 60, 30, 30, 0);  // start on an obstacle/unknown cell: nothing
  EXPECT_FALSE(any(component_near(clearance_components(blocked, 60, 60, 0.1, 0.75), blocked, 60, 60,
    Cell{30, 30}, 10)));
}

TEST(MissionPlanner, HomeApproachMovesTheReturnGoalOffARockAtHome) {
  PlannerParams p;
  p.home_approach_radius_m = 0.5;
  p.home_approach_clearance_m = 0.5;
  p.free_cost_max = 35;
  MissionPlanner planner(p);
  planner.set_pose({5.0, 5.0, 0.3});  // home
  planner.set_map(map(100, 100, 0));
  bool moved = true;
  auto goal = planner.home_goal(&moved);
  EXPECT_FALSE(moved);  // clear home: unchanged
  auto rock = map(100, 100, 0);
  for (int r = 47; r < 53; ++r) {
    for (int c = 51; c < 55; ++c) {
      rock.data[static_cast<size_t>(r) * 100 + c] = 100;
      // 0.1-0.5 m east of home
    }
  }
  planner.set_map(rock);
  goal = planner.home_goal(&moved);
  const double offset = std::hypot(goal.x - 5.0, goal.y - 5.0);
  EXPECT_TRUE(moved);
  EXPECT_GT(offset, 0.3);
  EXPECT_LE(offset, 0.5);
  EXPECT_LT(goal.x, 5.0);
  EXPECT_EQ(goal.yaw, 0.3);
  planner.params().home_approach_radius_m = 0.0;
  planner.home_goal(&moved);
  EXPECT_FALSE(moved);  // disabled
}

TEST(MissionPlanner, VisitMarginIgnoresPosesNearACellEdge) {
  PlannerParams p;
  p.home_radius_m = 8.0;
  p.mission_cell_size_m = 2.0;
  p.mission_visit_margin_m = 0.0;
  MissionPlanner no_margin(p);
  no_margin.set_pose({0, 0, 0});
  no_margin.set_pose({1.95, 1.0, 0});  // 5 cm from the x = 2 edge
  EXPECT_TRUE(no_margin.visited().count({4, 4}));
  p.mission_visit_margin_m = 0.15;
  MissionPlanner margin(p);
  margin.set_pose({0.0, 0.0, 0});  // home, on a cell corner: not counted
  margin.set_pose({1.95, 1.0, 0});
  EXPECT_TRUE(margin.visited().empty());
  margin.set_pose({1.0, 1.0, 0});
  EXPECT_EQ(margin.visited(), (std::set<Cell>{{4, 4}}));
}

TEST(MissionPlanner, PathQualityScalesTheUnknownToleranceWithDistance) {
  MissionPlanner planner;
  planner.set_pose({0.0, 0.0, 0});
  auto g = map(100, 100, 0);
  g.origin_x = g.origin_y = -5.0;
  for (int c = 50; c < 100; ++c) {
    g.data[static_cast<size_t>(50) * 100 + c] = -1;
  // unknown strip along the path
}
  planner.set_map(g);
  std::vector<std::pair<double, double>> near_path, far_path;
  for (int i = 0; i <= 15; ++i) {
    near_path.push_back({0.1 * i, 0.0});
}
  for (int i = 0; i <= 45; ++i) {
    far_path.push_back({0.1 * i, 0.0});
}
  EXPECT_EQ(planner.path_quality_error(near_path), "");  // 1.5 m: near tolerance 0.15
  EXPECT_NE(planner.path_quality_error(far_path).find("path_unknown_fraction"), std::string::npos);
  EXPECT_EQ(planner.path_quality_error({}), "empty_path");
}

TEST(MissionPlanner, ExploreCandidatesSweepNearestUnvisitedCellFirst) {
  PlannerParams p;
  p.home_radius_m = 4.0;
  p.mission_cell_size_m = 1.0;
  p.reachable_seed_distance_m = 0.0;
  MissionPlanner planner(p);
  auto g = map(120, 120, 0);
  g.origin_x = g.origin_y = -6.0;
  planner.set_map(g);
  planner.set_pose({0.0, 0.0, 0});
  const auto reachable = planner.reachable_free();
  ASSERT_GT(planner.reachable_cells(), 0);
  const auto c = planner.mission_candidates(reachable, true, 0.0);
  ASSERT_FALSE(c.empty());
  for (size_t i = 1; i < c.size(); ++i) {
    EXPECT_LE(std::nearbyint(c[i - 1].distance / 0.5), std::nearbyint(c[i].distance / 0.5));
}
  EXPECT_GE(c.front().distance, p.min_goal_distance_m);
  planner.blacklist({c.front().x, c.front().y, 0}, 0.0);
  const auto after = planner.mission_candidates(reachable, true, 1.0);
  for (const auto & k : after) {
    EXPECT_GT(std::hypot(k.x - c.front().x, k.y - c.front().y), p.blacklist_radius_m);
}
}

}  // namespace astrobot_exploration
