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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "astrobot_terrain/grid_ops.hpp"

namespace astrobot_terrain
{

TEST(DistanceTo, MatchesBruteForceOnRandomGrids) {
  std::mt19937 rng(7);
  for (int trial = 0; trial < 20; ++trial) {
    const int w = 5 + trial % 13, h = 3 + trial % 11;
    std::vector<uint8_t> feature(static_cast<size_t>(w) * h);
    for (auto & f : feature) {
      f = rng() % 9 == 0;
    }
    const auto d = distance_to(feature, w, h);
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) {
        double best = std::numeric_limits<double>::infinity();
        for (int rr = 0; rr < h; ++rr) {
          for (int cc = 0; cc < w; ++cc) {
            if (feature[static_cast<size_t>(rr) * w + cc]) {
              best = std::min(best, std::hypot(rr - r, cc - c));
            }
          }
        }
        EXPECT_EQ(d[static_cast<size_t>(r) * w + c], best) << trial << " " << r << " " << c;
      }
    }
  }
}

TEST(Inflate, TapersKnownGroundOnlyAndKeepsUnknown) {
  ClassGrid g(5, 1, 0);
  g.at(0, 0) = 100;
  g.at(0, 4) = -1;
  g.at(0, 2) = 20;
  const auto out = inflate(g);
  EXPECT_EQ(out.at(0, 0), 100);
  EXPECT_EQ(out.at(0, 1), 90);  // ceil(99 * (1 - 0.1))
  EXPECT_EQ(out.at(0, 2), 80);  // inferred ground is raised too
  EXPECT_EQ(out.at(0, 3), 70);
  EXPECT_EQ(out.at(0, 4), -1);
}

TEST(NavigationCosts, BoundedGapAndOccupiedShadow) {
  // Row of measured ground, a 3-cell unknown gap, measured ground; an occupied cell far away.
  ClassGrid classes(20, 3, 0);
  std::vector<uint8_t> reasons(classes.size(), 1);
  for (int c = 5; c < 8; ++c) {
    for (int r = 0; r < 3; ++r) {
      classes.at(r, c) = -1;
      reasons[static_cast<size_t>(r) * 20 + c] = 0;  // NO_RETURNS: plain missing
    }
  }
  classes.at(1, 19) = 100;
  for (int r = 0; r < 3; ++r) {
    classes.at(r, 18) = -1;  // unknown beside the obstacle -> lethal shadow
    reasons[static_cast<size_t>(r) * 20 + 18] = 0;
  }
  std::vector<uint8_t> uncertain;
  const auto costs = navigation_costs(classes, reasons, GapPolicy{}, 0.1, &uncertain);
  EXPECT_EQ(costs.at(1, 6), 70);
  EXPECT_TRUE(uncertain[static_cast<size_t>(1) * 20 + 6]);
  EXPECT_EQ(costs.at(1, 18), 100);
  EXPECT_FALSE(uncertain[static_cast<size_t>(1) * 20 + 18]);
  EXPECT_EQ(costs.at(1, 19), 100);
}

TEST(NavigationCosts, ConflictsStayUnknown) {
  ClassGrid classes(9, 9, 0);
  std::vector<uint8_t> reasons(classes.size(), 1);
  classes.at(4, 4) = -1;
  reasons[4 * 9 + 4] = 8;  // NEGATIVE_EDGE: an explicit conflict, never promoted
  classes.at(4, 5) = -1;
  reasons[4 * 9 + 5] = 0;  // plain missing, but within the hazard clearance of the conflict
  const auto costs = navigation_costs(classes, reasons, GapPolicy{}, 0.1);
  EXPECT_EQ(costs.at(4, 4), -1);
  EXPECT_EQ(costs.at(4, 5), -1);
}

TEST(ClearFootprint, ClearsUnknownUnderBodyOnly) {
  ClassGrid g(20, 20, -1);
  g.at(10, 10) = 100;
  clear_footprint(g, Body{0.3, 0.2}, 1.0, 1.0, 0.0, 0.1);
  EXPECT_EQ(g.at(10, 10), 100);  // occupied stays occupied
  EXPECT_EQ(g.at(10, 12), 0);
  EXPECT_EQ(g.at(12, 10), -1);   // 0.25 m lateral > 0.2 half width
  EXPECT_EQ(g.at(10, 13), -1);   // 0.35 m ahead > 0.3 half length
}

TEST(Mosaic, ObservationsWinOverPolicyCells) {
  Mosaic m(-1, -1, 1, 1, 0, 0, 0.1);
  ClassGrid w(4, 4, 0);
  std::vector<uint8_t> seen(w.size(), 1);
  m.update(w, 0, 0, &seen);
  ClassGrid policy(4, 4, 70);
  std::vector<uint8_t> unseen(w.size(), 0);
  unseen[0] = 1;
  m.update(policy, 0, 0, &unseen);
  const auto out = m.window(0, 0, 4, 4);
  EXPECT_EQ(out.at(0, 0), 70);  // observed again -> latest observation
  EXPECT_EQ(out.at(1, 1), 0);   // policy never overwrites an observation
  const auto outside = m.window(5, 5, 2, 2);
  EXPECT_EQ(outside.at(0, 0), -1);
  EXPECT_THROW(m.update(w, 0.05, 0, &seen), std::invalid_argument);
}

}  // namespace astrobot_terrain
