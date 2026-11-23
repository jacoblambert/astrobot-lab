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

#include <Eigen/Geometry>

#include "astrobot_terrain/pose_timeline.hpp"

namespace astrobot_terrain
{

namespace
{

Pose pose(double x, double yaw)
{
  Pose p = Pose::Identity();
  p.topLeftCorner<3, 3>() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p(0, 3) = x;
  return p;
}

FramePose frame(uint64_t id, int64_t stamp, double x, double yaw = 0)
{
  FramePose f;
  f.frame_id = id;
  f.stamp_ns = stamp;
  f.map_from_imu = pose(x, yaw);
  return f;
}

Revision odom(uint64_t revision, std::vector<FramePose> frames)
{
  Revision r;
  r.session = "s";
  r.epoch = 1;
  r.revision = revision;
  r.parent = revision - 1;
  r.frames = std::move(frames);
  return r;
}

}  // namespace

TEST(PoseTimeline, InterpolatesBodyPoseAndRefusesExtrapolation) {
  PoseTimeline t("s", 1);
  ASSERT_TRUE(t.apply(odom(1,
    {frame(1, 1'000'000'000, 0.0, 0.0), frame(2, 1'100'000'000, 1.0, 0.2)}), 1'200'000'000));
  const auto mid = t.associate(1'050'000'000, 1'200'000'000);
  EXPECT_NEAR(mid.map_from_lidar(0, 3), 0.5, 1e-12);
  EXPECT_NEAR(std::atan2(mid.map_from_lidar(1, 0), mid.map_from_lidar(0, 0)), 0.1, 1e-12);
  EXPECT_THROW(t.associate(1'150'000'000, 1'200'000'000), std::invalid_argument);
  // Not yet available at the output time.
  EXPECT_THROW(t.associate(1'050'000'000, 1'000'000'000), std::invalid_argument);
}

TEST(PoseTimeline, GapAndOrderContracts) {
  PoseTimeline t("s", 1);
  EXPECT_THROW(t.apply(odom(1, {frame(1, 1'000'000'000, 0), frame(2, 1'300'000'000, 1)}),
    1'000'000'000),
               std::invalid_argument);  // measurement after arrival
  ASSERT_TRUE(t.apply(odom(1, {frame(1, 1'000'000'000, 0), frame(2, 1'300'000'000, 1)}),
    1'400'000'000));
  EXPECT_THROW(t.associate(1'100'000'000, 1'400'000'000), std::invalid_argument);  // 0.3 s gap
  EXPECT_FALSE(t.apply(odom(1, {frame(1, 1'000'000'000, 0), frame(2, 1'300'000'000, 1)}),
    1'500'000'000));
  EXPECT_THROW(t.apply(odom(1, {frame(1, 1'000'000'000, 5)}), 1'500'000'000),
    std::invalid_argument);
  EXPECT_THROW(t.apply(odom(3, {frame(3, 1'400'000'000, 2)}), 1'500'000'000),
    std::invalid_argument);
}

TEST(Bookkeeping, AcceptsNoOpAndFaultsOnCorrection) {
  PoseTimeline t("s", 1);
  BookkeepingLedger ledger(10);
  const auto frames = std::vector<FramePose>{frame(1, 1'000'000'000, 0),
    frame(2, 1'100'000'000, 1)};
  ledger.validate_odom(frames);
  ASSERT_TRUE(t.apply(odom(1, frames), 1'200'000'000));
  ledger.commit_odom(frames);
  Revision global = odom(2, frames);
  global.kind = Revision::GLOBAL;
  EXPECT_TRUE(ledger.accept(global, 1'300'000'000, t));
  EXPECT_EQ(t.revision(), 2u);
  EXPECT_FALSE(ledger.accept(global, 1'300'000'000, t));  // retransmission
  Revision moved = odom(3, {frame(1, 1'000'000'000, 0.01)});
  moved.kind = Revision::SUBMAP;
  EXPECT_THROW(ledger.accept(moved, 1'400'000'000, t), std::invalid_argument);
}

}  // namespace astrobot_terrain
