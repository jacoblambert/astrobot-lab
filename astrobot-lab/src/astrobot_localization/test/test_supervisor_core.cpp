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

// Contracts of the localization supervisor.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include "astrobot_localization/child_config.hpp"
#include "astrobot_localization/supervisor_core.hpp"

namespace astrobot_localization
{

namespace
{

struct FakeEffects : SupervisorEffects
{
  std::vector<State> statuses;
  bool last_motion{false};
  int fixes{0}, poses{0}, resets{0}, clears{0}, launches{0}, stops{0}, cancels{0};
  uint64_t last_reset_id{0}, last_clear_id{0}, last_reset_epoch{0};
  bool ready{true}, exited{false};
  Launch launch_result{Launch::STARTED};
  void publish_status(
    State s, uint64_t, const std::string &, bool motion,
    const std::optional<StampKey> &) override
  {
    statuses.push_back(s);
    last_motion = motion;
  }
  void publish_fix(const Fix &) override {++fixes;}
  void publish_pose(const PoseSample &) override {++poses;}
  bool reset_ready() override {return ready;}
  void request_reset(uint64_t id, uint64_t epoch) override
  {
    ++resets;
    last_reset_id = id;
    last_reset_epoch = epoch;
  }
  bool clear_ready() override {return ready;}
  void request_clear(uint64_t id) override
  {
    ++clears;
    last_clear_id = id;
  }
  void cancel_navigation() override {++cancels;}
  Launch launch_child(uint64_t, const std::array<double, 7> &) override
  {
    ++launches;
    return launch_result;
  }
  void stop_child() override {++stops;}
  bool child_exited() override {return exited;}
  void log_error(const std::string &) override {}
};

struct Supervisor : ::testing::Test
{
  FakeEffects effects;
  SupervisorCore core{effects};
  void SetUp() override
  {
    core.tick(9.9, 100.0);  // clock running
    core.tick(10.0, 100.01);
  }
};

PoseSample pose(double t = 10, double x = 0, double z = 0)
{
  PoseSample p;
  p.stamp = t;
  p.key = {static_cast<int32_t>(t), static_cast<uint32_t>(std::lround((t - std::floor(t)) * 1e9))};
  p.position = {x, 0, z};
  return p;
}

Fix fix(double t, double x = 0, double y = 0, double z = 0, double sigma = 0.075)
{
  Fix f;
  f.stamp = t;
  f.position = {x, y, z};
  f.covariance[0] = f.covariance[7] = sigma * sigma;
  f.covariance[14] = (2 * sigma) * (2 * sigma);  // synthetic_rtk sigma_z_scale
  f.covariance[21] = f.covariance[28] = 1e6;
  f.covariance[35] = 1e-4;
  return f;
}

std::vector<int8_t> grid(size_t n, int8_t fill) {return std::vector<int8_t>(n, fill);}

SupervisorCore::MapView view(const std::vector<int8_t> & data, double stamp)
{
  SupervisorCore::MapView v;
  v.stamp = stamp;
  v.frame_id = "map";
  v.resolution = 0.1;
  v.width = 100;
  v.height = 20;
  v.data = data.data();
  v.size = data.size();
  return v;
}

}  // namespace

TEST_F(Supervisor, TerminalFaultInvalidatesPendingReset) {
  core.reference_override = [](double) {return std::array<double, 7>{0, 0, 0, 0, 0, 0, 1};};
  core.tick(10.02, 100.03);
  ASSERT_EQ(effects.resets, 1);
  core.fault("injected", true);
  core.reset_result(effects.last_reset_id, true, core.epoch());
  core.tick(10.04, 100.05);
  EXPECT_EQ(effects.launches, 0);
  EXPECT_EQ(core.state(), State::FAULTED);
  EXPECT_FALSE(effects.last_motion);
}

TEST_F(Supervisor, BadPoseIsNotPublishedAndStops) {
  core.set_state(State::HEALTHY);
  auto bad = pose();
  bad.position[0] = std::nan("");
  core.pose(bad, 10.0);
  EXPECT_EQ(effects.poses, 0);
  EXPECT_EQ(core.state(), State::STOPPING);
}

TEST_F(Supervisor, DuplicateFrameRevisionDoesNotTriggerRestart) {
  core.set_state(State::REBUILDING);
  core.pose(pose(), 10.0);
  core.pose(pose(), 10.0);
  EXPECT_EQ(core.state(), State::REBUILDING);
  EXPECT_EQ(effects.poses, 1);
}

TEST_F(Supervisor, DeferredOutputsWaitForTheirPose) {
  core.set_state(State::REBUILDING);
  int clouds = 0;
  core.cloud(10.0, pose().key, [&] {++clouds;}, 10.0);
  EXPECT_EQ(clouds, 0);
  core.pose(pose(), 10.0);
  EXPECT_EQ(clouds, 1);
}

TEST_F(Supervisor, RebuildNeedsLocalFreshSupport) {
  core.set_last_pose(pose(10, 5));
  auto data = grid(2000, -1);
  std::fill(data.begin(), data.begin() + 30, 0);  // free cells exist, but nowhere near the rover
  core.map(view(data, 10), 10.0);
  EXPECT_FALSE(core.map_ready());
  for (int row = 0; row < 5; ++row) {
    std::fill(data.begin() + row * 100 + 45, data.begin() + row * 100 + 50, 0);
}
  core.map(view(data, 10), 10.0);
  EXPECT_TRUE(core.map_ready());
  core.map(view(data, 8), 10.0);
  EXPECT_FALSE(core.map_ready());
}

TEST_F(Supervisor, OneLateMapMessageDoesNotEraseTheLastFreshMap) {
  core.set_last_pose(pose(10, 5));
  const auto data = grid(2000, 0);
  core.map(view(data, 10), 10.0);
  EXPECT_TRUE(core.map_ready());
  ASSERT_TRUE(core.map_stamp());
  core.map(view(data, 8), 10.0);  // 2 s old on arrival: rejected, but the 10 s map still counts
  EXPECT_FALSE(core.map_ready());
  EXPECT_EQ(*core.map_stamp(), 10.0);
}

TEST_F(Supervisor, RestartRequiresStationaryFiniteSixAxisTwist) {
  for (int axis = 0; axis < 6; ++axis) {
    std::array<double, 6> twist{};
    twist[axis] = axis == 0 ? std::nan("") : (axis < 3 ? 0.03 : 0.1);
    core.odom(10.0, twist);
    EXPECT_FALSE(core.stationary_reference(10.0)) << axis;
  }
}

TEST_F(Supervisor, StationaryReferenceFromRtkAndGravity) {
  double t = 9.0;
  for (int k = 0; k <= 25; ++k, t += 0.04) {
    core.fix(fix(t, 1.0, 2.0, 0.0), t);
    core.imu(t, {0.0, 0.0, 1.62});
  }
  core.odom(10.0, {});
  EXPECT_FALSE(core.stationary_reference(10.0));  // stationary for < 1 s
  core.odom(11.0, {});
  for (; t <= 11.0; t += 0.04) {
    core.fix(fix(t, 1.0, 2.0, 0.0), t);
    core.imu(t, {0.0, 0.0, 1.62});
  }
  const auto reference = core.stationary_reference(11.0);
  ASSERT_TRUE(reference);
  EXPECT_NEAR((*reference)[0], 1.0, 1e-12);
  EXPECT_NEAR((*reference)[6], 1.0, 1e-12);
}

TEST_F(Supervisor, FailedReinitializationsAreBounded) {
  core.reference_override = [](double) {return std::array<double, 7>{0, 0, 0, 0, 0, 0, 1};};
  double now = 10.02, wall = 100.03;
  for (int i = 0; i < 3; ++i, now += 0.02, wall += 0.02) {
    core.tick(now, wall);
    if (effects.resets > i) {core.reset_result(effects.last_reset_id, false, core.epoch());}
    now += 0.02;
    wall += 0.02;
    core.tick(now, wall);
  }
  EXPECT_EQ(effects.resets, 2);
  EXPECT_EQ(core.state(), State::FAULTED);
  EXPECT_EQ(core.reason(), "restart_budget_exhausted");
  EXPECT_EQ(effects.launches, 0);
}

TEST_F(Supervisor, WrongEpochResetAckDoesNotLaunch) {
  core.reference_override = [](double) {return std::array<double, 7>{0, 0, 0, 0, 0, 0, 1};};
  core.tick(10.02, 100.03);
  core.reset_result(effects.last_reset_id, true, 999);
  core.tick(10.04, 100.05);
  EXPECT_EQ(core.state(), State::STOPPING);
  EXPECT_EQ(effects.launches, 0);
}

TEST_F(Supervisor, ValidatedAfterMapAndCostmapClear) {
  core.reference_override = [](double) {return std::array<double, 7>{0, 0, 0, 0, 0, 0, 1};};
  core.tick(10.02, 100.03);
  core.reset_result(effects.last_reset_id, true, core.epoch());
  core.tick(10.04, 100.05);
  ASSERT_EQ(effects.launches, 1);
  EXPECT_EQ(core.state(), State::REINITIALIZING);
  core.reference_override = nullptr;
  double t = 10.06;
  for (; t < 12.2; t += 0.05) {
    core.fix(fix(t), t);
    core.pose(pose(t), t);
    core.map(view(grid(2000, 0), t), t);
    core.tick(t, 100.05 + (t - 10.04));
  }
  ASSERT_GE(effects.clears, 1);
  core.clear_result(effects.last_clear_id, true);
  core.clear_result(effects.last_clear_id, true);
  core.fix(fix(t), t);
  core.pose(pose(t), t);
  core.map(view(grid(2000, 0), t), t);
  core.tick(t, 100.05 + (t - 10.04));
  EXPECT_EQ(core.state(), State::HEALTHY) << core.reason();
  EXPECT_EQ(core.reason(), "validated");
  EXPECT_TRUE(effects.last_motion);
}

TEST_F(Supervisor, MappingCapacityFaultIsTerminal) {
  core.set_state(State::HEALTHY);
  core.map_integrity(false);
  EXPECT_EQ(core.state(), State::FAULTED);
  EXPECT_EQ(core.reason(), "map_capacity_exceeded");
}

TEST_F(Supervisor, StaleMapHoldsEvenWhenPoseIsFresh) {
  core.set_state(State::HEALTHY);
  core.set_last_pose(pose());
  core.set_map_stamp(8.0);
  core.tick(10.02, 100.03);
  EXPECT_EQ(core.state(), State::STOPPING);
  EXPECT_EQ(core.reason(), "map_stale");
}

TEST_F(Supervisor, EstimatorExitFaults) {
  core.set_state(State::REBUILDING);
  effects.exited = true;
  core.tick(10.02, 100.03);
  EXPECT_EQ(core.reason(), "estimator_exited");
}

TEST_F(Supervisor, PausedClockForbidsMotion) {
  core.set_state(State::HEALTHY);
  core.tick(10.0, 101.0);  // simulation clock did not advance for 1 s of wall time
  EXPECT_FALSE(effects.last_motion);
}

TEST_F(Supervisor, VerticalVarianceDoesNotWidenTheHorizontalJumpGate) {
  core.fix(fix(9.6), 9.6);
  core.fix(fix(9.633, 1.0), 9.633);
  EXPECT_EQ(core.fixes(), 1u);
  core.fix(fix(9.666, 0.1, 0, 0.3), 9.666);  // plausible noise + motion: accepted
  EXPECT_EQ(core.fixes(), 2u);
}

TEST_F(Supervisor, PersistentOutlierStaysRejectedWhileSlamCoversTheInterval) {
  for (int k = 0; k < 21; ++k) {
    core.add_recent_pose(9.0 + k * 0.05, {0, 0, 0});
}
  core.fix(fix(9.5), 9.5);
  for (double t : {9.55, 9.7, 9.85, 9.95}) {
    core.fix(fix(t, 1.0), t);
}
  ASSERT_EQ(core.fixes(), 1u);
  EXPECT_EQ(core.accepted_fixes().front().position[0], 0.0);
}

TEST_F(Supervisor, OneNoisyOutlierFixCannotCaptureTheGate) {
  for (int k = 0; k < 21; ++k) {
    core.add_recent_pose(9.0 + k * 0.05, {0, 0, 0});
}
  core.fix(fix(9.5, 0.5), 9.5);  // burst fix pulled back by noise: inside 0.55 m
  for (double t : {9.55, 9.6, 9.65}) {
    core.fix(fix(t, 1.0), t);
}
  ASSERT_EQ(core.fixes(), 1u);
  EXPECT_EQ(core.accepted_fixes().front().position[0], 0.5);
}

TEST_F(Supervisor, RealMotionFollowsSlamDisplacementAcrossAGap) {
  for (int k = 0; k < 21; ++k) {
    core.add_recent_pose(9.0 + k * 0.05, {0.5 * k * 0.05, 0, 0});
  // 0.5 m/s
}
  core.fix(fix(9.2, 0.1), 9.2);
  core.fix(fix(9.95, 0.475), 9.95);  // 0.75 s gap, moved with SLAM
  EXPECT_EQ(core.fixes(), 2u);
  const auto estimate = core.estimated_position(9.95);
  ASSERT_TRUE(estimate);
  EXPECT_NEAR(estimate->first[0], 0.475, 1e-9);
  EXPECT_EQ(estimate->second, 0.0);
  EXPECT_FALSE(core.estimated_position(10.6));  // SLAM silent > 0.5 s: speed bound instead
}

TEST_F(Supervisor, DisagreementFaultIsPerAxis) {
  // sigma 7.5 cm: horizontal limit max(0.5, 0.45) = 0.5 m, vertical max(0.5, 6 * 0.15) = 0.9 m
  for (auto [x, z, faults] : {std::tuple{0.6, 0.0, true}, std::tuple{0.0, 0.6, false}}) {
    FakeEffects fx;
    SupervisorCore c(fx);
    c.tick(9.9, 100.0);
    c.tick(10.0, 100.01);
    c.set_state(State::HEALTHY);
    c.fix(fix(10.0), 10.0);
    c.pose(pose(10.0, x, z), 10.0);
    EXPECT_EQ(c.reason() == "absolute_position_disagreement", faults);
    EXPECT_EQ(c.state() == State::STOPPING, faults);
  }
}

TEST(ChildConfig, OptimizationRequiresRevisionConsumerAndMissingConfigFailsClosed) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "astrobot_localization_test_config";
  fs::remove_all(dir);
  fs::create_directories(dir);
  EXPECT_EQ(check_revision_config(dir), ConfigCheck::INVALID);
  for (const char * enabled : {"sub_mapping", "global_mapping", ""}) {
    nlohmann::json global = {{"global",
      {{"config_sub_mapping", "sub.json"}, {"config_global_mapping", "global.json"}}}};
    std::ofstream(dir / "config.json") << global.dump();
    std::ofstream(dir / "sub.json") << nlohmann::json{{"sub_mapping",
      {{"enable_optimization", std::string(enabled) == "sub_mapping"}}}}.dump();
    std::ofstream(dir / "global.json") << nlohmann::json{{"global_mapping",
      {{"enable_optimization", std::string(enabled) == "global_mapping"}}}}.dump();
    EXPECT_EQ(check_revision_config(dir),
      *enabled ? ConfigCheck::REVISION_CONSUMER_REQUIRED : ConfigCheck::OK) << enabled;
  }
  std::ofstream(dir / "config_odometry_cpu.json") << R"({"odometry_estimation": {"x": 1}})";
  std::ofstream(dir / "config_ros.json") << R"({"abs_constraint": {"rtk_topic": "old"}})";
  const fs::path run = fs::temp_directory_path() / "astrobot_localization_test_run";
  fs::remove_all(run);
  fs::create_directories(run);
  const auto out = prepare_child_config(dir, run, 3, {1, 2, 3, 0, 0, 0, 1},
    "/astrobot_0/localization/absolute_pose");
  const auto odom = nlohmann::json::parse(std::ifstream(out / "config_odometry_cpu.json"));
  EXPECT_EQ(odom["odometry_estimation"]["init_T_world_imu"][2], 3.0);
  const auto ros = nlohmann::json::parse(std::ifstream(out / "config_ros.json"));
  EXPECT_EQ(ros["abs_constraint"]["rtk_topic"], "/astrobot_0/localization/absolute_pose");
  EXPECT_EQ(ros["revision_export"]["localization_epoch"], 3);
  fs::remove_all(dir);
  fs::remove_all(run);
}

}  // namespace astrobot_localization
