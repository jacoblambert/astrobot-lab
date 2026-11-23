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

#include <algorithm>
#include <cmath>

#include "astrobot_terrain/pipeline.hpp"

namespace astrobot_terrain
{

namespace
{

constexpr int64_t kSec = 1'000'000'000;
constexpr int64_t kTick = kSec / 60;

FramePose frame(uint64_t id, int64_t stamp, double speed = 0.0)
{
  FramePose f;
  f.frame_id = id;
  f.stamp_ns = stamp;
  f.map_from_imu(0, 3) = speed * (stamp - kSec) / 1e9;  // driving along x
  f.map_from_imu(2, 3) = 0.45;  // sensor height over flat ground
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

// One sixth of a 16-ring revolution over flat ground, in the sensor frame.
Chunk ground_chunk(uint64_t sequence, int64_t stamp)
{
  Chunk c;
  c.capture = "revision-0123";
  c.sequence = sequence;
  c.stamp_ns = stamp;
  const double start = (sequence % 6) * M_PI / 3;
  for (int ring = 0; ring < 16; ++ring) {
    const double elevation = (-15.0 + ring) * M_PI / 180.0;
    const double range = 0.45 / std::sin(-elevation);
    // horizontal ring never hits
    if (!(range > 0 && range <= 3.5)) {continue;}
    for (int k = 0; k < 120; ++k) {
      const double azimuth = start + k * (M_PI / 3) / 120;
      c.xyz.push_back(static_cast<float>(range * std::cos(elevation) * std::cos(azimuth)));
      c.xyz.push_back(static_cast<float>(range * std::cos(elevation) * std::sin(azimuth)));
      // Deterministic +-3 mm roughness: a perfectly flat return set has no plane residual.
      c.xyz.push_back(static_cast<float>(-0.45 + 0.003 *
        std::sin(1e3 * azimuth + 7.0 * ring + sequence)));
    }
  }
  return c;
}

}  // namespace

TEST(LocalReconstructor, ProjectsMeasuredGroundFromADrivingStream) {
  LocalReconstructor core(Pose::Identity());
  ReceiptScheduler scheduler(core);
  uint64_t revision = 1, frame_id = 0, sequence = 0;
  int projections = 0;
  size_t ground = 0, occupied = 0;
  for (int64_t now = kSec; now < 4 * kSec; now += kTick) {
    if ((now - kSec) % (kSec / 10) < kTick) {
      scheduler.receive(odom(revision++, {frame(frame_id++, now, 0.2)}));
    }
    scheduler.receive(ground_chunk(sequence++, now));
    if (auto projection = scheduler.clock(now)) {
      ++projections;
      ground = std::count(projection->layers.cost.begin(), projection->layers.cost.end(), 0);
      occupied = std::count(projection->layers.cost.begin(), projection->layers.cost.end(), 100);
    }
  }
  EXPECT_FALSE(core.failed()) << core.status().failure;
  EXPECT_GT(projections, 10);
  EXPECT_GT(ground, 100u);
  EXPECT_EQ(occupied, 0u);
}

TEST(LocalReconstructor, ToleratesLateChunksButRejectsReordering) {
  LocalReconstructor core(Pose::Identity());
  core.chunk(ground_chunk(0, kSec), kSec);
  core.chunk(ground_chunk(3, kSec + kTick), kSec + kTick + kDelayNs + 1);  // late: skipped, counted
  EXPECT_EQ(core.status().late_chunks, 1u);
  EXPECT_EQ(core.status().skipped_sequences, 2u);
  EXPECT_THROW(core.chunk(ground_chunk(2, kSec + 2 * kTick), kSec + kDelayNs + kTick + 2),
    std::invalid_argument);
  EXPECT_TRUE(core.failed());
}

TEST(LocalReconstructor, FixedReferenceWitnessChangeFaults) {
  LocalReconstructor core(Pose::Identity());
  ReceiptScheduler scheduler(core);
  uint64_t revision = 1, sequence = 0;
  std::optional<Projection> projection;
  for (int64_t now = kSec; now < 2 * kSec && !projection; now += kTick) {
    if ((now - kSec) % (kSec / 10) < kTick) {
      scheduler.receive(odom(revision++, {frame(revision, now)}));
    }
    scheduler.receive(ground_chunk(sequence++, now));
    projection = scheduler.clock(now);
  }
  ASSERT_TRUE(projection);
  // Re-estimate the oldest frame (the reference anchor) by 1 cm: frozen evidence cannot follow.
  FramePose moved = frame(2, kSec);
  moved.map_from_imu(0, 3) = 0.01;
  EXPECT_THROW(core.revision(odom(revision, {moved}), 2 * kSec), std::invalid_argument);
  EXPECT_TRUE(core.failed());
}

TEST(ReceiptScheduler, HoldsRevisionsAheadOfTheClock) {
  LocalReconstructor core(Pose::Identity());
  ReceiptScheduler scheduler(core);
  scheduler.receive(odom(1, {frame(1, kSec + kTick)}));
  scheduler.clock(kSec);  // the measurement is newer than this clock: held, not faulted
  EXPECT_FALSE(core.failed());
  scheduler.clock(kSec + kTick);
  EXPECT_FALSE(core.failed());
  // A second revision now continues the chain (the first one was applied).
  scheduler.receive(odom(2, {frame(2, kSec + 2 * kTick)}));
  scheduler.clock(kSec + 2 * kTick);
  EXPECT_FALSE(core.failed()) << core.status().failure;
}

TEST(TerrainPipeline, IgnoresRevisionsUntilResetAndRefusesStaleEpochs) {
  PipelineConfig config;
  TerrainPipeline pipeline(config);
  pipeline.revision(odom(1, {frame(1, kSec)}));  // before any reset: ignored
  std::string reason;
  ASSERT_TRUE(pipeline.reset(2, 0, &reason));
  EXPECT_FALSE(pipeline.reset(1, 0, &reason));
  EXPECT_EQ(reason, "stale_epoch");
  EXPECT_FALSE(pipeline.failed());
}

}  // namespace astrobot_terrain
