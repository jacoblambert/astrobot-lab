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
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "astrobot_terrain/grid_ops.hpp"
#include "astrobot_terrain/reconstructor.hpp"

namespace astrobot_terrain
{

// Static rover geometry the terrain pipeline needs (config/robot/astrobot.yaml).
struct RobotGeometry
{
  Pose base_from_imu{Pose::Identity()};
  std::array<double, 3> base_from_lidar_translation{{0, 0, 0}};
  // Chassis self-return box in base coordinates (lo xyz, hi xyz); disabled if lo >= hi.
  std::array<double, 6> chassis_aabb{{0, 0, 0, 0, 0, 0}};
  Body body;
};

struct PipelineConfig
{
  RobotGeometry robot;
  LocalConfig local;
  GapPolicy gap;
  double mosaic_half_extent_m{30.0};
};

// A native chunk after wire decoding (sensor-frame float32 XYZ and per-return intensity).
struct NativeChunk
{
  std::string capture;
  uint64_t sequence{0};
  int64_t stamp_ns{0};
  std::vector<float> xyz;
  std::vector<uint8_t> intensity;
};

// One planner window for Nav2's local costmap, cut from the cost mosaic.
struct PlannerWindow
{
  ClassGrid costs;
  Pose map_from_grid{Pose::Identity()};
  int64_t stamp_ns{0};
};

// ROS-free terrain pipeline: epoch-gated revision intake, stream joining,
// self filtering, receipt scheduling, rolling reconstruction, the bounded-shadow gap policy,
// and the two map-frame mosaics (Nav2 costs; observed classes for exploration).
// The ROS node is a thin adapter around it, so the pipeline can be replayed offline.
class TerrainPipeline {
public:
  explicit TerrainPipeline(PipelineConfig config);
  // Supervisor map reset before each SLAM child: fresh core and mosaics for the new epoch.
  // Returns false (stale_epoch) for an older epoch.
  bool reset(uint64_t epoch, int64_t minimum_ns, std::string * reason = nullptr);
  void revision(const Revision & revision);
  void chunk(NativeChunk chunk);
  // Drives all processing. Returns a planner window when a projection is committed.
  std::optional<PlannerWindow> clock(int64_t stamp_ns);
  // Any error: fault the core (only a reset recovers) and drop queued events.
  void fault(const std::string & reason);

  bool valid() const {return core_->status().valid;}
  bool failed() const {return core_->failed();}
  const LocalReconstructor & core() const {return *core_;}
  std::optional<uint64_t> epoch() const {return epoch_;}
  const Pose & last_map_from_grid() const {return last_map_from_grid_;}
  const Mosaic * cost_mosaic() const {return cost_.get();}
  const Mosaic * explore_mosaic() const {return explore_.get();}
  const PipelineConfig & config() const {return config_;}
  // Self returns: inside the chassis box (base frame) or zero intensity.
  std::vector<uint8_t> rejected(const NativeChunk & chunk) const;

private:
  void stitch(const ClassGrid & planner, const ClassGrid & classes, const Projection & projection);
  PipelineConfig config_;
  std::unique_ptr<LocalReconstructor> core_;
  std::unique_ptr<ReceiptScheduler> scheduler_;
  std::optional<uint64_t> epoch_;
  int64_t minimum_ns_{0};
  std::optional<int64_t> armed_ns_;
  std::string stream_capture_;
  std::unique_ptr<Mosaic> cost_, explore_;
  Pose last_map_from_grid_{Pose::Identity()};
};

}  // namespace astrobot_terrain
