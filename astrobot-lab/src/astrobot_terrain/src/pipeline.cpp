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

#include "astrobot_terrain/pipeline.hpp"

#include <algorithm>
#include <stdexcept>

namespace astrobot_terrain
{

TerrainPipeline::TerrainPipeline(PipelineConfig config)
: config_(std::move(config))
{
  core_ = std::make_unique<LocalReconstructor>(config_.robot.base_from_imu, config_.local);
  scheduler_ = std::make_unique<ReceiptScheduler>(*core_);
}

bool TerrainPipeline::reset(uint64_t epoch, int64_t minimum_ns, std::string * reason)
{
  if (epoch_ && epoch < *epoch_) {
    if (reason) {*reason = "stale_epoch";}
    return false;
  }
  auto core = std::make_unique<LocalReconstructor>(config_.robot.base_from_imu, config_.local);
  scheduler_ = std::make_unique<ReceiptScheduler>(*core);
  core_ = std::move(core);
  epoch_ = epoch;
  minimum_ns_ = minimum_ns;
  armed_ns_.reset();
  cost_.reset();
  explore_.reset();
  return true;
}

void TerrainPipeline::fault(const std::string & reason)
{
  core_->fault(reason);
  scheduler_->clear();
}

void TerrainPipeline::revision(const Revision & revision)
{
  if (core_->failed()) {return;}
  // Only the epoch the supervisor reset the map for; a previous child's late revisions
  // (or any before the first reset) never reach the core.
  if (!epoch_ || revision.epoch != *epoch_) {return;}
  if (!armed_ns_) {
    if (revision.kind != Revision::ODOM || revision.frames.empty()) {return;}
    int64_t first = revision.frames.front().stamp_ns;
    for (const auto & frame : revision.frames) {
      first = std::min(first, frame.stamp_ns);
    }
    armed_ns_ = std::max(minimum_ns_, first);
  }
  try {
    scheduler_->receive(revision);
  } catch (const std::exception & e) {
    fault(e.what());
  }
}

std::vector<uint8_t> TerrainPipeline::rejected(const NativeChunk & chunk) const
{
  const size_t n = chunk.xyz.size() / 3;
  std::vector<uint8_t> out(n, 0);
  const auto & box = config_.robot.chassis_aabb;
  const auto & t = config_.robot.base_from_lidar_translation;
  const bool enabled = box[0] < box[3] && box[1] < box[4] && box[2] < box[5];
  for (size_t i = 0; i < n; ++i) {
    bool inside = enabled;
    for (int axis = 0; axis < 3 && inside; ++axis) {
      const double base = static_cast<double>(chunk.xyz[3 * i + axis]) + t[axis];
      inside = base >= box[axis] && base <= box[3 + axis];
    }
    out[i] = inside || chunk.intensity[i] == 0;
  }
  return out;
}

void TerrainPipeline::chunk(NativeChunk native)
{
  if (core_->failed()) {return;}
  try {
    if (stream_capture_.empty()) {stream_capture_ = native.capture;}
    if (native.capture != stream_capture_) {throw std::invalid_argument("foreign native capture");}
    // no pose state for this epoch yet
    if (!armed_ns_ || native.stamp_ns < *armed_ns_) {return;}
    if (core_->capture().empty() && native.sequence > 0) {
      core_->resume_after(native.capture, native.sequence - 1);
    }
    // Zero rejected XYZ in the classifier input (dropped by the range filter), preserving tick
    // identity.
    const auto reject = rejected(native);
    for (size_t i = 0; i < reject.size(); ++i) {
      if (reject[i]) {native.xyz[3 * i] = native.xyz[3 * i + 1] = native.xyz[3 * i + 2] = 0.0f;}
    }
    Chunk chunk{std::move(native.capture), native.sequence, native.stamp_ns, std::move(native.xyz)};
    scheduler_->receive(std::move(chunk));
  } catch (const std::exception & e) {
    fault(e.what());
  }
}

std::optional<PlannerWindow> TerrainPipeline::clock(int64_t stamp_ns)
{
  if (core_->failed()) {return std::nullopt;}
  try {
    auto projection = scheduler_->clock(stamp_ns);
    if (!projection) {return std::nullopt;}
    const int cells = config_.local.cells;
    const double r = config_.local.resolution;
    ClassGrid classes(cells, cells, -1);
    std::vector<uint8_t> reasons(classes.size());
    for (size_t i = 0; i < classes.size(); ++i) {
      classes.data[i] = projection->layers.cost[i];
      reasons[i] = projection->layers.reason[i];
    }
    last_map_from_grid_ = projection->map_from_grid();
    ClassGrid planner = navigation_costs(classes, reasons, config_.gap, r);
    double x, y, yaw;
    projection->grid_pose(x, y, yaw);
    clear_footprint(planner, config_.robot.body, x, y, yaw, r);
    stitch(planner, classes, *projection);
    // Nav2's local costmap reads the window from the mosaic, so an obstacle out of view (behind
    // the front LiDAR) or past the evidence lifetime stays lethal.
    PlannerWindow window;
    window.costs = cost_->window(last_map_from_grid_(0, 3), last_map_from_grid_(1, 3), cells,
        cells);
    window.map_from_grid = last_map_from_grid_;
    window.stamp_ns = std::max<int64_t>(0, core_->watermark_ns());
    return window;
  } catch (const std::exception & e) {
    fault(e.what());
    return std::nullopt;
  }
}

void TerrainPipeline::stitch(
  const ClassGrid & planner, const ClassGrid & classes,
  const Projection & projection)
{
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  if (((last_map_from_grid_.topLeftCorner<3,
    3>() - identity).array().abs() > 1e-9 + 1e-5 * identity.array().abs()).any())
  {
    throw std::invalid_argument("rotated planner window; the mosaic is axis-aligned");
  }
  const double ox = last_map_from_grid_(0, 3), oy = last_map_from_grid_(1, 3);
  const double r = config_.local.resolution;
  if (!cost_) {
    const double cx = ox + 0.5 * planner.width * r, cy = oy + 0.5 * planner.height * r;
    const double h = config_.mosaic_half_extent_m;
    cost_ = std::make_unique<Mosaic>(cx - h, cy - h, cx + h, cy + h, ox, oy, r);
    explore_ = std::make_unique<Mosaic>(cx - h, cy - h, cx + h, cy + h, ox, oy, r);
  }
  std::vector<uint8_t> observed(classes.size());
  for (size_t i = 0; i < classes.size(); ++i) {
    observed[i] = classes.data[i] != -1;
  }
  cost_->update(planner, ox, oy, &observed);
  // Exploration map: 0 free, 100 occupied, -1 unknown (observations only, no gap policy), plus
  // the unobserved ground under the unpadded body as free (the LiDAR never sees it).
  ClassGrid body(classes.width, classes.height, -1);
  double x, y, yaw;
  projection.grid_pose(x, y, yaw);
  clear_footprint(body, config_.robot.body, x, y, yaw, r);
  ClassGrid explore = classes;
  for (size_t i = 0; i < explore.size(); ++i) {
    if (explore.data[i] == 20) {explore.data[i] = 0;}
    if (body.data[i] == 0 && classes.data[i] == -1) {explore.data[i] = 0;}
  }
  explore_->update(explore, ox, oy, &observed);
}

}  // namespace astrobot_terrain
