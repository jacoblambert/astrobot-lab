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

#include "astrobot_terrain/reconstructor.hpp"

#include <cmath>
#include <stdexcept>

namespace astrobot_terrain
{
namespace
{

constexpr double kRangeTolerance = 1e-5;
constexpr size_t kRevisionFrameBytes = 700;  // serialized size bound per frame (queue budget)

int64_t floor_div(int64_t a, int64_t b) {return a / b - ((a % b != 0) && ((a < 0) != (b < 0)));}

bool all_close(const Pose & a, const Pose & b, double atol)
{
  return ((a - b).array().abs() <= atol).all();
}

}  // namespace

Pose Projection::map_from_grid() const
{
  Pose local_from_grid = Pose::Identity();
  local_from_grid(0, 3) = origin_x;
  local_from_grid(1, 3) = origin_y;
  return local_from_map.inverse() * local_from_grid;
}

void Projection::grid_pose(double & x, double & y, double & yaw) const
{
  x = local_from_base(0, 3) - origin_x;
  y = local_from_base(1, 3) - origin_y;
  yaw = std::atan2(local_from_base(1, 0), local_from_base(0, 0));
}

LocalReconstructor::LocalReconstructor(const Pose & base_from_imu, LocalConfig config)
: config_(config), base_from_imu_(base_from_imu), bookkeeping_(config.max_bookkeeping_frames)
{
  validate_transform(base_from_imu_);
  if (config_.cells <= 0 || !(config_.resolution > 0) || config_.window_s <= 0 ||
    config_.evidence_lifetime_s <= 0)
  {
    throw std::invalid_argument("invalid terrain grid configuration");
  }
  if (!(0 <= kRangeTolerance && kRangeTolerance < config_.min_range_m &&
    config_.min_range_m < config_.max_range_m))
  {
    throw std::invalid_argument("invalid sensor range limits");
  }
}

void LocalReconstructor::check() const
{
  if (!failure_.empty()) {throw std::runtime_error(failure_);}
  if (closed_) {throw std::runtime_error("local reconstruction closed");}
}

void LocalReconstructor::release()
{
  last_.reset();
  observations_.clear();
  bytes_ = 0;
  mapper_.reset();
}

void LocalReconstructor::fault(const std::string & reason)
{
  if (failure_.empty()) {failure_ = reason.empty() ? "unspecified terrain fault" : reason;}
  release();
}

void LocalReconstructor::arrival(int64_t stamp)
{
  if (stamp < 0) {throw std::invalid_argument("invalid receipt bound");}
  if (stamp < last_arrival_ns_) {throw std::invalid_argument("receipt order regressed");}
  last_arrival_ns_ = stamp;
}

void LocalReconstructor::revision(const Revision & rev, int64_t arrival_ns)
{
  check();
  try {
    arrival(arrival_ns);
    if (rev.kind == Revision::SUBMAP || rev.kind == Revision::GLOBAL) {
      if (!timeline_) {throw std::invalid_argument("bookkeeping before ODOM prefix");}
      bookkeeping_.accept(rev, arrival_ns, *timeline_);
      return;
    }
    if (rev.kind != Revision::ODOM || rev.snapshot) {
      throw std::invalid_argument("only contiguous ODOM deltas are supported");
    }
    if (rev.frames.size() > config_.max_frames) {
      throw std::invalid_argument("revision frame budget exceeded");
    }
    if (!timeline_) {
      if (rev.revision != 1) {throw std::invalid_argument("missing revision prefix");}
      timeline_ = std::make_unique<PoseTimeline>(rev.session, rev.epoch);
    }
    bookkeeping_.validate_odom(rev.frames);
    if (bookkeeping_.has_event(rev.revision)) {
      throw std::invalid_argument("ODOM conflicts with bookkeeping revision");
    }
    if (anchor_ && !timeline_->has(*anchor_)) {throw std::invalid_argument("shared anchor lost");}
    if (timeline_->apply(rev, arrival_ns)) {
      bookkeeping_.commit_odom(rev.frames);
      // Frozen evidence cannot follow a re-anchored local frame: compare with the INITIAL
      // witness (numeric jitter only), never ratchet through successive small updates.
      if (anchor_ && !all_close(timeline_->newest(*anchor_), reference_witness_, 1e-6)) {
        throw std::invalid_argument("fixed-map reference witness changed; rebuild/reset required");
      }
      timeline_->compact();
      prune_frames();
      if (timeline_->size() > config_.max_frames) {
        throw std::invalid_argument("pose frame budget exceeded");
      }
    }
  } catch (const std::exception & e) {
    fault(e.what());
    throw;
  }
}

void LocalReconstructor::prune_frames()
{
  if (!timeline_ || watermark_ns_ < 0) {return;}
  timeline_->prune_before(watermark_ns_ - int64_t(config_.window_s) * 1'000'000'000, anchor_);
}

void LocalReconstructor::resume_after(const std::string & capture, uint64_t sequence)
{
  check();
  if (!capture_.empty() || received_) {throw std::logic_error("stream already joined");}
  if (capture.empty()) {throw std::invalid_argument("invalid capture identity");}
  capture_ = capture;
  sequence_ = static_cast<int64_t>(sequence);
}

void LocalReconstructor::chunk(Chunk chunk, int64_t arrival_ns)
{
  check();
  try {
    arrival(arrival_ns);
    if (chunk.capture.empty()) {throw std::invalid_argument("invalid capture identity");}
    if (capture_.empty()) {capture_ = chunk.capture;}
    const auto sequence = static_cast<int64_t>(chunk.sequence);
    const bool tolerant = config_.tolerate_stream_gaps;
    if (chunk.capture != capture_ || sequence <= sequence_ ||
      (!tolerant && sequence != sequence_ + 1) ||
      chunk.stamp_ns <= last_input_ns_)
    {
      throw std::invalid_argument("native identity/sequence/stamp discontinuity");
    }
    const bool late = chunk.stamp_ns <= watermark_ns_ || arrival_ns > chunk.stamp_ns + kDelayNs;
    if (tolerant && late) {
      skipped_sequences_ += static_cast<size_t>(sequence - sequence_ - 1);
      ++late_chunks_;
      sequence_ = sequence;
      return;
    }
    if (chunk.stamp_ns <= watermark_ns_) {
      throw std::invalid_argument("native identity/sequence/stamp discontinuity");
    }
    if (!(chunk.stamp_ns <= arrival_ns && arrival_ns <= chunk.stamp_ns + kDelayNs)) {
      throw std::invalid_argument("native acquisition deadline exceeded");
    }
    const size_t n = chunk.points();
    if (chunk.xyz.size() != 3 * n || n == 0 || n > kMaxPoints) {
      throw std::invalid_argument("invalid native return array");
    }
    for (float value : chunk.xyz) {
      if (!std::isfinite(value)) {throw std::invalid_argument("invalid native return array");}
    }
    const size_t nbytes = chunk.xyz.size() * sizeof(float);
    if (observations_.size() >= config_.max_chunks || bytes_ + nbytes > config_.max_bytes) {
      throw std::invalid_argument("retained native budget exceeded");
    }
    observations_.push_back({chunk.stamp_ns, chunk.sequence, std::move(chunk.xyz)});
    bytes_ += nbytes;
    skipped_sequences_ += static_cast<size_t>(sequence - sequence_ - 1);
    sequence_ = sequence;
    last_input_ns_ = chunk.stamp_ns;
    ++received_;
  } catch (const std::exception & e) {
    fault(e.what());
    throw;
  }
}

std::optional<Projection> LocalReconstructor::advance(int64_t now_ns)
{
  check();
  try {
    if (now_ns < 0) {throw std::invalid_argument("invalid publication bound");}
    if (now_ns < std::max(last_clock_ns_,
        last_arrival_ns_) || (last_clock_ns_ >= 0 && now_ns - last_clock_ns_ > kDelayNs))
    {
      throw std::invalid_argument("publication clock reset/gap or future receipt");
    }
    last_clock_ns_ = now_ns;
    if (capture_.empty()) {return std::nullopt;}
    if (now_ns - last_input_ns_ > kDelayNs) {
      if (config_.tolerate_stream_gaps) {return std::nullopt;}
      throw std::invalid_argument("native input stale");
    }
    const int64_t target = floor_div(now_ns - kDelayNs, kPeriodNs) * kPeriodNs;
    if (target <= watermark_ns_ || target < 0 || observations_.empty() ||
      target < observations_.front().stamp_ns)
    {
      return std::nullopt;
    }
    if (!timeline_) {throw std::invalid_argument("no pose state by deadline");}
    const int64_t cutoff = target - int64_t(config_.window_s) * 1'000'000'000;
    while (!observations_.empty() && observations_.front().stamp_ns < cutoff) {
      bytes_ -= observations_.front().xyz.size() * sizeof(float);
      observations_.pop_front();
    }
    // State is latest-at-publication, not final-revised offline state.
    const Association current = timeline_->associate(target, now_ns);
    const Pose map_from_imu = current.map_from_lidar * timeline_->extrinsic().inverse();
    const Pose map_from_base = map_from_imu * base_from_imu_.inverse();
    if (!local_from_map_) {
      Pose local = Pose::Identity();
      local.topRightCorner<3, 1>() = -map_from_base.topRightCorner<3, 1>();
      local_from_map_ = local;
    }
    const Pose local_from_base = *local_from_map_ * map_from_base;
    const double r = config_.resolution;
    const double origin_x = std::floor(local_from_base(0, 3) / r) * r - config_.cells * r / 2;
    const double origin_y = std::floor(local_from_base(1, 3) / r) * r - config_.cells * r / 2;
    if (!mapper_) {
      TerrainConfig cfg;
      cfg.width = cfg.height = config_.cells;
      cfg.resolution = r;
      cfg.origin_x = origin_x;
      cfg.origin_y = origin_y;
      cfg.lifetime = config_.evidence_lifetime_s;
      cfg.interpolate = true;
      mapper_ = std::make_unique<LayeredTerrain>(cfg);
    }
    mapper_->shift_origin(origin_x, origin_y);
    std::vector<double> world;
    for (const auto & observation : observations_) {
      if (observation.stamp_ns > target) {break;}
      if (static_cast<int64_t>(observation.sequence) <= integrated_sequence_) {continue;}
      // Each observation is placed once, at its first use (frozen observation).
      const Pose pose = *local_from_map_ * timeline_->associate(observation.stamp_ns,
          now_ns).map_from_lidar;
      world.clear();
      const size_t n = observation.xyz.size() / 3;
      // Range validity in the native float32 precision (zeroed self returns fall out here).
      const auto near = static_cast<float>(config_.min_range_m + kRangeTolerance);
      const auto far = static_cast<float>(config_.max_range_m - kRangeTolerance);
      for (size_t i = 0; i < n; ++i) {
        const float fx = observation.xyz[3 * i], fy = observation.xyz[3 * i + 1],
          fz = observation.xyz[3 * i + 2];
        const float range = std::sqrt(fx * fx + fy * fy + fz * fz);
        if (range <= near || range >= far) {continue;}
        const double x = fx, y = fy, z = fz;
        for (int row = 0; row < 3; ++row) {
          world.push_back(x * pose(row, 0) + y * pose(row, 1) + z * pose(row, 2) + pose(row, 3));
        }
      }
      if (!mapper_->integrate(world.data(), world.size() / 3, observation.stamp_ns / 1e9,
          pose.topRightCorner<3, 1>(),
                              std::atan2(pose(1, 0), pose(0, 0))))
      {
        throw std::invalid_argument("invalid/nonmonotonic scan");
      }
      integrated_sequence_ = static_cast<int64_t>(observation.sequence);
    }
    Projection projection;
    projection.layers = mapper_->project(target / 1e9);
    const size_t cells = static_cast<size_t>(config_.cells) * config_.cells;
    if (projection.layers.cost.size() != cells) {
      throw std::invalid_argument("unexpected layered output shape");
    }
    for (int8_t value : projection.layers.cost) {
      if (value != -1 && value != 0 && value != 20 && value != 100) {
        throw std::invalid_argument("invalid terrain costs");
      }
    }
    projection.evidence_ns = target;
    projection.publication_ns = now_ns;
    projection.origin_x = origin_x;
    projection.origin_y = origin_y;
    projection.local_from_map = *local_from_map_;
    projection.local_from_base = local_from_base;
    // Commit only a completed projection. The anchor is the oldest session frame, not the
    // continually revised recent rover pose, so map-to-local stays constant.
    if (!anchor_) {
      anchor_ = timeline_->oldest_frame();
      reference_witness_ = timeline_->newest(*anchor_);
    }
    watermark_ns_ = target;
    last_ = projection;
    ++projected_;
    prune_frames();
    return projection;
  } catch (const std::exception & e) {
    fault(e.what());
    throw;
  }
}

ReconstructorStatus LocalReconstructor::status() const
{
  ReconstructorStatus s;
  s.valid = last_.has_value() && failure_.empty() && !closed_;
  s.closed = closed_;
  s.failure = failure_;
  s.retained_bytes = bytes_;
  s.received = received_;
  s.projected = projected_;
  s.skipped_sequences = skipped_sequences_;
  s.late_chunks = late_chunks_;
  s.bookkeeping_accepted = bookkeeping_.accepted();
  s.evidence_ns = watermark_ns_;
  s.integrated_sequence = integrated_sequence_;
  s.capture = capture_;
  return s;
}

void ReceiptScheduler::push(Event event, size_t size)
{
  core_.check();
  if (pending_.size() >= max_events_ || bytes_ + size > max_bytes_) {
    core_.fault("receipt queue budget exceeded");
    clear();
    core_.check();
  }
  pending_.emplace_back(std::move(event), size);
  bytes_ += size;
}

void ReceiptScheduler::receive(Revision revision)
{
  const size_t size = 256 + revision.frames.size() * kRevisionFrameBytes;
  push(std::move(revision), size);
}

void ReceiptScheduler::receive(Chunk chunk)
{
  const size_t size = chunk.points() * 40;  // native wire bytes
  push(std::move(chunk), size);
}

std::optional<Projection> ReceiptScheduler::clock(int64_t stamp_ns)
{
  // A callback is bounded by the NEXT received clock (deterministic in live runs and replay).
  if (stamp_ns < core_.last_clock_ns() ||
    (core_.last_clock_ns() >= 0 && stamp_ns - core_.last_clock_ns() > kDelayNs))
  {
    core_.fault("receipt clock reset/gap");
  }
  try {
    core_.check();
    while (!pending_.empty()) {
      auto & [event, size] = pending_.front();
      if (auto * revision = std::get_if<Revision>(&event);
        revision && revision->newest_stamp_ns() > stamp_ns) {break;}
      Event current = std::move(event);
      bytes_ -= size;
      pending_.pop_front();
      if (auto * revision = std::get_if<Revision>(&current)) {
        core_.revision(*revision, stamp_ns);
      } else {
        core_.chunk(std::move(std::get<Chunk>(current)), stamp_ns);
      }
    }
    return core_.advance(stamp_ns);
  } catch (...) {
    clear();
    throw;
  }
}

}  // namespace astrobot_terrain
