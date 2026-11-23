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

#include "astrobot_terrain/pose_timeline.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace astrobot_terrain
{
namespace
{

constexpr size_t kRetainedEvents = 128;

uint64_t mix(uint64_t hash, const void * data, size_t size)
{
  // FNV-1a over the exact bytes; only used to tell retransmissions from conflicts.
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ull;
  }
  return hash;
}

template<typename T>
uint64_t mix_value(uint64_t hash, const T & value) {return mix(hash, &value, sizeof(T));}

bool all_close(const Pose & a, const Pose & b, double atol)
{
  return ((a - b).array().abs() <= atol).all();
}

template<typename Map>
void keep_newest(Map & events)
{
  while (events.size() > kRetainedEvents) {events.erase(events.begin());}
}

}  // namespace

void validate_transform(const Pose & pose)
{
  const Eigen::Matrix3d rotation = pose.topLeftCorner<3, 3>();
  if (!pose.allFinite() || (pose.row(3) - Eigen::RowVector4d(0, 0, 0,
      1)).cwiseAbs().maxCoeff() > 1e-10 ||
    ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).array().abs() > 1e-8).any() ||
    std::abs(rotation.determinant() - 1.0) > 1e-8)
  {
    throw std::invalid_argument("pose is not finite rigid SE(3)");
  }
}

int64_t Revision::newest_stamp_ns() const
{
  int64_t newest = -1;
  for (const auto & frame : frames) {
    newest = std::max(newest, frame.stamp_ns);
  }
  return newest;
}

uint64_t Revision::digest(bool with_metadata) const
{
  std::vector<const FramePose *> sorted;
  for (const auto & frame : frames) {
    sorted.push_back(&frame);
  }
  std::sort(sorted.begin(), sorted.end(), [](auto a, auto b) {return a->frame_id < b->frame_id;});
  uint64_t hash = 1469598103934665603ull;
  hash = mix_value(hash, parent);
  hash = mix_value(hash, snapshot);
  if (with_metadata) {
    hash = mix_value(hash, kind);
    hash = mix_value(hash, revision);
    hash = mix_value(hash, epoch);
    hash = mix(hash, session.data(), session.size());
  }
  for (const auto * frame : sorted) {
    hash = mix_value(hash, frame->frame_id);
    hash = mix_value(hash, frame->stamp_ns);
    hash = mix(hash, frame->map_from_imu.data(), sizeof(double) * 16);
    hash = mix(hash, frame->imu_from_lidar.data(), sizeof(double) * 16);
    if (with_metadata) {hash = mix_value(hash, frame->deskew_recomputed);}
  }
  return hash;
}

PoseTimeline::PoseTimeline(std::string session, uint64_t epoch, int64_t max_gap_ns)
: session_(std::move(session)), epoch_(epoch), max_gap_ns_(max_gap_ns)
{
  if (session_.empty()) {throw std::invalid_argument("invalid session");}
  if (max_gap_ns_ <= 0) {throw std::invalid_argument("maximum gap must be positive");}
}

void PoseTimeline::trim_events() {keep_newest(events_);}

bool PoseTimeline::apply(const Revision & rev, int64_t arrival_ns)
{
  if (rev.session != session_ || rev.epoch != epoch_) {
    throw std::invalid_argument("session/epoch reset barrier required");
  }
  if (arrival_ns < 0) {throw std::invalid_argument("invalid arrival");}
  if (rev.revision == 0 || rev.parent >= rev.revision) {
    throw std::invalid_argument("invalid revision sequence");
  }
  std::vector<uint64_t> ids;
  for (const auto & frame : rev.frames) {
    ids.push_back(frame.frame_id);
  }
  std::sort(ids.begin(), ids.end());
  if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
    throw std::invalid_argument("duplicate frame in revision");
  }
  const uint64_t digest = rev.digest(false);
  if (auto found = events_.find(rev.revision); found != events_.end()) {
    if (found->second != digest) {throw std::invalid_argument("conflicting duplicate revision");}
    return false;  // retransmission must not change original availability
  }
  if (arrival_ns < last_arrival_ns_ || rev.revision <= revision_ ||
    (!rev.snapshot && (rev.parent != revision_ || rev.revision != revision_ + 1)))
  {
    throw std::invalid_argument("revision gap/order: authoritative snapshot required");
  }
  if (rev.snapshot) {
    for (const auto & [id, history] : frames_) {
      if (!std::binary_search(ids.begin(), ids.end(), id)) {
        throw std::invalid_argument("snapshot omitted known frames");
      }
    }
  }
  auto stamps = stamps_;
  bool have_extrinsic = have_extrinsic_;
  Pose extrinsic = extrinsic_;
  for (const auto & frame : rev.frames) {
    validate_transform(frame.map_from_imu);
    validate_transform(frame.imu_from_lidar);
    if (frame.stamp_ns > arrival_ns) {
      throw std::invalid_argument("measurement after arrival: clock domains disagree");
    }
    if (!have_extrinsic) {
      extrinsic = frame.imu_from_lidar;
      have_extrinsic = true;
    }
    if (!all_close(extrinsic, frame.imu_from_lidar, 1e-10)) {
      throw std::invalid_argument("sensor extrinsic changed within session");
    }
    if (auto s = stamps.find(frame.stamp_ns); s != stamps.end() && s->second != frame.frame_id) {
      throw std::invalid_argument("ambiguous duplicate measurement timestamp");
    }
    if (auto known = frames_.find(frame.frame_id);
      known != frames_.end() && known->second.front().second.stamp_ns != frame.stamp_ns)
    {
      throw std::invalid_argument("immutable frame timestamp changed");
    }
    stamps[frame.stamp_ns] = frame.frame_id;
  }
  for (const auto & frame : rev.frames) {
    frames_[frame.frame_id].push_back({Row{arrival_ns, rev.revision}, frame});
  }
  stamps_ = std::move(stamps);
  extrinsic_ = extrinsic;
  have_extrinsic_ = have_extrinsic;
  events_[rev.revision] = digest;
  revision_ = rev.revision;
  last_arrival_ns_ = arrival_ns;
  return true;
}

void PoseTimeline::advance(uint64_t revision, uint64_t parent, int64_t arrival_ns, uint64_t digest)
{
  if (revision == 0 || parent >= revision) {
    throw std::invalid_argument("invalid revision sequence");
  }
  if (auto found = events_.find(revision); found != events_.end()) {
    if (found->second != digest) {throw std::invalid_argument("conflicting duplicate revision");}
    return;
  }
  if (arrival_ns < last_arrival_ns_ || parent != revision_ || revision != revision_ + 1) {
    throw std::invalid_argument("revision gap/order: authoritative snapshot required");
  }
  events_[revision] = digest;
  revision_ = revision;
  last_arrival_ns_ = arrival_ns;
  trim_events();
}

Association PoseTimeline::associate(int64_t acquisition_ns, int64_t output_ns) const
{
  if (acquisition_ns < 0 || acquisition_ns > output_ns) {
    throw std::invalid_argument("acquisition after publication");
  }
  // Newest row of every frame available by output_ns, ordered by measurement stamp.
  const std::pair<Row, FramePose> * before = nullptr;
  const std::pair<Row, FramePose> * after = nullptr;
  for (const auto & [id, history] : frames_) {
    const std::pair<Row, FramePose> * known = nullptr;
    for (const auto & row : history) {
      if (row.first.arrival_ns <= output_ns) {known = &row;}
    }
    if (!known) {continue;}
    const int64_t stamp = known->second.stamp_ns;
    if (stamp <= acquisition_ns && (!before || stamp > before->second.stamp_ns)) {before = known;}
    if (stamp >= acquisition_ns && (!after || stamp < after->second.stamp_ns)) {after = known;}
  }
  if (!before || !after) {
    throw std::invalid_argument("missing causal bracket: extrapolation forbidden");
  }
  Pose pose = Pose::Identity();
  const Pose & a = before->second.map_from_imu;
  const Pose & b = after->second.map_from_imu;
  if (before == after) {
    pose = a;
  } else {
    const int64_t gap = after->second.stamp_ns - before->second.stamp_ns;
    if (gap > max_gap_ns_) {
      throw std::invalid_argument("pose interpolation gap exceeds 0.2-second contract");
    }
    const double fraction = static_cast<double>(acquisition_ns - before->second.stamp_ns) /
      static_cast<double>(gap);
    pose.topRightCorner<3, 1>() = (1 - fraction) * a.topRightCorner<3,
        1>() + fraction * b.topRightCorner<3, 1>();
    // Geodesic interpolation r0 * exp(f * log(r0^-1 r1)), as scipy Slerp.
    const Eigen::Matrix3d r0 = a.topLeftCorner<3, 3>();
    const Eigen::AngleAxisd delta(Eigen::Quaterniond(r0.transpose() * b.topLeftCorner<3,
      3>()).normalized());
    pose.topLeftCorner<3, 3>() = r0 * Eigen::AngleAxisd(fraction * delta.angle(),
        delta.axis()).toRotationMatrix();
  }
  // Interpolate the IMU body first; a LiDAR lever arm does not translate linearly in a turn.
  return Association{acquisition_ns, std::max(before->first.arrival_ns, after->first.arrival_ns),
    pose * before->second.imu_from_lidar};
}

void PoseTimeline::compact()
{
  for (auto & [id, history] : frames_) {
    history.erase(history.begin(), history.end() - 1);
  }
  trim_events();
}

void PoseTimeline::prune_before(int64_t cutoff_ns, std::optional<uint64_t> keep)
{
  std::vector<std::pair<int64_t, uint64_t>> old;
  for (const auto & [id, history] : frames_) {
    if (history.back().second.stamp_ns < cutoff_ns) {
      old.push_back({history.back().second.stamp_ns, id});
    }
  }
  std::sort(old.begin(), old.end());
  if (old.empty()) {return;}
  old.pop_back();  // one predecessor for interpolation at the window edge
  for (const auto & [stamp, id] : old) {
    if (keep && *keep == id) {continue;}
    frames_.erase(id);
    stamps_.erase(stamp);
  }
}

uint64_t PoseTimeline::oldest_frame() const
{
  if (frames_.empty()) {throw std::logic_error("empty timeline");}
  auto oldest = frames_.begin();
  for (auto it = frames_.begin(); it != frames_.end(); ++it) {
    if (it->second.back().second.stamp_ns < oldest->second.back().second.stamp_ns) {oldest = it;}
  }
  return oldest->first;
}

void BookkeepingLedger::validate_odom(const std::vector<FramePose> & frames) const
{
  size_t added = 0;
  for (const auto & frame : frames) {
    added += frames_.count(frame.frame_id) ? 0 : 1;
  }
  if (frames_.size() + added > capacity_) {
    throw std::invalid_argument("canonical pose ledger budget exceeded");
  }
  for (const auto & frame : frames) {
    if (auto old =
      frames_.find(frame.frame_id); old != frames_.end() && old->second.stamp_ns != frame.stamp_ns)
    {
      throw std::invalid_argument("immutable historical frame timestamp changed");
    }
    if (auto s = stamps_.find(frame.stamp_ns); s != stamps_.end() && s->second != frame.frame_id) {
      throw std::invalid_argument("ambiguous historical measurement timestamp");
    }
  }
}

void BookkeepingLedger::commit_odom(const std::vector<FramePose> & frames)
{
  for (const auto & frame : frames) {
    frames_[frame.frame_id] = frame;
    stamps_[frame.stamp_ns] = frame.frame_id;
  }
}

bool BookkeepingLedger::accept(const Revision & rev, int64_t arrival_ns, PoseTimeline & timeline)
{
  if (rev.snapshot || (rev.kind != Revision::SUBMAP && rev.kind != Revision::GLOBAL)) {
    throw std::invalid_argument("unsupported bookkeeping kind/snapshot");
  }
  if (rev.session != timeline.session() || rev.epoch != timeline.epoch()) {
    throw std::invalid_argument("session/epoch reset barrier required");
  }
  if (rev.frames.empty() || rev.frames.size() > capacity_) {
    throw std::invalid_argument("bookkeeping batch budget/empty payload");
  }
  const uint64_t digest = rev.digest(true);
  if (auto found = events_.find(rev.revision); found != events_.end()) {
    if (found->second != digest) {
      throw std::invalid_argument("conflicting duplicate bookkeeping revision");
    }
    return false;
  }
  if (rev.parent != timeline.revision() || rev.revision != rev.parent + 1 ||
    arrival_ns < timeline.last_arrival_ns())
  {
    throw std::invalid_argument("bookkeeping revision gap/order");
  }
  std::vector<uint64_t> ids;
  for (const auto & frame : rev.frames) {
    ids.push_back(frame.frame_id);
    const auto old = frames_.find(frame.frame_id);
    if (old == frames_.end()) {
      throw std::invalid_argument("bookkeeping frame lacks canonical ODOM reference");
    }
    if (frame.stamp_ns != old->second.stamp_ns || frame.stamp_ns > arrival_ns) {
      throw std::invalid_argument("bookkeeping measurement stamp changed/future");
    }
    if (frame.deskew_recomputed) {throw std::invalid_argument("bookkeeping invalidates deskew");}
  }
  std::sort(ids.begin(), ids.end());
  if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
    throw std::invalid_argument("duplicate frame in bookkeeping revision");
  }
  for (const auto & frame : rev.frames) {
    const auto & old = frames_.at(frame.frame_id);
    if (!frame.imu_from_lidar.allFinite() || !frame.map_from_imu.allFinite()) {
      throw std::invalid_argument("invalid bookkeeping transform shape/nonfinite");
    }
    if (!all_close(frame.imu_from_lidar, old.imu_from_lidar, 1e-10)) {
      throw std::invalid_argument(
          "bookkeeping changes canonical imu_from_lidar; rebuild/reset required");
    }
    if (!all_close(frame.map_from_imu, old.map_from_imu, 1e-10)) {
      throw std::invalid_argument(
          "bookkeeping changes canonical map_from_imu; rebuild/reset required");
    }
  }
  // Advance the revision chain only: no frames enter the interpolation window.
  Revision empty;
  empty.parent = rev.parent;
  timeline.advance(rev.revision, rev.parent, arrival_ns, empty.digest(false));
  events_[rev.revision] = digest;
  keep_newest(events_);
  ++accepted_;
  return true;
}

}  // namespace astrobot_terrain
