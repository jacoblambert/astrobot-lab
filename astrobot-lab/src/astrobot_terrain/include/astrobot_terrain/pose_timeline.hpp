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
#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace astrobot_terrain
{

using Pose = Eigen::Matrix4d;

// Finite rigid SE(3) check (bottom row, orthonormal rotation, det 1).
void validate_transform(const Pose & pose);

struct FramePose
{
  uint64_t frame_id{0};
  int64_t stamp_ns{0};
  Pose map_from_imu{Pose::Identity()};
  Pose imu_from_lidar{Pose::Identity()};
  bool deskew_recomputed{false};
};

// One SLAM revision as published on slam_raw/revisions (astrobot_interfaces/SlamRevision).
struct Revision
{
  enum Kind : uint8_t { ODOM = 1, SUBMAP = 2, GLOBAL = 3 };
  std::string session;
  uint64_t epoch{0}, revision{0}, parent{0};
  uint8_t kind{ODOM};
  bool snapshot{false};
  std::vector<FramePose> frames;
  int64_t newest_stamp_ns() const;
  // Order-independent canonical digest for duplicate (retransmission) detection.
  uint64_t digest(bool with_metadata) const;
};

struct Association
{
  int64_t acquisition_ns{0}, available_ns{0};
  Pose map_from_lidar{Pose::Identity()};
};

// Latest-available pose timeline for one session/epoch. Interpolates the IMU body pose
// (linear translation, geodesic rotation) between bracketing frames at most max_gap apart,
// then applies the session's constant IMU-from-LiDAR extrinsic. Extrapolation is refused.
class PoseTimeline {
public:
  PoseTimeline(std::string session, uint64_t epoch, int64_t max_gap_ns = 200'000'000);
  // Atomic: validates the whole revision before changing state. False = identical retransmission.
  bool apply(const Revision & revision, int64_t arrival_ns);
  // Pose state available at output_ns (latest revision of each frame received by then).
  Association associate(int64_t acquisition_ns, int64_t output_ns) const;
  const Pose & newest(uint64_t frame_id) const
  {
    return frames_.at(frame_id).back().second.map_from_imu;
  }
  bool has(uint64_t frame_id) const {return frames_.count(frame_id) != 0;}
  // Keeps only the newest revision of every frame (bounds memory; the newest is what associate
  // uses).
  void compact();
  // Drops frames older than cutoff_ns except one predecessor (interpolation at the window edge)
  // and the keep frame (the fixed-map reference anchor).
  void prune_before(int64_t cutoff_ns, std::optional<uint64_t> keep);
  // Oldest frame by stamp (fixed-map reference anchor candidate).
  uint64_t oldest_frame() const;
  size_t size() const {return frames_.size();}
  const Pose & extrinsic() const {return extrinsic_;}
  const std::string & session() const {return session_;}
  uint64_t epoch() const {return epoch_;}
  uint64_t revision() const {return revision_;}
  int64_t last_arrival_ns() const {return last_arrival_ns_;}
  // Advances the revision chain without frames (validated bookkeeping no-ops).
  void advance(uint64_t revision, uint64_t parent, int64_t arrival_ns, uint64_t digest);

private:
  struct Row { int64_t arrival_ns; uint64_t revision; };
  std::string session_;
  uint64_t epoch_;
  int64_t max_gap_ns_;
  uint64_t revision_{0};
  int64_t last_arrival_ns_{-1};
  bool have_extrinsic_{false};
  Pose extrinsic_{Pose::Identity()};
  std::map<uint64_t, std::vector<std::pair<Row, FramePose>>> frames_;
  std::map<int64_t, uint64_t> stamps_;  // measurement stamp -> frame id
  std::map<uint64_t, uint64_t> events_;  // revision -> digest (newest 128)
  void trim_events();
};

// Canonical ODOM pose of every frame seen this epoch, to prove that SUBMAP/GLOBAL
// bookkeeping revisions change nothing (their frames must equal ODOM to 1e-10).
// A real global correction cannot be applied to frozen terrain evidence: it faults.
class BookkeepingLedger {
public:
  explicit BookkeepingLedger(size_t capacity)
  : capacity_(capacity) {}
  void validate_odom(const std::vector<FramePose> & frames) const;
  void commit_odom(const std::vector<FramePose> & frames);
  // True if accepted, false for an identical retransmission; throws on any change.
  bool accept(const Revision & revision, int64_t arrival_ns, PoseTimeline & timeline);
  bool has_event(uint64_t revision) const {return events_.count(revision) != 0;}
  size_t accepted() const {return accepted_;}

private:
  size_t capacity_;
  std::map<uint64_t, FramePose> frames_;
  std::map<int64_t, uint64_t> stamps_;
  std::map<uint64_t, uint64_t> events_;
  size_t accepted_{0};
};

}  // namespace astrobot_terrain
