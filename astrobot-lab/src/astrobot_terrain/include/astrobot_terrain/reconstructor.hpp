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
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <variant>
#include <vector>

#include "astrobot_terrain/layered_terrain.hpp"
#include "astrobot_terrain/pose_timeline.hpp"

namespace astrobot_terrain
{

constexpr int64_t kDelayNs = 200'000'000;   // projection delay and receipt deadline
constexpr int64_t kPeriodNs = 100'000'000;  // 10 Hz output lattice
constexpr size_t kMaxPoints = 100'000;

// One native LiDAR tick in the sensor frame (rejected returns already zeroed).
struct Chunk
{
  std::string capture;
  uint64_t sequence{0};
  int64_t stamp_ns{0};
  std::vector<float> xyz;  // 3 per point
  size_t points() const {return xyz.size() / 3;}
};

struct LocalConfig
{
  int window_s{5};
  int evidence_lifetime_s{60};
  int cells{60};
  double resolution{0.1};
  size_t max_bytes{256u * 1024 * 1024};
  size_t max_chunks{2048};
  size_t max_frames{2048};
  size_t max_bookkeeping_frames{32768};
  double min_range_m{0.5}, max_range_m{100.0};
  // Live stream: a dropped or late chunk is skipped and counted, and stale input pauses
  // projection; the localization supervisor's map freshness gate owns outages.
  bool tolerate_stream_gaps{true};
};

// One committed projection: the six terrain channels plus the frames that place it.
struct Projection
{
  TerrainOutput layers;
  int64_t evidence_ns{0}, publication_ns{0};
  double origin_x{0}, origin_y{0};  // grid corner in the local frame
  Pose local_from_map{Pose::Identity()}, local_from_base{Pose::Identity()};
  Pose map_from_grid() const;
  // Rover (x, y, yaw) in grid-corner coordinates.
  void grid_pose(double & x, double & y, double & yaw) const;
};

struct ReconstructorStatus
{
  bool valid{false}, closed{false};
  std::string failure;
  size_t retained_bytes{0}, received{0}, projected{0}, skipped_sequences{0}, late_chunks{0},
  bookkeeping_accepted{0};
  int64_t evidence_ns{-1};
  int64_t integrated_sequence{-1};
  std::string capture;
};

// Rolling incremental native terrain on latest-available ODOM poses: frozen first placement, one
// persistent rolling mapper with a fixed map reference whose witness must not move, bookkeeping
// revisions only as proven no-ops.
// Every error faults the core; only a fresh instance (supervisor map reset) recovers.
class LocalReconstructor {
public:
  explicit LocalReconstructor(const Pose & base_from_imu, LocalConfig config = {});
  void revision(const Revision & revision, int64_t arrival_ns);
  // Join a running stream (map reset): the next chunk is sequence + 1.
  void resume_after(const std::string & capture, uint64_t sequence);
  void chunk(Chunk chunk, int64_t arrival_ns);
  // Advance publication time; returns a committed projection when a new 10 Hz tick is due.
  std::optional<Projection> advance(int64_t now_ns);
  void fault(const std::string & reason);
  void check() const;
  ReconstructorStatus status() const;
  bool failed() const {return !failure_.empty();}
  const std::string & capture() const {return capture_;}
  int64_t last_clock_ns() const {return last_clock_ns_;}
  int64_t watermark_ns() const {return watermark_ns_;}
  const std::optional<Projection> & last() const {return last_;}

private:
  struct Observation { int64_t stamp_ns; uint64_t sequence; std::vector<float> xyz; };
  void arrival(int64_t stamp);
  void prune_frames();
  void release();
  LocalConfig config_;
  Pose base_from_imu_;
  std::unique_ptr<PoseTimeline> timeline_;
  BookkeepingLedger bookkeeping_;
  std::deque<Observation> observations_;
  size_t bytes_{0};
  std::string capture_;
  int64_t sequence_{-1}, last_input_ns_{-1}, last_clock_ns_{-1}, watermark_ns_{-1},
  last_arrival_ns_{-1};
  std::optional<Pose> local_from_map_;
  std::optional<uint64_t> anchor_;
  Pose reference_witness_{Pose::Identity()};
  std::unique_ptr<LayeredTerrain> mapper_;
  int64_t integrated_sequence_{-1};
  std::string failure_;
  bool closed_{false};
  std::optional<Projection> last_;
  size_t received_{0}, projected_{0}, skipped_sequences_{0}, late_chunks_{0};
};

// Receipt-order adapter: every revision/chunk received between two /clock messages is bounded
// by the NEXT clock (never backdated). A revision whose newest measurement is ahead of that
// clock (the subscriber's clock queue lagged) waits for a covering clock; later receipts keep
// their order behind it. Bounded queue; overflow faults the core.
class ReceiptScheduler {
public:
  explicit ReceiptScheduler(
    LocalReconstructor & core, size_t max_events = 128,
    size_t max_bytes = 32u * 1024 * 1024)
  : core_(core), max_events_(max_events), max_bytes_(max_bytes) {}
  void receive(Revision revision);
  void receive(Chunk chunk);
  std::optional<Projection> clock(int64_t stamp_ns);
  void clear() {pending_.clear(); bytes_ = 0;}

private:
  using Event = std::variant<Revision, Chunk>;
  void push(Event event, size_t size);
  LocalReconstructor & core_;
  std::deque<std::pair<Event, size_t>> pending_;
  size_t bytes_{0}, max_events_, max_bytes_;
};

}  // namespace astrobot_terrain
