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
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace astrobot_localization
{

// LocalizationStatus states (astrobot_interfaces/msg/LocalizationStatus).
enum class State : uint8_t { STARTING = 0, HEALTHY = 1, DEGRADED = 2, STOPPING = 3,
  REINITIALIZING = 4, REBUILDING = 5,
  FAULTED = 6 };

using StampKey = std::pair<int32_t, uint32_t>;  // (sec, nanosec): exact pose/cloud pairing

struct PoseSample
{
  double stamp{0};
  StampKey key{0, 0};
  std::string frame_id{"map"};
  std::array<double, 3> position{{0, 0, 0}};
  std::array<double, 4> quaternion{{0, 0, 0, 1}};  // xyzw
};

struct Fix
{
  double stamp{0};
  std::string frame_id{"map"};
  std::array<double, 3> position{{0, 0, 0}};
  std::array<double, 4> quaternion{{0, 0, 0, 1}};
  std::array<double, 36> covariance{};
};

// Kinematic plausibility of consecutive SLAM poses (age, speed, turn rate).
class PoseValidator {
public:
  explicit PoseValidator(
    double position_slack = 0.25, double max_age = 0.5, double max_speed = 2.0,
    double max_angular_speed = 4.0)
  : position_slack_(position_slack), max_age_(max_age), max_speed_(max_speed),
    max_angular_speed_(max_angular_speed) {}
  void reset() {previous_.reset();}
  // Empty string if valid, else the reason.
  std::string validate(const PoseSample & sample, double now);

private:
  double position_slack_, max_age_, max_speed_, max_angular_speed_;
  std::optional<PoseSample> previous_;
};

// Roll/pitch from a stationary gravity observation (lunar g) plus a yaw; quaternion xyzw.
// Throws if the observation is not a plausible stationary lunar gravity vector.
std::array<double, 4> orientation_from_gravity(
  const std::array<double, 3> & acceleration, double yaw,
  double gravity = 1.62);

// Everything the supervisor does to the outside world (ROS node, or fakes in tests).
struct SupervisorEffects
{
  virtual ~SupervisorEffects() = default;
  virtual void publish_status(
    State state, uint64_t epoch, const std::string & reason, bool motion_permitted,
    const std::optional<StampKey> & last_pose) = 0;
  virtual void publish_fix(const Fix & fix) = 0;
  virtual void publish_pose(const PoseSample & pose) = 0;
  virtual bool reset_ready() = 0;
  // Asynchronous map reset; the answer comes back through SupervisorCore::reset_result(id, ...).
  virtual void request_reset(uint64_t id, uint64_t epoch) = 0;
  virtual bool clear_ready() = 0;
  // Asynchronous clear of both Nav2 costmaps; answers via SupervisorCore::clear_result(id, ok) (two
  // each).
  virtual void request_clear(uint64_t id) = 0;
  virtual void cancel_navigation() = 0;
  enum class Launch { STARTED, REVISION_CONSUMER_REQUIRED, INVALID_CONFIG };
  // Start one estimator child for this epoch at the reference pose [x y z qx qy qz qw]. Throws on
  // failure.
  virtual Launch launch_child(uint64_t epoch, const std::array<double, 7> & reference) = 0;
  virtual void stop_child() = 0;
  virtual bool child_exited() = 0;
  virtual void log_error(const std::string & message) = 0;
};

// Fail-closed owner of exactly one SLAM child. Gates absolute fixes (per-axis innovation
// against the SLAM estimate), relays SLAM outputs only from a validated epoch, and restarts the
// estimator from a stationary RTK+gravity reference after a map reset, within a bounded budget.
// Motion is permitted only while HEALTHY/DEGRADED and the simulation clock runs.
class SupervisorCore {
public:
  explicit SupervisorCore(SupervisorEffects & effects, int restart_budget = 2);

  // Inputs (now = simulation time in seconds of the callback).
  void fix(const Fix & fix, double now);
  void imu(double stamp, const std::array<double, 3> & acceleration);
  void odom(double stamp, const std::array<double, 6> & twist);
  void pose(const PoseSample & pose, double now);
  // Deferred SLAM outputs (LiDAR pose, aligned cloud): published once the pose of the same stamp is
  // accepted.
  void lidar(double stamp, const StampKey & key, std::function<void()> publish, double now);
  void cloud(double stamp, const StampKey & key, std::function<void()> publish, double now);
  struct MapView
  {
    double stamp{0};
    std::string frame_id;
    double resolution{0}, origin_x{0}, origin_y{0};
    uint32_t width{0}, height{0};
    const int8_t * data{nullptr};
    size_t size{0};
  };
  void map(const MapView & map, double now);
  void map_integrity(bool ok);
  void navigation_active(bool active) {nav_active_ = active;}
  void reset_result(uint64_t id, bool success, uint64_t epoch);
  void clear_result(uint64_t id, bool ok);
  // 50 Hz supervision step: now = simulation seconds, wall = monotonic seconds.
  void tick(double now, double wall);
  void fault(const std::string & reason, bool terminal = false);

  // Introspection (tests, status).
  State state() const {return state_;}
  const std::string & reason() const {return reason_;}
  uint64_t epoch() const {return epoch_;}
  bool map_ready() const {return map_ready_;}
  std::optional<double> map_stamp() const {return map_stamp_;}
  size_t fixes() const {return fixes_.size();}
  const std::deque<Fix> & accepted_fixes() const {return fixes_;}
  // GLIM position at t (interpolated, held at the nearest end) and the seconds it is held.
  std::optional<std::pair<std::array<double, 3>, double>> estimated_position(double t) const;
  std::optional<std::array<double, 7>> stationary_reference(double now);

  // Test hooks mirroring the node's internal state.
  void set_state(State state) {state_ = state;}
  void set_last_pose(const PoseSample & pose) {last_pose_ = pose;}
  void set_map_stamp(double stamp) {map_stamp_ = stamp;}
  void add_recent_pose(double stamp, const std::array<double, 3> & position)
  {
    recent_poses_.push_back({stamp, position});
  }
  std::function<std::optional<std::array<double, 7>>(double)> reference_override;

private:
  struct Deferred { double stamp; StampKey key; std::function<void()> publish; };
  void publish_health();
  void drain();
  template<typename T>
  static void push_bounded(std::deque<T> & queue, T value, size_t limit)
  {
    queue.push_back(std::move(value));
    while (queue.size() > limit) {queue.pop_front();}
  }

  SupervisorEffects & effects_;
  int budget_limit_, attempts_{0};
  State state_{State::STARTING};
  std::string reason_{"waiting_for_reference"};
  uint64_t epoch_{0};
  PoseValidator validator_{0.25};
  std::deque<Fix> fixes_;
  std::deque<std::pair<double, std::array<double, 3>>> imu_;
  std::optional<std::pair<double, std::array<double, 6>>> odom_;
  std::optional<PoseSample> last_pose_;
  std::deque<std::pair<double, std::array<double, 3>>> recent_poses_;
  std::deque<StampKey> accepted_;
  std::deque<Deferred> pending_clouds_, pending_lidar_;
  double drain_now_{0};
  double last_sim_{0}, clock_wall_{0}, last_tick_wall_{0};
  bool clock_paused_{true}, wall_started_{false};
  std::optional<double> stationary_since_;
  double minimum_stamp_{0}, attempt_wall_{0};
  std::optional<double> healthy_since_;
  bool map_ready_{false};
  std::optional<double> map_stamp_;
  bool nav_active_{false};
  uint64_t next_request_{1};
  std::optional<uint64_t> reset_pending_;
  std::optional<std::pair<bool, uint64_t>> reset_answer_;
  std::optional<uint64_t> clear_pending_;
  int clear_answers_{0};
  bool clear_failed_{false};
  std::array<double, 7> restart_pose_{};
};

}  // namespace astrobot_localization
