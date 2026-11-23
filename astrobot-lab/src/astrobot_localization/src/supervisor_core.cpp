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

#include "astrobot_localization/supervisor_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace astrobot_localization
{
namespace
{

// Absolute-fix gates (2026-10-20). Per axis, never one 3-D trace sigma: RTK's vertical variance
// must not widen the horizontal gate. While SLAM runs, each fix is gated against SLAM's own
// position at the fix stamp (innovation), so an outlier burst cannot capture the reference the
// way a chain of fix-to-fix comparisons can. Without SLAM: fix to last accepted fix with a
// speed bound.
constexpr double kGateSigma = 6.0;      // per-axis sigma; a 2-D 6-sigma radius has p = exp(-18)
// m/s, rover motion bound over intervals SLAM does not cover
constexpr double kMaxSpeed = 0.5;
constexpr double kGlimAllowance = 0.1;  // m, SLAM's own error against truth
constexpr size_t kFixes = 100, kImu = 200, kRecent = 100, kClouds = 5, kLidar = 100;

bool finite(std::initializer_list<double> values)
{
  return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
}

std::pair<double, double> horizontal_vertical_sigma(const std::array<double, 36> & c)
{
  return {std::sqrt((c[0] + c[7]) / 2), std::sqrt(c[14])};
}

bool active(State s) {return s == State::HEALTHY || s == State::DEGRADED;}

}  // namespace

std::string PoseValidator::validate(const PoseSample & sample, double now)
{
  const auto & p = sample.position;
  const auto & q = sample.quaternion;
  if (!finite({sample.stamp, now, p[0], p[1], p[2], q[0], q[1], q[2], q[3]})) {return "non_finite";}
  const double norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (std::abs(norm - 1.0) > 0.02) {return "quaternion_norm";}
  const double age = now - sample.stamp;
  if (age < -0.05) {return "future_stamp";}
  if (age > max_age_) {return "stale";}
  if (previous_) {
    const double dt = sample.stamp - previous_->stamp;
    if (dt <= 0) {return "non_increasing_stamp";}
    const auto & o = previous_->position;
    const double displacement = std::sqrt((p[0] - o[0]) * (p[0] - o[0]) + (p[1] - o[1]) *
        (p[1] - o[1]) +
        (p[2] - o[2]) * (p[2] - o[2]));
    if (displacement > position_slack_ + max_speed_ * dt) {return "kinematic_position";}
    const auto & r = previous_->quaternion;
    const double dot = std::abs(q[0] * r[0] + q[1] * r[1] + q[2] * r[2] + q[3] * r[3]);
    const double rnorm = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3];
    const double angle = 2 * std::acos(std::min(1.0, dot / std::sqrt(norm * rnorm)));
    if (angle > 0.05 + max_angular_speed_ * dt) {return "kinematic_rotation";}
  }
  previous_ = sample;
  return "";
}

std::array<double, 4> orientation_from_gravity(
  const std::array<double, 3> & a, double yaw,
  double gravity)
{
  const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (!finite({a[0], a[1], a[2], yaw}) || !(0.5 * gravity <= norm && norm <= 1.5 * gravity)) {
    throw std::invalid_argument("stationary lunar gravity observation required");
  }
  const double roll = std::atan2(a[1], a[2]);
  const double pitch = std::atan2(-a[0], std::hypot(a[1], a[2]));
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

SupervisorCore::SupervisorCore(SupervisorEffects & effects, int restart_budget)
: effects_(effects), budget_limit_(restart_budget) {}

void SupervisorCore::imu(double stamp, const std::array<double, 3> & a)
{
  if (finite({a[0], a[1], a[2]})) {push_bounded(imu_, {stamp, a}, kImu);}
}

void SupervisorCore::odom(double stamp, const std::array<double, 6> & twist)
{
  odom_ = {{stamp, twist}};
}

std::optional<std::pair<std::array<double, 3>,
  double>> SupervisorCore::estimated_position(double t) const
{
  if (recent_poses_.empty()) {return std::nullopt;}
  const double first = recent_poses_.front().first, last = recent_poses_.back().first;
  const double uncovered = std::max({0.0, first - t, t - last});
  if (uncovered > 0.5) {return std::nullopt;}
  std::array<double, 3> xyz{};
  if (t <= first) {
    xyz = recent_poses_.front().second;
  } else if (t >= last) {
    xyz = recent_poses_.back().second;
  } else {
    // np.interp: the first sample at or after t and its predecessor.
    size_t i = 1;
    while (i < recent_poses_.size() && recent_poses_[i].first < t) {++i;}
    const auto & [t0, p0] = recent_poses_[i - 1];
    const auto & [t1, p1] = recent_poses_[i];
    for (int k = 0; k < 3; ++k) {
      xyz[k] = t1 == t0 ? p1[k] : p0[k] + (t - t0) * (p1[k] - p0[k]) / (t1 - t0);
    }
  }
  return {{xyz, uncovered}};
}

void SupervisorCore::fix(const Fix & msg, double now)
{
  const auto & p = msg.position;
  const auto & q = msg.quaternion;
  const auto & c = msg.covariance;
  if (msg.frame_id != "map" || !finite({msg.stamp, p[0], p[1], p[2], q[0], q[1], q[2], q[3], c[0],
      c[7], c[14], c[35]})) {return;}
  if (std::min({c[0], c[7], c[14]}) <= 0 || !(0 < c[35] && c[35] < 0.1)) {return;}
  if (std::abs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1) > 0.02 ||
    !(-0.05 <= now - msg.stamp && now - msg.stamp <= 0.5))
  {
    return;
  }
  if (!fixes_.empty() && msg.stamp <= fixes_.back().stamp) {return;}
  std::optional<std::array<double, 3>> reference;
  double allowance = 0;
  std::array<double, 36> cov = c;
  if (const auto expected = estimated_position(msg.stamp)) {
    reference = expected->first;
    allowance = kGlimAllowance + kMaxSpeed * expected->second;
  } else if (!fixes_.empty()) {
    const auto & last = fixes_.back();
    reference = last.position;
    allowance = 0.05 + kMaxSpeed * (msg.stamp - last.stamp);
    for (int i = 0; i < 15; ++i) {
      cov[i] = c[i] + last.covariance[i];
    }
  }
  if (reference) {
    const auto [sh, sz] = horizontal_vertical_sigma(cov);
    const auto & r = *reference;
    if (std::hypot(p[0] - r[0], p[1] - r[1]) > allowance + kGateSigma * sh ||
      std::abs(p[2] - r[2]) > allowance + kGateSigma * sz)
    {
      return;
    }
  }
  push_bounded(fixes_, msg, kFixes);
  effects_.publish_fix(msg);
}

void SupervisorCore::fault(const std::string & reason, bool terminal)
{
  if (state_ == State::FAULTED) {return;}
  if (active(state_)) {attempts_ = 0;}
  state_ = terminal ? State::FAULTED : State::STOPPING;
  reason_ = reason;
  // Invalidate asynchronous work before stopping the estimator: a reset completing for a failed
  // attempt must never start another child later.
  reset_pending_.reset();
  reset_answer_.reset();
  clear_pending_.reset();
  map_ready_ = false;
  map_stamp_.reset();
  last_pose_.reset();
  recent_poses_.clear();
  accepted_.clear();
  pending_clouds_.clear();
  pending_lidar_.clear();
  healthy_since_.reset();
  stationary_since_.reset();
  publish_health();
  effects_.cancel_navigation();
  effects_.stop_child();
  effects_.log_error("localization held: " + reason);
}

void SupervisorCore::pose(const PoseSample & msg, double now)
{
  if (state_ != State::REINITIALIZING && state_ != State::REBUILDING && !active(state_)) {return;}
  if (clock_paused_ || msg.stamp < minimum_stamp_) {return;}
  // SLAM can revise a published frame during smoothing: never pair a revised pose with an
  // already relayed cloud, or treat it as motion.
  if (last_pose_ && msg.stamp <= last_pose_->stamp) {return;}
  const std::string reason = msg.frame_id != "map" ? "wrong_frame" : validator_.validate(msg, now);
  if (!reason.empty()) {
    fault(reason);
    return;
  }
  const auto & p = msg.position;
  if (!fixes_.empty() && std::abs(fixes_.back().stamp - msg.stamp) < 0.1) {
    const auto & f = fixes_.back();
    const auto [sh, sz] = horizontal_vertical_sigma(f.covariance);
    if (std::hypot(p[0] - f.position[0], p[1] - f.position[1]) > std::max(0.5, kGateSigma * sh) ||
      std::abs(p[2] - f.position[2]) > std::max(0.5, kGateSigma * sz))
    {
      fault("absolute_position_disagreement");
      return;
    }
  }
  last_pose_ = msg;
  push_bounded(recent_poses_, {msg.stamp, p}, kRecent);
  push_bounded(accepted_, msg.key, kRecent);
  effects_.publish_pose(msg);
  if (!healthy_since_) {healthy_since_ = msg.stamp;}
  if (state_ == State::REINITIALIZING) {state_ = State::REBUILDING;}
  drain_now_ = now;
  drain();
}

void SupervisorCore::lidar(
  double stamp, const StampKey & key, std::function<void()> publish,
  double now)
{
  push_bounded(pending_lidar_, Deferred{stamp, key, std::move(publish)}, kLidar);
  drain_now_ = now;
  drain();
}

void SupervisorCore::cloud(
  double stamp, const StampKey & key, std::function<void()> publish,
  double now)
{
  push_bounded(pending_clouds_, Deferred{stamp, key, std::move(publish)}, kClouds);
  drain_now_ = now;
  drain();
}

void SupervisorCore::drain()
{
  if (state_ != State::REBUILDING && !active(state_)) {return;}
  for (auto * queue : {&pending_lidar_, &pending_clouds_}) {
    std::deque<Deferred> waiting;
    while (!queue->empty()) {
      Deferred item = std::move(queue->front());
      queue->pop_front();
      if (item.stamp < minimum_stamp_ || drain_now_ - item.stamp > 0.5) {continue;}
      if (std::find(accepted_.begin(), accepted_.end(), item.key) != accepted_.end()) {
        item.publish();
      } else {
        waiting.push_back(std::move(item));
      }
    }
    *queue = std::move(waiting);
  }
}

void SupervisorCore::map(const MapView & m, double now)
{
  // Freshness follows the newest valid map: one late or rejected message (a load spike on the
  // terrain node) must not read as a stale map and restart SLAM.
  map_ready_ = false;
  if (m.frame_id != "map" || !last_pose_ || m.stamp < minimum_stamp_ ||
    !(0 <= now - m.stamp && now - m.stamp <= 0.75) ||
    !std::isfinite(m.resolution) || m.resolution <= 0 ||
    m.size != static_cast<size_t>(m.width) * m.height)
  {
    return;
  }
  // A map-rebuild readiness check, not a collision/path approval: Nav2 still checks the
  // footprint and route before driving.
  const auto & p = last_pose_->position;
  const auto col = static_cast<int64_t>(std::floor((p[0] - m.origin_x) / m.resolution));
  const auto row = static_cast<int64_t>(std::floor((p[1] - m.origin_y) / m.resolution));
  if (!(0 <= row && row < m.height && 0 <= col && col < m.width)) {return;}
  const auto radius = static_cast<int64_t>(std::ceil(1.5 / m.resolution));
  int64_t known_free = 0;
  for (int64_t r = std::max<int64_t>(0, row - radius);
    r < std::min<int64_t>(m.height, row + radius + 1); ++r)
  {
    for (int64_t c = std::max<int64_t>(0, col - radius);
      c < std::min<int64_t>(m.width, col + radius + 1); ++c)
    {
      const int8_t v = m.data[r * m.width + c];
      known_free += v >= 0 && v < 50;
    }
  }
  map_ready_ = known_free >= 25;
  map_stamp_ = m.stamp;
}

void SupervisorCore::map_integrity(bool ok)
{
  if (!ok) {fault("map_capacity_exceeded", true);}
}

std::optional<std::array<double, 7>> SupervisorCore::stationary_reference(double now)
{
  if (reference_override) {return reference_override(now);}
  if (!odom_ || !(0 <= now - odom_->first && now - odom_->first < 0.5)) {
    stationary_since_.reset();
    return std::nullopt;
  }
  const auto & v = odom_->second;
  if (!finite({v[0], v[1], v[2], v[3], v[4],
      v[5]}) || std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) > 0.01 ||
    std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]) > 0.02)
  {
    stationary_since_.reset();
    return std::nullopt;
  }
  if (!stationary_since_) {stationary_since_ = now;}
  std::vector<const Fix *> fixes;
  for (const auto & f : fixes_) {
    if (now - 1 <= f.stamp && f.stamp <= now) {fixes.push_back(&f);}
  }
  std::vector<std::array<double, 3>> imu;
  for (const auto & [stamp, a] : imu_) {
    if (now - 1 <= stamp && stamp <= now) {imu.push_back(a);}
  }
  if (now - *stationary_since_ < 1 || fixes.size() < 10 || imu.size() < 10 ||
    now - fixes.back()->stamp > 0.25)
  {
    return std::nullopt;
  }
  std::array<double, 3> xyz{}, gravity{};
  double sin_sum = 0, cos_sum = 0;
  for (const auto * f : fixes) {
    for (int k = 0; k < 3; ++k) {
      xyz[k] += f->position[k] / fixes.size();
    }
    const double qz = f->quaternion[2], qw = f->quaternion[3];
    const double yaw = std::atan2(2 * (qw * qz), 1 - 2 * qz * qz);
    sin_sum += std::sin(yaw);
    cos_sum += std::cos(yaw);
  }
  for (const auto & a : imu) {
    for (int k = 0; k < 3; ++k) {
      gravity[k] += a[k] / imu.size();
    }
  }
  try {
    const auto q = orientation_from_gravity(gravity, std::atan2(sin_sum, cos_sum));
    return std::array<double, 7>{xyz[0], xyz[1], xyz[2], q[0], q[1], q[2], q[3]};
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  }
}

void SupervisorCore::reset_result(uint64_t id, bool success, uint64_t epoch)
{
  if (reset_pending_ && *reset_pending_ == id) {
    reset_answer_ = std::make_pair(success, epoch);
  }
}

void SupervisorCore::clear_result(uint64_t id, bool ok)
{
  if (!clear_pending_ || *clear_pending_ != id) {return;}
  ++clear_answers_;
  clear_failed_ |= !ok;
}

void SupervisorCore::publish_health()
{
  std::optional<StampKey> last;
  if (last_pose_) {last = last_pose_->key;}
  effects_.publish_status(state_, epoch_, reason_, active(state_) && !clock_paused_, last);
}

void SupervisorCore::tick(double now, double wall)
{
  if (!wall_started_) {
    wall_started_ = true;
    clock_wall_ = last_tick_wall_ = wall;
  }
  const double elapsed_wall = wall - last_tick_wall_;
  last_tick_wall_ = wall;
  if (now < last_sim_) {fault("clock_reversed_new_mission_required", true);}
  if (now > last_sim_) {clock_wall_ = wall;}
  clock_paused_ = wall - clock_wall_ > 0.5 || now <= 0;
  last_sim_ = now;
  if (clock_paused_) {
    attempt_wall_ += elapsed_wall;
    publish_health();
    return;
  }
  if (effects_.child_exited()) {fault("estimator_exited");}
  if ((active(state_) || state_ == State::REBUILDING) && last_pose_ &&
    now - last_pose_->stamp > 0.5) {fault("pose_stale");}
  if (active(state_) && (!map_stamp_ || now - *map_stamp_ > 1.0)) {fault("map_stale");}
  if (active(state_)) {
    const double age = fixes_.empty() ? std::numeric_limits<double>::infinity() : now -
      fixes_.back().stamp;
    if (age > 2) {
      fault("absolute_sensor_outage");
    } else {
      state_ = age > 0.25 ? State::DEGRADED : State::HEALTHY;
    }
  }
  if (state_ == State::STARTING || state_ == State::STOPPING) {
    const auto reference = stationary_reference(now);
    if (reference && !nav_active_ && effects_.reset_ready()) {
      if (attempts_ >= budget_limit_) {
        fault("restart_budget_exhausted", true);
      } else {
        ++attempts_;
        ++epoch_;
        minimum_stamp_ = now;
        map_ready_ = false;
        map_stamp_.reset();
        clear_pending_.reset();
        restart_pose_ = *reference;
        state_ = State::REINITIALIZING;
        reason_ = "resetting_map";
        attempt_wall_ = wall;
        reset_pending_ = next_request_++;
        reset_answer_.reset();
        effects_.request_reset(*reset_pending_, epoch_);
      }
    }
  }
  if (reset_pending_ && reset_answer_) {
    const auto [success, epoch] = *reset_answer_;
    reset_pending_.reset();
    reset_answer_.reset();
    try {
      if (!success || epoch != epoch_) {throw std::runtime_error("map reset rejected");}
      switch (effects_.launch_child(epoch_, restart_pose_)) {
        case SupervisorEffects::Launch::REVISION_CONSUMER_REQUIRED:
          fault("revision_consumer_required", true);
          break;
        case SupervisorEffects::Launch::INVALID_CONFIG:
          fault("invalid_mapping_revision_config", true);
          break;
        case SupervisorEffects::Launch::STARTED:
          state_ = State::REINITIALIZING;
          reason_ = "estimator_starting";
          attempt_wall_ = wall;
          validator_.reset();
          break;
      }
    } catch (const std::exception & e) {
      fault(std::string("restart_failed:") + e.what());
    }
  }
  if ((state_ == State::REINITIALIZING || state_ == State::REBUILDING) &&
    wall - attempt_wall_ > 30)
  {
    reset_pending_.reset();
    fault("restart_timeout");
  }
  const bool map_fresh = map_stamp_ && 0 <= now - *map_stamp_ && now - *map_stamp_ <= 0.75;
  if (state_ == State::REBUILDING && map_ready_ && map_fresh && healthy_since_ &&
    now - *healthy_since_ >= 2)
  {
    if (!clear_pending_ && effects_.clear_ready()) {
      clear_pending_ = next_request_++;
      clear_answers_ = 0;
      clear_failed_ = false;
      effects_.request_clear(*clear_pending_);
    }
    if (clear_pending_ && clear_answers_ >= 2 && !clear_failed_ && !nav_active_) {
      state_ = State::HEALTHY;
      reason_ = "validated";
      attempts_ = 0;
    }
  }
  publish_health();
}

}  // namespace astrobot_localization
