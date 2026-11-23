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
#include "astrobot_glim_extensions/abs_constraint_module.hpp"

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>

#include "astrobot_glim_extensions/absolute_association.hpp"

namespace glim
{

AbsConstraintModule::AbsConstraintModule()
: logger(create_module_logger("abs_constraint")),
  n_msgs(0),
  n_added(0),
  n_yaw_added(0),
  n_no_match(0),
  n_gated(0),
  last_log_stamp(0.0),
  last_added_dist(0.0)
{
  const Config config(GlobalConfig::get_config_path("config_ros"));

  enabled = config.param<bool>("abs_constraint", "enable", true);
  rtk_topic = config.param<std::string>("abs_constraint", "rtk_topic", "/rtk/pose");
  assoc_tolerance = config.param<double>("abs_constraint", "assoc_tolerance", 0.05);
  require_closed_window = config.param<bool>("abs_constraint", "require_closed_window", false);
  gate_dist = config.param<double>("abs_constraint", "gate_dist", 5.0);
  huber_k = config.param<double>("abs_constraint", "huber_k", 1.0);
  sigma_floor = config.param<double>("abs_constraint", "sigma_floor", 0.05);
  use_msg_covariance = config.param<bool>("abs_constraint", "use_msg_covariance", true);
  use_yaw = config.param<bool>("abs_constraint", "use_yaw", true);
  yaw_huber_k = config.param<double>("abs_constraint", "yaw_huber_k", 0.0);
  yaw_sigma_floor = config.param<double>("abs_constraint", "yaw_sigma_floor", 0.01);
  yaw_cov_threshold = config.param<double>("abs_constraint", "yaw_cov_threshold", 1.0);

  logger->info(
    "abs_constraint: enable={} topic={} assoc_tol={}s gate={}m huber_k={} sigma_floor={}m "
    "use_cov={} use_yaw={} yaw_huber_k={} yaw_sigma_floor={}rad",
    enabled, rtk_topic, assoc_tolerance, gate_dist, huber_k, sigma_floor, use_msg_covariance,
    use_yaw, yaw_huber_k, yaw_sigma_floor);

  if (enabled) {
    set_callbacks();
  } else {
    logger->warn("abs_constraint disabled by config -- no factors will be added");
  }
}

AbsConstraintModule::~AbsConstraintModule()
{
  if (smoother_callback_id >= 0) {
    OdometryEstimationCallbacks::on_smoother_update.remove(smoother_callback_id);
  }
  logger->info(
    "abs_constraint shutdown: msgs={} added={} yaw_added={} no_match={} gated={}",
    n_msgs, n_added, n_yaw_added, n_no_match, n_gated);
}

std::vector<GenericTopicSubscription::Ptr> AbsConstraintModule::create_subscriptions(
  rclcpp::Node & node)
{
  if (!enabled) {
    return {};
  }
  using Msg = geometry_msgs::msg::PoseWithCovarianceStamped;
  auto sub = std::make_shared<TopicSubscription<Msg>>(
    rtk_topic, [this](const std::shared_ptr<const Msg> & msg) {rtk_callback(msg);});
  return {sub};
}

void AbsConstraintModule::set_callbacks()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  smoother_callback_id = OdometryEstimationCallbacks::on_smoother_update.add(
    std::bind(&AbsConstraintModule::on_smoother_update, this, _1, _2, _3, _4));
}

void AbsConstraintModule::rtk_callback(
  const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> & msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex);
  const auto & orientation = msg->pose.pose.orientation;
  const double norm = orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (msg->header.frame_id != "map" || !std::isfinite(norm) || std::abs(norm - 1.0) > .02) {
    ++n_gated; return;
  }
  for (const int i : {0, 7, 14, 35}) {
    if (!std::isfinite(msg->pose.covariance[i]) || msg->pose.covariance[i] <= 0) {
      ++n_gated; return;
    }
  }
  Sample s;
  s.stamp = static_cast<double>(msg->header.stamp.sec) +
    static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  if (!std::isfinite(s.stamp) || s.stamp < 0 ||
    (require_closed_window && s.stamp <= latest_ordered_stamp))
  {
    ++n_gated; return;
  }
  s.pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);
  if (!s.pos.allFinite()) {++n_gated; return;}
  if (consumed_stamps.count(s.stamp) || std::any_of(buffer.begin(), buffer.end(),
    [&](const auto & old) {return old.stamp == s.stamp;})) {return;}

  // Per-axis sigma from the covariance diagonal (row-major 6x6: xx=0, yy=7, zz=14).
  const double cxx = msg->pose.covariance[0];
  const double cyy = msg->pose.covariance[7];
  const double czz = msg->pose.covariance[14];
  if (use_msg_covariance && cxx > 0.0 && cyy > 0.0 && czz > 0.0) {
    s.sigma = Eigen::Vector3d(
      std::max(sigma_floor, std::sqrt(cxx)),
      std::max(sigma_floor, std::sqrt(cyy)),
      std::max(sigma_floor, std::sqrt(czz)));
  } else {
    s.sigma = Eigen::Vector3d::Constant(sigma_floor);
  }

  // Absolute heading (GNSS-compass / dual-antenna). Carried in the orientation +
  // the yaw covariance (row-major 6x6 index 35). A finite, small yaw variance means
  // heading was measured; huge variance = position-only fix this epoch.
  const auto & q = msg->pose.pose.orientation;
  s.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cyaw = msg->pose.covariance[35];
  s.has_yaw = use_yaw && cyaw > 0.0 && cyaw < yaw_cov_threshold;
  s.yaw_sigma = s.has_yaw ? std::max(yaw_sigma_floor, std::sqrt(cyaw)) : 0.0;

  n_msgs++;
  if (require_closed_window) {latest_ordered_stamp = s.stamp;}
  // Keep the buffer time-ordered (messages may arrive slightly out of order under latency).
  if (!buffer.empty() && s.stamp < buffer.back().stamp) {
    auto it = std::upper_bound(
      buffer.begin(), buffer.end(), s.stamp, [](double v, const Sample & e) {return v < e.stamp;});
    buffer.insert(it, s);
  } else {
    buffer.push_back(s);
  }
  // Bound memory if the consumer is not draining (e.g. SLAM not yet running).
  while (buffer.size() > 2000) {
    buffer.pop_front();
  }
}

void AbsConstraintModule::on_smoother_update(
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback & smoother,
  gtsam::NonlinearFactorGraph & new_factors,
  gtsam::Values & new_values,
  std::map<std::uint64_t, double> & new_stamps)
{
  std::lock_guard<std::mutex> lock(buffer_mutex);
  std::map<uint64_t, double> incoming_poses;
  for (const auto & kv : new_stamps) {
    if (gtsam::Symbol(kv.first).chr() == 'x') {
      incoming_poses[kv.first] = kv.second;
    }
  }
  const auto & linearization = smoother.getLinearizationPoint();
  const double newest_frame_stamp = update_absolute_pose_keys(live_pose_keys, incoming_poses,
      [&](uint64_t key){return new_values.exists(key) || linearization.exists(key);});
  for (auto it = constrained_keys.begin(); it != constrained_keys.end(); ) {
    if (!live_pose_keys.count(*it)) {it = constrained_keys.erase(it);} else {++it;}
  }
  // Track keys even while the sample buffer is empty: a delayed fix can arrive
  // after this callback, while the pose remains inside the actual smoother lag.
  const auto matches = associate_absolute_samples(live_pose_keys, constrained_keys, consumed_stamps,
      buffer, assoc_tolerance,
    require_closed_window ? latest_ordered_stamp : std::numeric_limits<double>::infinity());
  n_no_match += live_pose_keys.size() - constrained_keys.size() - matches.size();
  for (const auto & kv : matches) {
    const Sample & s = kv.second;

    // --- Position prior (gtsam::GPSFactor) ---
    // Delayed matches must pass the same innovation gate as newly inserted keys.
    // Surviving old keys live in the smoother, not in this update's new_values.
    bool add_pos = true;
    double innov = -1.0;
    {
      const auto & values = new_values.exists(kv.first) ? new_values : linearization;
      const Eigen::Vector3d predicted = values.at<gtsam::Pose3>(kv.first).translation();
      innov = (s.pos - predicted).norm();
      if (!std::isfinite(innov) || (gate_dist > 0.0 && innov > gate_dist)) {
        n_gated++;
        add_pos = false;
      }
    }
    if (add_pos) {
      gtsam::SharedNoiseModel base = gtsam::noiseModel::Diagonal::Sigmas(s.sigma);
      gtsam::SharedNoiseModel model =
        (huber_k > 0.0) ?
        gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Huber::Create(huber_k), base)) :
        base;
      new_factors.emplace_shared<gtsam::GPSFactor>(kv.first, gtsam::Point3(s.pos), model);
      n_added++;
      if (innov >= 0.0) {
        last_added_dist = innov;
      }
    }

    // --- Heading prior (yaw only) ---
    // Independent of the position gate: a wrong frame yaw is exactly what makes the
    // position look like an outlier, so the heading must get through to fix it.
    // PriorFactor<Pose3> with huge roll/pitch/translation sigmas = yaw-only prior
    // (gtsam tangent order [rx,ry,rz, x,y,z]; rz==yaw for level attitude). Heading is
    // left un-robustified by default so a trustworthy compass can correct a bad frame.
    if (s.has_yaw) {
      const double kBig = 1e3;
      gtsam::Vector6 sig;
      sig << kBig, kBig, s.yaw_sigma, kBig, kBig, kBig;
      gtsam::SharedNoiseModel ynoise = gtsam::noiseModel::Diagonal::Sigmas(sig);
      if (yaw_huber_k > 0.0) {
        ynoise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(yaw_huber_k), ynoise);
      }
      const gtsam::Pose3 meas(gtsam::Rot3::Yaw(s.yaw), gtsam::Point3(s.pos));
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(kv.first, meas, ynoise);
      n_yaw_added++;
    }
    if (add_pos || s.has_yaw) {
      consumed_stamps.insert(s.stamp);
      constrained_keys.insert(kv.first);
    }
  }

  // Use actual surviving keys, not an assumed lag or the fallback wrapper's
  // inherited timestamps. The inner smoother owns the live linearization point.
  if (newest_frame_stamp > 0.0) {
    double oldest_frame_stamp = newest_frame_stamp;
    for (const auto & key : live_pose_keys) {
      oldest_frame_stamp = std::min(oldest_frame_stamp, key.second);
    }
    const double cutoff = oldest_frame_stamp - assoc_tolerance;
    while (!buffer.empty() && buffer.front().stamp < cutoff) {
      buffer.pop_front();
    }
    consumed_stamps.erase(consumed_stamps.begin(), consumed_stamps.lower_bound(cutoff));
  }

  // Throttled diagnostics (uses sim/frame time, ~every 5s of data).
  if (newest_frame_stamp - last_log_stamp >= 5.0) {
    last_log_stamp = newest_frame_stamp;
    logger->info(
      "abs_constraint: added={} yaw_added={} no_match={} gated={} msgs={} last_innov={:.3f}m "
      "buffer={}",
      n_added, n_yaw_added, n_no_match, n_gated, n_msgs, last_added_dist, buffer.size());
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule * create_extension_module()
{
  return new glim::AbsConstraintModule();
}
