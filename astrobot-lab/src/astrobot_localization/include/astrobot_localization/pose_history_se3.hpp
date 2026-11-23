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
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <utility>

namespace astrobot_localization
{
class PoseHistorySE3 {
public:
  void clear() {values_.clear();}
  void insert(double stamp, const Eigen::Isometry3d & pose)
  {
    if (!std::isfinite(stamp) || !pose.matrix().allFinite()) {return;}
    auto it = std::lower_bound(values_.begin(), values_.end(), stamp,
        [](const auto & value, double t){return value.first < t;});
    if (it != values_.end() && it->first == stamp) {it->second = pose;} else {
      values_.insert(it, {stamp, pose});
    }
    while (values_.size() > 1 &&
      (values_.size() > 2000 || values_.back().first - values_.front().first > 10.0))
    {
      values_.pop_front();
    }
  }
  std::optional<Eigen::Isometry3d> at(double stamp, double max_gap = .2) const
  {
    if (!std::isfinite(stamp) || !std::isfinite(max_gap) || max_gap <= 0) {return std::nullopt;}
    auto hi = std::lower_bound(values_.begin(), values_.end(), stamp,
        [](const auto & value, double t){return value.first < t;});
    if (hi != values_.end() && hi->first == stamp) {return hi->second;}
    if (hi == values_.begin() || hi == values_.end()) {return std::nullopt;}
    auto lo = std::prev(hi);
    if (hi->first - lo->first > max_gap) {return std::nullopt;}
    const double f = (stamp - lo->first) / (hi->first - lo->first);
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = (1 - f) * lo->second.translation() + f * hi->second.translation();
    pose.linear() = Eigen::Quaterniond(lo->second.linear()).slerp(f,
        Eigen::Quaterniond(hi->second.linear())).toRotationMatrix();
    return pose;
  }

private:
  std::deque<std::pair<double, Eigen::Isometry3d>> values_;
};
}  // namespace astrobot_localization
