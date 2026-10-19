#ifndef SLAM_MAPPING__POSE_HISTORY_HPP_
#define SLAM_MAPPING__POSE_HISTORY_HPP_
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
namespace slam_mapping {
struct Pose2D { double x; double y; double yaw; };
inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
class PoseHistory {
public:
  void clear() { values_.clear(); }
  void insert(double stamp, Pose2D pose) {
    if (!std::isfinite(stamp) || !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) return;
    auto it = std::lower_bound(values_.begin(), values_.end(), stamp,
      [](const auto & v, double t) { return v.first < t; });
    if (it != values_.end() && it->first == stamp) it->second = pose;
    else values_.insert(it, {stamp, pose});
    while (values_.size() > 1 && values_.back().first - values_.front().first > 10.0) values_.pop_front();
    while (values_.size() > 2000) values_.pop_front();
  }
  std::optional<Pose2D> at(double stamp, double max_gap = 0.2) const {
    auto hi = std::lower_bound(values_.begin(), values_.end(), stamp,
      [](const auto & v, double t) { return v.first < t; });
    if (hi != values_.end() && hi->first == stamp) return hi->second;
    if (hi == values_.begin() || hi == values_.end()) return std::nullopt;
    auto lo = std::prev(hi);
    if (hi->first - lo->first > max_gap) return std::nullopt;
    double a = (stamp - lo->first) / (hi->first - lo->first);
    return Pose2D{lo->second.x + a * (hi->second.x - lo->second.x),
                  lo->second.y + a * (hi->second.y - lo->second.y),
                  wrap_angle(lo->second.yaw + a * wrap_angle(hi->second.yaw - lo->second.yaw))};
  }
private:
  std::deque<std::pair<double, Pose2D>> values_;
};
}
#endif
