#ifndef SLAM_MAPPING__OBSTACLE_MEMORY_HPP_
#define SLAM_MAPPING__OBSTACLE_MEMORY_HPP_

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include "slam_mapping/probabilistic_voxel_map.hpp"

namespace slam_mapping {
// Navigation-output memory only: never changes ground support or 3D occupancy.
// Call with timestamps of actual obstacle-height returns, not cloud/publication
// timestamps. Publication cannot renew evidence age. Clear on every map epoch.
class ObstacleMemory {
public:
  void clear() { entries_.clear(); have_geometry_ = false; last_stamp_ = -1; }

  void apply(BevGrid & grid, const std::vector<double> & observed_at, const Point3 & origin,
             double stamp, double radius, double lifetime) {
    if (grid.width <= 0 || grid.height <= 0 || !std::isfinite(grid.resolution) || grid.resolution <= 0 ||
        static_cast<size_t>(grid.width)*static_cast<size_t>(grid.height) != grid.data.size() ||
        !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
        observed_at.size() != grid.data.size() || !std::isfinite(stamp) ||
        !std::isfinite(radius) || !std::isfinite(lifetime) || radius < 0 || lifetime < 0) {
      throw std::invalid_argument("invalid obstacle memory input");
    }
    if (stamp < last_stamp_ || !have_geometry_ || width_ != grid.width || height_ != grid.height ||
        resolution_ != grid.resolution || x_ != grid.origin_x || y_ != grid.origin_y) clear();
    width_ = grid.width; height_ = grid.height; resolution_ = grid.resolution;
    x_ = grid.origin_x; y_ = grid.origin_y; have_geometry_ = true; last_stamp_ = stamp;
    if (radius == 0 || lifetime == 0) { entries_.clear(); return; }
    for (size_t i=0; i<grid.data.size(); ++i) {
      const double ts = observed_at[i];
      if (grid.data[i] == 100 && std::isfinite(ts) && ts <= stamp && stamp-ts <= lifetime) {
        const auto found = entries_.find(i);
        if (found == entries_.end() || ts > found->second) entries_[i] = ts;
      }
    }
    for (auto it=entries_.begin(); it!=entries_.end();) {
      if (stamp-it->second > lifetime) { it=entries_.erase(it); continue; }
      const auto i=it->first;
      const double dx=x_+(i%width_+.5)*resolution_-origin.x;
      const double dy=y_+(i/width_+.5)*resolution_-origin.y;
      if (dx*dx+dy*dy <= radius*radius) grid.data[i]=100;
      ++it;
    }
  }

private:
  std::unordered_map<size_t,double> entries_;
  bool have_geometry_{false};
  int width_{0},height_{0};
  double resolution_{0},x_{0},y_{0},last_stamp_{-1};
};
}  // namespace slam_mapping
#endif
