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
#include <vector>

namespace astrobot_terrain
{

// Row-major grid (row = y, col = x), as nav_msgs/OccupancyGrid data.
template<typename T>
struct Grid
{
  int width{0}, height{0};
  std::vector<T> data;
  Grid() = default;
  Grid(int w, int h, T fill)
  : width(w), height(h), data(static_cast<size_t>(w) * h, fill) {}
  T & at(int row, int col) {return data[static_cast<size_t>(row) * width + col];}
  const T & at(int row, int col) const {return data[static_cast<size_t>(row) * width + col];}
  size_t size() const {return data.size();}
};

using ClassGrid = Grid<int8_t>;  // -1 unknown, 0 measured ground, 20 inferred ground, 100 occupied

// Exact Euclidean distance (in cells) from every cell to the nearest feature cell, as
// scipy.ndimage.distance_transform_edt(~feature). Infinity everywhere if no feature exists.
std::vector<double> distance_to(const std::vector<uint8_t> & feature, int width, int height);

// Advisory display costs: occupied 100, unknown -1, known ground raised by a 1 m taper.
ClassGrid inflate(const ClassGrid & classes, double resolution = 0.1, double radius = 1.0);

// Planner gap policy ("nav2_bounded_shadow"): plain-missing unknown cells within
// max_support_gap of measured ground and farther than hazard_clearance from any conflict
// get a finite uncertainty cost; unknown within occupied_shadow_lethal of an occupied cell is
// published lethal, since MPPI with track_unknown_space tolerates unknown under the footprint.
struct GapPolicy
{
  int cost{70};
  double hazard_clearance_m{0.30};
  double max_support_gap_m{0.5};
  double occupied_shadow_lethal_m{0.20};
  double max_cell_m{0.100000001};
};

// reasons: the terrain library's per-cell reason channel (NO_RETURNS=0 ... UNSUPPORTED_EXTENT=12).
// Returns costs in [-1, 100]; uncertain marks cells that received the policy cost.
ClassGrid navigation_costs(
  const ClassGrid & classes, const std::vector<uint8_t> & reasons, const GapPolicy & policy,
  double resolution, std::vector<uint8_t> * uncertain = nullptr);

// Rover body (unpadded rectangle) in grid-corner metres; unknown cells whose centre lies under it
// become measured free (0): the LiDAR never sees the ground the rover stands on.
struct Body
{
  double half_length{0.495}, half_width{0.335};
};
void clear_footprint(
  ClassGrid & grid, const Body & body, double x, double y, double yaw,
  double resolution);

// Persistent map-frame mosaic of axis-aligned windows on one lattice. Observed cells keep the
// latest observation; unobserved policy cells fill only cells nothing has observed.
class Mosaic {
public:
  Mosaic(
    double lo_x, double lo_y, double hi_x, double hi_y, double first_x, double first_y,
    double resolution = 0.1,
    double tolerance = 1e-3);
  void update(
    const ClassGrid & window, double origin_x, double origin_y,
    const std::vector<uint8_t> * observed);
  ClassGrid window(double origin_x, double origin_y, int width, int height) const;
  const ClassGrid & grid() const {return grid_;}
  double origin_x() const {return origin_x_;}
  double origin_y() const {return origin_y_;}
  double resolution() const {return resolution_;}

private:
  void index(double x, double y, int & col, int & row) const;
  double resolution_, tolerance_, origin_x_, origin_y_;
  ClassGrid grid_;
  std::vector<uint8_t> observed_;
};

}  // namespace astrobot_terrain
