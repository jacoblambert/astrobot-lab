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

#include "astrobot_terrain/grid_ops.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace astrobot_terrain
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// Felzenszwalb & Huttenlocher 1-D squared distance transform of f (in place).
void squared_1d(
  std::vector<double> & f, std::vector<double> & d, std::vector<int> & v,
  std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    if (f[q] == kInf) {continue;}
    if (f[v[k]] == kInf) {  // no finite parabola yet
      v[k] = q;
      continue;
    }
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  if (f[v[0]] == kInf) {
    std::fill(d.begin(), d.end(), kInf);
  } else {
    k = 0;
    for (int q = 0; q < n; ++q) {
      while (z[k + 1] < q) {++k;}
      d[q] = (q - v[k]) * static_cast<double>(q - v[k]) + f[v[k]];
    }
  }
  f.swap(d);
}

bool known_ground(int8_t value) {return value == 0 || value == 20;}

bool plain_missing(uint8_t reason)
{
  return reason == 0 || reason == 4 || reason == 9 || reason == 12;
}

int8_t taper(double distance_m, double radius)
{
  return static_cast<int8_t>(std::ceil(99.0 * std::max(0.0, 1.0 - distance_m / radius)));
}

}  // namespace

std::vector<double> distance_to(const std::vector<uint8_t> & feature, int width, int height)
{
  if (feature.size() != static_cast<size_t>(width) * height) {
    throw std::invalid_argument("feature grid size");
  }
  // Squared integer distances are exact in double; sqrt matches scipy's result.
  std::vector<double> out(feature.size());
  for (size_t i = 0; i < feature.size(); ++i) {
    out[i] = feature[i] ? 0.0 : kInf;
  }
  const int n = std::max(width, height);
  std::vector<double> f, d;
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  f.resize(height);
  d.resize(height);
  for (int col = 0; col < width; ++col) {
    for (int row = 0; row < height; ++row) {
      f[row] = out[static_cast<size_t>(row) * width + col];
    }
    squared_1d(f, d, v, z);
    for (int row = 0; row < height; ++row) {
      out[static_cast<size_t>(row) * width + col] = f[row];
    }
  }
  f.resize(width);
  d.resize(width);
  for (int row = 0; row < height; ++row) {
    std::copy(out.begin() + static_cast<ptrdiff_t>(row) * width,
        out.begin() + static_cast<ptrdiff_t>(row + 1) * width,
              f.begin());
    squared_1d(f, d, v, z);
    std::copy(f.begin(), f.end(), out.begin() + static_cast<ptrdiff_t>(row) * width);
  }
  for (auto & value : out) {
    value = std::sqrt(value);
  }
  return out;
}

ClassGrid inflate(const ClassGrid & classes, double resolution, double radius)
{
  if (!(resolution > 0) || !(radius > 0)) {
    throw std::invalid_argument("positive resolution and inflation radius required");
  }
  ClassGrid result = classes;
  std::vector<uint8_t> occupied(classes.size());
  bool any = false;
  for (size_t i = 0; i < classes.size(); ++i) {
    const int8_t value = classes.data[i];
    if (value != -1 && value != 0 && value != 20 && value != 100) {
      throw std::invalid_argument("expected exploration classes (-1, 0, 20, 100)");
    }
    occupied[i] = value == 100;
    any |= occupied[i] != 0;
  }
  if (!any) {return result;}
  const auto cells = distance_to(occupied, classes.width, classes.height);
  for (size_t i = 0; i < classes.size(); ++i) {
    if (known_ground(classes.data[i])) {
      result.data[i] = std::max(result.data[i], taper(cells[i] * resolution, radius));
    }
  }
  return result;
}

ClassGrid navigation_costs(
  const ClassGrid & classes, const std::vector<uint8_t> & reasons, const GapPolicy & policy,
  double resolution, std::vector<uint8_t> * uncertain_out)
{
  if (reasons.size() != classes.size()) {throw std::invalid_argument("reason grid size");}
  if (!(policy.cost > 20 && policy.cost < 100)) {
    throw std::invalid_argument("uncertainty cost must lie between 20 and 100");
  }
  ClassGrid costs = inflate(classes, resolution);
  const size_t n = classes.size();
  std::vector<uint8_t> uncertain(n, 0), occupied(n, 0);
  bool any_occupied = false;
  for (size_t i = 0; i < n; ++i) {
    occupied[i] = classes.data[i] == 100;
    any_occupied |= occupied[i] != 0;
  }
  std::vector<double> to_occupied;
  if (any_occupied) {to_occupied = distance_to(occupied, classes.width, classes.height);}
  if (std::min(classes.width, classes.height) >= 3 && resolution <= policy.max_cell_m) {
    std::vector<uint8_t> measured(n, 0), conflicts(n, 0);
    bool any_eligible = false, any_measured = false, any_conflict = false;
    for (size_t i = 0; i < n; ++i) {
      const bool plain = plain_missing(reasons[i]);
      uncertain[i] = classes.data[i] == -1 && plain;
      measured[i] = classes.data[i] == 0;  // not class 20: inference cannot extend the support
      conflicts[i] = classes.data[i] == 100 || (classes.data[i] == -1 && !plain);
      any_eligible |= uncertain[i] != 0;
      any_measured |= measured[i] != 0;
      any_conflict |= conflicts[i] != 0;
    }
    if (any_eligible && any_measured) {
      const auto support = distance_to(measured, classes.width, classes.height);
      std::vector<double> clearance;
      if (any_conflict) {clearance = distance_to(conflicts, classes.width, classes.height);}
      for (size_t i = 0; i < n; ++i) {
        if (!uncertain[i]) {continue;}
        bool keep = support[i] * resolution <= policy.max_support_gap_m;
        if (any_conflict) {keep = keep && clearance[i] * resolution > policy.hazard_clearance_m;}
        uncertain[i] = keep;
      }
      for (size_t i = 0; i < n; ++i) {
        if (!uncertain[i]) {continue;}
        costs.data[i] = static_cast<int8_t>(policy.cost);
        if (any_occupied) {
          costs.data[i] = std::max(costs.data[i], taper(to_occupied[i] * resolution, 1.0));
        }
      }
    } else {
      std::fill(uncertain.begin(), uncertain.end(), 0);
    }
  }
  if (policy.occupied_shadow_lethal_m > 0 && any_occupied) {
    for (size_t i = 0; i < n; ++i) {
      if (classes.data[i] == -1 &&
        to_occupied[i] * resolution <= policy.occupied_shadow_lethal_m + 1e-9)
      {
        costs.data[i] = 100;
        uncertain[i] = 0;
      }
    }
  }
  if (uncertain_out) {*uncertain_out = std::move(uncertain);}
  return costs;
}

void clear_footprint(
  ClassGrid & grid, const Body & body, double x, double y, double yaw,
  double resolution)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const double dx = (0.0 + col * resolution + resolution / 2) - x;
      const double dy = (0.0 + row * resolution + resolution / 2) - y;
      const double u = dx * c + dy * s;
      const double v = -dx * s + dy * c;
      if (std::abs(u) <= body.half_length && std::abs(v) <= body.half_width && grid.at(row,
          col) == -1) {grid.at(row, col) = 0;}
    }
  }
}

Mosaic::Mosaic(
  double lo_x, double lo_y, double hi_x, double hi_y, double first_x, double first_y,
  double resolution,
  double tolerance)
: resolution_(resolution), tolerance_(tolerance)
{
  origin_x_ = first_x + std::floor((lo_x - first_x) / resolution) * resolution;
  origin_y_ = first_y + std::floor((lo_y - first_y) / resolution) * resolution;
  const int width = static_cast<int>(std::ceil((hi_x - origin_x_) / resolution));
  const int height = static_cast<int>(std::ceil((hi_y - origin_y_) / resolution));
  if (width <= 0 || height <= 0) {throw std::invalid_argument("empty mosaic");}
  grid_ = ClassGrid(width, height, -1);
  observed_.assign(grid_.size(), 0);
}

void Mosaic::index(double x, double y, int & col, int & row) const
{
  const double ox = (x - origin_x_) / resolution_, oy = (y - origin_y_) / resolution_;
  col = static_cast<int>(std::nearbyint(ox));
  row = static_cast<int>(std::nearbyint(oy));
  if (std::max(std::abs(ox - col), std::abs(oy - row)) > tolerance_) {
    throw std::invalid_argument("window is off the mosaic lattice");
  }
}

void Mosaic::update(
  const ClassGrid & window, double origin_x, double origin_y,
  const std::vector<uint8_t> * observed)
{
  int col0, row0;
  index(origin_x, origin_y, col0, row0);
  for (int r = std::max(0, -row0); r < window.height && row0 + r < grid_.height; ++r) {
    for (int c = std::max(0, -col0); c < window.width && col0 + c < grid_.width; ++c) {
      const int8_t value = window.at(r, c);
      if (value == -1) {continue;}
      const size_t target = static_cast<size_t>(row0 + r) * grid_.width + (col0 + c);
      const bool seen = observed ? (*observed)[static_cast<size_t>(r) * window.width + c] !=
        0 : true;
      if (seen || !observed_[target]) {grid_.data[target] = value;}
      if (seen && observed) {observed_[target] = 1;}
    }
  }
}

ClassGrid Mosaic::window(double origin_x, double origin_y, int width, int height) const
{
  int col0, row0;
  index(origin_x, origin_y, col0, row0);
  ClassGrid out(width, height, -1);
  for (int r = std::max(0, -row0); r < height && row0 + r < grid_.height; ++r) {
    for (int c = std::max(0, -col0); c < width && col0 + c < grid_.width; ++c) {
      out.at(r, c) = grid_.at(row0 + r, col0 + c);
    }
  }
  return out;
}

}  // namespace astrobot_terrain
