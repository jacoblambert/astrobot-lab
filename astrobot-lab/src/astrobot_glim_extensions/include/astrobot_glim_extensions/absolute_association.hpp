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
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <limits>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace glim
{
template<class IsLive>
double update_absolute_pose_keys(
  std::map<uint64_t, double> & keys,
  const std::map<uint64_t, double> & incoming, IsLive is_live)
{
  for (const auto & entry : incoming) {
    if (std::isfinite(entry.second)) {
      keys[entry.first] = entry.second;
    }
  }
  double newest = 0;
  for (const auto & entry : keys) {
    newest = std::max(newest, entry.second);
  }
  for (auto it = keys.begin(); it != keys.end(); ) {
    if (!is_live(it->first)) {it = keys.erase(it);} else {++it;}
  }
  return newest;
}

// Smallest time-error pair first; deterministic ties. Each fix and pose is used
// at most once. This avoids older keys stealing fixes from an exact newer key.
template<class Sample>
std::vector<std::pair<uint64_t, Sample>> associate_absolute_samples(
  const std::map<uint64_t, double> & keys, const std::set<uint64_t> & constrained,
  const std::set<double> & consumed, const std::deque<Sample> & samples, double tolerance,
  double window_watermark = std::numeric_limits<double>::infinity())
{
  using Edge = std::tuple<double, double, uint64_t, size_t>;
  std::vector<Edge> edges;
  for (const auto & key : keys) {
    if (constrained.count(key.first)) {continue;}
    // For an explicitly ordered source, wait until the entire matching window
    // has arrived. Otherwise optimization timing can permanently select an
    // older fix immediately before a closer one arrives. This is causal delay,
    // never an invented future measurement or a retimestamped old fix.
    if (key.second + tolerance > window_watermark) {continue;}
    for (size_t i = 0; i < samples.size(); ++i) {
      const auto ts = samples[i].stamp;
      if (!std::isfinite(ts) || consumed.count(ts)) {continue;}
      const double error = std::abs(key.second - ts);
      if (error <= tolerance) {edges.emplace_back(error, ts, key.first, i);}
    }
  }
  std::sort(edges.begin(), edges.end());
  std::set<uint64_t> used_keys;
  std::set<double> used_stamps;
  std::vector<std::pair<uint64_t, Sample>> matches;
  for (const auto & edge : edges) {
    const auto ts = std::get<1>(edge); const auto key = std::get<2>(edge);
    if (used_keys.count(key) || used_stamps.count(ts)) {continue;}
    matches.emplace_back(key, samples[std::get<3>(edge)]);
    used_keys.insert(key); used_stamps.insert(ts);
  }
  return matches;
}
}  // namespace glim
