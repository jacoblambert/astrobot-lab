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
#include <iostream>
#include <stdexcept>

#include "astrobot_glim_extensions/absolute_association.hpp"
struct Sample { double stamp; };
void check(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
int main()
{
  std::map<uint64_t, double> keys;
  auto newest = glim::update_absolute_pose_keys(keys, {{1, 1.0}, {2, 1.05}}, [](auto){
        return true;
    });
  check(newest == 1.05 && keys.size() == 2, "pose registry lost keys before a fix arrived");
  std::deque<Sample> samples{{1.049}};
  auto matches = glim::associate_absolute_samples(keys, {}, {}, samples, .05);
  check(matches.size() == 1 && matches[0].first == 2, "old key stole a better-matched fix");
  matches = glim::associate_absolute_samples(keys, {2}, {1.049}, samples, .05);
  check(matches.empty(), "a consumed fix was reused");
  samples = {{1.0}, {1.0}, {1.05}};
  matches = glim::associate_absolute_samples(keys, {}, {}, samples, .05);
  check(matches.size() == 2, "duplicate fix changed one-to-one association");
  samples = {{2.0}};
  check(glim::associate_absolute_samples(keys, {}, {}, samples, .05).empty(),
    "out-of-window fix was associated");
  // Actual surviving keys, not a hardcoded five-second window.
  glim::update_absolute_pose_keys(keys, {{3, 4.0}}, [](auto key){return key == 3;});
  check(keys.size() == 1 && keys.count(3), "marginalization horizon was ignored");
  glim::update_absolute_pose_keys(keys, {}, [](auto){return false;});
  check(keys.empty(), "factor could target a key missing from the smoother");
  keys = {{1, 1.0}};
  samples = {{.98}};
  check(glim::associate_absolute_samples(keys, {}, {}, samples, .05, .98).empty(),
    "open window consumed an older fix");
  samples.push_back({1.001});
  check(glim::associate_absolute_samples(keys, {}, {}, samples, .05, 1.001).empty(),
    "window closed before declared tolerance");
  samples.push_back({1.06});
  matches = glim::associate_absolute_samples(keys, {}, {}, samples, .05, 1.06);
  check(matches.size() == 1 && matches[0].second.stamp == 1.001,
    "closed window failed to choose nearest fix");
  check(glim::associate_absolute_samples(keys, {1}, {1.001}, samples, .05, 1.06).empty(),
    "closed window reused a factor");
  keys = {{2, 2.0}};
  check(glim::associate_absolute_samples(keys, {}, {}, samples, .05, 1.06).empty(),
    "outage fabricated future availability");
  std::cout << "absolute association tests passed\n";
}
