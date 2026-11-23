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

#include <stdexcept>

#include "astrobot_localization/pose_history_se3.hpp"
void check(bool ok) {if (!ok) {throw std::runtime_error("SE3 history contract failed");}}
int main()
{
  astrobot_localization::PoseHistorySE3 h;
  auto a = Eigen::Isometry3d::Identity(), b = a;
  b.translation() << .1, .2, .3;
  b.linear() = (Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(.6,
    Eigen::Vector3d::UnitY())).toRotationMatrix();
  h.insert(1.1, b); h.insert(1, a);
  auto mid = h.at(1.05); check(mid.has_value());
  check(mid->translation().isApprox(b.translation() * .5));
  check(mid->linear().isApprox(Eigen::Quaterniond(a.linear()).slerp(.5,
    Eigen::Quaterniond(b.linear())).toRotationMatrix()));
  auto extrinsic = a; extrinsic.translation() << .264, .017, -.262;
  extrinsic.linear() = Eigen::AngleAxisd(3.141592653589793,
    Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Isometry3d map_base = b * extrinsic;
  const Eigen::Isometry3d map_odom = map_base * mid->inverse();
  check((map_odom * (*mid)).isApprox(map_base));
  check(!h.at(.9)); check(!h.at(1.2)); check(!h.at(1.05, .01));
  h.insert(1.1, a); check(h.at(1.1)->isApprox(a));
  h.clear(); check(!h.at(1));
}
