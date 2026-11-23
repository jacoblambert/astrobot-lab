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
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>

namespace astrobot_localization
{

enum class ConfigCheck { OK, REVISION_CONSUMER_REQUIRED, INVALID };

// The navigation consumers only accept latest poses: SLAM sub/global mapping optimization must be
// explicitly disabled (a revision exporter alone cannot make accumulated terrain revisable).
ConfigCheck check_revision_config(const std::filesystem::path & config);

// Per-epoch copy of the SLAM configuration with the stationary reference pose
// [x y z qx qy qz qw] as initial state, the gated absolute-pose topic and the epoch.
std::filesystem::path prepare_child_config(
  const std::filesystem::path & config, const std::filesystem::path & run_dir,
  uint64_t epoch, const std::array<double, 7> & pose,
  const std::string & absolute_pose_topic);

}  // namespace astrobot_localization
