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
#include <string>

#include <astrobot_interfaces/msg/slam_revision.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "astrobot_terrain/pipeline.hpp"

namespace astrobot_terrain
{

constexpr const char * kNativeFrame = "vlp16";

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp);
Pose pose_matrix(const geometry_msgs::msg::Pose & pose);
Revision revision_from_msg(const astrobot_interfaces::msg::SlamRevision & msg);
// Versioned native chunk wire contract (40-byte points: x y z depth float32, intensity uint8 @16,
// sequence uint32 @20, capture uuid 16 x uint8 @24); throws on any layout/identity mismatch.
NativeChunk decode_native(const sensor_msgs::msg::PointCloud2 & msg);
nav_msgs::msg::OccupancyGrid grid_message(
  const ClassGrid & grid, const Pose & map_from_grid, double resolution,
  int64_t stamp_ns, const std::string & frame_id);

}  // namespace astrobot_terrain
