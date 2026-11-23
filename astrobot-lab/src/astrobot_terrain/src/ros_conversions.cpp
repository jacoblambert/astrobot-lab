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

#include "astrobot_terrain/ros_conversions.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <tuple>

namespace astrobot_terrain
{

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
}

Pose pose_matrix(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm2) ||
    std::abs(norm2 - 1.0) > 1e-6)
  {
    throw std::invalid_argument("invalid recorded pose/quaternion");
  }
  Pose out = Pose::Identity();
  out.topLeftCorner<3, 3>() = Eigen::Quaterniond(q.w, q.x, q.y,
      q.z).normalized().toRotationMatrix();
  out.topRightCorner<3, 1>() = Eigen::Vector3d(p.x, p.y, p.z);
  return out;
}

Revision revision_from_msg(const astrobot_interfaces::msg::SlamRevision & msg)
{
  if (msg.header.frame_id != "map") {throw std::invalid_argument("revision must be in map frame");}
  Revision rev;
  rev.session = msg.session_id;
  rev.epoch = msg.localization_epoch;
  rev.revision = msg.revision;
  rev.parent = msg.parent_revision;
  rev.kind = msg.kind;
  rev.snapshot = msg.full_snapshot;
  rev.frames.reserve(msg.frames.size());
  for (const auto & f : msg.frames) {
    if (f.frame_id < 0) {
      throw std::invalid_argument("invalid frame id: expected nonnegative integer");
    }
    FramePose frame;
    frame.frame_id = static_cast<uint64_t>(f.frame_id);
    frame.stamp_ns = stamp_ns(f.measurement_stamp);
    if (frame.stamp_ns < 0) {throw std::invalid_argument("invalid measurement stamp");}
    frame.map_from_imu = pose_matrix(f.map_from_imu);
    frame.imu_from_lidar = pose_matrix(f.imu_from_lidar);
    frame.deskew_recomputed = f.deskew_recomputed;
    rev.frames.push_back(frame);
  }
  return rev;
}

NativeChunk decode_native(const sensor_msgs::msg::PointCloud2 & msg)
{
  using sensor_msgs::msg::PointField;
  static const std::tuple<const char *, uint32_t, uint8_t, uint32_t> kFields[] = {
    {"x", 0, PointField::FLOAT32, 1}, {"y", 4, PointField::FLOAT32, 1},
    {"z", 8, PointField::FLOAT32, 1}, {"depth", 12, PointField::FLOAT32, 1},
    {"intensity", 16, PointField::UINT8, 1}, {"sequence", 20, PointField::UINT32, 1},
    {"capture_uuid", 24, PointField::UINT8, 16}};
  bool layout = msg.fields.size() == 7;
  for (size_t i = 0; layout && i < 7; ++i) {
    const auto & [name, offset, type, count] = kFields[i];
    const auto & f = msg.fields[i];
    layout = f.name == name && f.offset == offset && f.datatype == type && f.count == count;
  }
  const size_t width = msg.width;
  if (msg.header.frame_id != kNativeFrame || msg.header.stamp.sec < 0 ||
    msg.header.stamp.nanosec >= 1'000'000'000u ||
    msg.height != 1 || width == 0 || width > kMaxPoints || !layout || msg.is_bigendian ||
    !msg.is_dense ||
    msg.point_step != 40 || msg.row_step != 40 * width || msg.data.size() != msg.row_step)
  {
    throw std::invalid_argument("invalid native wire layout/frame/stamp");
  }
  NativeChunk out;
  out.stamp_ns = stamp_ns(msg.header.stamp);
  out.xyz.resize(3 * width);
  out.intensity.resize(width);
  const uint8_t * data = msg.data.data();
  uint32_t sequence = 0;
  std::memcpy(&sequence, data + 20, 4);
  for (size_t i = 0; i < width; ++i) {
    const uint8_t * row = data + 40 * i;
    float values[4];
    std::memcpy(values, row, 16);
    uint32_t row_sequence;
    std::memcpy(&row_sequence, row + 20, 4);
    if (row_sequence != sequence || std::memcmp(row + 24, data + 24, 16) != 0) {
      throw std::invalid_argument("mixed native chunk identities");
    }
    for (float v : values) {
      if (!std::isfinite(v)) {throw std::invalid_argument("nonfinite native returns");}
    }
    out.xyz[3 * i] = values[0];
    out.xyz[3 * i + 1] = values[1];
    out.xyz[3 * i + 2] = values[2];
    out.intensity[i] = row[16];
  }
  static const char * kHex = "0123456789abcdef";
  out.capture = "revision-";
  for (int i = 0; i < 16; ++i) {
    out.capture += kHex[data[24 + i] >> 4];
    out.capture += kHex[data[24 + i] & 15];
  }
  out.sequence = sequence;
  return out;
}

nav_msgs::msg::OccupancyGrid grid_message(
  const ClassGrid & grid, const Pose & map_from_grid, double resolution,
  int64_t stamp, const std::string & frame_id)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.sec = static_cast<int32_t>(stamp / 1'000'000'000);
  msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1'000'000'000);
  msg.info.width = static_cast<uint32_t>(grid.width);
  msg.info.height = static_cast<uint32_t>(grid.height);
  msg.info.resolution = static_cast<float>(resolution);
  msg.info.origin.position.x = map_from_grid(0, 3);
  msg.info.origin.position.y = map_from_grid(1, 3);
  msg.info.origin.position.z = map_from_grid(2, 3);
  const Eigen::Quaterniond q(Eigen::Matrix3d(map_from_grid.topLeftCorner<3, 3>()));
  msg.info.origin.orientation.x = q.x();
  msg.info.origin.orientation.y = q.y();
  msg.info.origin.orientation.z = q.z();
  msg.info.origin.orientation.w = q.w();
  msg.data.assign(grid.data.begin(), grid.data.end());
  return msg;
}

}  // namespace astrobot_terrain
