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

#include <gtest/gtest.h>

#include <cstring>

#include "astrobot_terrain/ros_conversions.hpp"

namespace astrobot_terrain
{

namespace
{

sensor_msgs::msg::PointCloud2 cloud(size_t n, uint32_t sequence)
{
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = kNativeFrame;
  msg.header.stamp.sec = 3;
  msg.header.stamp.nanosec = 500;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(n);
  const std::tuple<const char *, uint32_t, uint8_t, uint32_t> fields[] = {
    {"x", 0, PointField::FLOAT32, 1}, {"y", 4, PointField::FLOAT32, 1},
    {"z", 8, PointField::FLOAT32, 1}, {"depth", 12, PointField::FLOAT32, 1},
    {"intensity", 16, PointField::UINT8, 1}, {"sequence", 20, PointField::UINT32, 1},
    {"capture_uuid", 24, PointField::UINT8, 16}};
  for (const auto & [name, offset, type, count] : fields) {
    PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = type;
    f.count = count;
    msg.fields.push_back(f);
  }
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 40;
  msg.row_step = static_cast<uint32_t>(40 * n);
  msg.data.assign(40 * n, 0);
  for (size_t i = 0; i < n; ++i) {
    const float xyzd[4] = {1.0f + i, 2.0f, -0.5f, 2.3f};
    std::memcpy(&msg.data[40 * i], xyzd, 16);
    msg.data[40 * i + 16] = static_cast<uint8_t>(i);
    std::memcpy(&msg.data[40 * i + 20], &sequence, 4);
    for (int k = 0; k < 16; ++k) {
      msg.data[40 * i + 24 + k] = static_cast<uint8_t>(k * 17);
    }
  }
  return msg;
}

}  // namespace

TEST(DecodeNative, RoundTripsTheWireContract) {
  const auto chunk = decode_native(cloud(3, 42));
  EXPECT_EQ(chunk.capture, "revision-00112233445566778899aabbccddeeff");
  EXPECT_EQ(chunk.sequence, 42u);
  EXPECT_EQ(chunk.stamp_ns, 3'000'000'500);
  ASSERT_EQ(chunk.xyz.size(), 9u);
  EXPECT_FLOAT_EQ(chunk.xyz[3], 2.0f);
  EXPECT_EQ(chunk.intensity[2], 2);
}

TEST(DecodeNative, RejectsLayoutAndMixedIdentities) {
  auto wrong_frame = cloud(2, 1);
  wrong_frame.header.frame_id = "lidar";
  EXPECT_THROW(decode_native(wrong_frame), std::invalid_argument);
  auto mixed = cloud(2, 1);
  mixed.data[40 + 20] = 9;
  EXPECT_THROW(decode_native(mixed), std::invalid_argument);
  auto field = cloud(2, 1);
  field.fields[3].name = "range";
  EXPECT_THROW(decode_native(field), std::invalid_argument);
}

}  // namespace astrobot_terrain
