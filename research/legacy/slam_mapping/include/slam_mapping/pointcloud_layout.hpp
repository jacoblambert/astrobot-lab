#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace slam_mapping {
inline bool valid_pointcloud_layout(uint32_t width, uint32_t height, uint32_t point_step,
                                   uint32_t row_step, size_t data_size, std::array<uint32_t,3> offsets) {
  if (point_step < sizeof(float)) return false;
  for (const auto offset : offsets) if (offset > point_step-sizeof(float)) return false;
  const uint64_t row_bytes=static_cast<uint64_t>(width)*point_step;
  if (row_bytes > row_step) return false;
  const uint64_t required=height == 0 ? 0 : static_cast<uint64_t>(height-1)*row_step+row_bytes;
  return required <= data_size;
}
}  // namespace slam_mapping
