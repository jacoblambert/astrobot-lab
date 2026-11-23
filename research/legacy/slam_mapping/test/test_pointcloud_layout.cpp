#include "slam_mapping/pointcloud_layout.hpp"
#include <limits>
#include <stdexcept>
#include <iostream>
using slam_mapping::valid_pointcloud_layout;
void check(bool ok) { if (!ok) throw std::runtime_error("PointCloud2 layout contract failed"); }
int main() {
  check(valid_pointcloud_layout(2,2,12,24,48,{0,4,8}));
  check(valid_pointcloud_layout(2,2,12,32,56,{0,4,8}));  // row padding
  check(!valid_pointcloud_layout(2,2,12,24,47,{0,4,8}));
  check(!valid_pointcloud_layout(2,2,12,23,48,{0,4,8}));
  check(!valid_pointcloud_layout(1,1,12,12,12,{0,4,std::numeric_limits<uint32_t>::max()}));
  check(!valid_pointcloud_layout(1,1,0,0,0,{0,0,0}));
  check(!valid_pointcloud_layout(std::numeric_limits<uint32_t>::max(),2,12,24,48,{0,4,8}));
  check(valid_pointcloud_layout(0,1,12,0,0,{0,4,8}));
  std::cout << "PointCloud2 layout tests passed\n";
}
