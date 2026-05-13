cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(voxel_map_builder_node src/voxel_map_builder_node.cpp)
target_compile_features(voxel_map_builder_node PUBLIC cxx_std_17)
ament_target_dependencies(voxel_map_builder_node rclcpp sensor_msgs)

add_executable(voxel_to_bev_costmap_node src/voxel_to_bev_costmap_node.cpp)
target_compile_features(voxel_to_bev_costmap_node PUBLIC cxx_std_17)
ament_target_dependencies(voxel_to_bev_costmap_node rclcpp sensor_msgs nav_msgs)

install(TARGETS voxel_map_builder_node voxel_to_bev_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
