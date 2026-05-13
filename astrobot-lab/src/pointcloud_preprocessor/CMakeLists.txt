cmake_minimum_required(VERSION 3.8)
project(pointcloud_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_crop_filter_node src/pointcloud_crop_filter_node.cpp)
target_compile_features(pointcloud_crop_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_crop_filter_node rclcpp sensor_msgs)

install(TARGETS pointcloud_crop_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
