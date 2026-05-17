cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(probabilistic_voxel_map src/probabilistic_voxel_map.cpp)
target_compile_features(probabilistic_voxel_map PUBLIC cxx_std_17)
target_include_directories(probabilistic_voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(probabilistic_voxel_mapper_node src/probabilistic_voxel_mapper_node.cpp)
target_compile_features(probabilistic_voxel_mapper_node PUBLIC cxx_std_17)
target_link_libraries(probabilistic_voxel_mapper_node probabilistic_voxel_map)
ament_target_dependencies(probabilistic_voxel_mapper_node rclcpp sensor_msgs nav_msgs geometry_msgs)

add_executable(slam_map_to_odom_tf_node src/slam_map_to_odom_tf_node.cpp)
target_compile_features(slam_map_to_odom_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(slam_map_to_odom_tf_node rclcpp geometry_msgs nav_msgs tf2_ros)

if(BUILD_TESTING)
  add_executable(test_probabilistic_voxel_map test/test_probabilistic_voxel_map.cpp)
  target_compile_features(test_probabilistic_voxel_map PUBLIC cxx_std_17)
  target_link_libraries(test_probabilistic_voxel_map probabilistic_voxel_map)
  add_test(NAME test_probabilistic_voxel_map COMMAND test_probabilistic_voxel_map)
endif()

install(TARGETS
  probabilistic_voxel_map
  probabilistic_voxel_mapper_node
  slam_map_to_odom_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
