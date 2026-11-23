cmake_minimum_required(VERSION 3.16)
project(astrobot_terrain)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for clang-tidy (scripts/check.sh).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(astrobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROS-free core: layered terrain, pose timeline, reconstruction, cost policy, mosaics.
add_library(${PROJECT_NAME}_core SHARED
  src/layered_terrain.cpp
  src/pose_timeline.cpp
  src/grid_ops.cpp
  src/reconstructor.cpp
  src/pipeline.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen PRIVATE OpenMP::OpenMP_CXX)

add_library(${PROJECT_NAME}_ros SHARED src/ros_conversions.cpp)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_ros PUBLIC astrobot_interfaces geometry_msgs nav_msgs sensor_msgs)

add_library(terrain_node SHARED src/terrain_node.cpp)
target_link_libraries(terrain_node ${PROJECT_NAME}_ros nlohmann_json::nlohmann_json)
ament_target_dependencies(terrain_node rclcpp rclcpp_components astrobot_interfaces rosgraph_msgs std_msgs)
rclcpp_components_register_nodes(terrain_node "astrobot_terrain::TerrainNode")
add_executable(terrain src/terrain_main.cpp)
target_link_libraries(terrain terrain_node)
ament_target_dependencies(terrain rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS terrain DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros terrain_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_ops test/test_grid_ops.cpp)
  target_link_libraries(test_grid_ops ${PROJECT_NAME}_core)
  ament_add_gtest(test_pose_timeline test/test_pose_timeline.cpp)
  target_link_libraries(test_pose_timeline ${PROJECT_NAME}_core)
  ament_add_gtest(test_reconstructor test/test_reconstructor.cpp)
  target_link_libraries(test_reconstructor ${PROJECT_NAME}_core)
  ament_add_gtest(test_native_decode test/test_native_decode.cpp)
  target_link_libraries(test_native_decode ${PROJECT_NAME}_ros)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 astrobot_interfaces geometry_msgs nav_msgs sensor_msgs)
ament_package()
