cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
find_package(astrobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(layered_terrain SHARED src/layered_terrain.cpp src/layered_terrain_c.cpp)
target_compile_features(layered_terrain PUBLIC cxx_std_17)
target_include_directories(layered_terrain PUBLIC include)
target_link_libraries(layered_terrain Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(layered_terrain OpenMP::OpenMP_CXX)
endif()

# Explicit diagnostic target, excluded from ordinary builds and installation.
# Preprocessor isolation keeps instrumentation out of the default mapper path.
add_library(layered_terrain_diagnostics SHARED EXCLUDE_FROM_ALL src/layered_terrain.cpp src/layered_terrain_c.cpp)
target_compile_features(layered_terrain_diagnostics PUBLIC cxx_std_17)
target_compile_definitions(layered_terrain_diagnostics PUBLIC TERRAIN_DIAGNOSTICS)
target_include_directories(layered_terrain_diagnostics PUBLIC include)
target_link_libraries(layered_terrain_diagnostics Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
  target_link_libraries(layered_terrain_diagnostics OpenMP::OpenMP_CXX)
endif()

# Diagnostic evidence target, never default-built or installed. The rejected
# observation-only classifier is archived, not selectable in runtime source.
add_library(layered_terrain_observations SHARED EXCLUDE_FROM_ALL src/layered_terrain.cpp src/layered_terrain_c.cpp)
target_compile_features(layered_terrain_observations PUBLIC cxx_std_17)
target_compile_definitions(layered_terrain_observations PUBLIC TERRAIN_OBSERVATIONS TERRAIN_DIAGNOSTICS)
target_include_directories(layered_terrain_observations PUBLIC include)
target_link_libraries(layered_terrain_observations Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(layered_terrain_observations OpenMP::OpenMP_CXX)
endif()

# Isolated two-path experiment; never installed or enabled in navigation.
add_library(layered_terrain_two_path SHARED EXCLUDE_FROM_ALL src/layered_terrain.cpp src/layered_terrain_c.cpp)
target_compile_features(layered_terrain_two_path PUBLIC cxx_std_17)
target_compile_definitions(layered_terrain_two_path PUBLIC TERRAIN_OBSERVATIONS TERRAIN_TWO_PATH)
target_include_directories(layered_terrain_two_path PUBLIC include)
target_link_libraries(layered_terrain_two_path Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(layered_terrain_two_path OpenMP::OpenMP_CXX)
endif()
# Separate classifier experiment; known occupied sources keep their occupied
# label but do not add the supplemental native-only unknown halo.
add_library(layered_terrain_resolved_boundary SHARED EXCLUDE_FROM_ALL src/layered_terrain.cpp src/layered_terrain_c.cpp)
target_compile_features(layered_terrain_resolved_boundary PUBLIC cxx_std_17)
target_compile_definitions(layered_terrain_resolved_boundary PUBLIC TERRAIN_OBSERVATIONS TERRAIN_TWO_PATH TERRAIN_RESOLVED_BOUNDARY)
target_include_directories(layered_terrain_resolved_boundary PUBLIC include)
target_link_libraries(layered_terrain_resolved_boundary Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(layered_terrain_resolved_boundary OpenMP::OpenMP_CXX)
endif()
add_library(probabilistic_voxel_map src/probabilistic_voxel_map.cpp)
target_compile_features(probabilistic_voxel_map PUBLIC cxx_std_17)
target_include_directories(probabilistic_voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(probabilistic_voxel_mapper_node src/probabilistic_voxel_mapper_node.cpp)
target_compile_features(probabilistic_voxel_mapper_node PUBLIC cxx_std_17)
target_link_libraries(probabilistic_voxel_mapper_node probabilistic_voxel_map)
ament_target_dependencies(probabilistic_voxel_mapper_node rclcpp sensor_msgs nav_msgs geometry_msgs astrobot_interfaces std_msgs)

add_executable(slam_map_to_odom_tf_node src/slam_map_to_odom_tf_node.cpp)
target_compile_features(slam_map_to_odom_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(slam_map_to_odom_tf_node rclcpp geometry_msgs nav_msgs tf2_ros astrobot_interfaces)
target_include_directories(slam_map_to_odom_tf_node PRIVATE include)
target_link_libraries(slam_map_to_odom_tf_node Eigen3::Eigen)

if(BUILD_TESTING)
  option(SLAM_MAPPING_TEST_TWO_PATH "Build and test the unpromoted two-path terrain experiment" OFF)
  if(SLAM_MAPPING_TEST_TWO_PATH)
    # Inherit the candidate's PUBLIC definitions: compiling this runner with
    # default macros would give LayeredTerrain a different C++ class layout.
    add_executable(test_layered_terrain_two_path test/test_layered_terrain.cpp)
    target_link_libraries(test_layered_terrain_two_path layered_terrain_two_path)
    add_test(NAME test_layered_terrain_two_path COMMAND test_layered_terrain_two_path)
    add_executable(test_layered_terrain_resolved_boundary test/test_layered_terrain.cpp)
    target_link_libraries(test_layered_terrain_resolved_boundary layered_terrain_resolved_boundary)
    add_test(NAME test_layered_terrain_resolved_boundary COMMAND test_layered_terrain_resolved_boundary)
  endif()
  add_executable(test_observation_moments test/test_observation_moments.cpp)
  target_compile_features(test_observation_moments PRIVATE cxx_std_17)
  target_include_directories(test_observation_moments PRIVATE include)
  target_link_libraries(test_observation_moments Eigen3::Eigen)
  add_test(NAME test_observation_moments COMMAND test_observation_moments)
  add_executable(test_layered_terrain test/test_layered_terrain.cpp)
  target_link_libraries(test_layered_terrain layered_terrain)
  add_test(NAME test_layered_terrain COMMAND test_layered_terrain)
  add_executable(test_pose_history_se3 test/test_pose_history_se3.cpp)
  target_compile_features(test_pose_history_se3 PRIVATE cxx_std_17)
  target_include_directories(test_pose_history_se3 PRIVATE include)
  target_link_libraries(test_pose_history_se3 Eigen3::Eigen)
  add_test(NAME test_pose_history_se3 COMMAND test_pose_history_se3)
  add_executable(test_probabilistic_voxel_map test/test_probabilistic_voxel_map.cpp)
  target_compile_features(test_probabilistic_voxel_map PUBLIC cxx_std_17)
  target_link_libraries(test_probabilistic_voxel_map probabilistic_voxel_map)
  add_test(NAME test_probabilistic_voxel_map COMMAND test_probabilistic_voxel_map)
  add_executable(test_pose_history test/test_pose_history.cpp)
  target_compile_features(test_pose_history PUBLIC cxx_std_17)
  target_include_directories(test_pose_history PRIVATE include)
  add_test(NAME test_pose_history COMMAND test_pose_history)
  add_executable(test_surface_evidence test/test_surface_evidence.cpp)
  target_link_libraries(test_surface_evidence probabilistic_voxel_map)
  add_test(NAME test_surface_evidence COMMAND test_surface_evidence)
  add_executable(test_obstacle_memory test/test_obstacle_memory.cpp)
  target_link_libraries(test_obstacle_memory probabilistic_voxel_map)
  add_test(NAME test_obstacle_memory COMMAND test_obstacle_memory)
  add_executable(test_pointcloud_layout test/test_pointcloud_layout.cpp)
  target_include_directories(test_pointcloud_layout PRIVATE include)
  target_compile_features(test_pointcloud_layout PRIVATE cxx_std_17)
  add_test(NAME test_pointcloud_layout COMMAND test_pointcloud_layout)
endif()

install(TARGETS
  layered_terrain
  probabilistic_voxel_map
  probabilistic_voxel_mapper_node
  slam_map_to_odom_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

option(SLAM_MAPPING_BUILD_TERRAIN_SHADOW "Install advisory-only native terrain shadow tools" OFF)
if(SLAM_MAPPING_BUILD_TERRAIN_SHADOW)
  set_target_properties(layered_terrain_resolved_boundary PROPERTIES EXCLUDE_FROM_ALL FALSE)
  # No dependency from the normal navigation mapper to this candidate. The
  # runtime also requires an explicit frozen library path and grid/range bounds.
  install(TARGETS layered_terrain_resolved_boundary DESTINATION lib/${PROJECT_NAME})
  set(SHADOW_TOOLS ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools)
  install(PROGRAMS ${SHADOW_TOOLS}/live_terrain_shadow.py DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS ${SHADOW_TOOLS}/live_local_terrain.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY ${SHADOW_TOOLS}/local_terrain DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py" PATTERN "*.json")
  install(FILES ${SHADOW_TOOLS}/build_local_terrain.py ${SHADOW_TOOLS}/selected_lidar_profile.py
    DESTINATION lib/${PROJECT_NAME})
  install(FILES
    ${SHADOW_TOOLS}/terrain_shadow_core.py ${SHADOW_TOOLS}/terrain_shadow_runtime.py
    ${SHADOW_TOOLS}/native_chunk_transport.py ${SHADOW_TOOLS}/layered_terrain.py
    ${SHADOW_TOOLS}/shadow_terrain.py ${SHADOW_TOOLS}/raw_return_filter.py
    ${SHADOW_TOOLS}/revision_evidence.py ${SHADOW_TOOLS}/archive_guard.py
    ${SHADOW_TOOLS}/local_terrain_seal.py
    ${SHADOW_TOOLS}/export_shadow_trajectory.py DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
