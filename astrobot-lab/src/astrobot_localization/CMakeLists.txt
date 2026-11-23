cmake_minimum_required(VERSION 3.16)
project(astrobot_localization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for clang-tidy (scripts/check.sh).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# nav2_msgs comes from the apt Jazzy install, built against Jazzy's message packages. The Space ROS
# underlay is earlier on CMAKE_PREFIX_PATH and ships those packages without the fastrtps typesupport
# targets nav2_msgs links, so resolve this package's ROS dependencies from Jazzy (the libraries
# loaded at runtime too). Remove once Nav2 is built against the Space ROS underlay (0.2).
if(EXISTS /opt/ros/$ENV{ROS_DISTRO}/share/nav2_msgs)
  list(PREPEND CMAKE_PREFIX_PATH /opt/ros/$ENV{ROS_DISTRO})
endif()
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(astrobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-free supervision logic and SLAM child configuration.
add_library(${PROJECT_NAME}_core SHARED src/supervisor_core.cpp src/child_config.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PRIVATE nlohmann_json::nlohmann_json)

add_library(supervisor_node SHARED src/supervisor_node.cpp)
target_link_libraries(supervisor_node ${PROJECT_NAME}_core)
ament_target_dependencies(supervisor_node rclcpp rclcpp_components action_msgs ament_index_cpp astrobot_interfaces
  geometry_msgs nav2_msgs nav_msgs sensor_msgs std_msgs)
rclcpp_components_register_nodes(supervisor_node "astrobot_localization::LocalizationSupervisor")
add_executable(localization_supervisor src/localization_supervisor_main.cpp)
target_link_libraries(localization_supervisor supervisor_node)
ament_target_dependencies(localization_supervisor rclcpp)

add_executable(map_to_odom_tf src/map_to_odom_tf_node.cpp)
target_include_directories(map_to_odom_tf PRIVATE include)
target_link_libraries(map_to_odom_tf Eigen3::Eigen)
ament_target_dependencies(map_to_odom_tf rclcpp geometry_msgs nav_msgs tf2_ros astrobot_interfaces)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS localization_supervisor DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core supervisor_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS map_to_odom_tf DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_supervisor_core test/test_supervisor_core.cpp)
  target_link_libraries(test_supervisor_core ${PROJECT_NAME}_core nlohmann_json::nlohmann_json)
  add_executable(test_pose_history_se3 test/test_pose_history_se3.cpp)
  target_include_directories(test_pose_history_se3 PRIVATE include)
  target_link_libraries(test_pose_history_se3 Eigen3::Eigen)
  add_test(NAME test_pose_history_se3 COMMAND test_pose_history_se3)
endif()

ament_package()
