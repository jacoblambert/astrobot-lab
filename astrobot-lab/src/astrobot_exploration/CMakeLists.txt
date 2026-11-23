cmake_minimum_required(VERSION 3.16)
project(astrobot_exploration)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for clang-tidy (scripts/check.sh).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# nav2_msgs comes from the apt Jazzy install; see astrobot_localization/CMakeLists.txt.
if(EXISTS /opt/ros/$ENV{ROS_DISTRO}/share/nav2_msgs)
  list(PREPEND CMAKE_PREFIX_PATH /opt/ros/$ENV{ROS_DISTRO})
endif()
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(astrobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# ROS-free goal selection.
add_library(${PROJECT_NAME}_core SHARED src/mission_planner.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(${PROJECT_NAME}_strategies SHARED src/strategies.cpp)
target_link_libraries(${PROJECT_NAME}_strategies ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_strategies rclcpp pluginlib)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(exploration_node SHARED src/exploration_node.cpp)
target_link_libraries(exploration_node ${PROJECT_NAME}_core nlohmann_json::nlohmann_json)
ament_target_dependencies(exploration_node rclcpp rclcpp_action rclcpp_components pluginlib astrobot_interfaces
  geometry_msgs nav2_msgs nav_msgs std_msgs)
rclcpp_components_register_nodes(exploration_node "astrobot_exploration::ExplorationManager")
add_executable(exploration_manager src/exploration_manager_main.cpp)
target_link_libraries(exploration_manager exploration_node)
ament_target_dependencies(exploration_manager rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS exploration_manager DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_strategies exploration_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_planner test/test_mission_planner.cpp)
  target_link_libraries(test_mission_planner ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_package()
