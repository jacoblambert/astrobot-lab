cmake_minimum_required(VERSION 3.16)
project(astrobot_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for clang-tidy (scripts/check.sh).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(astrobot_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/command_mux.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(command_mux_node SHARED src/command_mux_node.cpp)
target_link_libraries(command_mux_node ${PROJECT_NAME}_core)
ament_target_dependencies(command_mux_node rclcpp rclcpp_components astrobot_interfaces diagnostic_msgs geometry_msgs std_msgs)
rclcpp_components_register_nodes(command_mux_node "astrobot_control::CommandMuxNode")
add_executable(command_mux src/command_mux_main.cpp)
target_link_libraries(command_mux command_mux_node)
ament_target_dependencies(command_mux rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS command_mux DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core command_mux_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_mux test/test_command_mux.cpp)
  target_link_libraries(test_command_mux ${PROJECT_NAME}_core)
endif()

ament_package()
