cmake_minimum_required(VERSION 3.16)
project(astrobot_glim_extensions)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for clang-tidy (scripts/check.sh).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(glim REQUIRED)
find_package(astrobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# GLIM loads extension modules by library name (config_ros.json extension_modules).
add_library(astrobot_abs_constraint_module SHARED src/abs_constraint_module.cpp)
target_include_directories(astrobot_abs_constraint_module PRIVATE include)
target_link_libraries(astrobot_abs_constraint_module glim::glim)
ament_target_dependencies(astrobot_abs_constraint_module rclcpp geometry_msgs)

add_library(astrobot_revision_module SHARED src/revision_module.cpp)
target_include_directories(astrobot_revision_module PRIVATE include)
target_link_libraries(astrobot_revision_module glim::glim)
ament_target_dependencies(astrobot_revision_module rclcpp astrobot_interfaces std_msgs)

install(TARGETS astrobot_abs_constraint_module astrobot_revision_module
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  add_executable(test_absolute_association test/test_absolute_association.cpp)
  target_include_directories(test_absolute_association PRIVATE include)
  add_test(NAME test_absolute_association COMMAND test_absolute_association)
  add_executable(test_revision_module test/test_revision_module.cpp)
  target_include_directories(test_revision_module PRIVATE include)
  target_link_libraries(test_revision_module glim::glim)
  ament_target_dependencies(test_revision_module rclcpp astrobot_interfaces std_msgs)
  add_test(NAME test_revision_module
    COMMAND test_revision_module ${CMAKE_CURRENT_SOURCE_DIR}/../astrobot_bringup/config/glim)
  set_tests_properties(test_revision_module PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=179")
endif()

ament_package()
