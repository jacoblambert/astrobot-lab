# Native parity tools, built against an installed release workspace (inside astrobot-dev):
#   source install/setup.bash
#   cmake -S ../research/tools/native -B /tmp/astrobot_research_tools && cmake --build /tmp/astrobot_research_tools
# terrain_replay       terrain pipeline over a recorded/synthetic stream (nav_terrain_parity.py,
#                      replay_terrain_ros.py)
# libastrobot_terrain_c  plain C API of the layered terrain (check_terrain_library_parity.py,
#                      layered_terrain.py ctypes tooling)
# candidates_tool      goal candidates for a JSON fixture (exploration_parity.py)
cmake_minimum_required(VERSION 3.16)
project(astrobot_research_tools CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(astrobot_terrain REQUIRED)
find_package(nlohmann_json REQUIRED)
find_path(EXPLORATION_INCLUDE astrobot_exploration/mission_planner.hpp REQUIRED)
find_library(EXPLORATION_CORE astrobot_exploration_core REQUIRED)

add_executable(terrain_replay terrain_replay.cpp)
target_link_libraries(terrain_replay astrobot_terrain::astrobot_terrain_core nlohmann_json::nlohmann_json)

add_library(astrobot_terrain_c SHARED terrain_c_api.cpp)
target_link_libraries(astrobot_terrain_c PRIVATE astrobot_terrain::astrobot_terrain_core)

add_executable(candidates_tool candidates_tool.cpp)
target_include_directories(candidates_tool PRIVATE ${EXPLORATION_INCLUDE})
target_link_libraries(candidates_tool ${EXPLORATION_CORE} nlohmann_json::nlohmann_json)
