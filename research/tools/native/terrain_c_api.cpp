// Copyright 2026 Jacob Lambert
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>

#include "astrobot_terrain/layered_terrain.hpp"
namespace {thread_local std::string error;}
extern "C" {
const char * terrain_error() {return error.c_str();}
void * terrain_create(
  int width, int height, double resolution, double x, double y, double lifetime,
  int interpolate)
{
  try {
    astrobot_terrain::TerrainConfig cfg;
    cfg.width = width; cfg.height = height; cfg.resolution = resolution; cfg.origin_x = x;
    cfg.origin_y = y;
    cfg.lifetime = lifetime; cfg.interpolate = interpolate;
    return new astrobot_terrain::LayeredTerrain(cfg);
  } catch (const std::exception & e) {
    error = e.what(); return nullptr;
  }
}
void terrain_destroy(void * map) {delete static_cast<astrobot_terrain::LayeredTerrain *>(map);}
int terrain_shift_origin(void * map, double x, double y)
{
  try {
    static_cast<astrobot_terrain::LayeredTerrain *>(map)->shift_origin(x, y); return 1;
  } catch (const std::exception & e) {
    error = e.what(); return -1;
  }
}
int terrain_integrate(
  void * map, const double * xyz, size_t count, double stamp, double x,
  double y, double z, double yaw)
{
  try {
    return static_cast<astrobot_terrain::LayeredTerrain *>(map)->integrate(xyz, count, stamp,
             {x, y, z}, yaw);
  } catch (const std::exception & e) {
    error = e.what(); return -1;
  }
}
int terrain_project(void * map, double now, float * output)
{
  try {
    const auto data = static_cast<astrobot_terrain::LayeredTerrain *>(map)->project(now);
    for (size_t i = 0; i < data.cost.size(); ++i) {
      output[6 * i] = data.cost[i]; output[6 * i + 1] = data.reason[i];
      output[6 * i + 2] = data.ground[i];
      output[6 * i + 3] = data.uncertainty[i]; output[6 * i + 4] = data.obstacle_height[i];
      output[6 * i + 5] = data.age[i];
    }
    return 1;
  } catch (const std::exception & e) {
    error = e.what(); return -1;
  }
}
}
