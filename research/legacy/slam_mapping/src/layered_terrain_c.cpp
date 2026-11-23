#include "slam_mapping/layered_terrain.hpp"
#include <algorithm>
#include <string>
namespace { thread_local std::string error; }
extern "C" {
const char* terrain_error() { return error.c_str(); }
#if defined(TERRAIN_OBSERVATIONS) && defined(TERRAIN_DIAGNOSTICS)
int terrain_observation_channels() { return 22; }
int terrain_observations(void* map,double* cells) {
  const auto& data=static_cast<slam_mapping::LayeredTerrain*>(map)->diagnostics().observations;
  if (data.empty()) return 0;
  for (size_t i=0;i<data.size();++i) std::copy(data[i].begin(),data[i].end(),cells+22*i);
  return 1;
}
#endif
#ifdef TERRAIN_DIAGNOSTICS
int terrain_diagnostics(void* map,uint32_t* cells,uint32_t* corners) {
  const auto& data=static_cast<slam_mapping::LayeredTerrain*>(map)->diagnostics();
  if (data.cells.empty() || data.corners.empty()) return 0;
  for (size_t i=0;i<data.cells.size();++i) std::copy(data.cells[i].begin(),data.cells[i].end(),cells+4*i);
  for (size_t i=0;i<data.corners.size();++i) std::copy(data.corners[i].begin(),data.corners[i].end(),corners+8*i);
  return 1;
}
#endif
void* terrain_create(int width,int height,double resolution,double x,double y,double lifetime,int interpolate) {
  try {
    slam_mapping::TerrainConfig cfg;
    cfg.width=width; cfg.height=height; cfg.resolution=resolution; cfg.origin_x=x; cfg.origin_y=y;
    cfg.lifetime=lifetime; cfg.interpolate=interpolate;
    return new slam_mapping::LayeredTerrain(cfg);
  } catch (const std::exception& e) { error=e.what(); return nullptr; }
}
void terrain_destroy(void* map) { delete static_cast<slam_mapping::LayeredTerrain*>(map); }
#ifdef TERRAIN_ROLLING_GRID
int terrain_shift_origin(void* map,double x,double y) {
  try { static_cast<slam_mapping::LayeredTerrain*>(map)->shift_origin(x,y); return 1; }
  catch (const std::exception& e) { error=e.what(); return -1; }
}
#endif
int terrain_integrate(void* map,const double* xyz,size_t count,double stamp,double x,double y,double z,double yaw) {
  try { return static_cast<slam_mapping::LayeredTerrain*>(map)->integrate(xyz,count,stamp,{x,y,z},yaw); }
  catch (const std::exception& e) { error=e.what(); return -1; }
}
int terrain_project(void* map,double now,float* output) {
  try {
    const auto data=static_cast<slam_mapping::LayeredTerrain*>(map)->project(now);
    for (size_t i=0;i<data.cost.size();++i) {
      output[6*i]=data.cost[i]; output[6*i+1]=data.reason[i]; output[6*i+2]=data.ground[i];
      output[6*i+3]=data.uncertainty[i]; output[6*i+4]=data.obstacle_height[i]; output[6*i+5]=data.age[i];
    }
    return 1;
  } catch (const std::exception& e) { error=e.what(); return -1; }
}
}
