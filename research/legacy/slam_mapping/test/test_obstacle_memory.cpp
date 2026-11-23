#include "slam_mapping/obstacle_memory.hpp"
#include <stdexcept>
#include <iostream>
using namespace slam_mapping;
void check(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
BevGrid grid() { BevGrid g; g.width=10; g.height=10; g.resolution=.1; g.data.assign(100,-1); return g; }
int main() {
  ObstacleMemory memory;
  std::vector<double> stamps(100,std::numeric_limits<double>::quiet_NaN());
  auto g=grid(); g.data[55]=100; g.data[56]=0; stamps[55]=1;
  memory.apply(g,stamps,{.5,.5,1},1,.9,3);
  g=grid(); memory.apply(g,stamps,{.5,.5,1},2,.9,3);
  check(g.data[55]==100,"confirmed obstacle disappeared in near-field occlusion");
  check(g.data[56]==-1,"memory invented ground support");
  g=grid(); memory.apply(g,stamps,{10,10,1},2.1,.9,3);
  check(g.data[55]==-1,"memory escaped its near-field radius");
  // Frequent publication must not renew the timestamp of an old observation.
  for (double t=2.2;t<4;t+=.05) { g=grid(); memory.apply(g,stamps,{.5,.5,1},t,.9,3); }
  g=grid(); memory.apply(g,stamps,{.5,.5,1},4.1,.9,3);
  check(g.data[55]==-1,"publication made an obstacle immortal");
  g=grid(); g.data[55]=100; stamps[55]=5;
  memory.apply(g,stamps,{.5,.5,1},5,.9,3);
  memory.clear(); g=grid(); memory.apply(g,stamps,{.5,.5,1},5.1,.9,3);
  check(g.data[55]==-1,"map epoch failed to clear obstacle memory");
  g=grid(); g.data[55]=14; stamps[55]=6;
  memory.apply(g,stamps,{.5,.5,1},6,.9,3);
  g=grid(); memory.apply(g,stamps,{.5,.5,1},6.1,.9,3);
  check(g.data[55]==-1,"soft terrain risk was latched as a lethal obstacle");
  // End-to-end: rays just ABOVE an 8 cm rock miss-clear its 10 cm 3D voxel.
  // The navigation layer retains only the confirmed obstacle, then expires it.
  OccupancyConfig occupancy; occupancy.voxel_size=.1; occupancy.point_stride=1;
  occupancy.use_surface_evidence=true; occupancy.use_bounds=false;
  occupancy.hit_log_odds=.5; occupancy.miss_log_odds=-.2;
  ProbabilisticVoxelMap map(occupancy);
  BevProjectionConfig projection; projection.width_m=projection.height_m=2;
  projection.origin_x=projection.origin_y=0; projection.use_plane_fit=true;
  projection.use_ground_floor=false; projection.obstacle_height=.08;
  projection.obstacle_cost_min=projection.obstacle_cost_max=100; projection.obstacle_inflation_radius=0;
  std::vector<Point3> scene;
  for(int y=0;y<20;++y) for(int x=0;x<20;++x) scene.push_back({x*.1+.05,y*.1+.05,.001});
  scene.push_back({1.05,1.05,.081});
  for(int scan=0;scan<3;++scan) map.integratePointCloud({1,1,1},scene,1+scan*.05);
  g=map.projectToBev(projection);
  check(g.data[210]==100,"approach fixture did not confirm its rock");
  memory.clear(); memory.apply(g,g.obstacle_observed_at,{1.75,1.05,.091},1.1,.9,3);
  int raw_cost=100;
  for(int scan=1;scan<=16;++scan) {
    const double ts=1.1+scan*.05;
    map.integratePointCloud({1.75,1.05,.091},{{.15,1.05,.091}},ts);
    g=map.projectToBev(projection); raw_cost=g.data[210];
    memory.apply(g,g.obstacle_observed_at,{1.75,1.05,.091},ts,.9,3);
    check(g.data[210]==100,"near-field occlusion lost the confirmed obstacle");
  }
  check(raw_cost!=100,"fixture must reproduce ray clearing independently of navigation memory");
  map.integratePointCloud({1.75,1.05,.091},{},5);
  g=map.projectToBev(projection); memory.apply(g,g.obstacle_observed_at,{1.75,1.05,.091},5,.9,3);
  check(g.data[210]!=100,"navigation obstacle survived its observation lifetime");
  std::cout << "bounded obstacle memory tests passed\n";
}
