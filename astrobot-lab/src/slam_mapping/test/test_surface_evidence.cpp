#include "slam_mapping/probabilistic_voxel_map.hpp"
#include <stdexcept>
#include <iostream>
using namespace slam_mapping;
void check(bool ok, const char * why) { if (!ok) throw std::runtime_error(why); }
int cell(const BevGrid & g, double x, double y) {
  return g.data.at(static_cast<size_t>((y-g.origin_y)/g.resolution)*g.width +
                   static_cast<size_t>((x-g.origin_x)/g.resolution));
}
BevProjectionConfig bev() {
  BevProjectionConfig b; b.width_m=2; b.height_m=2; b.origin_x=0; b.origin_y=0;
  b.use_plane_fit=true; b.use_ground_floor=false; b.obstacle_height=.08;
  b.obstacle_cost_min=100; b.obstacle_cost_max=100; b.obstacle_inflation_radius=0;
  b.min_occupied_voxels_per_cell=3; return b;
}
OccupancyConfig occupancy() {
  OccupancyConfig o; o.voxel_size=.1; o.point_stride=1; o.use_bounds=false;
  o.use_surface_evidence=true; o.decay_factor=.999; return o;
}
int main() {
  for (double offset : {.001, .031, .071}) for (double height : {.05,.08,.10,.14,.20,.25}) {
    ProbabilisticVoxelMap map(occupancy()); std::vector<Point3> pts;
    for(int y=0;y<20;++y) for(int x=0;x<20;++x) pts.push_back({x*.1+.05,y*.1+.05,offset});
    pts.push_back({1.05,1.05,offset+height});
    for(int i=1;i<=3;++i) map.integratePointCloud({1,1,1},pts,i*.05);
    auto b=bev(); b.terrain_risk_height=.05;
    auto grid=map.projectToBev(b);
    check(cell(grid,.55,.55)==0,"flat ground incorrectly occupied");
    check(height < .08 ? (cell(grid,1.05,1.05)>0 && cell(grid,1.05,1.05)<100) :
                        cell(grid,1.05,1.05)==100,"height/grid-offset regression");
    b.ground_window_radius=8; map.projectToBev(b); // formerly stack overflow
  }
  ProbabilisticVoxelMap air(occupancy());
  air.integratePointCloud({.05,.05,1.05},{{1.05,.05,1.05}},1);
  check(cell(air.projectToBev(bev()),.55,.05)==-1,"air ray must not prove ground support");
  auto capped=occupancy(); capped.max_voxels=5;
  ProbabilisticVoxelMap limited(capped);
  auto stats=limited.integratePointCloud({.05,.05,.05},{{1.05,.05,.05}},1);
  check(limited.logOdds(limited.worldToKey({1.05,.05,.05}))>0,"endpoint admission priority");
  check(stats.skipped_capacity>0,"capacity drops must be explicit");
  auto o=occupancy(); ProbabilisticVoxelMap a(o),b(o);
  a.integratePointCloud({0,0,1},{{0,0,0}},1); b.integratePointCloud({0,0,1},{{0,0,0}},1);
  a.integratePointCloud({0,0,1},{},2);
  b.integratePointCloud({0,0,1},{},1.5); b.integratePointCloud({0,0,1},{},2);
  check(std::abs(a.logOdds({0,0,0})-b.logOdds({0,0,0}))<1e-6,"decay depends on replay call rate");
  // Smooth slopes and deterministic centimetre-scale sensor noise must not seed rocks.
  for (double slope : {0., .1, .25}) {
    ProbabilisticVoxelMap smooth(occupancy());
    for (int scan=1;scan<=5;++scan) {
      std::vector<Point3> ground;
      for (int y=0;y<20;++y) for (int x=0;x<20;++x) {
        const double noise = .005 * ((x*7+y*3+scan)%3-1);
        ground.push_back({x*.1+.05,y*.1+.05,slope*(x*.1+.05)+noise});
      }
      smooth.integratePointCloud({1,1,1},ground,scan*.05);
    }
    auto grid=smooth.projectToBev(bev());
    for (int y=3;y<17;++y) for (int x=3;x<17;++x)
      check(cell(grid,x*.1+.05,y*.1+.05)<100,"smooth slope/noise created lethal obstacle");
  }
  // A single new observation after expiry must reconfirm, not revive stale confidence.
  ProbabilisticVoxelMap expired(occupancy());
  std::vector<Point3> surrounding;
  for(int y=0;y<20;++y) for(int x=0;x<20;++x)
    if(x!=10 || y!=10) surrounding.push_back({x*.1+.05,y*.1+.05,.001});
  auto old_scene=surrounding;
  old_scene.push_back({1.05,1.05,.001}); old_scene.push_back({1.05,1.05,.081});
  for(int scan=0;scan<3;++scan) expired.integratePointCloud({1,1,1},old_scene,1+scan*.05);
  check(cell(expired.projectToBev(bev()),1.05,1.05)==100,"fixture must start with a confirmed rock");
  auto ground_only=surrounding; ground_only.push_back({1.05,1.05,.001});
  expired.integratePointCloud({1,1,1},ground_only,1.2);
  auto intermediate=expired.projectToBev(bev());
  check(cell(intermediate,1.05,1.05)==100,"fixture needs two retained obstacle observations");
  check(intermediate.obstacle_observed_at.at(10*20+10)<=1.1,
        "new ground return renewed old obstacle timestamp");
  expired.integratePointCloud({1,1,1},surrounding,39.9);
  expired.integratePointCloud({1,1,1},surrounding,39.95);
  expired.integratePointCloud({1,1,1},{{1.05,1.05,.081}},40);
  check(cell(expired.projectToBev(bev()),1.05,1.05)==-1,"expired evidence was revived by a single hit");
  auto invalid=bev(); invalid.width_m=1e100;
  bool rejected=false;
  try { expired.projectToBev(invalid); } catch (const std::invalid_argument&) { rejected=true; }
  check(rejected,"huge projection must be rejected before integer conversion/allocation");
  invalid=bev(); invalid.origin_x=std::numeric_limits<double>::quiet_NaN(); rejected=false;
  try { expired.projectToBev(invalid); } catch (const std::invalid_argument&) { rejected=true; }
  check(rejected,"nonfinite projection origin was accepted");
  std::cout << "surface evidence tests passed\n";
}
