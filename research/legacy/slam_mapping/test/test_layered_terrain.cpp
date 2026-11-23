#include "slam_mapping/layered_terrain.hpp"
#include "slam_mapping/angular_support.hpp"
#include "slam_mapping/terrain_diagnostics.hpp"
#include "terrain_boundary_fixture.hpp"
#include "angular_boundary_fixture.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace slam_mapping;
void check(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
int id(int x,int y) { return y*60+x; }
TerrainConfig config() { TerrainConfig c; c.width=c.height=60; c.origin_x=c.origin_y=-3; return c; }
std::vector<double> floor_points(double slope=0,int gap_width=0) {
  std::vector<double> points;
  for (int y=1;y<59;++y) for (int x=1;x<59;++x) {
    if (gap_width && x>=30 && x<30+gap_width) continue;
    double xx=-3+(x+.5)*.1,yy=-3+(y+.5)*.1;
    for (int k=0;k<3;++k) for (int j=0;j<3;++j) {
      points.push_back(xx+.005*k); points.push_back(yy+.005*j); points.push_back(slope*(xx+.005*k));
    }
  }
  return points;
}
int main() {
  check(!wall_extends_below_ground(.02,0),"upward face classified as dropoff");
  check(!wall_extends_below_ground(-1e-7,0),"numerical floor residual classified as dropoff");
  check(wall_extends_below_ground(-.002,0),"shallow resolved downward wall lost");
  check(!wall_extends_below_ground(-.002,.002),"uncertain wall direction treated as resolved");
  check(wall_extends_below_ground(-.02,.002),"resolved downward wall lost");
  // Rejection telemetry must distinguish too few directions, an actual angular
  // gap, and diameter failure; it never supplies a support certificate.
  std::array<Eigen::Vector2d,8> diagnostic_points;
  std::array<double,8> diagnostic_radii{};
  for (int i=0;i<6;++i) diagnostic_points[i]={.12*std::cos(i*M_PI/3),.12*std::sin(i*M_PI/3)};
  check(rejected_support_flags(diagnostic_points,diagnostic_radii,15,.30)&FEW_DIRECTIONS,"few-direction diagnostic lost");
  check(rejected_support_flags(diagnostic_points,diagnostic_radii,31,.30)&ANGULAR_GAP,"gap diagnostic lost");
  check(rejected_support_flags(diagnostic_points,diagnostic_radii,63,.30)==0,"supported spokes diagnosed as rejected");
  for (int i=0;i<6;++i) diagnostic_points[i]*=2;
  check(rejected_support_flags(diagnostic_points,diagnostic_radii,63,.30)==EXCESSIVE_SPAN,"span conflated with angular gap");
  // Reproduce the discarded-evidence case: six 60-degree spokes safely bound
  // the corner, but the legacy prefilter discards the two on the X axis.
  for (double radius: {0.,.005}) {
    BoundedAngularWitnesses witnesses,missing_wedge;
    std::array<Eigen::Vector2d,8> legacy;
    std::array<double,8> legacy_radii{};
    unsigned retained=0;
    for (int i=0;i<6;++i) {
      const double dx=.12*std::cos(i*M_PI/3),dy=.12*std::sin(i*M_PI/3);
      witnesses.add(dx,dy,radius,.30);
      if (i<5) missing_wedge.add(dx,dy,radius,.30);
      const double half=radius/std::sqrt(2.);
      if (std::abs(dx)<=half+1e-6 || std::abs(dy)<=half+1e-6 ||
          std::abs(std::abs(dx)-std::abs(dy))<=2*half+1e-6) continue;
      legacy[i]={dx,dy}; legacy_radii[i]=radius; retained|=1u<<i;
    }
    check(!angular_support(legacy,legacy_radii,retained,.30),"boundary-filter reproducer no longer fails");
    check(witnesses.certified(.30),"actual boundary-crossing support was discarded");
    check(!missing_wedge.certified(.30),"boundary fallback accepted a missing 120-degree wedge");
  }
  BoundedAngularWitnesses uncertain,too_wide,invalid_witnesses;
  for (int i=0;i<8;++i) {
    const double angle=i*M_PI/4;
    uncertain.add(.12*std::cos(angle),.12*std::sin(angle),.05,.30);
    too_wide.add(.2*std::cos(angle),.2*std::sin(angle),0.,.30);
    invalid_witnesses.add(.01*std::cos(angle),.01*std::sin(angle),.02,.30);
  }
  check(!uncertain.certified(.30),"region center treated as a return across uncertain sectors");
  check(!too_wide.certified(.30),"boundary fallback exceeded pairwise support span");
  check(!invalid_witnesses.certified(.30),"region containing the corner acquired a bearing");
  // Enumerate all masks, including the circular 7/0 boundary, and vary points
  // within their true sectors. Rejection must never discard an actual angular
  // certificate. The mask is only a cheap necessary condition, not evidence.
  check(!sector_gaps_can_support(256),"invalid sector mask accepted");
  for (unsigned mask=0;mask<256;++mask) {
    bool expected=true;
    for (int i=0;i<8;++i)
      if (!(mask&(1u<<i)) && !(mask&(1u<<((i+1)%8)))) expected=false;
    check(sector_gaps_can_support(mask)==expected,"circular empty-sector rejection differs");
    for (int pattern=0;pattern<256;++pattern) {
      std::array<Eigen::Vector2d,8> points;
      std::array<double,8> radii{};
      for (int i=0;i<8;++i) {
        const double angle=(i+((pattern&(1<<i)) ? .99999 : .00001))*M_PI/4;
        points[i]={.12*std::cos(angle),.12*std::sin(angle)};
      }
      if (!sector_gaps_can_support(mask))
        check(!angular_support(points,radii,mask,.30),"mask optimization removed a valid angular certificate");
    }
  }
  for (double phase: {0.,.13,.31,.77,1.1}) {
    std::array<Eigen::Vector2d,8> spokes;
    std::array<double,8> radii{};
    for (int i=0;i<6;++i) spokes[i]={.12*std::cos(phase+i*M_PI/3),.12*std::sin(phase+i*M_PI/3)};
    check(angular_support(spokes,radii,63,.30),"axis-independent six-spoke support rejected");
    check(!angular_support(spokes,radii,31,.30),"120-degree unsupported wedge accepted");
    radii.fill(.005);
    check(angular_support(spokes,radii,63,.30),"bounded observed XY regions rejected");
    radii.fill(.05);
    check(!angular_support(spokes,radii,63,.30),"uncertain region centers treated as exact returns");
    radii.fill(0.);
    for (int i=0;i<6;++i) spokes[i]*=2.;
    check(!angular_support(spokes,radii,63,.30),"angular support exceeded pairwise diameter");
    for (int i=0;i<4;++i) spokes[i]={.12*std::cos(phase+i*M_PI/2),.12*std::sin(phase+i*M_PI/2)};
    check(!angular_support(spokes,radii,15,.30),"90-degree missing wedge accepted");
  }
  auto points=floor_points(); LayeredTerrain flat(config());
  check(flat.integrate(points.data(),points.size()/3,1,{0,0,.5},0),"flat integration failed");
  auto out=flat.project(1);
  check(out.cost[id(35,35)]==0,"flat ground not free");
  check(!flat.integrate(points.data(),points.size()/3,1,{0,0,.5},0),"duplicate input accepted");
  for (int i=0;i<4;++i) { points.push_back(.54+.005*i); points.push_back(.55); points.push_back(.08); }
  LayeredTerrain rock(config()); rock.integrate(points.data(),points.size()/3,1,{0,0,.5},0);
  out=rock.project(1);
  check(out.cost[id(35,35)]==100,"single-ring 8cm rock averaged into floor");
  auto ground=floor_points(); rock.integrate(ground.data(),ground.size()/3,20,{0,0,.5},0);
  check(rock.project(20).cost[id(35,35)]==100,"later ground observations erased rock");
  out=rock.project(32);
  check(out.cost[id(35,35)]==0,"old evidence expiry discarded recent ground observations");
  rock.integrate(ground.data(),ground.size()/3,32,{0,0,.5},0);
  check(rock.project(32).cost[id(35,35)]==0,"expired rock did not clear on fresh ground");
  check(rock.project(63).cost[id(35,35)]==-1,"expired ground remained free");
  check(!rock.integrate(ground.data(),ground.size()/3,62,{0,0,.5},0),"integration rewound destructive expiry");
  bool rewind_rejected=false;
  try { rock.project(62); } catch (const std::invalid_argument&) { rewind_rejected=true; }
  check(rewind_rejected,"projection rewound destructive expiry");
  auto ramp_points=floor_points(.2); LayeredTerrain ramp(config());
  ramp.integrate(ramp_points.data(),ramp_points.size()/3,1,{0,0,.5},0);
  check(ramp.project(1).cost[id(35,35)]==0,"continuous ramp became an obstacle");
  auto sparse_points=floor_points(0,1); LayeredTerrain sparse(config());
  sparse.integrate(sparse_points.data(),sparse_points.size()/3,1,{0,0,.5},0);
  check(sparse.project(1).reason[id(30,35)]==UNSUPPORTED_EXTENT,
        "center-only gap brackets falsely certified whole-cell support");
  // The old chord-only fixture proves the center, not every output corner.
  // Add actual collinear floor returns: they still cannot be measured ground
  // on their own, but can support bounded inference together with the sides.
  std::vector<double> line_points;
  for (int y=0;y<300;++y) for (int x=0;x<300;++x) {
    const double xx=-2.99+x*.02,yy=-2.99+y*.02;
    if (xx>=0 && xx<.1) continue;
    line_points.insert(line_points.end(),{xx,yy,0.});
  }
  for (int k=0;k<2900;++k) {
    line_points.push_back(.05); line_points.push_back(-2.899+k*.002); line_points.push_back(0.);
  }
  LayeredTerrain line_gap(config());
  line_gap.integrate(line_points.data(),line_points.size()/3,1,{0,0,.5},0);
  check(line_gap.project(1).reason[id(30,35)]==INFERRED_GROUND,"supported line gap not inferred");
  auto cfg=config(); cfg.interpolate=false; LayeredTerrain measured(cfg);
  measured.integrate(sparse_points.data(),sparse_points.size()/3,1,{0,0,.5},0);
  check(measured.project(1).cost[id(30,35)]==-1,"measured-only ablation interpolated");
  auto gap_points=floor_points(0,6); LayeredTerrain gap(config());
  gap.integrate(gap_points.data(),gap_points.size()/3,1,{0,0,.5},0);
  check(gap.project(1).cost[id(32,35)]==-1,"wide unsupported gap filled");
  // A missing slab with observed vertical rim walls. Millimetric XY offsets
  // put wall returns inside the hole's cells; those returns are NOT drivable.
  auto hole_points=floor_points();
  std::vector<double> hole_cloud;
  for (size_t k=0;k<hole_points.size();k+=3)
    if (!(hole_points[k]>=.5 && hole_points[k]<1.1 && hole_points[k+1]>=.5 && hole_points[k+1]<1.1))
      hole_cloud.insert(hole_cloud.end(),hole_points.begin()+k,hole_points.begin()+k+3);
  const auto hole_floor=hole_cloud;
  for (double depth_step:{.005,.020}) {
  hole_cloud=hole_floor;
  for (int depth=0;depth<4;++depth) for (int k=0;k<60;++k) {
    const double z=-.005-depth*depth_step;
    const double at=.501+k*.01;
    hole_cloud.insert(hole_cloud.end(),{.501,at,z,1.099,at,z,at,.501,z,at,1.099,z});
  }
  LayeredTerrain hole(config()); hole.integrate(hole_cloud.data(),hole_cloud.size()/3,1,{0,0,.5},0);
  const auto hole_out=hole.project(1);
  for (int y=35;y<41;++y) for (int x=35;x<41;++x)
    check(hole_out.cost[id(x,y)]<0 || hole_out.cost[id(x,y)]==100,"vertical hole rim became free");
  check(hole_out.cost[id(25,25)]==0,"hole guard erased distant flat ground");
  }
  auto thin=floor_points();
  std::vector<double> thin_cloud;
  for (size_t k=0;k<thin.size();k+=3)
    if (!(thin[k]>.5 && thin[k]<.6 && thin[k+1]>.5 && thin[k+1]<.6))
      thin_cloud.insert(thin_cloud.end(),thin.begin()+k,thin.begin()+k+3);
  for (int k=0;k<10;++k) thin_cloud.insert(thin_cloud.end(),{.505+k*.009,.55,0.});
  LayeredTerrain line_only(cfg); line_only.integrate(thin_cloud.data(),thin_cloud.size()/3,1,{0,0,.5},0);
  check(line_only.project(1).reason[id(35,35)]==INSUFFICIENT_AREA,"line returns certified a measured ground patch");
  std::vector<double> corner_cloud;
  for (size_t k=0;k<thin.size();k+=3)
    if (thin[k]<.5 || thin[k+1]<.5)
      corner_cloud.insert(corner_cloud.end(),thin.begin()+k,thin.begin()+k+3);
  LayeredTerrain corner(config()); corner.integrate(corner_cloud.data(),corner_cloud.size()/3,1,{0,0,.5},0);
  check(corner.project(1).cost[id(35,35)]==-1,"one diagonal chord filled an unsupported corner");
  // Identical repeated hits are not independent spatial evidence.
  auto duplicate=floor_points();
  for (int k=0;k<30;++k) duplicate.insert(duplicate.end(),{.55,.55,.08});
  LayeredTerrain repeat(config()); repeat.integrate(duplicate.data(),duplicate.size()/3,1,{0,0,.5},0);
  check(repeat.project(1).reason[id(35,35)]==UNCERTAIN_RAISED,"duplicate returns promoted to a cluster");
  repeat.integrate(duplicate.data(),duplicate.size()/3,2,{0,0,.5},0);
  check(repeat.project(2).reason[id(35,35)]==UNCERTAIN_RAISED,"stationary repetition promoted to a cluster");
  repeat.integrate(duplicate.data(),duplicate.size()/3,3,{.2,0,.5},0);
  check(repeat.project(3).reason[id(35,35)]==OBSTACLE,"independent view support discarded");
  // A nearby pit must not become the robot's support surface simply because
  // its bottom is the lowest observed plane.
  for (double offset: {-.4,.4}) {
    auto raised=floor_points();
    for (size_t k=0;k<raised.size();k+=3)
      if (raised[k]>.9 && raised[k]<2.3 && std::abs(raised[k+1])<.7) raised[k+2]+=offset;
    LayeredTerrain step(config()); step.integrate(raised.data(),raised.size()/3,1,{0,0,.5},0);
    check(step.project(1).cost[id(46,30)]!=0,"disconnected pit/plateau classified as safe floor");
    check(step.project(1).cost[id(25,25)]==0,"pit/plateau stole local ground anchor");
  }
  auto overflow=floor_points();
  for (double z: {.1,.2,.3}) overflow.insert(overflow.end(),{.55,.55,z});
  LayeredTerrain capacity(config()); capacity.integrate(overflow.data(),overflow.size()/3,1,{0,0,.5},0);
  check(capacity.project(1).reason[id(35,35)]==LAYER_CAPACITY,"layer capacity silently merged geometry");
  for (double z: {.12,.16,.22}) for (int k=0;k<3;++k)
    overflow.insert(overflow.end(),{.54+.005*k,.55,z});
  LayeredTerrain face(config()); face.integrate(overflow.data(),overflow.size()/3,1,{0,0,.5},0);
  check(face.project(1).cost[id(35,35)]==100,"vertical-face layer overflow erased known obstacle");
  auto invalid_cfg=config(); invalid_cfg.layer_band=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;
  try { LayeredTerrain invalid_map(invalid_cfg); } catch (const std::invalid_argument&) { rejected=true; }
  check(rejected,"NaN parameter accepted");
  std::cout<<"layered terrain contracts passed\n";
}
