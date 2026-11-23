#include "slam_mapping/observation_moments.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
using namespace slam_mapping;
void check(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
int main() {
  // Exhaust every sequence of eight class/boundary events and every bucket
  // split. Compact merging must preserve the original full-count predicates.
  for (unsigned sequence=0;sequence<65536;++sequence) for (unsigned split=0;split<=8;++split) {
    NativeSurfaceEvidence left,right;
    unsigned ground_votes=0; bool veto=false,boundary=false;
    for (unsigned i=0;i<8;++i) {
      const unsigned event=(sequence>>(2*i))&3u;
      const unsigned kind=(event&1u) ? 0u : 2u;
      const bool edge=event&2u;
      (i<split ? left : right).add(kind,edge);
      ground_votes+=kind==2; veto|=kind!=2; boundary|=edge;
    }
    left.merge(right);
    check(left.only_ground()==(ground_votes>=2 && !veto),"compact ground predicate changed");
    check(left.boundary()==boundary,"compact merge lost boundary veto");
  }
  ObservationMoments empty,line,ground,steep,volume,translated;
  check(empty.normal_class(.35)==0,"empty observation supplied a normal");
  for (int x=-4;x<=4;++x) {
    line.add({x*.005,0,0});
    for (int y=-4;y<=4;++y) {
      const Eigen::Vector3d p(x*.005,y*.005,.2*x*.005);
      ground.add(p); translated.add(p+Eigen::Vector3d(21,-19,.4));
      steep.add({p.x(),p.y(),.8*p.x()});
      for (int z=-4;z<=4;++z) volume.add({x*.005,y*.005,z*.005});
    }
  }
  check(line.normal_class(.35)==0,"scan-ring line supplied a normal");
  check(ground.normal_class(.35)==2,"traversable inclined plane lost");
  check(steep.normal_class(.35)==3,"steep face became ground");
  check(volume.normal_class(.35)==1,"volume supplied a plane normal");
  check(translated.normal_class(.35)==2,"translation changed normal class");
  check((ground.scatter-translated.scatter).norm()<1e-10,"absolute gauge changed within scatter");
  ObservationMoments rotated;
  Eigen::Matrix3d rotation=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for (int x=-4;x<=4;++x) for (int y=-4;y<=4;++y)
    rotated.add(rotation*Eigen::Vector3d(x*.005,y*.005,.2*x*.005));
  check(rotated.normal_class(.35)==2,"yaw changed normal class");
  check((rotated.scatter-rotation*ground.scatter*rotation.transpose()).norm()<1e-10,"scatter is not equivariant");
  ObservationMoments merged;
  merged.merge(ground); merged.merge(translated);
  const Eigen::Vector3d delta=translated.mean-ground.mean;
  check((merged.scatter-ground.scatter-translated.scatter-
    delta*delta.transpose()*(ground.count*.5)).norm()<1e-9,"merged scatter lost between-observation variation");
  ObservationMoments flat,ground_line,raised_line,hole_line;
  for (int x=-20;x<=20;++x) for (int y=-20;y<=20;++y) flat.add({x*.01,y*.01,0});
  for (int x=-4;x<=4;++x) {
    ground_line.add({x*.005,0,0});
    raised_line.add({x*.005,0,.008});
    hole_line.add({x*.005,0,-.008});
  }
  check(ground_line.supported_by(flat,.35,.005),"local planar context failed to constrain a ground line");
  check(!raised_line.supported_by(flat,.35,.005),"above-plane line accepted by local context");
  check(!hole_line.supported_by(flat,.35,.005),"below-plane line accepted by local context");
  check(!ground_line.supported_by(line,.35,.005),"rank-deficient context invented a normal");
  check(!ground_line.supported_by(volume,.35,.005),"volumetric context invented a normal");
  check(!ground_line.supported_by(steep,.35,.005),"steep context certified ground");
  check(raised_line.centered_residual_bound(flat,.35)<1e-9,"constant offset invented within-surface spread");
  ObservationMoments partial_face;
  for (int y=-4;y<=4;++y) for (int z=0;z<6;++z) partial_face.add({0,y*.005,z*.004});
  check(partial_face.centered_residual_bound(flat,.35)>.005,"partial face lost its native height variation");
  check(!std::isfinite(ground_line.centered_residual_bound(line,.35)),"unresolved context acquired a residual certificate");
  check(ground.bounded_gradient_residual(.35)<1e-8,"traversable slope became native boundary");
  check(translated.bounded_gradient_residual(.35)<1e-7,"rigid gauge changed native boundary");
  check(raised_line.bounded_gradient_residual(.35)<1e-8,"uniform low top became native boundary");
  check(partial_face.bounded_gradient_residual(.35)>.005,"vertical face fitted as drivable height gradient");
  check(steep.bounded_gradient_residual(.35)>.005,"excessive slope bypassed the gradient cap");
#ifdef TERRAIN_PATCH_NORMAL_CACHE
  for (const auto* patch:{&empty,&line,&ground,&steep,&volume,&translated,&rotated,&flat,&partial_face}) {
    const auto normal=patch->ground_normal(.35);
    check(normal.valid==(patch->normal_class(.35)==2),"cached normal class differs");
    for (const auto* sample:{&empty,&line,&ground,&steep,&raised_line,&hole_line,&partial_face})
      for (double tolerance:{0.,.001,.005,.008,.1})
        check(sample->supported_by_cached(*patch,normal,tolerance)==sample->supported_by(*patch,.35,tolerance),
              "cached support decision differs");
  }
#endif
  std::cout << "observation moments contracts passed\n";
}
