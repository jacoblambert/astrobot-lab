#include "slam_mapping/pose_history_se3.hpp"
#include <stdexcept>
void check(bool ok) { if (!ok) throw std::runtime_error("SE3 history contract failed"); }
int main() {
  slam_mapping::PoseHistorySE3 h;
  auto a=Eigen::Isometry3d::Identity(), b=a;
  b.translation()<<.1,.2,.3;
  b.linear()=(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitY())).toRotationMatrix();
  h.insert(1.1,b); h.insert(1,a);
  auto mid=h.at(1.05); check(mid.has_value());
  check(mid->translation().isApprox(b.translation()*.5));
  check(mid->linear().isApprox(Eigen::Quaterniond(a.linear()).slerp(.5,Eigen::Quaterniond(b.linear())).toRotationMatrix()));
  auto extrinsic=a; extrinsic.translation()<<.264,.017,-.262;
  extrinsic.linear()=Eigen::AngleAxisd(3.141592653589793,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Isometry3d map_base=b*extrinsic;
  const Eigen::Isometry3d map_odom=map_base*mid->inverse();
  check((map_odom*(*mid)).isApprox(map_base));
  check(!h.at(.9)); check(!h.at(1.2)); check(!h.at(1.05,.01));
  h.insert(1.1,a); check(h.at(1.1)->isApprox(a));
  h.clear(); check(!h.at(1));
}
