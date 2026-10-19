#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <Eigen/Core>

namespace slam_mapping {
// Necessary, not sufficient: bits must denote actual consecutive 45-degree
// sectors about the origin, not arbitrary point indices. Two adjacent empty
// sectors force a >=90-degree gap even at their limiting boundary positions.
// Use only as a rejection before the unchanged actual-bearing certificate.
inline bool sector_gaps_can_support(unsigned sectors) {
  if (sectors&~255u) return false;
  const unsigned missing=(~sectors)&255u;
  return !(missing&((missing<<1)|(missing>>7)));
}
// Rotation-invariant sufficient angular support around a cell corner. Points
// are actual returns, or centers of observed regions with a conservative disk
// radius. The centers of nonzero-radius regions are never treated as returns.
// Every possible consecutive angular gap must be strictly less than 90 degrees,
// and every possible pair must satisfy the existing support diameter. Filling
// all eight fixed 45-degree sectors already implies the point-gap bound, but is
// unnecessarily axis-dependent. Neither certificate proves absence of all tiny
// unseen voids: this remains the mapper's bounded local-support assumption.
inline bool angular_support(const std::array<Eigen::Vector2d,8>& points,
                            const std::array<double,8>& radii,unsigned mask,double diameter) {
  if (!std::isfinite(diameter) || diameter<=0 || (mask&~255u)) return false;
  struct Bearing { double angle,error; };
  std::array<Bearing,8> bearings;
  std::array<int,8> indices;
  int count=0;
  for (int i=0;i<8;++i) if (mask&(1u<<i)) indices[count++]=i;
  // Four or fewer directions cannot have all circular gaps strictly below 90°.
  if (count<5) return false;
  for (int j=0;j<count;++j) {
    const int i=indices[j];
    const double distance=points[i].norm(),radius=radii[i];
    if (!points[i].allFinite() || !std::isfinite(radius) || radius<0 || distance<=radius) return false;
    bearings[j]={std::atan2(points[i].y(),points[i].x()),std::asin(radius/distance)};
  }
  std::sort(bearings.begin(),bearings.begin()+count,[](const auto& a,const auto& b){return a.angle<b.angle;});
  for (int j=0;j<count;++j) {
    const int next=(j+1)%count;
    const double gap=bearings[next].angle-bearings[j].angle+(next==0 ? 2*M_PI : 0.);
    if (gap+bearings[j].error+bearings[next].error>=M_PI/2.-1e-9) return false;
  }
  for (int a=0;a<count;++a) for (int b=a+1;b<count;++b) {
    const int i=indices[a],j=indices[b];
    if ((points[i]-points[j]).norm()+radii[i]+radii[j]>diameter+1e-9) return false;
  }
  return true;
}

}
