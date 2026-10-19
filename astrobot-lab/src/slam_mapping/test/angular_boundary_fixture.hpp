#pragma once
#include "slam_mapping/angular_support.hpp"
#include <limits>

// Test-only prototype. Native evaluation showed no final reference-coverage
// benefit and a runtime regression; the active mapper does not use this pool.
namespace slam_mapping {
// Supplemental witnesses may lie on/cross fixed-sector boundaries. Retain
// nearest bounded regions by center bearing, but never accept their mask as a
// certificate: every angular gap includes the region's full uncertainty disk.
// Used only after the legacy certificate fails, so it cannot replace a known
// legacy witness with a worse candidate or regress an existing certificate.
class BoundedAngularWitnesses {
 public:
  BoundedAngularWitnesses() { distances_.fill(std::numeric_limits<double>::infinity()); }
  void add(double dx,double dy,double radius,double diameter) {
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(radius) || radius<0 ||
        !std::isfinite(diameter) || diameter<=0) return;
    const double distance=std::hypot(dx,dy);
    if (distance<=radius || distance+radius>diameter+1e-9) return;
    const unsigned sector=dy>0 ? (dx>0 ? (dy<dx?0:1) : (dy>-dx?2:3))
                              : (dx<0 ? (-dy<-dx?4:5) : (-dy>dx?6:7));
    if (distance+radius>=distances_[sector]) return;
    points_[sector]={dx,dy}; radii_[sector]=radius; distances_[sector]=distance+radius;
    mask_ |= 1u<<sector;
  }
  bool certified(double diameter) const {
    // Empty center sectors already fail the conservative center-gap bound;
    // adding nonnegative angular uncertainty cannot turn that into a pass.
    return sector_gaps_can_support(mask_) && angular_support(points_,radii_,mask_,diameter);
  }
 private:
  std::array<Eigen::Vector2d,8> points_;
  std::array<double,8> radii_{},distances_;
  unsigned mask_{0};
};
}
