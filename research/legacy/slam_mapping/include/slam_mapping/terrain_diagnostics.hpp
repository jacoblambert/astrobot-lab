#pragma once
#include "slam_mapping/angular_support.hpp"
#include <vector>
#include <cstdint>

namespace slam_mapping {
// Diagnostic-only flags. Never used to decide a map label or retain evidence.
enum CornerDiagnostic : uint32_t {
  CORNER_NEEDED=1, NO_ANCHOR=2, ADJACENT_CERTIFIED=4, WITNESS_CERTIFIED=8,
  FEW_DIRECTIONS=16, ANGULAR_GAP=32, EXCESSIVE_SPAN=64
};
inline uint32_t rejected_support_flags(const std::array<Eigen::Vector2d,8>& points,
                                      const std::array<double,8>& radii,unsigned mask,double diameter) {
  int count=0;
  for (int i=0;i<8;++i) if (mask&(1u<<i)) ++count;
  uint32_t flags=count<5 ? uint32_t(FEW_DIRECTIONS) : 0u;
  // Twice the bounded search radius admits every pair. This counterfactual
  // distinguishes angular uncertainty from span, without changing the decision.
  if (count>=5 && mask!=255 && !angular_support(points,radii,mask,2*diameter+1.)) flags|=ANGULAR_GAP;
  for (int i=0;i<8;++i) if (mask&(1u<<i))
    for (int j=i+1;j<8;++j) if (mask&(1u<<j))
      if ((points[i]-points[j]).norm()+radii[i]+radii[j]>diameter+1e-9) flags|=EXCESSIVE_SPAN;
  return flags;
}
struct TerrainDiagnostics {
  // Cells: source edge bits (1=below plane,2=wall normal,4=raised obstacle,
  // 8=native boundary in the opt-in two-path experiment),
  // propagated edge bits, rejected corner flags, plane-disagreement corner mask.
  std::vector<std::array<uint32_t,4>> cells;
  // Vertices: status flags, selected sector mask, boundary rejects, range rejects,
  // point-height rejects, bucket-height rejects, considered witnesses, reserved.
  std::vector<std::array<uint32_t,8>> corners;
#ifdef TERRAIN_OBSERVATIONS
  // Pooled-wall layers only: source count; observations by class 0..3;
  // total/within/between scatter trace; all-observation-ground source count.
  // Additional channels describe all active layer observation classes, 0..3.
  // 13..16: source observation classes after bounded spatial context;
  // 17: source layers with only resolved ground context (>=2 observations);
  // 18..21: context classes for all layers with a valid local ground model.
  std::vector<std::array<double,22>> observations;
#endif
};
}
