#pragma once
#include <cmath>

namespace slam_mapping {
// REJECTED TEST-ONLY PROTOTYPE: sign alone cannot remove a wall's boundary halo.
// A rotated face crosses an unobserved cell-corner sliver that this rule frees.
// A vertical normal has no up/down sign. Classify its direction only when the
// trough is below the local plane beyond the existing two-sigma uncertainty.
// The 1 micrometre tolerance is numerical, not a new traversability threshold.
// Explicit deep below-plane evidence has a separate, unchanged guard.
inline bool wall_extends_below_ground(double trough_residual,double plane_sigma) {
  return std::isfinite(trough_residual) && std::isfinite(plane_sigma) && plane_sigma>=0 &&
         trough_residual+2*plane_sigma < -1e-6;
}
}
