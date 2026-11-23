#pragma once
#include <algorithm>
#include <Eigen/Eigenvalues>
#include <cstdint>
#include <limits>

namespace slam_mapping {
// Sufficient statistics for the two-path decision, not diagnostic counts.
// Fits one padding byte in a time bucket; saturation preserves >=2 exactly.
struct NativeSurfaceEvidence {
  uint8_t bits{0};
  void add(unsigned kind,bool boundary) {
    if (kind==2) bits=(bits&~3u)|std::min(2u,unsigned(bits&3u)+1u);
    else bits|=4u;
    if (boundary) bits|=8u;
  }
  void merge(const NativeSurfaceEvidence& other) {
    bits=((bits|other.bits)&12u)|std::min(2u,unsigned(bits&3u)+unsigned(other.bits&3u));
  }
  bool boundary() const { return bits&8u; }
  bool only_ground() const { return (bits&7u)==2u; }
};
static_assert(sizeof(NativeSurfaceEvidence)==1,"native evidence must fit bucket padding");
// One rigid measurement only. Cross-time displacement must never enter this
// scatter. Classification is geometric evidence, not a calibrated covariance.
struct ObservationMoments {
  Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d scatter{Eigen::Matrix3d::Zero()};
  uint32_t count{0};
  void add(const Eigen::Vector3d& point) {
    const Eigen::Vector3d delta=point-mean;
    mean+=delta/double(++count);
    scatter+=delta*(point-mean).transpose();
  }
  void merge(const ObservationMoments& other) {
    if (!other.count) return;
    if (!count) { *this=other; return; }
    const Eigen::Vector3d delta=other.mean-mean;
    scatter+=other.scatter+delta*delta.transpose()*(double(count)*other.count/(double(count)+other.count));
    mean+=delta*(double(other.count)/(double(count)+other.count));
    count+=other.count;
  }
  // Spatial orientation evidence, never a whole-cell clearance certificate.
  // Require a well-resolved ground plane and
  // a conservative bound on EVERY return in the center observation. sqrt of
  // unnormalized residual scatter bounds each centered residual; unlike a
  // standard-deviation multiple, this never discards an inconvenient outlier.
  bool supported_by(const ObservationMoments& patch,double max_slope,double tolerance) const {
    if (!count || patch.normal_class(max_slope)!=2) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(patch.scatter/double(patch.count));
    const Eigen::Vector3d normal=eig.eigenvectors().col(0);
    const double bound=std::abs((mean-patch.mean).dot(normal))+
      std::sqrt(std::max(0.,normal.dot(scatter*normal)));
    return bound<=tolerance;
  }
#ifdef TERRAIN_PATCH_NORMAL_CACHE
  struct GroundNormal {
    Eigen::Vector3d value{Eigen::Vector3d::Zero()};
    bool valid{false};
  };
  // Same eigensolve and predicates as normal_class + supported_by. Compute
  // once per ordered native patch, never reuse across observations/revisions.
  GroundNormal ground_normal(double max_slope) const {
    if (count<3) return {};
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter/double(count));
    if (eig.info()!=Eigen::Success || eig.eigenvalues()[1]<=1e-6 ||
        eig.eigenvalues()[0]>.1*eig.eigenvalues()[1] ||
        !(std::abs(eig.eigenvectors()(2,0))>=1./std::sqrt(1.+max_slope*max_slope))) return {};
    return {eig.eigenvectors().col(0),true};
  }
  bool supported_by_cached(const ObservationMoments& patch,const GroundNormal& normal,double tolerance) const {
    if (!count || !normal.valid) return false;
    const double bound=std::abs((mean-patch.mean).dot(normal.value))+
      std::sqrt(std::max(0.,normal.value.dot(scatter*normal.value)));
    return bound<=tolerance;
  }
#endif
  double centered_residual_bound(const ObservationMoments& patch,double max_slope) const {
    if (!count || patch.normal_class(max_slope)!=2) return std::numeric_limits<double>::infinity();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(patch.scatter/double(patch.count));
    const Eigen::Vector3d normal=eig.eigenvectors().col(0);
    return std::sqrt(std::max(0.,normal.dot(scatter*normal)));
  }
  // Independent native boundary evidence: residual from a locally fitted
  // height gradient capped at the retained traversable slope. Uniform Z gauge
  // and offset from a curved neighborhood do not enter this centered residual.
  // A line can have low residual, but still does NOT supply a surface normal.
  Eigen::Vector3d bounded_gradient_direction(double max_slope) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(scatter.topLeftCorner<2,2>());
    if (eig.info()!=Eigen::Success) return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const Eigen::Vector2d xz=scatter.block<2,1>(0,2);
    Eigen::Vector2d gradient=Eigen::Vector2d::Zero();
    for (int k=0;k<2;++k) if (eig.eigenvalues()[k]>1e-6*count) {
      const Eigen::Vector2d direction=eig.eigenvectors().col(k);
      gradient+=direction*(direction.dot(xz)/eig.eigenvalues()[k]);
    }
    if (gradient.norm()>max_slope) gradient*=max_slope/gradient.norm();
    return {-gradient.x(),-gradient.y(),1.};
  }
  double bounded_gradient_residual(double max_slope) const {
    if (!count) return 0.;
    const Eigen::Vector3d residual=bounded_gradient_direction(max_slope);
    if (!residual.allFinite()) return std::numeric_limits<double>::infinity();
    return std::sqrt(std::max(0.,residual.dot(scatter*residual)));
  }
  // 0=rank deficient, 1=nonplanar, 2=traversable normal, 3=steep normal.
  unsigned normal_class(double max_slope) const {
    if (count<3) return 0;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter/double(count));
    if (eig.info()!=Eigen::Success || eig.eigenvalues()[1]<=1e-6) return 0;
    if (eig.eigenvalues()[0]>.1*eig.eigenvalues()[1]) return 1;
    return std::abs(eig.eigenvectors()(2,0))>=1./std::sqrt(1.+max_slope*max_slope) ? 2 : 3;
  }
};
}
