// Actual candidate target storage/reconstruction contracts. No simulator or GT.
#include <stdexcept>
#include <iostream>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/util/config.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim {
struct TargetReconstructionTest {
  static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  }
  static void run() {
    OdometryEstimationCPUParams params;
    params.revise_target_poses = true;
    params.registration_type = "GICP";
    params.lru_thresh = 2;
    params.target_downsampling_rate = 1.;
    OdometryEstimationCPU estimator(params);
    auto add = [&](int id, double x) {
      auto frame = std::make_shared<EstimationFrame>();
      frame->id = id;
      frame->T_world_imu = Eigen::Isometry3d::Identity();
      std::vector<Eigen::Vector4d> points = {Eigen::Vector4d(x, 0., 0., 1.)};
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(points);
      estimator.frames.push_back(frame);
      estimator.update_target(id, frame->T_world_imu);
      return frame;
    };
    auto first = add(0, 1.);
    auto second = add(1, 10.);
    require(estimator.target_ivox->voxel_points().empty(), "target inserted before fusion");
    first->T_world_imu.translation() = Eigen::Vector3d(2., 3., 4.);
    first->T_world_imu.linear() = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    estimator.rebuild_target();
    auto points = estimator.target_ivox->voxel_points();
    auto contains = [&](const Eigen::Vector3d& expected) {
      for (const auto& p : points) if ((p.head<3>()-expected).norm()<1e-8) return true;
      return false;
    };
    require(points.size()==2 && contains(Eigen::Vector3d(2.,4.,4.)) && contains(Eigen::Vector3d(10.,0.,0.)),
            "rebuild did not apply revised full SE3 pose");
    require((first->frame->points[0]-Eigen::Vector4d(1.,0.,0.,1.)).norm()==0., "sensor evidence was mutated");
    // Repeated reconstruction cannot double-transform or accumulate stale copies.
    estimator.rebuild_target();
    points = estimator.target_ivox->voxel_points();
    require(points.size()==2 && contains(Eigen::Vector3d(2.,4.,4.)), "rebuild is not idempotent");
    add(2, 20.);
    estimator.rebuild_target();
    points = estimator.target_ivox->voxel_points();
    require(estimator.target_observations.size()==2 && points.size()==2 && !contains(Eigen::Vector3d(2.,4.,4.)),
            "bounded observation expiry failed");
    // A surviving observation keeps its last pose after the smoother releases it.
    estimator.frames[1].reset();
    second.reset();
    estimator.rebuild_target();
    points = estimator.target_ivox->voxel_points();
    require(points.size()==2 && contains(Eigen::Vector3d(10.,0.,0.)), "marginalization invalidated evidence");
    params.lru_thresh=0;
    bool rejected=false;
    try { OdometryEstimationCPU invalid(params); } catch (const std::invalid_argument&) { rejected=true; }
    require(rejected, "unbounded target accepted");
  }
};
}

int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    glim::GlobalConfig::instance(argv[1]);
    glim::TargetReconstructionTest::run();
    std::cout << "Target reconstruction contracts passed\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
