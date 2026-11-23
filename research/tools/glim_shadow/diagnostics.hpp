// Standalone diagnostic listeners. No changes to installed estimator or factors.
#pragma once
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <cstring>
#include <nlohmann/json.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <glim/common/imu_integration.hpp>
#include <glim/odometry/callbacks.hpp>

namespace shadow {
using Json=nlohmann::json;
inline Json matrix(const Eigen::Matrix4d& m) {
  Json result=Json::array();
  for(int i=0;i<4;++i) result.push_back({m(i,0),m(i,1),m(i,2),m(i,3)});
  return result;
}
inline void gravity_probe(const std::string& output) {
  glim::IMUIntegrationParams params;
  glim::IMUIntegration integration(params);
  for(int i=0;i<=60;++i) integration.insert_imu(1.+i/60.,Eigen::Vector3d(0,0,1.62),Eigen::Vector3d::Zero());
  int count=0;
  integration.integrate_imu(1.,2.,gtsam::imuBias::ConstantBias(),&count);
  auto predicted=integration.integrated_measurements().predict(gtsam::NavState(),gtsam::imuBias::ConstantBias());
  const auto gravity=integration.integrated_measurements().params()->n_gravity;
  Json result={{"configured_gravity",params.gravity_magnitude},{"gtsam_gravity",{gravity.x(),gravity.y(),gravity.z()}},
    {"stationary_lunar_1s_position_error_m",predicted.position().norm()},
    {"stationary_lunar_1s_velocity_error_m_s",predicted.velocity().norm()},
    {"samples_integrated",count},{"passed",std::abs(params.gravity_magnitude-1.62)<1e-12 && predicted.position().norm()<1e-8 && predicted.velocity().norm()<1e-8}};
  std::ofstream(output+"/runtime_gravity.json")<<result.dump(2);
  if(!result["passed"].get<bool>()) throw std::runtime_error("loaded GLIM lunar gravity probe failed");
}
class Trace {
  std::ofstream stream;
  std::mutex mutex;
  int before=-1, after=-1, factors=-1, input=-1, imu=-1;
  size_t imu_count=0;
  double imu_last=0;
  // Diagnostic byte fingerprint, not a cryptographic provenance hash.
  static void fingerprint(uint64_t& hash,const void* data,size_t size) {
    const auto* bytes=static_cast<const unsigned char*>(data);
    for(size_t i=0;i<size;++i) { hash^=bytes[i]; hash*=1099511628211ULL; }
  }
  void emit(const Json& value) { std::lock_guard<std::mutex> lock(mutex); stream<<value.dump()<<'\n'; }
  void pose(const char* stage,const glim::EstimationFrame::ConstPtr& f) {
    emit({{"stage",stage},{"frame_id",f->id},{"stamp_ns",int64_t(std::llround(f->stamp*1e9))},
      {"map_from_imu",matrix(f->T_world_imu.matrix())},
      {"velocity",{f->v_world_imu.x(),f->v_world_imu.y(),f->v_world_imu.z()}}});
  }
public:
  Trace(const std::string& output) {
    if(!std::getenv("GLIM_SHADOW_TRACE") || std::string(std::getenv("GLIM_SHADOW_TRACE"))!="1") return;
    stream.open(output+"/estimator_trace.jsonl");
    if(!stream) throw std::runtime_error("cannot create estimator trace");
    using C=glim::OdometryEstimationCallbacks;
    imu=C::on_insert_imu.add([this](double stamp,const auto&,const auto&){++imu_count;imu_last=stamp;});
    input=C::on_insert_frame.add([this](const auto& f){
      uint64_t xyz=14695981039346656037ULL, neighbors=xyz, times=xyz;
      for(const auto& p:f->points) fingerprint(xyz,p.data(),4*sizeof(double));
      for(const auto& n:f->neighbors) fingerprint(neighbors,&n,sizeof(n));
      for(const auto& t:f->times) fingerprint(times,&t,sizeof(t));
      emit({{"stage","processed_input"},{"stamp_ns",int64_t(std::llround(f->stamp*1e9))},
        {"points",f->points.size()},{"xyz_fnv1a",xyz},{"neighbors_fnv1a",neighbors},{"times_fnv1a",times},
        {"imu_count",imu_count},{"last_imu_stamp_ns",int64_t(std::llround(imu_last*1e9))}});
    });
    before=C::on_new_frame.add([this](const auto& f){pose("imu_prediction",f);});
    after=C::on_update_new_frame.add([this](const auto& f){pose("optimized",f);});
    // Register after GlimROS constructs its absolute-factor extension, so its
    // factors are included. Read only; do not insert/update values or factors.
    factors=C::on_smoother_update.add([this](auto&,auto& graph,auto& values,auto& stamps){
      Json poses=Json::array(), factors=Json::array();
      for(const auto& [key,time]:stamps) if(gtsam::Symbol(key).chr()=='x' && values.exists(key))
        poses.push_back({{"frame_id",gtsam::Symbol(key).index()},{"stamp_ns",int64_t(std::llround(time*1e9))},
          {"map_from_imu",matrix(values.template at<gtsam::Pose3>(key).matrix())}});
      for(const auto& factor:graph) {
        if(!factor) continue;
        Json f={{"type",typeid(*factor).name()},{"keys",factor->keys()}};
        if(auto n=dynamic_cast<const gtsam::NoiseModelFactor*>(factor.get())) {
          auto sig=n->noiseModel()->sigmas(); std::vector<double> s(sig.data(),sig.data()+sig.size()); f["sigmas"]=s;
        }
        if(auto p=dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(factor.get())) f["prior_pose"]=matrix(p->prior().matrix());
        if(auto p=dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factor.get())) f["between_pose"]=matrix(p->measured().matrix());
        if(auto p=dynamic_cast<const gtsam::GPSFactor*>(factor.get())) {
          auto m=p->measurementIn(); f["gps_position"]={m.x(),m.y(),m.z()};
        }
        factors.push_back(f);
      }
      emit({{"stage","pre_smoother"},{"poses",poses},{"factors",factors}});
    });
  }
  ~Trace() {
    using C=glim::OdometryEstimationCallbacks;
    if(before>=0) C::on_new_frame.remove(before);
    if(after>=0) C::on_update_new_frame.remove(after);
    if(factors>=0) C::on_smoother_update.remove(factors);
    if(input>=0) C::on_insert_frame.remove(input);
    if(imu>=0) C::on_insert_imu.remove(imu);
  }
};
}
