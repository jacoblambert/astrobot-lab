// Directly dispatch recorded CDR to the SAME installed GLIM callbacks. No GT,
// sensor regeneration, message mutation, DDS input loss, or estimator changes.
#include <chrono>
#include <fstream>
#include <future>
#include <thread>
#include <filesystem>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <astrobot_interfaces/msg/slam_revision.hpp>
#include <glim_ros/glim_ros.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include "diagnostics.hpp"
#include "absolute_schedule.hpp"

using json = nlohmann::json;
using Steady = std::chrono::steady_clock;
using Revision = astrobot_interfaces::msg::SlamRevision;
int64_t ns(const builtin_interfaces::msg::Time& t) { return int64_t(t.sec)*1000000000+t.nanosec; }
template<class T> std::shared_ptr<T> decode(const rclcpp::SerializedMessage& data) {
  auto result=std::make_shared<T>(); rclcpp::Serialization<T>().deserialize_message(&data,result.get()); return result;
}
json matrix(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-6) throw std::runtime_error("invalid pose quaternion");
  Eigen::Matrix4d m=Eigen::Matrix4d::Identity(); m.block<3,3>(0,0)=q.toRotationMatrix();
  m.block<3,1>(0,3)=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  if (!m.allFinite()) throw std::runtime_error("nonfinite pose");
  json result=json::array(); for(int i=0;i<4;++i) result.push_back({m(i,0),m(i,1),m(i,2),m(i,3)}); return result;
}
int main(int argc,char** argv) {
  if(argc<4) return 2; // bag, output, legacy|physics|lidar_only, then ROS options
  const std::string bag=argv[1], output=argv[2], mode=argv[3];
  if(mode!="legacy" && mode!="physics" && mode!="lidar_only") return 2;
  const bool lidar_only=mode=="lidar_only";
  if(std::filesystem::exists(output+"/trajectory.jsonl")) return 2;
  std::ofstream trajectory(output+"/trajectory.jsonl");
  if(!trajectory) return 2;
  rclcpp::init(argc,argv);
  try {
    const bool scheduled=std::getenv("GLIM_SHADOW_ABSOLUTE_SCHEDULE") &&
      std::string(std::getenv("GLIM_SHADOW_ABSOLUTE_SCHEDULE"))=="measurement";
    if(scheduled && lidar_only) throw std::runtime_error("absolute schedule requires assisted estimator");
    shadow::AbsoluteSchedule absolute_schedule(bag,scheduled);
    auto glim=std::make_shared<glim::GlimROS>(rclcpp::NodeOptions());
    if(!lidar_only) shadow::gravity_probe(output);
    shadow::Trace trace(output);
    auto recorder=std::make_shared<rclcpp::Node>("offline_revision_recorder");
    int64_t clock=0, last_arrival=0; size_t revisions=0;
    auto sub=recorder->create_subscription<Revision>("/astrobot_0/slam_raw/revisions",
      rclcpp::QoS(1000).reliable().transient_local(),[&](const Revision& msg){
        if(msg.header.frame_id!="map") throw std::runtime_error("unexpected revision frame");
        json frames=json::array(); int64_t maximum=0;
        for(const auto& f:msg.frames) {
          maximum=std::max(maximum,ns(f.measurement_stamp));
          frames.push_back({{"frame_id",f.frame_id},{"stamp_ns",ns(f.measurement_stamp)},
            {"map_from_imu",matrix(f.map_from_imu)},{"imu_from_lidar",matrix(f.imu_from_lidar)},
            {"deskew_recomputed",f.deskew_recomputed}});
        }
        last_arrival=std::max({last_arrival,clock,maximum});
        trajectory<<json({{"session",msg.session_id},{"epoch",msg.localization_epoch},
          {"revision",msg.revision},{"parent",msg.parent_revision},{"snapshot",msg.full_snapshot},
          {"arrival_ns",last_arrival},{"frames",frames}}).dump()<<'\n';
        ++revisions;
      });
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(glim); exec.add_node(recorder);
    // Discovery only; no measurements are dispatched before output subscription.
    const auto discovery=Steady::now()+std::chrono::seconds(1);
    while(Steady::now()<discovery) { exec.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    rosbag2_cpp::Reader reader; reader.open(bag);
    const std::string imu="/astrobot_0/"+std::string(mode=="legacy"?"imu":"imu_physics");
    const std::string points="/astrobot_0/lidar_0/pointcloud/filtered", absolute="/astrobot_0/localization/absolute_pose";
    auto extensions=glim->extension_subscriptions();
    std::vector<glim::GenericTopicSubscription::Ptr> absolute_subs;
    for(auto& s:extensions) if(s->topic==absolute) absolute_subs.push_back(s);
    if(absolute_subs.size()!=(lidar_only?0:1)) throw std::runtime_error("unexpected absolute-input consumer count");
    if(scheduled) absolute_schedule.set_consumer(absolute_subs[0]);
    json counts=json::object(); int64_t first_receipt=0, previous=0; const auto start=Steady::now();
    while(reader.has_next()) {
      auto msg=reader.read_next(); const auto receipt=msg->recv_timestamp;
      if(previous && receipt<previous) throw std::runtime_error("recorded ordering reversed"); previous=receipt;
      if(!first_receipt) first_receipt=receipt;
      // Fixed recorded wall-receipt pace in BOTH arms, not fastest-possible input.
      const auto deadline=start+std::chrono::nanoseconds(receipt-first_receipt);
      while(Steady::now()<deadline) { exec.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
      const rclcpp::SerializedMessage data(*msg->serialized_data);
      if(!lidar_only && msg->topic_name==imu) glim->imu_callback(decode<sensor_msgs::msg::Imu>(data));
      else if(msg->topic_name==points) glim->points_callback(decode<sensor_msgs::msg::PointCloud2>(data));
      else if(msg->topic_name==absolute) {
        if(!lidar_only && !scheduled) absolute_subs[0]->insert_message_instance(data,"geometry_msgs/msg/PoseWithCovarianceStamped");
      }
      else if(msg->topic_name=="/clock") {
        auto next=ns(decode<rosgraph_msgs::msg::Clock>(data)->clock);
        if(next<clock) throw std::runtime_error("replay clock reversed"); clock=next;
      } else if(msg->topic_name!="/astrobot_0/imu" && msg->topic_name!="/astrobot_0/imu_physics")
        throw std::runtime_error("undeclared replay input");
      if((!lidar_only && (msg->topic_name==imu || msg->topic_name==absolute)) || msg->topic_name==points || msg->topic_name=="/clock")
        counts[msg->topic_name]=counts.value(msg->topic_name,0)+1;
      glim->timer_callback(); exec.spin_some();
      if(!rclcpp::ok()) throw std::runtime_error("interrupted replay");
    }
    auto joined=std::async(std::launch::async,[&](){glim->wait(true);});
    while(joined.wait_for(std::chrono::milliseconds(2))!=std::future_status::ready) exec.spin_some();
    joined.get();
    if(scheduled && absolute_schedule.finish()!=counts[absolute].get<size_t>())
      throw std::runtime_error("scheduled absolute dispatch count mismatch");
    { std::ifstream maps("/proc/self/maps"); std::ofstream saved(output+"/loaded_maps.txt"); saved<<maps.rdbuf(); }
    const auto tail=Steady::now()+std::chrono::seconds(1);
    while(Steady::now()<tail) { exec.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    trajectory.flush();
    if(!trajectory || !revisions) throw std::runtime_error("missing revision output");
    std::ofstream report(output+"/dispatch.json");
    report<<json({{"completed",true},{"counts",counts},{"revisions",revisions},
      {"mode",mode},{"wall_seconds",std::chrono::duration<double>(Steady::now()-start).count()},
      {"input_dispatch",scheduled?"original CDR; clouds/IMU receipt order; RTK measurement-time snapshot":"direct installed callbacks; original CDR and receipt order"},
      {"absolute_schedule",scheduled?"measurement-time snapshot before each solve; offline non-causal":"recorded receipt"},
      {"causal_timing_acceptance",false},{"gt_published",false}}).dump(2);
    exec.remove_node(glim); exec.remove_node(recorder); glim.reset(); rclcpp::shutdown(); return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<std::endl; rclcpp::shutdown(); return 1; }
}
