#pragma once
#include <deque>
#include <gtsam/inference/Symbol.h>
#include <glim/odometry/callbacks.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <glim/util/extension_module_ros2.hpp>

namespace shadow {
// Explicit non-causal diagnostic: availability is frozen to measurement time,
// not wall/receipt timing. No new measurements, fits, covariance edits or GT.
class AbsoluteSchedule {
  struct Sample { int64_t stamp; rclcpp::SerializedMessage data; };
  std::deque<Sample> pending;
  glim::GenericTopicSubscription::Ptr consumer;
  int callback=-1;
  size_t delivered=0;
  int64_t previous_cutoff=-1;
public:
  AbsoluteSchedule(const std::string& bag, bool enabled) {
    if (!enabled) return;
    rosbag2_cpp::Reader reader; reader.open(bag);
    int64_t previous=-1;
    while(reader.has_next()) {
      auto msg=reader.read_next();
      if(msg->topic_name!="/astrobot_0/localization/absolute_pose") continue;
      rclcpp::SerializedMessage data(*msg->serialized_data);
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      rclcpp::Serialization<decltype(pose)>().deserialize_message(&data,&pose);
      int64_t stamp=int64_t(pose.header.stamp.sec)*1000000000+pose.header.stamp.nanosec;
      if(stamp<=previous) throw std::runtime_error("scheduled absolute stamps not strictly increasing");
      previous=stamp; pending.push_back({stamp,std::move(data)});
    }
    if(pending.empty()) throw std::runtime_error("missing scheduled absolute inputs");
    // Construct before GlimROS registers the actual factor-building extension.
    callback=glim::OdometryEstimationCallbacks::on_smoother_update.add(
      [this](auto&,auto&,auto&,auto& stamps) {
        int64_t cutoff=0;
        for(const auto& [key,time]:stamps) if(gtsam::Symbol(key).chr()=='x')
          cutoff=std::max(cutoff,int64_t(std::llround(time*1e9)));
        dispatch_until(cutoff);
      });
  }
  bool enabled() const { return callback>=0; }
  void set_consumer(glim::GenericTopicSubscription::Ptr value) { consumer=std::move(value); }
  void dispatch_until(int64_t cutoff) {
    if(!consumer) throw std::runtime_error("absolute schedule has no consumer");
    if(cutoff<previous_cutoff) throw std::runtime_error("absolute snapshot cutoff reversed");
    previous_cutoff=cutoff;
    while(!pending.empty() && pending.front().stamp<=cutoff) {
      consumer->insert_message_instance(pending.front().data,"geometry_msgs/msg/PoseWithCovarianceStamped");
      pending.pop_front(); ++delivered;
    }
  }
  size_t finish() { dispatch_until(std::numeric_limits<int64_t>::max()); return delivered; }
  ~AbsoluteSchedule() { if(callback>=0) glim::OdometryEstimationCallbacks::on_smoother_update.remove(callback); }
};
}
