#include <iostream>
#include "absolute_schedule.hpp"

struct Capture : glim::GenericTopicSubscription {
  std::vector<std::vector<uint8_t>> bytes;
  Capture():GenericTopicSubscription("/astrobot_0/localization/absolute_pose") {}
  void create_subscriber(rclcpp::Node&) override {}
  void insert_message_instance(const rclcpp::SerializedMessage& data,const std::string&) override {
    const auto& raw=data.get_rcl_serialized_message();
    bytes.emplace_back(raw.buffer,raw.buffer+raw.buffer_length);
  }
};

int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    using Pose=geometry_msgs::msg::PoseWithCovarianceStamped;
    rosbag2_cpp::Reader reader; reader.open(argv[1]);
    std::vector<int64_t> stamps;
    auto reference=std::make_shared<Capture>();
    while(reader.has_next()) {
      auto msg=reader.read_next();
      if(msg->topic_name!=reference->topic) continue;
      rclcpp::SerializedMessage data(*msg->serialized_data);
      Pose pose; rclcpp::Serialization<Pose>().deserialize_message(&data,&pose);
      stamps.push_back(int64_t(pose.header.stamp.sec)*1000000000+pose.header.stamp.nanosec);
      reference->insert_message_instance(data,"");
    }
    if(stamps.empty()) throw std::runtime_error("fixture has no absolute inputs");
    shadow::AbsoluteSchedule schedule(argv[1],true);
    auto output=std::make_shared<Capture>(); schedule.set_consumer(output);
    for(size_t i=0;i<stamps.size();++i) {
      schedule.dispatch_until(stamps[i]-1);
      if(output->bytes.size()!=i) throw std::runtime_error("future fix dispatched early");
      schedule.dispatch_until(stamps[i]);
      schedule.dispatch_until(stamps[i]);
      if(output->bytes.size()!=i+1) throw std::runtime_error("lost or duplicate fix");
    }
    if(schedule.finish()!=stamps.size() || output->bytes!=reference->bytes)
      throw std::runtime_error("scheduled CDR changed");
    bool reversed=false;
    try { schedule.dispatch_until(stamps.front()); } catch(const std::runtime_error&) { reversed=true; }
    if(!reversed) throw std::runtime_error("reversing cutoff accepted");
    std::cout<<"Absolute schedule contracts passed: "<<stamps.size()<<" exact CDR messages\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
