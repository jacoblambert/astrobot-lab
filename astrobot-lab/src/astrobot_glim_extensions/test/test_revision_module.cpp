// Copyright 2026 Jacob Lambert
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Exercise the real callback adapter without a simulator or estimator worker.
#include "../src/revision_module.cpp"  // NOLINT(build/include)
#include <stdexcept>
#include <gtsam_points/types/point_cloud_cpu.hpp>
void check(bool ok, const char * what) {if (!ok) {throw std::runtime_error(what);}}
Eigen::Isometry3d at(double x)
{
  auto p = Eigen::Isometry3d::Identity(); p.translation().x() = x; return p;
}
glim::EstimationFrame::Ptr frame(int id, double x)
{
  auto f = std::make_shared<glim::EstimationFrame>();
  f->id = id; f->stamp = id + 1.; f->T_world_imu = at(x); f->T_world_lidar = at(x);
  f->T_lidar_imu = at(0); f->frame_id = glim::FrameID::IMU;
  f->frame =
    std::make_shared<gtsam_points::PointCloudCPU>(std::vector<Eigen::Vector4d>{Eigen::Vector4d(0, 0,
      .08, 1)});
  return f;
}
glim::SubMap::Ptr submap(int id, const glim::EstimationFrame::Ptr & f)
{
  auto s = std::make_shared<glim::SubMap>(); s->id = id;
  s->T_world_origin = f->T_world_imu; s->T_origin_endpoint_L = s->T_origin_endpoint_R = at(0);
  s->frames = {f}; s->odom_frames = {f->clone()}; return s;
}
int main(int argc, char ** argv)
{
  check(argc > 1, "config path required");
  glim::GlobalConfig::instance(argv[1], true);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("revision_contract");
  {
    glim::RevisionModule module; module.create_subscriptions(*node);
    auto a = frame(0, 10), b = frame(1, 20);
    b->stamp = 49.700002592;
    glim::OdometryEstimationCallbacks::on_update_frames(
      std::vector<glim::EstimationFrame::ConstPtr>{a, b});
    auto first = module.snapshot();
    check(first.revision == 1 && first.frames.size() == 2, "missing active frames");
    check(first.frames[1].measurement_stamp == glim::from_sec(b->stamp),
      "fractional nanosecond stamp disagrees with cloud");
    a->T_world_imu = at(11);
    check(module.snapshot().frames[0].map_from_imu.position.x == 10,
      "mutable estimator object escaped callback");
    glim::OdometryEstimationCallbacks::on_update_frames(
      std::vector<glim::EstimationFrame::ConstPtr>{a, b});
    check(module.snapshot().frames[0].map_from_imu.position.x == 11, "historical revision lost");
    auto sa = submap(0, a), sb = submap(1, b);
    glim::SubMappingCallbacks::on_new_submap(sa); glim::SubMappingCallbacks::on_new_submap(sb);
    sa->T_world_origin = at(13); sb->T_world_origin = at(24);
    glim::GlobalMappingCallbacks::on_update_submaps(std::vector<glim::SubMap::Ptr>{sa, sb});
    auto last = module.snapshot();
    check(last.frames[0].map_from_imu.position.x == 13, "first submap correction wrong");
    check(last.frames[1].map_from_imu.position.x == 24, "second submap correction wrong");
    check(last.frames[0].submap_id == 0 && last.frames[1].submap_id == 1, "membership lost");
    check(last.frames[0].measurement_stamp.sec == 1, "correction refreshed observation time");
    check(!last.frames[0].deskew_recomputed, "rigid correction claimed deskew");
    check(first.frames[0].map_from_imu.position.x == 10, "old snapshot mutated");
    check(last.parent_revision + 1 == last.revision && last.full_snapshot,
      "revision contract broken");
  // Late subscribers recover the bounded retained observations; full graph
  // recovery uses the actual ROS snapshot service, not a visualization topic.
    size_t observations = 0;
    auto subscription =
      node->create_subscription<astrobot_interfaces::msg::SlamObservation>("~/observations",
    rclcpp::QoS(100).reliable().transient_local(),
        [&](const astrobot_interfaces::msg::SlamObservation & msg){
          check(msg.revision == 1 && msg.points.width == 1,
        "observation lost original revision/evidence");
          check(msg.points.header.stamp == msg.frame.measurement_stamp,
        "observation stamp mismatch");
          ++observations;
    });
    auto client = node->create_client<astrobot_interfaces::srv::GetSlamSnapshot>("~/get_snapshot");
    check(client->wait_for_service(std::chrono::seconds(3)), "snapshot service absent");
    auto future =
      client->async_send_request(
      std::make_shared<astrobot_interfaces::srv::GetSlamSnapshot::Request>());
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((observations < 2 ||
      future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    check(observations == 2, "initial local observations missing or re-emitted by revisions");
    check(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready,
      "snapshot service timed out");
    check(future.get()->success, "snapshot service not ready");
  }
  check(!glim::OdometryEstimationCallbacks::on_update_frames,
    "destroyed revision module left a dangling callback");
  check(!glim::SubMappingCallbacks::on_new_submap,
    "destroyed revision module left a submap callback");
  rclcpp::shutdown();
}
