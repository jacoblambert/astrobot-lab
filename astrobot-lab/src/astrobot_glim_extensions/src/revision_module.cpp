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
// SLAM revision export for the terrain node (slam_raw/revisions). Not a navigation
// authorization interface: motion permission comes from the localization supervisor.
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <random>
#include <sstream>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/config.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <astrobot_interfaces/msg/slam_revision.hpp>
#include <astrobot_interfaces/msg/slam_observation.hpp>
#include <astrobot_interfaces/srv/get_slam_snapshot.hpp>
#define GLIM_ROS2
#include <glim/util/ros_cloud_converter.hpp>

namespace glim
{
class RevisionModule : public ExtensionModuleROS2 {
  using Revision = astrobot_interfaces::msg::SlamRevision;
  using FramePose = astrobot_interfaces::msg::SlamFramePose;
  using Snapshot = astrobot_interfaces::srv::GetSlamSnapshot;
  using Observation = astrobot_interfaces::msg::SlamObservation;
  struct Frame
  {
    FramePose message;
    Eigen::Isometry3d local;  // estimator-local if unassigned, submap-local otherwise
  };
  struct Submap
  {
    Eigen::Isometry3d origin;
  };

public:
  RevisionModule()
  {
    const Config config(GlobalConfig::get_config_path("config_ros"));
    const int epoch = config.param<int>("revision_export", "localization_epoch", 0);
    if (epoch < 0) {throw std::invalid_argument("negative localization epoch");}
    epoch_ = epoch;
    map_frame_ = config.param<std::string>("glim_ros", "map_frame_id", "map");
    imu_frame_ = config.param<std::string>("glim_ros", "imu_frame_id", "Imu_Sensor");
    lidar_frame_ = config.param<std::string>("glim_ros", "lidar_frame_id", "vlp16");
    std::ostringstream id;
    id << std::chrono::system_clock::now().time_since_epoch().count() << '-' <<
      std::random_device{}();
    session_ = id.str();
    // Copy data under callbacks' ownership. No shared GLIM object escapes a callback.
    const auto on_frames = [this](const auto & frames) {odometry(frames);};
    const auto on_submap = [this](const auto & submap) {new_submap(submap);};
    const auto on_submaps = [this](const auto & submaps) {global(submaps);};
    const auto on_corruption = [this](double) {valid_ = false;};
    odometry_callback_ = OdometryEstimationCallbacks::on_update_frames.add(on_frames);
    submap_callback_ = SubMappingCallbacks::on_new_submap.add(on_submap);
    global_callback_ = GlobalMappingCallbacks::on_update_submaps.add(on_submaps);
    corruption_callback_ = OdometryEstimationCallbacks::on_smoother_corruption.add(on_corruption);
  }
  ~RevisionModule() override
  {
    // GlimROS joins all estimator/mapping workers before destroying extensions.
    OdometryEstimationCallbacks::on_update_frames.remove(odometry_callback_);
    SubMappingCallbacks::on_new_submap.remove(submap_callback_);
    GlobalMappingCallbacks::on_update_submaps.remove(global_callback_);
    OdometryEstimationCallbacks::on_smoother_corruption.remove(corruption_callback_);
  }
  bool ok() const override {return valid_;}
  Revision snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = header(Revision::GLOBAL);
    result.full_snapshot = true;
    for(const auto & entry : frames_) {result.frames.push_back(materialize(entry.second));}
    return result;
  }
  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node & node) override
  {
    pub_ = node.create_publisher<Revision>("~/revisions",
        rclcpp::QoS(100).reliable().transient_local());
    observations_ = node.create_publisher<Observation>("~/observations",
        rclcpp::QoS(100).reliable().transient_local());
    service_ = node.create_service<Snapshot>("~/get_snapshot",
        [this](const std::shared_ptr<Snapshot::Request>,
        std::shared_ptr<Snapshot::Response> out){
          std::lock_guard<std::mutex> lock(mutex_);
          out->success = valid_ && revision_ > 0;
          out->reason = out->success ? "complete_known_frame_snapshot" : "not_ready_or_invalid";
          out->snapshot = header(Revision::GLOBAL);
          out->snapshot.full_snapshot = true;
          for(const auto & entry : frames_) {
            out->snapshot.frames.push_back(materialize(entry.second));
          }
    });
    return {};
  }

private:
  static geometry_msgs::msg::Pose pose(const Eigen::Isometry3d & value)
  {
    if (!value.matrix().allFinite()) {throw std::runtime_error("nonfinite graph pose");}
    geometry_msgs::msg::Pose msg;
    msg.position.x = value.translation().x(); msg.position.y = value.translation().y();
    msg.position.z = value.translation().z();
    const Eigen::Quaterniond q(value.linear());
    msg.orientation.x = q.x(); msg.orientation.y = q.y(); msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
  }
  static builtin_interfaces::msg::Time stamp(double seconds)
  {
    if (!std::isfinite(seconds) || seconds < 0) {throw std::runtime_error("invalid graph stamp");}
    // Must match GLIM's pose/cloud conversion exactly (fractional ns truncate).
    return from_sec(seconds);
  }
  Revision header(uint8_t kind) const
  {
    Revision msg; msg.header.frame_id = map_frame_; msg.header.stamp = stamp(last_stamp_);
    msg.session_id = session_; msg.localization_epoch = epoch_; msg.revision = revision_;
    msg.parent_revision = revision_ ? revision_ - 1 : 0; msg.kind = kind;
    return msg;
  }
  FramePose materialize(const Frame & frame) const
  {
    auto msg = frame.message;
    const Eigen::Isometry3d transform = msg.submap_id <
      0 ? correction_ : submaps_.at(msg.submap_id).origin;
    msg.map_from_imu = pose(transform * frame.local);
    return msg;
  }
  void publish(uint8_t kind, const std::vector<int64_t> & changed)
  {
    if (!pub_ || !valid_) {return;}
    if (frames_.size() > 100000) {valid_ = false; return;}
    ++revision_;
    auto msg = header(kind);
    for(auto id : changed) {msg.frames.push_back(materialize(frames_.at(id)));}
    pub_->publish(msg);  // Publish under lock, preserving revision order across callback threads.
  }
  void odometry(const std::vector<EstimationFrame::ConstPtr> & updates)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_) {return;}
    try {
      std::vector<int64_t> changed;
      std::vector<Observation> observations;
      for(const auto & input : updates) {
        if (!input) {continue;}
        auto old = frames_.find(input->id);
        if (old != frames_.end() && old->second.message.submap_id >= 0) {continue;}
        Frame frame; frame.message.frame_id = input->id; frame.message.submap_id = -1;
        frame.message.measurement_stamp = stamp(input->stamp);
        frame.message.imu_from_lidar = pose(input->T_lidar_imu.inverse());
        frame.message.deskew_recomputed = false;
        frame.local = input->T_world_imu;
        if (old == frames_.end()) {
          if (!input->frame || input->frame_id == FrameID::WORLD) {
            throw std::runtime_error("new frame lacks immutable sensor-local evidence");
          }
          Observation observation;
          observation.session_id = session_; observation.localization_epoch = epoch_;
          observation.revision = revision_ + 1;
          observation.frame = materialize(frame);
          observation.preprocessing_lossy = true;
          observation.points = *frame_to_pointcloud2(input->frame_id ==
              FrameID::IMU ? imu_frame_ : lidar_frame_,
                                                   input->stamp, *input->frame);
          observations.push_back(std::move(observation));
        } else if (old->second.message.measurement_stamp != frame.message.measurement_stamp) {
          throw std::runtime_error("frame identity changed measurement time");
        }
        frames_[input->id] = frame; changed.push_back(input->id);
        last_stamp_ = std::max(last_stamp_, input->stamp);
      }
      if (!changed.empty()) {publish(Revision::ODOM, changed);}
      if (valid_ && observations_) {
        for(const auto & observation : observations) {
          observations_->publish(observation);
        }
      }
    } catch(const std::exception & e) {
      spdlog::error("revision export: {}", e.what()); valid_ = false;
    }
  }
  void new_submap(const SubMap::ConstPtr & input)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_ || input->frames.empty() || input->odom_frames.empty()) {return;}
    try {
      const auto cached = global_origins_.find(input->id);
      const Eigen::Isometry3d origin = cached ==
        global_origins_.end() ? correction_ * input->T_world_origin : cached->second;
      submaps_[input->id] = {origin};
      std::vector<int64_t> changed;
      for(const auto & source : input->frames) {
        Frame frame; frame.message.frame_id = source->id; frame.message.submap_id = input->id;
        frame.message.measurement_stamp = stamp(source->stamp);
        frame.message.imu_from_lidar = pose(source->T_lidar_imu.inverse());
        frame.message.deskew_recomputed = false;
        frame.local = input->T_world_origin.inverse() * source->T_world_imu;
        frames_[source->id] = frame; changed.push_back(source->id);
      }
      publish(Revision::SUBMAP, changed);
    } catch(const std::exception & e) {
      spdlog::error("revision submap export: {}", e.what()); valid_ = false;
    }
  }
  void global(const std::vector<SubMap::Ptr> & updates)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_ || updates.empty()) {return;}
    try {
      for(const auto & source : updates) {
        global_origins_[source->id] = source->T_world_origin;
        auto cached = submaps_.find(source->id);
        if (cached != submaps_.end()) {cached->second.origin = source->T_world_origin;}
      }
      // Same current-world anchor as GLIM's visualization trajectory, but copied
      // from this callback. Historical submaps retain their INDIVIDUAL transforms.
      const auto & latest = updates.back();
      if (!latest->odom_frames.empty()) {
        correction_ = latest->T_world_origin * latest->T_origin_endpoint_R *
          latest->odom_frames.back()->T_world_imu.inverse();
      }
      std::vector<int64_t> changed;
      for(const auto & entry : frames_) {changed.push_back(entry.first);}
      if (!changed.empty()) {publish(Revision::GLOBAL, changed);}
    } catch(const std::exception & e) {
      spdlog::error("revision global export: {}", e.what()); valid_ = false;
    }
  }
  std::mutex mutex_;
  int odometry_callback_, submap_callback_, global_callback_, corruption_callback_;
  std::atomic<bool> valid_{true};
  uint64_t epoch_{0}, revision_{0};
  std::string session_, map_frame_, imu_frame_, lidar_frame_;
  double last_stamp_{0};
  Eigen::Isometry3d correction_{Eigen::Isometry3d::Identity()};
  std::map<int64_t, Frame> frames_;
  std::map<int64_t, Submap> submaps_;
  std::map<int64_t, Eigen::Isometry3d> global_origins_;
  rclcpp::Publisher<Revision>::SharedPtr pub_;
  rclcpp::Publisher<Observation>::SharedPtr observations_;
  rclcpp::Service<Snapshot>::SharedPtr service_;
};
}  // namespace glim
extern "C" glim::ExtensionModule * create_extension_module() {return new glim::RevisionModule;}
