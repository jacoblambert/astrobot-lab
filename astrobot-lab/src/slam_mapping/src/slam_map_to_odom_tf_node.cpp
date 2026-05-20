#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace slam_mapping
{
struct Pose2D
{
  double x;
  double y;
  double yaw;
};

class SlamMapToOdomTfNode : public rclcpp::Node
{
public:
  SlamMapToOdomTfNode()
  : Node("slam_map_to_odom_tf")
  {
    slam_pose_topic_ = declare_parameter<std::string>("slam_pose_topic", "slam/pose_corrected");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    base_offset_x_ = declare_parameter<double>("base_offset_x", 0.264);
    base_offset_y_ = declare_parameter<double>("base_offset_y", 0.017);
    base_yaw_offset_ = declare_parameter<double>("base_yaw_offset", M_PI);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    transform_tolerance_sec_ = declare_parameter<double>("transform_tolerance_sec", 0.02);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      slam_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamMapToOdomTfNode::slam_pose_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamMapToOdomTfNode::odom_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)),
      std::bind(&SlamMapToOdomTfNode::publish_tf, this));

    RCLCPP_INFO(
      get_logger(),
      "SLAM map->odom TF: slam_pose=%s odom=%s frame=%s->%s base=%s imu_to_base=[%.3f, %.3f, yaw %.3f]",
      slam_pose_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), base_offset_x_, base_offset_y_, base_yaw_offset_);
  }

private:
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static double normalize_angle(const double yaw)
  {
    return std::atan2(std::sin(yaw), std::cos(yaw));
  }

  static Pose2D pose_from_msg(const geometry_msgs::msg::Pose & pose)
  {
    return Pose2D{pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)};
  }

  static Pose2D compose(const Pose2D & a, const Pose2D & b)
  {
    const double c = std::cos(a.yaw);
    const double s = std::sin(a.yaw);
    return Pose2D{
      a.x + c * b.x - s * b.y,
      a.y + s * b.x + c * b.y,
      normalize_angle(a.yaw + b.yaw)};
  }

  static Pose2D inverse(const Pose2D & pose)
  {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    return Pose2D{
      -c * pose.x - s * pose.y,
      s * pose.x - c * pose.y,
      normalize_angle(-pose.yaw)};
  }

  void slam_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_slam_pose_ = *msg;
    have_slam_pose_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  void publish_tf()
  {
    if (!have_slam_pose_ || !have_odom_) {
      return;
    }

    const Pose2D map_imu = pose_from_msg(latest_slam_pose_.pose);
    const Pose2D imu_base{base_offset_x_, base_offset_y_, base_yaw_offset_};
    const Pose2D map_base = compose(map_imu, imu_base);
    const Pose2D odom_base = pose_from_msg(latest_odom_.pose.pose);
    const Pose2D map_odom = compose(map_base, inverse(odom_base));

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = latest_slam_pose_.header.stamp;
    transform.header.stamp.nanosec += static_cast<uint32_t>(transform_tolerance_sec_ * 1e9);
    while (transform.header.stamp.nanosec >= 1000000000U) {
      transform.header.stamp.sec += 1;
      transform.header.stamp.nanosec -= 1000000000U;
    }
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_odom.x;
    transform.transform.translation.y = map_odom.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = quaternion_from_yaw(map_odom.yaw);
    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "publishing %s->%s x=%.3f y=%.3f yaw=%.3f",
      map_frame_.c_str(), odom_frame_.c_str(), map_odom.x, map_odom.y, map_odom.yaw);
  }

  std::string slam_pose_topic_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double base_offset_x_;
  double base_offset_y_;
  double base_yaw_offset_;
  double publish_rate_hz_;
  double transform_tolerance_sec_;
  bool have_slam_pose_{false};
  bool have_odom_{false};
  geometry_msgs::msg::PoseStamped latest_slam_pose_;
  nav_msgs::msg::Odometry latest_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace slam_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_mapping::SlamMapToOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
