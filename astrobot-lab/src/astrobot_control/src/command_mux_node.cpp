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

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <astrobot_interfaces/msg/localization_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "astrobot_control/command_mux.hpp"

namespace astrobot_control
{

class CommandMuxNode : public rclcpp::Node {
public:
  explicit CommandMuxNode(const rclcpp::NodeOptions & options)
  : Node("command_mux", options)
  {
    MuxConfig config;
    const double rate = declare_parameter("publish_rate_hz", 20.0);
    config.nav_timeout_s = declare_parameter("nav_timeout_sec", 0.5);
    config.teleop_timeout_s = declare_parameter("teleop_timeout_sec", 0.5);
    config.max_linear = declare_parameter("max_linear_x", 0.5);
    config.max_angular = declare_parameter("max_angular_z", 1.2);
    config.linear_scale = declare_parameter("linear_scale", 1.0);
    config.angular_scale = declare_parameter("angular_scale", 1.0);
    config.require_localization = declare_parameter("require_localization", false);
    mux_ = CommandMux(config);
    mux_.set_mode(declare_parameter<std::string>("default_mode", "auto"));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("control/status", 10);
    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10,
      [this](const geometry_msgs::msg::Twist & m) {
        mux_.nav({m.linear.x, m.angular.z}, wall());
      });
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_teleop", 10,
      [this](const geometry_msgs::msg::Twist & m) {
        mux_.teleop({m.linear.x, m.angular.z}, wall());
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "control/estop", 10,
      [this](const std_msgs::msg::Bool & m) {mux_.set_estop(m.data);});
    mode_sub_ = create_subscription<std_msgs::msg::String>("control/mode", 10,
        [this](const std_msgs::msg::String & m) {
          std::string mode = m.data;
          mode.erase(0, mode.find_first_not_of(" \t\n"));
          mode.erase(mode.find_last_not_of(" \t\n") + 1);
          std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
          if (!CommandMux::valid_mode(mode)) {
            RCLCPP_WARN(get_logger(), "ignoring unsupported control mode '%s'", m.data.c_str());
          } else if (mode != mux_.mode()) {
            mux_.set_mode(mode);
            RCLCPP_INFO(get_logger(), "control mode -> %s", mode.c_str());
          }
    });
    localization_sub_ = create_subscription<astrobot_interfaces::msg::LocalizationStatus>(
      "localization/status", 10,
      [this](const astrobot_interfaces::msg::LocalizationStatus & m) {
        mux_.localization(m.motion_permitted, wall());
      });
    loop_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate, 1.0)), [this] {
          const Command out = mux_.step(wall());
          geometry_msgs::msg::Twist twist;
          twist.linear.x = out.linear;
          twist.angular.z = out.angular;
          cmd_pub_->publish(twist);
    });
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_status();});
    RCLCPP_INFO(get_logger(), "command mux ready: mode=%s", mux_.mode().c_str());
  }

private:
  double wall() {return steady_.now().nanoseconds() * 1e-9;}

  static std::string format(const std::optional<double> & value)
  {
    if (!value) {return "n/a";}
    char text[32];
    std::snprintf(text, sizeof(text), "%.3f", *value);
    return text;
  }

  void publish_status()
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;
    DiagnosticStatus status;
    status.name = "command_mux";
    status.hardware_id = "astrobot";
    if (mux_.estop()) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "estop";
    } else if (mux_.last_source() == "hold") {
      status.level = DiagnosticStatus::WARN;
      status.message = "holding";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = mux_.last_source();
    }
    const double now = wall();
    auto kv = [](const std::string & key, const std::string & value) {
        KeyValue k;
        k.key = key;
        k.value = value;
        return k;
      };
    status.values = {kv("mode", mux_.mode()), kv("estop", mux_.estop() ? "true" : "false"),
      kv("active_source", mux_.last_source()), kv("nav_age_sec", format(mux_.nav_age(now))),
      kv("teleop_age_sec", format(mux_.teleop_age(now))),
      kv("linear_x", format(mux_.last_output().linear)),
      kv("angular_z", format(mux_.last_output().angular))};
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now_ros();
    msg.status = {status};
    status_pub_->publish(msg);
  }

  rclcpp::Time now_ros() {return get_clock()->now();}

  CommandMux mux_;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_, teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<astrobot_interfaces::msg::LocalizationStatus>::SharedPtr localization_sub_;
  rclcpp::TimerBase::SharedPtr loop_, status_timer_;
};

rclcpp::Node::SharedPtr make_command_mux(const rclcpp::NodeOptions & options)
{
  return std::make_shared<CommandMuxNode>(options);
}

}  // namespace astrobot_control

RCLCPP_COMPONENTS_REGISTER_NODE(astrobot_control::CommandMuxNode)
