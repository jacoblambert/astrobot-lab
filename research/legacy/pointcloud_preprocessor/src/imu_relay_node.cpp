#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRelayNode : public rclcpp::Node
{
public:
  ImuRelayNode() : Node("imu_relay")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "imu");

    auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic,
      qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        publisher_->publish(*msg);
      });

    RCLCPP_INFO(get_logger(), "Relaying IMU %s -> %s", input_topic.c_str(), output_topic.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRelayNode>());
  rclcpp::shutdown();
  return 0;
}
