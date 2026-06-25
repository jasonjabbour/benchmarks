#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robotperf { namespace localization {

class JointStateInputComponent : public rclcpp::Node
{
public:
  explicit JointStateInputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  void cb(sensor_msgs::msg::JointState::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  static uint32_t generate_unique_key();
};

}} // namespace robotperf::localization
