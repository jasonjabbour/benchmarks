#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robotperf { namespace control {

class JointStateOutputComponent : public rclcpp::Node
{
public:
  explicit JointStateOutputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  void cb(sensor_msgs::msg::JointState::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::JointState::ConstSharedPtr msg);
};

}} // namespace robotperf::control
