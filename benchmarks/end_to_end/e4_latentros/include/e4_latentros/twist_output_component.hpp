#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace robotperf { namespace planning {

class TwistOutputComponent : public rclcpp::Node
{
public:
  explicit TwistOutputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  void cb(geometry_msgs::msg::Twist::SharedPtr msg);
  size_t get_msg_size(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  static uint32_t generate_unique_key();
};

}} // namespace robotperf::planning
