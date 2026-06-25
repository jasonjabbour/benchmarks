#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace robotperf { namespace localization {

class OdometryInputComponent : public rclcpp::Node
{
public:
  explicit OdometryInputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  void cb(nav_msgs::msg::Odometry::SharedPtr msg);
  size_t get_msg_size(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  static uint32_t generate_unique_key();
};

}} // namespace robotperf::localization
