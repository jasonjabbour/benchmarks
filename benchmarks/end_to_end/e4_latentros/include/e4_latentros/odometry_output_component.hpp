#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace robotperf { namespace localization {

class OdometryOutputComponent : public rclcpp::Node
{
public:
  explicit OdometryOutputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  void cb(nav_msgs::msg::Odometry::SharedPtr msg);
  size_t get_msg_size(nav_msgs::msg::Odometry::ConstSharedPtr msg);
};

}} // namespace robotperf::localization
