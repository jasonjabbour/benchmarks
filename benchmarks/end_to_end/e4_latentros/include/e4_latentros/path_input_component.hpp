#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

namespace robotperf { namespace planning {

class PathInputComponent : public rclcpp::Node
{
public:
  explicit PathInputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  void cb(nav_msgs::msg::Path::SharedPtr msg);
  size_t get_msg_size(nav_msgs::msg::Path::ConstSharedPtr msg);
  static uint32_t generate_unique_key();
};

}} // namespace robotperf::planning
