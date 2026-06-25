#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

namespace robotperf { namespace planning {

class PathOutputComponent : public rclcpp::Node
{
public:
  explicit PathOutputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  void cb(nav_msgs::msg::Path::SharedPtr msg);
  size_t get_msg_size(nav_msgs::msg::Path::ConstSharedPtr msg);
};

}} // namespace robotperf::planning
