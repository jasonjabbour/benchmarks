#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace robotperf { namespace planning {

class PoseStampedInputComponent : public rclcpp::Node
{
public:
  explicit PoseStampedInputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  void cb(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  size_t get_msg_size(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  static uint32_t generate_unique_key();
};

}} // namespace robotperf::planning
