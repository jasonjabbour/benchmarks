#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace robotperf { namespace localization {

class PoseWithCovarianceInputComponent : public rclcpp::Node
{
public:
  explicit PoseWithCovarianceInputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
  void cb(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  size_t get_msg_size(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  static uint32_t generate_unique_key();
};

}} // namespace robotperf::localization
