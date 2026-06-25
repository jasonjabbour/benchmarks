#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace robotperf { namespace localization {

class PoseWithCovarianceOutputComponent : public rclcpp::Node
{
public:
  explicit PoseWithCovarianceOutputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
  void cb(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  size_t get_msg_size(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
};

}} // namespace robotperf::localization
