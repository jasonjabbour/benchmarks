#ifndef ROBOTPERF_TWIST_INPUT_COMPONENT_HPP_
#define ROBOTPERF_TWIST_INPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace robotperf
{

namespace control
{

class TwistInputComponent
  : public rclcpp::Node
{
public:
  explicit TwistInputComponent(const rclcpp::NodeOptions &);

protected:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  size_t get_msg_size(geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg);

  void twistCb(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);

};

}  // namespace control

}  // namespace robotperf

#endif  // ROBOTPERF_TWIST_INPUT_COMPONENT_HPP_