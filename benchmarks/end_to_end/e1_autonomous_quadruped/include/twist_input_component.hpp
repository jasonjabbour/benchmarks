#ifndef ROBOTPERF_TWIST_INPUT_COMPONENT_HPP_
#define ROBOTPERF_TWIST_INPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <variant>

namespace robotperf
{

namespace control
{

class TwistInputComponent : public rclcpp::Node
{
public:
  explicit TwistInputComponent(const rclcpp::NodeOptions &);

protected:

  // Flexible for Twist or TwistStamped Messages
  using TwistVariant = std::variant<
    geometry_msgs::msg::Twist,
    geometry_msgs::msg::TwistStamped>;

  std::string twist_type_;
  rclcpp::PublisherBase::SharedPtr pub_twist_;
  rclcpp::SubscriptionBase::SharedPtr sub_twist_;

  void process_twist(const TwistVariant &twist_msg);

  template <typename TwistMsgType>
  void twistCb(const typename TwistMsgType::SharedPtr twist_msg);

  template <typename TwistMsgType>
  size_t get_msg_size(typename TwistMsgType::ConstSharedPtr twist_msg);
};

}  // namespace control

}  // namespace robotperf

#endif  // ROBOTPERF_TWIST_INPUT_COMPONENT_HPP_
