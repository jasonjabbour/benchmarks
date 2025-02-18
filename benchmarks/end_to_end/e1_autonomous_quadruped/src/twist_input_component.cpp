#include <rclcpp/rclcpp.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "twist_input_component.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{

namespace control
{

TwistInputComponent::TwistInputComponent(const rclcpp::NodeOptions &options)
: rclcpp::Node("TwistInputComponent", options)
{
  // Get the output_topic_name parameter from the parameter server with default value "input"
  std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name", "input");

  // Get the twist type (Twist or TwistStamped)
  twist_type_ = this->declare_parameter<std::string>("twist_type", "TwistStamped");

  if (twist_type_ == "TwistStamped") {
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&TwistInputComponent::twistCb<geometry_msgs::msg::TwistStamped>, this, std::placeholders::_1)
    );
  } else {
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
      output_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&TwistInputComponent::twistCb<geometry_msgs::msg::Twist>, this, std::placeholders::_1)
    );
  }
}

template <typename TwistMsgType>
void TwistInputComponent::twistCb(const typename TwistMsgType::SharedPtr twist_msg)
{
  TRACEPOINT(
    robotperf_twist_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*twist_msg)),
    get_msg_size<TwistMsgType>(twist_msg));

  if (pub_twist_->get_subscription_count() < 1) {
    return;
  }

  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<TwistMsgType>>(pub_twist_);
  if (pub) {
    pub->publish(*twist_msg);
  }

  TRACEPOINT(
    robotperf_twist_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*twist_msg)),
    get_msg_size<TwistMsgType>(twist_msg));
}

template <typename TwistMsgType>
size_t TwistInputComponent::get_msg_size(typename TwistMsgType::ConstSharedPtr twist_msg)
{
  rclcpp::SerializedMessage serialized_data_twist;
  rclcpp::Serialization<TwistMsgType> twist_serialization;
  const void* twist_ptr = reinterpret_cast<const void*>(twist_msg.get());
  twist_serialization.serialize_message(twist_ptr, &serialized_data_twist);
  return serialized_data_twist.size();
}

}  // namespace control

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::control::TwistInputComponent)
