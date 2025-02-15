#include <rclcpp/rclcpp.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "twist_input_component.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{

namespace control
{

TwistInputComponent::TwistInputComponent (const rclcpp::NodeOptions & options)
: rclcpp::Node("TwistInputComponent", options)
{

  // Get the input_topic_name parameter from the parameter server with default value "input"
  std::string input_topic_name = this->declare_parameter<std::string>("input_topic_name", "input");


  // Create a twist publisher
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(input_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // Createtwist subscriber
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
    std::bind(&TwistInputComponent::twistCb, this, std::placeholders::_1)
  );

}

size_t TwistInputComponent::get_msg_size(geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg){
  //Serialize the twist messages
  rclcpp::SerializedMessage serialized_data_twist;
  rclcpp::Serialization<geometry_msgs::msg::TwistStamped> twist_serialization;
  const void* twist_ptr = reinterpret_cast<const void*>(twist_msg.get());
  twist_serialization.serialize_message(twist_ptr, &serialized_data_twist);
  size_t twist_msg_size = serialized_data_twist.size();
  return twist_msg_size;
}


void TwistInputComponent::twistCb(
  geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  
  TRACEPOINT(
    robotperf_twist_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*twist_msg)),
    get_msg_size(twist_msg));

  if (pub_twist_->get_subscription_count() < 1) {
    return;
  }

  pub_twist_->publish(*twist_msg);

  TRACEPOINT(
    robotperf_twist_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*twist_msg)),
    get_msg_size(twist_msg));
}

}  // namespace control

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::control::TwistInputComponent)