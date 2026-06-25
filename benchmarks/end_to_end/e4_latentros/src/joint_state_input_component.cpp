#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/joint_state_input_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace localization {

JointStateInputComponent::JointStateInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("JointStateInputComponent", options)
{
  auto src  = declare_parameter<std::string>("source_topic_name", "/joint_states");
  auto dest = declare_parameter<std::string>("input_topic_name",  "/robotperf/benchmark/joint_states");
  pub_ = create_publisher<sensor_msgs::msg::JointState>(dest, rclcpp::QoS(10).reliable());
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
    src, rclcpp::QoS(10).reliable(),
    std::bind(&JointStateInputComponent::cb, this, std::placeholders::_1));
}

size_t JointStateInputComponent::get_msg_size(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::JointState> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

uint32_t JointStateInputComponent::generate_unique_key() {
  static uint32_t counter = 1;
  return counter++;
}

void JointStateInputComponent::cb(sensor_msgs::msg::JointState::SharedPtr msg) {
  uint32_t key = generate_unique_key();
  msg->header.stamp.nanosec = key;

  TRACEPOINT(robotperf_joint_state_input_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  if (pub_->get_subscription_count() > 0)
    pub_->publish(*msg);

  TRACEPOINT(robotperf_joint_state_input_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::localization::JointStateInputComponent)
