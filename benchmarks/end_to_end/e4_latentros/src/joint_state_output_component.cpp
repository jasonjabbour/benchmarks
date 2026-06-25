#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/joint_state_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace control {

JointStateOutputComponent::JointStateOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("JointStateOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/joint_states");
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&JointStateOutputComponent::cb, this, std::placeholders::_1));
}

size_t JointStateOutputComponent::get_msg_size(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::JointState> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void JointStateOutputComponent::cb(sensor_msgs::msg::JointState::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_joint_state_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  TRACEPOINT(robotperf_joint_state_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::control::JointStateOutputComponent)
