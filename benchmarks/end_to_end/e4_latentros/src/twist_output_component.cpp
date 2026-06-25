#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/twist_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace planning {

TwistOutputComponent::TwistOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("TwistOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/cmd_vel");
  sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&TwistOutputComponent::cb, this, std::placeholders::_1));
}

size_t TwistOutputComponent::get_msg_size(geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<geometry_msgs::msg::Twist> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

uint32_t TwistOutputComponent::generate_unique_key() {
  static uint32_t counter = 1;
  return counter++;
}

void TwistOutputComponent::cb(geometry_msgs::msg::Twist::SharedPtr msg) {
  // Twist has no header — use a monotonic counter as key
  uint32_t key = generate_unique_key();

  TRACEPOINT(robotperf_twist_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    get_msg_size(msg), key);

  TRACEPOINT(robotperf_twist_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::planning::TwistOutputComponent)
