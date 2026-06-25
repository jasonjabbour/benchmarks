/// Pure LaserScan observer — fires laserscan_output tracepoints, does NOT republish.
/// Reads key from header.stamp.nanosec (consistent with all e4 components).
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/laserscan_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

E4LaserscanOutputComponent::E4LaserscanOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("E4LaserscanOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/scan_filtered");
  sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
    std::bind(&E4LaserscanOutputComponent::cb, this, std::placeholders::_1));
}

size_t E4LaserscanOutputComponent::get_msg_size(
  sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void E4LaserscanOutputComponent::cb(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_laserscan_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  TRACEPOINT(robotperf_laserscan_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::E4LaserscanOutputComponent)
