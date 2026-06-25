#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/path_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace planning {

PathOutputComponent::PathOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("PathOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/global_plan");
  // best_effort — compatible with both reliable and best_effort publishers
  sub_ = create_subscription<nav_msgs::msg::Path>(
    topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
    std::bind(&PathOutputComponent::cb, this, std::placeholders::_1));
}

size_t PathOutputComponent::get_msg_size(nav_msgs::msg::Path::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<nav_msgs::msg::Path> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void PathOutputComponent::cb(nav_msgs::msg::Path::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_path_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  TRACEPOINT(robotperf_path_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::planning::PathOutputComponent)
