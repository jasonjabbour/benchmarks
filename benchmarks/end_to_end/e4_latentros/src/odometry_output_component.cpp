#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/odometry_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace localization {

OdometryOutputComponent::OdometryOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("OdometryOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/odom");
  sub_ = create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&OdometryOutputComponent::cb, this, std::placeholders::_1));
}

size_t OdometryOutputComponent::get_msg_size(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<nav_msgs::msg::Odometry> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void OdometryOutputComponent::cb(nav_msgs::msg::Odometry::SharedPtr msg) {
  // Key propagated via header.stamp.nanosec by the input component;
  // in live-stack mode key is 0 (no injection) — both cases are handled.
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_odometry_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  TRACEPOINT(robotperf_odometry_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::localization::OdometryOutputComponent)
