#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/odometry_input_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace localization {

OdometryInputComponent::OdometryInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("OdometryInputComponent", options)
{
  auto src  = declare_parameter<std::string>("source_topic_name", "/odom/raw");
  auto dest = declare_parameter<std::string>("input_topic_name",  "/robotperf/benchmark/odom_input");
  pub_ = create_publisher<nav_msgs::msg::Odometry>(dest, rclcpp::QoS(10).reliable());
  sub_ = create_subscription<nav_msgs::msg::Odometry>(
    src, rclcpp::QoS(10).reliable(),
    std::bind(&OdometryInputComponent::cb, this, std::placeholders::_1));
}

size_t OdometryInputComponent::get_msg_size(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<nav_msgs::msg::Odometry> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

uint32_t OdometryInputComponent::generate_unique_key() {
  static uint32_t counter = 1;
  return counter++;
}

void OdometryInputComponent::cb(nav_msgs::msg::Odometry::SharedPtr msg) {
  uint32_t key = generate_unique_key();
  msg->header.stamp.nanosec = key;

  TRACEPOINT(robotperf_odometry_input_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  if (pub_->get_subscription_count() > 0)
    pub_->publish(*msg);

  TRACEPOINT(robotperf_odometry_input_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::localization::OdometryInputComponent)
