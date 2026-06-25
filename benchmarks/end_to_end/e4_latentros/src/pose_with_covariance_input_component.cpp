#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/pose_with_covariance_input_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace localization {

PoseWithCovarianceInputComponent::PoseWithCovarianceInputComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("PoseWithCovarianceInputComponent", options)
{
  auto src  = declare_parameter<std::string>("source_topic_name", "/base_to_footprint_pose");
  auto dest = declare_parameter<std::string>("input_topic_name",  "/robotperf/benchmark/pose_cov_input");
  pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(dest, rclcpp::QoS(10).reliable());
  sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    src, rclcpp::QoS(10).reliable(),
    std::bind(&PoseWithCovarianceInputComponent::cb, this, std::placeholders::_1));
}

size_t PoseWithCovarianceInputComponent::get_msg_size(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

uint32_t PoseWithCovarianceInputComponent::generate_unique_key() {
  static uint32_t counter = 1;
  return counter++;
}

void PoseWithCovarianceInputComponent::cb(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  uint32_t key = generate_unique_key();
  msg->header.stamp.nanosec = key;

  TRACEPOINT(robotperf_pose_cov_input_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  if (pub_->get_subscription_count() > 0)
    pub_->publish(*msg);

  TRACEPOINT(robotperf_pose_cov_input_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::localization::PoseWithCovarianceInputComponent)
