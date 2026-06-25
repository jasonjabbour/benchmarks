#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/pose_with_covariance_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace localization {

PoseWithCovarianceOutputComponent::PoseWithCovarianceOutputComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("PoseWithCovarianceOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/amcl_pose");
  sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&PoseWithCovarianceOutputComponent::cb, this, std::placeholders::_1));
}

size_t PoseWithCovarianceOutputComponent::get_msg_size(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void PoseWithCovarianceOutputComponent::cb(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_pose_cov_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  TRACEPOINT(robotperf_pose_cov_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::localization::PoseWithCovarianceOutputComponent)
