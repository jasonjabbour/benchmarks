#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/disparity_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

DisparityOutputComponent::DisparityOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("DisparityOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/disparity");
  sub_ = create_subscription<stereo_msgs::msg::DisparityImage>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&DisparityOutputComponent::cb, this, std::placeholders::_1));
}

size_t DisparityOutputComponent::get_msg_size(
  stereo_msgs::msg::DisparityImage::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<stereo_msgs::msg::DisparityImage> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void DisparityOutputComponent::cb(stereo_msgs::msg::DisparityImage::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;
  TRACEPOINT(robotperf_disparity_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
  TRACEPOINT(robotperf_disparity_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::DisparityOutputComponent)
