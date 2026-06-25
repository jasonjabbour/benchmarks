/// Simple Image output component — NO image_transport, just plain rclcpp topics.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/image_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

E4ImageOutputComponent::E4ImageOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("E4ImageOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/robotperf/benchmark/output_image");
  // SensorDataQoS (best_effort) to match image_transport publishers
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&E4ImageOutputComponent::cb, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Image output: subscribing to %s", topic.c_str());
}

size_t E4ImageOutputComponent::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::Image> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void E4ImageOutputComponent::cb(sensor_msgs::msg::Image::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;
  TRACEPOINT(robotperf_image_output_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    static_cast<const void *>(&(*msg)),  // dummy info_msg ptr
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), 0);
  TRACEPOINT(robotperf_image_output_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), 0);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::E4ImageOutputComponent)
