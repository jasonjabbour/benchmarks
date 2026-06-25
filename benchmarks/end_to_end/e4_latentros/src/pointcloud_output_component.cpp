#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/pointcloud_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

PointCloudOutputComponent::PointCloudOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/points");
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&PointCloudOutputComponent::cb, this, std::placeholders::_1));
}

size_t PointCloudOutputComponent::get_msg_size(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void PointCloudOutputComponent::cb(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;
  TRACEPOINT(robotperf_pointcloud_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
  TRACEPOINT(robotperf_pointcloud_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::PointCloudOutputComponent)
