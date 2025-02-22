#include <rclcpp/rclcpp.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "laserscan_input_component.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{

namespace perception
{

LaserscanInputComponent::LaserscanInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("LaserscanInputComponent", options)
{
  // Get topic names from parameter server
  std::string input_topic_name = this->declare_parameter<std::string>("input_topic_name", "input_scan");
  std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name", "output_scan");

  // Create a LaserScan publisher
  pub_laserscan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    output_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort()
  );

  // Create a LaserScan subscriber
  sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
    std::bind(&LaserscanInputComponent::laserscanCb, this, std::placeholders::_1)
  );

}

size_t LaserscanInputComponent::get_msg_size(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  // Serialize the LaserScan message
  rclcpp::SerializedMessage serialized_data_scan;
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization;
  const void* scan_ptr = reinterpret_cast<const void*>(scan_msg.get());
  scan_serialization.serialize_message(scan_ptr, &serialized_data_scan);
  return serialized_data_scan.size();
}

void LaserscanInputComponent::laserscanCb(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // 🔥 Extract the unique key directly from nanosec
    uint32_t unique_key = scan_msg->header.stamp.nanosec;

    TRACEPOINT(
        robotperf_laserscan_input_cb_init,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*scan_msg)),
        scan_msg->header.stamp.nanosec,
        scan_msg->header.stamp.sec,
        get_msg_size(scan_msg), 
        unique_key);

    // If no subscribers, do nothing
    if (pub_laserscan_->get_subscription_count() < 1) {
        return;
    }

    // Republish the incoming LaserScan message
    pub_laserscan_->publish(*scan_msg);

    TRACEPOINT(
        robotperf_laserscan_input_cb_fini,    
        static_cast<const void *>(this),
        static_cast<const void *>(&(*scan_msg)),
        scan_msg->header.stamp.nanosec,
        scan_msg->header.stamp.sec,
        get_msg_size(scan_msg), 
        unique_key);
}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::LaserscanInputComponent)
