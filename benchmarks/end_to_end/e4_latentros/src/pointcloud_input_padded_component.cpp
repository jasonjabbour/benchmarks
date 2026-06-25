#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/pointcloud_input_padded_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

PointCloudInputPaddedComponent::PointCloudInputPaddedComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudInputPaddedComponent", options)
{
  auto src = declare_parameter<std::string>("source_topic_name", "/velodyne_points");
  auto dest = declare_parameter<std::string>("input_topic_name",
    "/robotperf/benchmark/velodyne_points");

  // Pad PointCloud2 data to match real-world sensor sizes.
  // Gazebo simulated Velodyne produces ~100 KB point clouds, but real sensors
  // output much larger data (VLP-16: ~700 KB, VLP-32: ~1.5 MB, OS1-64: ~3 MB).
  // Default 1500 KB ≈ VLP-32 sized point cloud.
  target_msg_size_kb_ = declare_parameter<int>("target_msg_size_kb", 1500);

  // Use SensorDataQoS (best_effort) to match the kernel's subscription QoS
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    dest, rclcpp::QoS(10).reliable());
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    src, rclcpp::QoS(10).reliable(),
    std::bind(&PointCloudInputPaddedComponent::cb, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "PointCloud input with padding: target_msg_size_kb=%d (%d bytes)",
    target_msg_size_kb_, target_msg_size_kb_ * 1024);
}

size_t PointCloudInputPaddedComponent::get_msg_size(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

uint32_t PointCloudInputPaddedComponent::generate_unique_key()
{
  static uint32_t counter = 1;
  return counter++;
}

void PointCloudInputPaddedComponent::cb(
  sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  uint32_t key = generate_unique_key();
  msg->header.stamp.nanosec = key;

  // Pad point cloud to target size to match real-world sensors.
  // Duplicate actual valid points so the kernel's PointCloud2Iterator
  // sees valid point data at the larger size.
  if (target_msg_size_kb_ > 0) {
    size_t target_bytes = static_cast<size_t>(target_msg_size_kb_) * 1024;
    if (msg->data.size() < target_bytes) {
      size_t orig_data_size = msg->data.size();
      uint32_t point_step = msg->point_step;
      // Ensure target is exact multiple of point_step
      uint32_t new_point_count = target_bytes / point_step;
      size_t new_data_size = new_point_count * point_step;

      msg->data.resize(new_data_size);
      // Fill by repeating original valid points
      for (size_t i = orig_data_size; i < new_data_size; i++) {
        msg->data[i] = msg->data[i % orig_data_size];
      }
      msg->width = new_point_count;
      msg->height = 1;
      msg->row_step = new_data_size;
    }
  }

  TRACEPOINT(robotperf_pointcloud_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), key);

  if (pub_->get_subscription_count() > 0)
    pub_->publish(*msg);

  TRACEPOINT(robotperf_pointcloud_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), key);
}

}}  // namespace robotperf::perception

RCLCPP_COMPONENTS_REGISTER_NODE(
  robotperf::perception::PointCloudInputPaddedComponent)
