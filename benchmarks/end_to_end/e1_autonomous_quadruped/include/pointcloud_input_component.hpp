#ifndef ROBOTPERF_POINTCLOUD_INPUT_COMPONENT_HPP_
#define ROBOTPERF_POINTCLOUD_INPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "e3_custom_messages/msg/custom_point_cloud2.hpp"

namespace robotperf
{

namespace perception
{

class PointCloudInputComponent
  : public rclcpp::Node
{
public:
  explicit PointCloudInputComponent(const rclcpp::NodeOptions &);

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  // For publishing quantized pointclouds
  rclcpp::Publisher<e3_custom_messages::msg::CustomPointCloud2>::SharedPtr pub_pointcloud_custom_;

  size_t get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  uint32_t generate_unique_key();

  bool quantization_enabled_;

  // Helper function to convert the incoming PointCloud2 to a custom "int16" version
  void convertPointCloud2ToCustom(
    const sensor_msgs::msg::PointCloud2 & in_msg,
    e3_custom_messages::msg::CustomPointCloud2 & out_msg);
};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPERF_POINTCLOUD_INPUT_COMPONENT_HPP_