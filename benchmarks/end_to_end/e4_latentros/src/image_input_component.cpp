/// Simple Image input component — NO image_transport, just plain rclcpp topics.
/// Subscribes to source image, stamps key, publishes image + matching CameraInfo
/// so downstream image_transport CameraSubscriber sync works.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/image_input_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

E4ImageInputComponent::E4ImageInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("E4ImageInputComponent", options)
{
  auto src = declare_parameter<std::string>("source_topic_name", "/camera/image_raw");
  auto dest = declare_parameter<std::string>("input_topic_name", "/robotperf/benchmark/image");
  auto cam_src = declare_parameter<std::string>("camera_info_in", "/camera/camera_info");
  auto cam_dest = declare_parameter<std::string>("camera_info_out", "/robotperf/benchmark/camera_info");

  // Subscribe RELIABLE (rosbag always publishes reliable)
  // Publish RELIABLE (compatible with both reliable and best_effort subscribers)
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    src, rclcpp::QoS(10).reliable(),
    std::bind(&E4ImageInputComponent::cb, this, std::placeholders::_1));
  pub_ = create_publisher<sensor_msgs::msg::Image>(
    dest, rclcpp::QoS(10).reliable());

  // CameraInfo: reliable in and out
  cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    cam_src, rclcpp::QoS(10).reliable(),
    [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { last_cam_info_ = msg; });
  cam_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    cam_dest, rclcpp::QoS(10).reliable());

  RCLCPP_INFO(get_logger(), "Image input: %s -> %s (cam: %s -> %s)",
    src.c_str(), dest.c_str(), cam_src.c_str(), cam_dest.c_str());
}

size_t E4ImageInputComponent::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::Image> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

uint32_t E4ImageInputComponent::generate_unique_key() {
  static uint32_t counter = 1;
  return counter++;
}

void E4ImageInputComponent::cb(sensor_msgs::msg::Image::SharedPtr msg) {
  uint32_t key = generate_unique_key();
  msg->header.stamp.nanosec = key;

  TRACEPOINT(robotperf_image_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), 0);

  // Always publish — no subscriber count gating (DDS discovery delay
  // causes missed messages in separate-process benchmarks)
  pub_->publish(*msg);
  // Publish CameraInfo with same stamp so image_transport sync matches
  if (last_cam_info_) {
    auto ci = *last_cam_info_;
    ci.header.stamp = msg->header.stamp;
    // Fix distortion_model for nodes that only accept "plumb_bob".
    // Some bags use "pinhole" or empty, which causes
    // "Cannot call rectifyPoint when distortion is unknown".
    if (ci.distortion_model.empty() || ci.distortion_model == "pinhole") {
      ci.distortion_model = "plumb_bob";
      ci.d.resize(5, 0.0);
    }
    cam_pub_->publish(ci);
  }

  TRACEPOINT(robotperf_image_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), 0);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::E4ImageInputComponent)
