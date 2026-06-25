/// Image input component that preserves the original timestamp.
/// For kernels using message_filters sync (e.g., stereo disparity)
/// where modifying nanosec would break synchronization.
/// Uses the original header.stamp.nanosec as the tracepoint key.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/image_input_nokey_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace perception {

E4ImageInputNoKeyComponent::E4ImageInputNoKeyComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("E4ImageInputNoKeyComponent", options)
{
  auto src = declare_parameter<std::string>("source_topic_name", "/camera/image_raw");
  auto dest = declare_parameter<std::string>("input_topic_name", "/robotperf/benchmark/image");
  auto cam_src = declare_parameter<std::string>("camera_info_in", "/camera/camera_info");
  auto cam_dest = declare_parameter<std::string>("camera_info_out", "/robotperf/benchmark/camera_info");

  sub_ = create_subscription<sensor_msgs::msg::Image>(
    src, rclcpp::QoS(10).reliable(),
    std::bind(&E4ImageInputNoKeyComponent::cb, this, std::placeholders::_1));
  pub_ = create_publisher<sensor_msgs::msg::Image>(
    dest, rclcpp::QoS(10).reliable());

  cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    cam_src, rclcpp::QoS(10).reliable(),
    [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { last_cam_info_ = msg; });
  cam_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    cam_dest, rclcpp::QoS(10).reliable());

  RCLCPP_INFO(get_logger(), "Image input (no-key): %s -> %s", src.c_str(), dest.c_str());
}

size_t E4ImageInputNoKeyComponent::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<sensor_msgs::msg::Image> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void E4ImageInputNoKeyComponent::cb(sensor_msgs::msg::Image::SharedPtr msg) {
  // Use original nanosec as key — do NOT modify the timestamp
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_image_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), 0);

  pub_->publish(*msg);
  if (last_cam_info_) {
    // Forward camera_info with ORIGINAL timestamp (preserves sync)
    cam_pub_->publish(*last_cam_info_);
  }

  TRACEPOINT(robotperf_image_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*msg)),
    static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec,
    get_msg_size(msg), 0);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::E4ImageInputNoKeyComponent)
