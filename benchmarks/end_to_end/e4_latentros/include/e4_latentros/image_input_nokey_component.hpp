#ifndef E4_LATENTROS__IMAGE_INPUT_NOKEY_COMPONENT_HPP_
#define E4_LATENTROS__IMAGE_INPUT_NOKEY_COMPONENT_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace robotperf { namespace perception {
/// Image input component that does NOT modify the timestamp.
/// For kernels using message_filters sync where changing nanosec
/// would break timestamp-based synchronization.
/// Tracepoints still fire with the original nanosec as the key.
class E4ImageInputNoKeyComponent : public rclcpp::Node {
public:
  explicit E4ImageInputNoKeyComponent(const rclcpp::NodeOptions & options);
private:
  void cb(sensor_msgs::msg::Image::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr msg);
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
  sensor_msgs::msg::CameraInfo::SharedPtr last_cam_info_;
};
}}
#endif
