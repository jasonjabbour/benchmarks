#ifndef E4_LATENTROS__IMAGE_OUTPUT_COMPONENT_HPP_
#define E4_LATENTROS__IMAGE_OUTPUT_COMPONENT_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace robotperf { namespace perception {
class E4ImageOutputComponent : public rclcpp::Node {
public:
  explicit E4ImageOutputComponent(const rclcpp::NodeOptions & options);
private:
  void cb(sensor_msgs::msg::Image::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
}}
#endif
