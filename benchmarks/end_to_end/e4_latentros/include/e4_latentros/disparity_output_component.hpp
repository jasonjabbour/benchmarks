#ifndef E4_LATENTROS__DISPARITY_OUTPUT_COMPONENT_HPP_
#define E4_LATENTROS__DISPARITY_OUTPUT_COMPONENT_HPP_
#include <rclcpp/rclcpp.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

namespace robotperf { namespace perception {
class DisparityOutputComponent : public rclcpp::Node {
public:
  explicit DisparityOutputComponent(const rclcpp::NodeOptions & options);
private:
  void cb(stereo_msgs::msg::DisparityImage::SharedPtr msg);
  size_t get_msg_size(stereo_msgs::msg::DisparityImage::ConstSharedPtr msg);
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_;
};
}}
#endif
