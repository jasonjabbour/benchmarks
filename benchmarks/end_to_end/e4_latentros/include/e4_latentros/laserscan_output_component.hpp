#ifndef E4_LATENTROS__LASERSCAN_OUTPUT_COMPONENT_HPP_
#define E4_LATENTROS__LASERSCAN_OUTPUT_COMPONENT_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace robotperf { namespace perception {
/// Pure observer — subscribes to LaserScan, fires output tracepoints, does NOT republish.
class E4LaserscanOutputComponent : public rclcpp::Node {
public:
  explicit E4LaserscanOutputComponent(const rclcpp::NodeOptions & options);
private:
  void cb(sensor_msgs::msg::LaserScan::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};
}}
#endif
