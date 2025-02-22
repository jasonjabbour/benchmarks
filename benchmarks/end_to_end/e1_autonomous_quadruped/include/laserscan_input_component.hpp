#ifndef ROBOTPERF_LASERSCAN_INPUT_COMPONENT_HPP_
#define ROBOTPERF_LASERSCAN_INPUT_COMPONENT_HPP_

// Include C++ Libraries and ROS2 Packages
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace robotperf
{

namespace perception
{

class LaserscanInputComponent : public rclcpp::Node
{
public:
  explicit LaserscanInputComponent(const rclcpp::NodeOptions &);

protected:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;

  size_t get_msg_size(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  void laserscanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPERF_LASERSCAN_INPUT_COMPONENT_HPP_
