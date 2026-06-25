#ifndef E4_LATENTROS__POINTCLOUD_OUTPUT_COMPONENT_HPP_
#define E4_LATENTROS__POINTCLOUD_OUTPUT_COMPONENT_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robotperf { namespace perception {
class PointCloudOutputComponent : public rclcpp::Node {
public:
  explicit PointCloudOutputComponent(const rclcpp::NodeOptions & options);
private:
  void cb(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
}}
#endif
