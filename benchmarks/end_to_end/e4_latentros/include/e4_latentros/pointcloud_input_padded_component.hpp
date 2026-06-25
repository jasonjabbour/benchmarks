#ifndef E4_LATENTROS__POINTCLOUD_INPUT_PADDED_COMPONENT_HPP_
#define E4_LATENTROS__POINTCLOUD_INPUT_PADDED_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robotperf { namespace perception {

/// PointCloud2 input component with message padding.
/// Gazebo simulated Velodyne produces ~100 KB point clouds, but real sensors
/// output much larger data (VLP-16: ~700 KB, VLP-32: ~1.5 MB, OS1-64: ~3 MB).
/// This component pads the PointCloud2 data to a configurable target size
/// so benchmarks reflect realistic transport overhead.
class PointCloudInputPaddedComponent : public rclcpp::Node
{
public:
  explicit PointCloudInputPaddedComponent(const rclcpp::NodeOptions & options);

private:
  void cb(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  size_t get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  uint32_t generate_unique_key();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  int target_msg_size_kb_;
};

}}  // namespace robotperf::perception

#endif
