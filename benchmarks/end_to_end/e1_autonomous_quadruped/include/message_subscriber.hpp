#ifndef ROBOTPERF_MESSAGE_SUBSCRIBER_HPP_
#define ROBOTPERF_MESSAGE_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tracetools_benchmark/tracetools.h"

namespace robotperf
{
namespace benchmark
{

class MessageSubscriber : public rclcpp::Node
{
public:
    explicit MessageSubscriber(const rclcpp::NodeOptions & options);

private:
    // Callback functions for each message type
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void occupancygrid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    template<typename T>
    size_t get_msg_size(const T & msg);

    // Subscriptions for different message types
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
};

}  // namespace benchmark
}  // namespace robotperf

#endif  // ROBOTPERF_MESSAGE_SUBSCRIBER_HPP_