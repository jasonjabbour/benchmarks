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

// Include custom message headers for quantized versions.
#include "e3_custom_messages/msg/custom_laser_scan.hpp"
#include "e3_custom_messages/msg/custom_point_cloud2.hpp"
#include "e3_custom_messages/msg/custom_pose_with_covariance_stamped.hpp"
#include "e3_custom_messages/msg/custom_imu.hpp"

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
    void pointcloud_custom_callback(const e3_custom_messages::msg::CustomPointCloud2::SharedPtr msg);
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void laserscan_custom_callback(const e3_custom_messages::msg::CustomLaserScan::SharedPtr msg);
    void occupancygrid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void pose_custom_callback(const e3_custom_messages::msg::CustomPoseWithCovarianceStamped::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void imu_custom_callback(const e3_custom_messages::msg::CustomImu::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    template<typename T>
    size_t get_msg_size(const T & msg);

    // Subscriptions for different message types
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<e3_custom_messages::msg::CustomPointCloud2>::SharedPtr sub_pointcloud_custom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;
    rclcpp::Subscription<e3_custom_messages::msg::CustomLaserScan>::SharedPtr sub_laserscan_custom_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<e3_custom_messages::msg::CustomPoseWithCovarianceStamped>::SharedPtr sub_pose_custom_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<e3_custom_messages::msg::CustomImu>::SharedPtr sub_imu_custom_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
};

}  // namespace benchmark
}  // namespace robotperf

#endif  // ROBOTPERF_MESSAGE_SUBSCRIBER_HPP_
