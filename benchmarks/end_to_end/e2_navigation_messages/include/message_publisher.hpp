#ifndef ROBOTPERF_MESSAGE_PUBLISHER_HPP_
#define ROBOTPERF_MESSAGE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tracetools_benchmark/tracetools.h"

// Include custom message headers from the correct package
#include "e3_custom_messages/msg/custom_laser_scan.hpp"
#include "e3_custom_messages/msg/custom_point_cloud2.hpp"
#include "e3_custom_messages/msg/custom_pose_with_covariance_stamped.hpp"
#include "e3_custom_messages/msg/custom_imu.hpp"

namespace robotperf
{
namespace benchmark
{

class MessagePublisher : public rclcpp::Node
{
public:
    explicit MessagePublisher(const rclcpp::NodeOptions & options);

private:
    void publish_message();
    uint32_t generate_unique_key();

    template<typename T>
    size_t get_msg_size(const T & msg);

    // Quantization Function
    uint16_t float64_to_float16(double f);

    // Publishers for different message types

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;
    
    // Standard and quantized message publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
    rclcpp::Publisher<e3_custom_messages::msg::CustomLaserScan>::SharedPtr pub_laserscan_custom_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<e3_custom_messages::msg::CustomPointCloud2>::SharedPtr pub_pointcloud_custom_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<e3_custom_messages::msg::CustomPoseWithCovarianceStamped>::SharedPtr pub_pose_custom_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<e3_custom_messages::msg::CustomImu>::SharedPtr pub_imu_custom_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string message_type_;
    int message_size_;
    bool quantization_enabled_;
};

}  // namespace benchmark
}  // namespace robotperf

#endif  // ROBOTPERF_MESSAGE_PUBLISHER_HPP_
