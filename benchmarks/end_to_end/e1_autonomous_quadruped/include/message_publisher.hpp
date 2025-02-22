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

    // Publishers for different message types
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string message_type_;
    int message_size_;
};

}  // namespace benchmark
}  // namespace robotperf

#endif  // ROBOTPERF_MESSAGE_PUBLISHER_HPP_
