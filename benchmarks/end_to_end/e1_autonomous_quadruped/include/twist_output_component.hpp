#ifndef ROBOTPERF_TRAJECTORY_OUTPUT_COMPONENT_HPP_
#define ROBOTPERF_TRAJECTORY_OUTPUT_COMPONENT_HPP_

// Include C++ Libraries and ROS2 Packages
#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace robotperf
{

namespace perception
{

class TrajectoryOutputComponent : public rclcpp::Node
{
    public:
        // Constructor
        explicit TrajectoryOutputComponent(const rclcpp::NodeOptions &options);

    private:
        // Member variable for subscriber to the trajectory messages
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr trajectory_sub;

        // Member function to get size of message
        size_t get_msg_size(geometry_msgs::msg::Twist::ConstSharedPtr trajectory_msg);
        // Member function to handle received trajectory messages
        void trajectoryCb(const geometry_msgs::msg::Twist::SharedPtr trajectory_msg);

};

} // namespace perception
    
} // namespace robotperf

#endif

