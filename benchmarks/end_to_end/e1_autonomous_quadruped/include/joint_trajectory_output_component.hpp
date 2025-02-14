#ifndef ROBOTPERF_JOINT_TRAJECTORY_OUTPUT_COMPONENT_HPP_
#define ROBOTPERF_JOINT_TRAJECTORY_OUTPUT_COMPONENT_HPP_

// Include C++ Libraries and ROS2 Packages
#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


namespace robotperf
{

namespace control
{

class JointTrajectoryOutputComponent : public rclcpp::Node
{
    public:
        // Constructor
        explicit JointTrajectoryOutputComponent(const rclcpp::NodeOptions &options);

    private:
        // Member variable for subscriber to the trajectory messages
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub;

        // Member function to get size of message
        size_t get_msg_size(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr joint_trajectory_msg);
        // Member function to handle received trajectory messages
        void jointTrajectoryCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg);

};

} // namespace control
    
} // namespace robotperf

#endif

