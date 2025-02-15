// Include C++ Libraries and ROS2 Packages
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Include custom header files
#include "tracetools_benchmark/tracetools.h"
#include "joint_trajectory_output_component.hpp"
#include <rclcpp/serialization.hpp>

// Include macro to register the node as a component with class_loader
#include "rclcpp_components/register_node_macro.hpp"

// Define namespace for the component
namespace robotperf
{

namespace control
{
    // Define the TrajectoryOutputComponent class which inherits from rclcpp::Node
    JointTrajectoryOutputComponent::JointTrajectoryOutputComponent(const rclcpp::NodeOptions & options)
    : rclcpp::Node("JointTrajectoryOutputComponent", options)
    {
        // Get the output topic name paramter from the parameter server with default value "/scan"
        std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name","/joint_trajectory");

        // Create a subscriber to listen to the laserscan messages
        // KeepLast(10) specifies subscriber stores last 10 messages received in case of disconnections
        // reliable() subscriber receives all the messages in order and without duplicates
        // best_effort() if subscriber can't keep up with messages being published, it will skip any messages it misses
        joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            output_topic_name, 
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(), 
            std::bind(&JointTrajectoryOutputComponent::jointTrajectoryCb, this, std::placeholders::_1)
        );


    }

    size_t JointTrajectoryOutputComponent::get_msg_size(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr joint_trajectory_msg){
        // Create SerializedMessage object which is a ROS 2 message type that can hold a serialized message
        rclcpp::SerializedMessage serialized_data_joint_trajectory;
        // Create Serialization object that is used to serialize a sensor_msgs::msg::LaserScan message
        rclcpp::Serialization<trajectory_msgs::msg::JointTrajectory> joint_trajectory_serialization;
        // Use reinterpret_cast to cast shared pointer to a const void pointer and create a pointer to trajectory_msg LaserScan message.
        const void* trajectory_ptr = reinterpret_cast<const void*>(joint_trajectory_msg.get());
        // Serialize the LaserScan message into the SerializedMessage object
        joint_trajectory_serialization.serialize_message(trajectory_ptr, &serialized_data_joint_trajectory);
        // Using the Serialization object, serialize the LaserScan message into the SerializedMessage object
        size_t joint_trajectory_msg_size = serialized_data_joint_trajectory.size();
        // Return the size of the serialized message
        return joint_trajectory_msg_size;
    }


    //Member function to handle the received LaserScan messages
    void JointTrajectoryOutputComponent::jointTrajectoryCb(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg)
    {
        TRACEPOINT(
            robotperf_joint_trajectory_output_cb_init,
            static_cast<const void *>(this),
            static_cast<const void *>(&(*joint_trajectory_msg)),
            joint_trajectory_msg->header.stamp.nanosec,
            joint_trajectory_msg->header.stamp.sec,
            get_msg_size(joint_trajectory_msg));

        TRACEPOINT(
            robotperf_joint_trajectory_output_cb_fini,    
            static_cast<const void *>(this),
            static_cast<const void *>(&(*joint_trajectory_msg)),
            joint_trajectory_msg->header.stamp.nanosec,
            joint_trajectory_msg->header.stamp.sec,
            get_msg_size(joint_trajectory_msg));

    }
} // namespace control

} // namespace robotperf

// Register component with class_loader
// Allows component to be discoverable when
// library is loaded to a running process
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::control::JointTrajectoryOutputComponent);