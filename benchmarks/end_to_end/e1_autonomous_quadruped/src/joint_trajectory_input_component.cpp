#include <rclcpp/rclcpp.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "joint_trajectory_input_component.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{

namespace control
{

JointTrajectoryInputComponent::JointTrajectoryInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("JointTrajectoryInputComponent", options)
{
  // Get the input_topic_name parameter from the parameter server with default value "input"
  std::string input_topic_name = this->declare_parameter<std::string>("input_topic_name", "input");
  std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name", "output");

  // Create a joint trajectory publisher
  pub_joint_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    output_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
  );

  // Create a joint trajectory subscriber
  sub_joint_trajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    input_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&JointTrajectoryInputComponent::jointTrajectoryCb, this, std::placeholders::_1)
  );
}

size_t JointTrajectoryInputComponent::get_msg_size(
  trajectory_msgs::msg::JointTrajectory::ConstSharedPtr joint_trajectory_msg)
{
  // Serialize the JointTrajectory message
  rclcpp::SerializedMessage serialized_data_joint_trajectory;
  rclcpp::Serialization<trajectory_msgs::msg::JointTrajectory> joint_trajectory_serialization;
  const void * joint_trajectory_ptr = reinterpret_cast<const void *>(joint_trajectory_msg.get());
  joint_trajectory_serialization.serialize_message(joint_trajectory_ptr, &serialized_data_joint_trajectory);
  return serialized_data_joint_trajectory.size();
}

void JointTrajectoryInputComponent::jointTrajectoryCb(
  trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg)
{

    TRACEPOINT(
        robotperf_joint_trajectory_input_cb_init,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*joint_trajectory_msg)),
        joint_trajectory_msg->header.stamp.nanosec,
        joint_trajectory_msg->header.stamp.sec,
        get_msg_size(joint_trajectory_msg));

    // If no subscribers, do nothing
    if (pub_joint_trajectory_->get_subscription_count() < 1) {
        return;
    }

    // Republish the incoming JointTrajectory
    pub_joint_trajectory_->publish(*joint_trajectory_msg);

    TRACEPOINT(
        robotperf_joint_trajectory_input_cb_fini,    
        static_cast<const void *>(this),
        static_cast<const void *>(&(*joint_trajectory_msg)),
        joint_trajectory_msg->header.stamp.nanosec,
        joint_trajectory_msg->header.stamp.sec,
        get_msg_size(joint_trajectory_msg));
    }

}  // namespace control

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::control::JointTrajectoryInputComponent)