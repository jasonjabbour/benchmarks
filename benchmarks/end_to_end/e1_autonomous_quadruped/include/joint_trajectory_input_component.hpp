#ifndef ROBOTPERF_JOINT_TRAJECTORY_INPUT_COMPONENT_HPP_
#define ROBOTPERF_JOINT_TRAJECTORY_INPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace robotperf
{

namespace control
{

class JointTrajectoryInputComponent
  : public rclcpp::Node
{
public:
  explicit JointTrajectoryInputComponent(const rclcpp::NodeOptions &);

protected:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_trajectory_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_joint_trajectory_;

  size_t get_msg_size(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr joint_trajectory_msg);

  void jointTrajectoryCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg);

};

}  // namespace control

}  // namespace robotperf

#endif  // ROBOTPERF_JOINT_TRAJECTORY_INPUT_COMPONENT_HPP_