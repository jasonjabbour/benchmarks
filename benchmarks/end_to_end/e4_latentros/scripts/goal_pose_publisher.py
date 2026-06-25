#!/usr/bin/env python3
"""
Publishes PoseStamped goals on /goal_pose at a configurable rate.
For use with topic-driven planner benchmarks (NavFn, Theta*).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)

        rate = self.get_parameter('rate').get_parameter_value().double_value
        self._goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self._goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        self._pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._count = 0
        self.get_logger().info(
            f'Publishing goals at {rate} Hz to ({self._goal_x}, {self._goal_y})')

    def _publish(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # Alternate between two positions for variety
        if self._count % 2 == 0:
            msg.pose.position.x = self._goal_x
            msg.pose.position.y = self._goal_y
        else:
            msg.pose.position.x = -self._goal_x
            msg.pose.position.y = -self._goal_y
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GoalPosePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
