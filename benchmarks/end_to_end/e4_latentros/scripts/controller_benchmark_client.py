#!/usr/bin/env python3
"""
Benchmark action client for Nav2 ControllerServer (DWB local planner).

Sends a FollowPath action goal with a synthetic path.  The controller then
runs at its configured frequency (20 Hz), publishing cmd_vel each tick.
TwistOutputComponent traces each cmd_vel publication.

For the benchmark table:
  - T_in  = path arrival overhead (one-time per FollowPath)
  - T_K   = per-tick computeVelocityCommands latency  (≈ 1/controller_freq)
  - T_out = cmd_vel observation overhead

Usage:  Launched alongside controller_server + local_costmap + lifecycle_manager
        in trace_dwb_controller.launch.py.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import math


class ControllerBenchmarkClient(Node):
    def __init__(self):
        super().__init__('controller_benchmark_client')
        self.declare_parameter('num_paths', 50)
        self.declare_parameter('path_length', 20)
        self.declare_parameter('delay_between_paths', 2.0)

        self._action_client = ActionClient(
            self, FollowPath, 'follow_path')

        # Publish path on topic so PathInputComponent can trace it
        self._path_pub = self.create_publisher(
            Path, '/received_global_plan', 10)

        self._num_paths = self.get_parameter(
            'num_paths').get_parameter_value().integer_value
        self._path_length = self.get_parameter(
            'path_length').get_parameter_value().integer_value
        self._delay = self.get_parameter(
            'delay_between_paths').get_parameter_value().double_value

        self.create_timer(8.0, self._start_benchmark_once)
        self._started = False

    def _start_benchmark_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info('Waiting for controller action server...')
        self._action_client.wait_for_server()
        self.get_logger().info(
            f'Starting controller benchmark: {self._num_paths} paths, '
            f'DWB runs at controller_frequency between each')
        self._run_benchmark()

    def _make_path(self, idx):
        """Generate a simple straight-line path."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        angle = (idx * 0.5) % (2 * math.pi)
        for j in range(self._path_length):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = j * 0.1 * math.cos(angle)
            ps.pose.position.y = j * 0.1 * math.sin(angle)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def _run_benchmark(self):
        completed = 0
        # Each path lets DWB run for delay seconds at 20 Hz → ~delay*20 cmd_vel
        # 50 paths × 2s × 20Hz = 2000 cmd_vel samples
        # Increase num_paths or delay for more samples
        for i in range(self._num_paths):
            path = self._make_path(i)

            # Publish path for input component tracing
            self._path_pub.publish(path)

            goal = FollowPath.Goal()
            goal.path = path
            future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f'Path {i} rejected')
                continue

            # Let controller run for a while (produces cmd_vel at 20Hz)
            time.sleep(self._delay)

            # Cancel so we can send next path
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            completed += 1

            if completed % 10 == 0:
                self.get_logger().info(
                    f'Completed {completed}/{self._num_paths} paths')

        self.get_logger().info(
            f'Controller benchmark done: {completed} paths executed, '
            f'~{int(completed * self._delay * 20)} cmd_vel samples collected')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBenchmarkClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
