#!/usr/bin/env python3
"""
Benchmark action client for Nav2 PlannerServer.

Publishes goals on /goal_pose for PoseStampedInputComponent to trace and key.
Subscribes to /robotperf/benchmark/goal_pose to get the keyed goal back.
Sends the keyed goal via ComputePathToPose action.
Re-publishes the result path on /global_plan for PathOutputComponent.

Full 6-tracepoint chain:
  pose_stamped_input_cb_init/fini -> msg_received_1 -> msg_published_1
    -> path_output_cb_init/fini
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import threading


class PlannerBenchmarkClient(Node):
    def __init__(self):
        super().__init__('planner_benchmark_client')
        self.declare_parameter('num_iterations', 5000)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('delay_between_goals', 0.05)

        self._action_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')

        # Publish raw goals for PoseStampedInputComponent to key
        self._goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        # Subscribe to keyed goals from PoseStampedInputComponent
        self._keyed_goal = None
        self._keyed_goal_event = threading.Event()
        self._keyed_sub = self.create_subscription(
            PoseStamped, '/robotperf/benchmark/goal_pose',
            self._keyed_goal_cb, 10)

        # Re-publish result path for PathOutputComponent
        self._path_pub = self.create_publisher(Path, '/global_plan', 10)

        self._num_iterations = self.get_parameter(
            'num_iterations').get_parameter_value().integer_value
        self._goal_x = self.get_parameter(
            'goal_x').get_parameter_value().double_value
        self._goal_y = self.get_parameter(
            'goal_y').get_parameter_value().double_value
        self._delay = self.get_parameter(
            'delay_between_goals').get_parameter_value().double_value

    def _keyed_goal_cb(self, msg):
        self._keyed_goal = msg
        self._keyed_goal_event.set()

    def _wait_for_keyed_goal(self, timeout=5.0):
        """Spin until we receive the keyed goal from input component."""
        self._keyed_goal = None
        self._keyed_goal_event.clear()
        end_time = time.monotonic() + timeout
        while not self._keyed_goal_event.is_set():
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return None
            rclpy.spin_once(self, timeout_sec=min(0.01, remaining))
        return self._keyed_goal

    def run_benchmark(self):
        self.get_logger().info('Waiting for planner action server...')
        self._action_client.wait_for_server()
        self.get_logger().info(
            f'Starting planner benchmark: {self._num_iterations} iterations')

        completed = 0
        for i in range(self._num_iterations):
            # Build raw goal
            raw_goal = PoseStamped()
            raw_goal.header.frame_id = 'map'
            raw_goal.header.stamp = self.get_clock().now().to_msg()
            if i % 2 == 0:
                raw_goal.pose.position.x = self._goal_x
                raw_goal.pose.position.y = self._goal_y
            else:
                raw_goal.pose.position.x = -self._goal_x
                raw_goal.pose.position.y = -self._goal_y
            raw_goal.pose.orientation.w = 1.0

            # Publish for PoseStampedInputComponent to key
            self._goal_pub.publish(raw_goal)

            # Wait for keyed goal back from input component
            keyed = self._wait_for_keyed_goal(timeout=5.0)
            if keyed is None:
                self.get_logger().warn(f'Goal {i}: no keyed goal received')
                continue

            # Send action with the key from input component
            action_goal = ComputePathToPose.Goal()
            action_goal.goal = keyed
            action_goal.use_start = True
            action_goal.start.header.frame_id = 'map'
            action_goal.start.header.stamp = keyed.header.stamp
            action_goal.start.pose.position.x = 0.0
            action_goal.start.pose.position.y = 0.0
            action_goal.start.pose.orientation.w = 1.0

            send_future = self._action_client.send_goal_async(action_goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f'Goal {i} rejected')
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            result = result_future.result()
            if result and result.result and result.result.path.poses:
                # Re-publish path with the key for output component
                self._path_pub.publish(result.result.path)
            completed += 1

            if completed % 500 == 0:
                self.get_logger().info(f'Completed {completed}/{self._num_iterations}')

            time.sleep(self._delay)

        self.get_logger().info(
            f'Planner benchmark done: {completed}/{self._num_iterations} succeeded')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerBenchmarkClient()
    node.run_benchmark()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
