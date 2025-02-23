import os

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():

    # Declare arguments
    message_type_arg = DeclareLaunchArgument(
        "message_type",
        default_value="pointcloud",
        description="Type of message to publish (pointcloud, laserscan, occupancy_grid, image, amcl_pose, twist_stamped, imu)"
    )

    # NOTE: Only pointcloud, laserscan, occupancy_grid, and image can have dffierent message sizes
    message_size_arg = DeclareLaunchArgument(
        "message_size",
        default_value="1000",
        description="Size of the message to publish"
    )

    # Tracing setup
    trace = Trace(
        session_name='message_benchmark_test',
        events_ust=[
            "robotperf_benchmarks:*",
            "ros2:*",
        ] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        }
    )

    # Message Publisher Node
    message_publisher_node = Node(
        namespace="robotperf",
        package="e1_autonomous_quadruped",
        executable="message_publisher_node",
        name="message_publisher",
        parameters=[
            {"message_type": LaunchConfiguration("message_type")},
            {"message_size": LaunchConfiguration("message_size")}
        ],
        output="screen",
    )

    # Message Subscriber Node
    message_subscriber_node = Node(
        namespace="robotperf",
        package="e1_autonomous_quadruped",
        executable="message_subscriber_node",
        name="message_subscriber",
        output="screen",
    )

    # Create LaunchDescription and add nodes
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(message_type_arg)
    ld.add_action(message_size_arg)

    # Add tracing tool
    ld.add_action(trace)

    # Add publisher and subscriber nodes
    ld.add_action(message_publisher_node)
    ld.add_action(message_subscriber_node)

    return ld
