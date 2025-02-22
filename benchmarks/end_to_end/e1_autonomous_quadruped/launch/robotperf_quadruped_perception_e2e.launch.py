import os

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT

# NOTE: Make sure you comment out the pointcloud_to_laserscan node in the navigation.launch.py file

def generate_launch_description():

    trace = Trace(
        session_name= 'quadruped_control_test',
        events_ust=[
            "robotperf_benchmarks:*",
            "ros2:*",
            "robotcore_power:*",
            # "lttng_ust_cyg_profile*",
            # "lttng_ust_statedump*",
            # "liblttng-ust-libc-wrapper",
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        # events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
    )
    


    # # Place Input Tracepoint
    # pointcloud_input_node = Node(
    #             namespace="robotperf",
    #             package="e1_autonomous_quadruped",
    #             plugin="robotperf::perception::PointCloudInputComponent",
    #             name="pointcloud_input_component",
    #             parameters=[
    #                 {"input_topic_name":"/robotperf/benchmark/velodyne_points"} #publishing to this
    #             ],
    #             remappings=[
    #                 ("cloud", "/velodyne_points"),
    #             ], 
    #             # extra_arguments=[{'use_intra_process_comms': True}],
    # )


    # # Convert PointCloud to Laserscan (node of interest)
    # laserscan_node = Node(
    #         namespace='robotperf/benchmark',
    #         package='e1_autonomous_quadruped',
    #         plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
    #         name='pointcloud_to_laserscan',
    #         remappings=[
    #             ('cloud_in', '/robotperf/benchmark/velodyne_points'),
    #             ('scan', '/robotperf/benchmark/scan')
    #         ], 
    #         parameters=[ {'scan_time': 0.000000001, 
    #                     'qos': {'reliability': 'best_effort', 'durability': 'transient_local'}}
    #         ]
    # )


    # # Record Final Tracepoint once scan is produced
    # laserscan_input_node = Node(
    #             namespace="robotperf",
    #             package='e1_autonomous_quadruped', 
    #             plugin='robotperf::perception::LaserscanInputComponent', 
    #             name='laserscan_input_component',
    #             parameters=[
    #                 {'input_topic_name': '/robotperf/benchmark/scan'}, # subbing to this
    #                 {'output_topic_name': '/scan'}, # outputting this
    #             ],
    #             # extra_arguments=[{'use_intra_process_comms': True}], # Uses shared memory instead of DDS to transport messages
    # )

    
    # # Put your two composable nodes (input and output) into a container
    # composable_node_container = ComposableNodeContainer(
    #     name="end2end_container",
    #     package="rclcpp_components",
    #     namespace='',
    #     executable="component_container_mt",  # Single-threaded "component_container" or Mutli-threaded "component_container_mt"
    #     composable_node_descriptions=[
    #         pointcloud_input_node,
    #         laserscan_input_node,
    #         laserscan_node
    #     ],
    #     output="screen"
    # )


    # Run PointCloud Input Node as a separate process
    pointcloud_input_node = Node(
        namespace="robotperf",
        package="e1_autonomous_quadruped",
        executable="pointcloud_input_component_node",
        name="pointcloud_input_component",
        parameters=[
            {"input_topic_name": "/robotperf/benchmark/velodyne_points"}
        ],
        remappings=[
            ("cloud", "/velodyne_points"),
        ],
        output="screen",
    )

    # Run PointCloud-to-LaserScan Node as a separate process
    laserscan_node = Node(
        namespace='robotperf/benchmark',
        package='e1_autonomous_quadruped',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/robotperf/benchmark/velodyne_points'),
            ('scan', '/robotperf/benchmark/scan')
        ],
        parameters=[{
            'scan_time': 0.000000001,
            'qos': {'reliability': 'best_effort', 'durability': 'transient_local'}
        }],
        output="screen",
    )

    # Run LaserScan Input Node as a separate process
    laserscan_input_node = Node(
        namespace="robotperf",
        package='e1_autonomous_quadruped',
        executable='laserscan_input_component_node',
        name='laserscan_input_component',
        parameters=[
            {'input_topic_name': '/robotperf/benchmark/scan'},
            {'output_topic_name': '/scan'},
        ],
        output="screen",
    )

    # Build the final LaunchDescription
    ld = LaunchDescription()

    # Add the tracing tool and all separate nodes
    ld.add_action(trace)
    ld.add_action(pointcloud_input_node)
    ld.add_action(laserscan_node)
    ld.add_action(laserscan_input_node)

    return ld
