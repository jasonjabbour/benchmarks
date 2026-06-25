"""
Benchmark: LaserScan -> OccupancyGrid  (SLAM Toolbox — async online)
Kernel:    slam_toolbox/async_slam_toolbox_node
Session:   e4_slam_toolbox

Flow:
  rosbag /scan → LaserscanInputComponent
    → /robotperf/benchmark/scan → async_slam_toolbox_node
    → /map → OccupancyGridOutputComponent

6-tracepoint chain:
  laserscan_input_cb_init/fini → msg_received_1 → msg_published_1
    → occupancy_grid_output_cb_init/fini

Note: SLAM is stateful — timing includes incremental map updates.
      Map output rate depends on slam_toolbox map_update_interval.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'


def generate_launch_description():
    trace = Trace(
        session_name='e4_slam_toolbox',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop', '--clock', '200',
             '--rate', '0.5',
             '--topics', '/scan', '/tf', '/tf_static'],
        output='screen',
    )

    # Input: intercept laser scans
    input_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::perception::LaserscanInputComponent',
        name='laserscan_input_component',
        parameters=[{
            'input_topic_name': '/scan',
            'output_topic_name': '/robotperf/benchmark/scan',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Output: trace map publications
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::mapping::OccupancyGridOutputComponent',
        name='occupancy_grid_output_component',
        parameters=[{'output_topic_name': '/map'}],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    container = ComposableNodeContainer(
        name='benchmark_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[input_component, output_component],
        output='screen',
    )

    # Kernel: SLAM Toolbox async node
    kernel = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/robotperf/benchmark/scan',
            'mode': 'mapping',
            'map_update_interval': 1.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_travel_distance': 0.0,
            'minimum_travel_heading': 0.0,
        }],
    )

    # Static TF: map→odom (identity) so TF chain resolves
    static_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='log',
    )

    # Disable CycloneDDS shared memory
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm,
        trace, rosbag_play, container,
        kernel, static_tf,
    ])
