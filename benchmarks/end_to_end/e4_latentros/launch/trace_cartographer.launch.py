"""
Benchmark: LaserScan -> OccupancyGrid  (Google Cartographer)
Kernel:    cartographer_ros/cartographer_node + cartographer_occupancy_grid_node
Session:   e4_cartographer

Flow:
  rosbag /scan → LaserscanInputComponent
    → /robotperf/benchmark/scan → cartographer_node
    → submap_list → cartographer_occupancy_grid_node
    → /map → OccupancyGridOutputComponent

6-tracepoint chain:
  laserscan_input_cb_init/fini → msg_received_1 → msg_published_1
    → occupancy_grid_output_cb_init/fini

Note: Cartographer is complex — requires .lua configuration files.
      The occupancy_grid_node publishes /map from submaps at a configurable rate.
      Config files may need to be created in e4_latentros/config/.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'
E4_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    # TODO: Create cartographer config .lua files in e4_latentros/config/
    # For now, point to the e4_latentros config directory
    cartographer_config_dir = os.path.join(E4_DIR, 'config')

    trace = Trace(
        session_name='e4_cartographer',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop', '--clock', '200',
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

    # Kernel: Cartographer SLAM node
    # TODO: Create cartographer_benchmark.lua config file
    kernel = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_benchmark.lua',
        ],
        remappings=[
            ('scan', '/robotperf/benchmark/scan'),
        ],
    )

    # Occupancy grid node — converts submaps to /map
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
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
        kernel, occupancy_grid, static_tf,
    ])
