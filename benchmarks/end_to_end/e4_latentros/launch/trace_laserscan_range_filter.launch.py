"""
Benchmark: LaserScan -> Filtered LaserScan  (laser_filters range filter)
Kernel:    laser_filters scan_to_scan_filter_chain
Session:   e4_laserscan_range_filter

Intercept pattern:
  rosbag /scan -> LaserscanInputComponent
    -> /robotperf/benchmark/scan -> scan_to_scan_filter_chain
    -> /robotperf/benchmark/scan_filtered -> LaserscanInputComponent -> /scan_filtered
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

# Dummy subscriber ensures output component's publisher has a
# downstream subscriber so the fini tracepoint fires.
DUMMY_SUB_CMD = [
    'ros2', 'topic', 'echo', '/scan_filtered',
    'sensor_msgs/msg/LaserScan', '--no-daemon', '--qos-reliability', 'best_effort',
]

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('e4_latentros'), 'config')
    range_filter_yaml = os.path.join(config_dir, 'range_filter.yaml')

    trace = Trace(
        session_name='e4_laserscan_range_filter',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/scan', '/tf', '/tf_static'],
        output='screen',
    )

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

    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::E4LaserscanOutputComponent',
        name='laserscan_output_component',
        parameters=[{
            'output_topic_name': '/robotperf/benchmark/scan_filtered',
        }],
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

    kernel = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        namespace='robotperf/benchmark',
        name='scan_to_scan_filter_chain',
        remappings=[
            ('scan', '/robotperf/benchmark/scan'),
            ('scan_filtered', '/robotperf/benchmark/scan_filtered'),
        ],
        parameters=[range_filter_yaml],
        output='screen',
    )

    # Dummy subscriber so output LaserscanInputComponent's fini tracepoint fires
    dummy_sub = ExecuteProcess(cmd=DUMMY_SUB_CMD, output='log')

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm, trace, rosbag_play, container, kernel, dummy_sub])
