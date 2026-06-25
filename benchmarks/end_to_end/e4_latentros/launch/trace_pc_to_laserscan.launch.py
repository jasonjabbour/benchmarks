"""
Benchmark: PointCloud2 -> LaserScan  (pointcloud_to_laserscan)
Kernel:    e1 forked pointcloud_to_laserscan_node  (has kernel tracepoints)
Session:   e4_pc_to_laserscan

Intercept pattern:
  rosbag /velodyne_points -> PointCloudInputComponent
    -> /robotperf/benchmark/velodyne_points -> pointcloud_to_laserscan_node
    -> /robotperf/benchmark/scan -> LaserscanInputComponent -> /scan

6-tracepoint chain (kernel has msg_received_1 / msg_published_1):
  pointcloud_input_cb_init/fini -> msg_received_1 -> msg_published_1
    -> laserscan_input_cb_init/fini
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

# Dummy subscriber ensures e1 output component's publisher has a
# downstream subscriber so the fini tracepoint fires.
DUMMY_SUB_CMD = [
    'ros2', 'topic', 'echo', '/scan',
    'sensor_msgs/msg/LaserScan', '--no-daemon', '--qos-reliability', 'best_effort',
]

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'


def generate_launch_description():
    trace = Trace(
        session_name='e4_pc_to_laserscan',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/velodyne_points', '/tf', '/tf_static'],
        output='screen',
    )

    input_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::perception::PointCloudInputComponent',
        name='pointcloud_input_component',
        parameters=[{
            'input_topic_name': '/robotperf/benchmark/velodyne_points',
        }],
        remappings=[('cloud', '/velodyne_points')],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    output_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::perception::LaserscanInputComponent',
        name='laserscan_output_component',
        parameters=[{
            'input_topic_name': '/robotperf/benchmark/scan',
            'output_topic_name': '/scan',
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
        package='e1_autonomous_quadruped',
        executable='pointcloud_to_laserscan_node',
        namespace='robotperf/benchmark',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/robotperf/benchmark/velodyne_points'),
            ('scan', '/robotperf/benchmark/scan'),
        ],
        parameters=[{'scan_time': 0.000000001}],
        output='screen',
    )

    # Dummy subscriber so LaserscanInputComponent's fini tracepoint fires
    dummy_sub = ExecuteProcess(cmd=DUMMY_SUB_CMD, output='log')

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm, trace, rosbag_play, container, kernel, dummy_sub])
