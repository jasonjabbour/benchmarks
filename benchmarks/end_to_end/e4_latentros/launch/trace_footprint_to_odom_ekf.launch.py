"""
Benchmark: Odometry -> Odometry
           (robot_localization EKF — footprint_to_odom fusion)
Kernel:    robot_localization/ekf_node  (fixed-rate 200 Hz output)
Session:   e4_footprint_to_odom_ekf

Intercept pattern:
  rosbag /odom/raw -> OdometryInputComponent
    -> /robotperf/benchmark/odom_input -> ekf_node (odom0 overridden)
    -> /odom (odometry/filtered remapped) -> OdometryOutputComponent

Note: EKF runs at 200 Hz and fuses odom + IMU. IMU from bag.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/rosbag2_end_to_end_1m'
E4_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    ekf_config = os.path.join(
        E4_DIR, 'config', 'ekf_footprint_to_odom_benchmark.yaml')

    trace = Trace(
        session_name='e4_footprint_to_odom_ekf',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/odom/raw', '/imu/data', '/tf', '/tf_static'],
        output='screen',
    )

    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::localization::OdometryInputComponent',
        name='odometry_input_component',
        parameters=[{
            'source_topic_name': '/odom/raw',
            'input_topic_name': '/robotperf/benchmark/odom_input',
        }],
    extra_arguments=[{'use_intra_process_comms': False}],
    )

    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::localization::OdometryOutputComponent',
        name='odometry_output_component',
        parameters=[{'output_topic_name': '/odom'}],
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
        package='robot_localization',
        executable='ekf_node',
        name='footprint_to_odom_ekf',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False},
        ],
        remappings=[('odometry/filtered', 'odom')],
    )

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm, trace, rosbag_play, container, kernel])
