"""
Benchmark: Twist -> JointState  (ros2_control diff_drive_controller)
Kernel:    diff_drive_controller (forked with tracepoints)
Session:   e4_diff_drive_controller

Flow:
  rosbag /cmd_vel -> TwistInputComponent (e1_autonomous_quadruped)
    -> /diff_drive_controller/cmd_vel_unstamped
    -> ros2_control_node (diff_drive_controller plugin)
    -> /diff_drive_controller/joint_commands -> JointStateOutputComponent

Uses mock_components/GenericSystem fake hardware (no Gazebo needed).

6-tracepoint chain:
  twist_output_cb_init/fini -> msg_received_1 -> msg_published_1
    -> odometry_output_cb_init/fini
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'
E4_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    urdf_path = os.path.join(E4_DIR, 'config', 'diff_drive_mock_hardware.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ros_control_config = os.path.join(
        E4_DIR, 'config', 'diff_drive_ros_control_benchmark.yaml')

    trace = Trace(
        session_name='e4_diff_drive_controller',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/cmd_vel'],
        output='screen',
    )

    # Input: intercept Twist from rosbag, stamp key, forward
    input_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::control::TwistInputComponent',
        name='twist_input_component',
        remappings=[('cmd_vel', '/cmd_vel')],
        parameters=[{
            'output_topic_name': '/diff_drive_controller/cmd_vel_unstamped',
            'twist_type': 'Twist',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Output: observe wheel commands published by forked diff_drive_controller
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::control::JointStateOutputComponent',
        name='joint_state_output_component',
        parameters=[{'output_topic_name': '/diff_drive_controller/joint_commands'}],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    container = ComposableNodeContainer(
        name='benchmark_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[input_component, output_component],
        output='screen',
    )

    # Kernel: ros2_control_node with mock hardware
    kernel = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            ros_control_config,
        ],
    )

    # Spawn controllers
    spawn_jsb = TimerAction(period=3.0, actions=[Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )])
    spawn_ddc = TimerAction(period=5.0, actions=[Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )])

    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm,
        trace, rosbag_play, container,
        kernel, spawn_jsb, spawn_ddc,
    ])
