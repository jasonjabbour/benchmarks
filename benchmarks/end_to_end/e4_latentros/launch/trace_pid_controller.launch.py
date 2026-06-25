"""
Benchmark: JointTrajectory -> JointState
           (ros2_control joint_trajectory_controller — PID)
Kernel:    controller_manager/ros2_control_node + joint_trajectory_controller
Session:   e4_pid_controller

Intercept pattern:
  rosbag /joint_group_effort_controller/joint_trajectory
    -> JointTrajectoryInputComponent
    -> /robotperf/benchmark/joint_trajectory
    -> ros2_control_node (joint_trajectory_controller)
    -> /joint_states (via joint_state_broadcaster) -> JointStateOutputComponent

Fix: Uses mock_components/GenericSystem fake hardware (no Gazebo needed).
     Minimal URDF in e4_latentros/config/go2_mock_hardware.urdf.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/rosbag2_end_to_end_1m'
E4_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    urdf_path = os.path.join(E4_DIR, 'config', 'go2_mock_hardware.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ros_control_config = os.path.join(
        E4_DIR, 'config', 'go2_ros_control_benchmark.yaml')

    trace = Trace(
        session_name='e4_pid_controller',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics',
             '/joint_group_effort_controller/joint_trajectory',
             '/tf', '/tf_static'],
        output='screen',
    )

    input_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::control::JointTrajectoryInputComponent',
        name='joint_trajectory_input_component',
        parameters=[{
            'input_topic_name':
                '/joint_group_effort_controller/joint_trajectory',
            'output_topic_name':
                '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory',
        }],
    extra_arguments=[{'use_intra_process_comms': False}],
    )

    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::control::JointStateOutputComponent',
        name='joint_state_output_component',
        parameters=[{'output_topic_name': '/joint_states'}],
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

    # Kernel: ros2_control_node with mock hardware
    kernel = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            ros_control_config,
        ],
        remappings=[
            ('joint_group_effort_controller/joint_trajectory',
             '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory'),
        ],
    )

    # Spawn controllers after controller_manager is ready
    spawn_jsb = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )],
    )
    spawn_jtc = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_effort_controller'],
            output='screen',
        )],
    )

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm, 
        trace, rosbag_play, container,
        kernel, spawn_jsb, spawn_jtc,
    ])
