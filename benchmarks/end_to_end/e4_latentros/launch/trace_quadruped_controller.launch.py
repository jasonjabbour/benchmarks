"""
Benchmark: Twist -> JointTrajectory  (CHAMP quadruped_controller — gait + IK)
Kernel:    e1 forked robotperf_quadruped_controller_node  (has kernel tracepoints)
Session:   e4_quadruped_controller

Intercept pattern:
  rosbag /cmd_vel -> TwistInputComponent
    -> /robotperf/benchmark/cmd_vel -> quadruped_controller_node
    -> /robotperf/benchmark/.../joint_trajectory -> JointTrajectoryInputComponent

6-tracepoint chain:
  twist_input_cb_init/fini -> msg_received_1 -> msg_published_1
    -> joint_trajectory_input_cb_init/fini
"""
import os
import launch_ros
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

# Dummy subscriber so JointTrajectoryInputComponent's fini tracepoint fires
DUMMY_SUB_CMD = [
    'ros2', 'topic', 'echo',
    '/joint_group_effort_controller/joint_trajectory',
    'trajectory_msgs/msg/JointTrajectory', '--no-daemon',
]

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/rosbag2_end_to_end_1m'


def generate_launch_description():
    config_pkg = launch_ros.substitutions.FindPackageShare(
        package='champ_config').find('champ_config')
    descr_pkg = launch_ros.substitutions.FindPackageShare(
        package='champ_description').find('champ_description')

    joints_config = os.path.join(config_pkg, 'config/joints/joints.yaml')
    gait_config = os.path.join(config_pkg, 'config/gait/gait.yaml')
    links_config = os.path.join(config_pkg, 'config/links/links.yaml')
    default_model = os.path.join(descr_pkg, 'urdf/champ.urdf.xacro')

    trace = Trace(
        session_name='e4_quadruped_controller',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/cmd_vel', '/tf', '/tf_static'],
        output='screen',
    )

    input_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::control::TwistInputComponent',
        name='twist_input_component',
        parameters=[{
            'output_topic_name': '/robotperf/benchmark/cmd_vel',
            'twist_type': 'Twist',
        }],
        remappings=[('cmd_vel', '/cmd_vel')],
    extra_arguments=[{'use_intra_process_comms': False}],
    )

    output_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::control::JointTrajectoryInputComponent',
        name='joint_trajectory_output_component',
        parameters=[{
            'input_topic_name':
                '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory',
            'output_topic_name':
                '/joint_group_effort_controller/joint_trajectory',
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
        executable='robotperf_quadruped_controller_node',
        namespace='robotperf/benchmark',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'gazebo': False},
            {'publish_joint_states': False},
            {'publish_joint_control': True},
            {'publish_foot_contacts': False},
            {'joint_controller_topic':
                'joint_group_effort_controller/joint_trajectory'},
            {'twist_type': 'Twist'},
            {'control_mode': 'event_based'},
            {'urdf': Command(['xacro ', default_model])},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=[
            ('cmd_vel/smooth', '/robotperf/benchmark/cmd_vel'),
            ('joint_group_effort_controller/joint_trajectory',
             '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory'),
        ],
    )

    dummy_sub = ExecuteProcess(cmd=DUMMY_SUB_CMD, output='log')

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm, trace, rosbag_play, container, kernel, dummy_sub])
