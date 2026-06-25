"""
Benchmark: JointState -> Odometry  (CHAMP state estimation — forward kinematics)
Kernel:    champ_base/state_estimation_node  (timer-based 50 Hz output)
Session:   e4_state_estimation

Intercept pattern:
  rosbag /joint_states -> JointStateInputComponent
    -> /robotperf/benchmark/joint_states -> state_estimation_node
    -> /odom/raw -> OdometryOutputComponent

Note: Kernel is timer-based (50 Hz). Also needs /foot_contacts and /imu/data
      from the rosbag (not intercepted). Output count != input count.
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
        session_name='e4_state_estimation',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/joint_states', '/foot_contacts', '/imu/data',
             '/tf', '/tf_static'],
        output='screen',
    )

    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::localization::JointStateInputComponent',
        name='joint_state_input_component',
        parameters=[{
            'source_topic_name': '/joint_states',
            'input_topic_name': '/robotperf/benchmark/joint_states',
        }],
    extra_arguments=[{'use_intra_process_comms': False}],
    )

    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::localization::OdometryOutputComponent',
        name='odometry_output_component',
        parameters=[{'output_topic_name': '/odom/raw'}],
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
        package='champ_base',
        executable='state_estimation_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'orientation_from_imu': True},
            {'urdf': Command(['xacro ', default_model])},
            joints_config, links_config, gait_config,
        ],
        remappings=[
            ('joint_states', '/robotperf/benchmark/joint_states'),
        ],
    )

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm, trace, rosbag_play, container, kernel])
