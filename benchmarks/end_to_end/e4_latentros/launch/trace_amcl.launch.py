"""
Benchmark: LaserScan -> PoseWithCovarianceStamped  (Nav2 AMCL — Particle Filter)
Kernel:    nav2_amcl/amcl  (lifecycle node)
Session:   e4_amcl

Intercept pattern:
  rosbag /scan -> LaserscanInputComponent
    -> /robotperf/benchmark/scan -> amcl (scan_topic overridden)
    -> /amcl_pose -> PoseWithCovarianceOutputComponent

Fixes applied:
  - update_min_d=0.0, update_min_a=0.0  → force pose update on every scan
  - Lifecycle manager auto-starts map_server + amcl
  - Map loaded from go2_config/maps_manual/square_map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/rosbag2_end_to_end_1m'


def generate_launch_description():
    go2_share = get_package_share_directory('go2_config')
    map_yaml = os.path.join(go2_share, 'maps_manual', 'square_map', 'square_map.yaml')
    nav_params = os.path.join(go2_share, 'config', 'navigation', 'nav2_params.yaml')

    trace = Trace(
        session_name='e4_amcl',
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
        plugin='robotperf::localization::PoseWithCovarianceOutputComponent',
        name='pose_cov_output_component',
        parameters=[{'output_topic_name': '/amcl_pose'}],
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

    # Map server (lifecycle)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
    )

    # Kernel: AMCL (lifecycle)
    # Override scan_topic + force update on every scan
    kernel = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav_params,
            {
                'use_sim_time': True,
                'scan_topic': '/robotperf/benchmark/scan',
                'update_min_d': 0.0,     # Force update on every scan
                'update_min_a': 0.0,     # Force update on every scan
                'set_initial_pose': True,
                'initial_pose': {
                    'x': 0.0, 'y': 0.0, 'z': 0.0,
                    'yaw': 0.0,
                },
            },
        ],
    )

    # Lifecycle manager — auto-activate map_server + amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # Disable CycloneDDS shared memory — force full UDP serialization
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    # Static TF: map→odom (identity) so AMCL TF chain resolves
    static_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='log',
    )

    return LaunchDescription([disable_cyclone_shm,
        trace, rosbag_play, container,
        map_server, kernel, lifecycle_manager, static_tf,
    ])
