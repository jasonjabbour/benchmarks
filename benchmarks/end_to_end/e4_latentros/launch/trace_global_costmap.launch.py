"""
Benchmark: LaserScan -> OccupancyGrid  (Nav2 global costmap — Costmap2D)
Kernel:    nav2_costmap_2d  (lifecycle, fixed-rate publish at 10 Hz)
Session:   e4_global_costmap

Intercept pattern:
  rosbag /scan -> LaserscanInputComponent
    -> /robotperf/benchmark/scan -> global_costmap (obstacle_layer.scan.topic)
    -> /global_costmap/costmap -> OccupancyGridOutputComponent

Note: Costmap publishes at publish_frequency (10 Hz), not per-scan.
      5000 costmaps ≈ 8.3 min of runtime.
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
    e4_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    costmap_params = os.path.join(e4_dir, 'config', 'global_costmap_benchmark.yaml')

    trace = Trace(
        session_name='e4_global_costmap',
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
        plugin='robotperf::mapping::OccupancyGridOutputComponent',
        name='occupancy_grid_output_component',
        parameters=[{'output_topic_name': '/costmap/costmap'}],
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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
    )

    # Kernel: global costmap (lifecycle)
    # nav2_costmap_2d standalone executable creates node "costmap" with
    # internal child also named "costmap" → lifecycle at /costmap/costmap.
    # Params are loaded from global_costmap.global_costmap namespace in YAML.
    kernel = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[costmap_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server', 'costmap/costmap'],
        }],
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
        map_server, kernel, lifecycle_manager,
        # Dummy static TF: map→odom (identity) so costmap TF chain resolves
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='log',
        ),
    ])
