"""
Benchmark: LaserScan -> OccupancyGrid  (Nav2 local costmap — VoxelLayer)
Kernel:    nav2_costmap_2d  (lifecycle, fixed-rate publish at 2 Hz)
Session:   e4_local_costmap

Intercept pattern:
  rosbag /scan -> LaserscanInputComponent
    -> /robotperf/benchmark/scan -> local_costmap (voxel_layer.scan.topic)
    -> /local_costmap/costmap -> OccupancyGridOutputComponent

Note: Local costmap publishes at 2 Hz. 5000 costmaps ≈ 42 min.
      Increase publish_frequency for faster benchmarking.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/rosbag2_end_to_end_1m'


def generate_launch_description():
    e4_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    costmap_params = os.path.join(e4_dir, 'config', 'local_costmap_benchmark.yaml')

    trace = Trace(
        session_name='e4_local_costmap',
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

    # Kernel: local costmap (lifecycle, no static map needed)
    # nav2_costmap_2d standalone executable creates node "costmap" with
    # internal child also named "costmap" → lifecycle at /costmap/costmap.
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
            'node_names': ['costmap/costmap'],
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
        kernel, lifecycle_manager,
        # Dummy static TF: map→odom + odom→base_footprint (identity)
        # so costmap TF chain resolves for benchmarking
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='log',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='log',
        ),
    ])
