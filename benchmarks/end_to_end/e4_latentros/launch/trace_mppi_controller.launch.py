"""
Benchmark: Path -> Twist  (Nav2 MPPI Controller)
Kernel:    mppi_benchmark_node  (topic-driven wrapper around MPPI)
Session:   e4_mppi_controller

Flow:
  rosbag /received_global_plan → PathInputComponent
    → /robotperf/benchmark/path_input → mppi_benchmark_node
    → /cmd_vel → TwistOutputComponent

6-tracepoint chain:
  path_input_cb_init/fini → msg_received_1 → msg_published_1
    → twist_output_cb_init/fini

Note: MPPI requires a local costmap internally for obstacle avoidance.
      The benchmark wrapper needs to set up a costmap subscriber or
      embed a costmap layer. For now this is a placeholder.

TODO: Create mppi_benchmark_node that wraps the MPPI critic pipeline
      with an embedded local costmap, subscribing to /scan and /odom.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'
E4_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    go2_share = get_package_share_directory('go2_config')
    nav_params = os.path.join(go2_share, 'config', 'navigation', 'nav2_params.yaml')
    local_costmap_params = os.path.join(E4_DIR, 'config', 'local_costmap_benchmark.yaml')

    trace = Trace(
        session_name='e4_mppi_controller',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    # TF + scan (for local costmap) + odom (for controller pose lookups)
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop', '--clock', '200',
             '--topics', '/scan', '/odom',
             '/received_global_plan', '/tf', '/tf_static'],
        output='screen',
    )

    # Input: trace path arrivals
    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::planning::PathInputComponent',
        name='path_input_component',
        parameters=[{
            'source_topic_name': '/received_global_plan',
            'input_topic_name': '/robotperf/benchmark/path_input',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Output: trace cmd_vel publications
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::planning::TwistOutputComponent',
        name='twist_output_component',
        parameters=[{'output_topic_name': '/cmd_vel'}],
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

    # Local costmap for MPPI obstacle avoidance
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[
            local_costmap_params,
            {'use_sim_time': True},
        ],
    )

    # Kernel: MPPI benchmark wrapper node (placeholder)
    # TODO: Create mppi_benchmark_node that wraps the MPPI controller
    #       Subscribes to /robotperf/benchmark/path_input (Path)
    #       Publishes to /cmd_vel (Twist)
    #       Uses local costmap for obstacle-aware control
    kernel = Node(
        package='e4_latentros',
        executable='mppi_benchmark_node',
        name='mppi_benchmark_node',
        output='screen',
        parameters=[
            nav_params,
            {
                'use_sim_time': True,
                'input_topic': '/robotperf/benchmark/path_input',
                'output_topic': '/cmd_vel',
            },
        ],
    )

    # Static TF for costmap
    static_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='log',
    )

    # Disable CycloneDDS shared memory
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm,
        trace, rosbag_play, container,
        local_costmap, kernel, static_tf,
    ])
