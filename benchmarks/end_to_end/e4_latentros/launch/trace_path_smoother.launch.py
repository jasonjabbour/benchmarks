"""
Benchmark: Path -> Path  (Nav2 path smoother)
Kernel:    smoother_benchmark_node  (topic-driven, no lifecycle/action)
Session:   e4_path_smoother

Flow:
  rosbag /received_global_plan → PathInputComponent
    → /robotperf/benchmark/path_input → smoother_benchmark_node
    → /benchmark/smoothed_path → PathOutputComponent

6-tracepoint chain:
  path_input_cb_init/fini → msg_received_1 → msg_published_1
    → path_output_cb_init/fini

No costmap needed — pure geometric smoothing on path waypoints.

TODO: Create smoother_benchmark_node that wraps nav2_smoother
      (e.g., SimpleSmoother or ConstrainedSmoother).
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/end_to_end/latentros_end_to_end_1m'


def generate_launch_description():
    trace = Trace(
        session_name='e4_path_smoother',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--topics', '/received_global_plan', '/tf', '/tf_static'],
        output='screen',
    )

    # Input: subscribe to path from rosbag, republish to benchmark topic
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

    # Output: subscribe to smoothed path
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::planning::PathOutputComponent',
        name='path_output_component',
        parameters=[{'output_topic_name': '/benchmark/smoothed_path'}],
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

    # Kernel: simple smoother wrapper node (placeholder)
    # TODO: Create smoother_benchmark_node that subscribes to
    #       /robotperf/benchmark/path_input and publishes to
    #       /benchmark/smoothed_path
    kernel = Node(
        package='e4_latentros',
        executable='smoother_benchmark_node',
        name='smoother_benchmark_node',
        output='screen',
        parameters=[{
            'input_topic': '/robotperf/benchmark/path_input',
            'output_topic': '/benchmark/smoothed_path',
        }],
    )

    # Disable CycloneDDS shared memory
    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm,
        trace, rosbag_play, container, kernel,
    ])
