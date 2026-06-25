"""
Benchmark: PoseStamped -> Path  (OMPL RRT-Connect sampling-based planner)
Kernel:    rrt_benchmark_node  (topic-driven, no lifecycle/action)
Session:   e4_rrt_planner

Flow:
  goal_pose_publisher.py generates goals at 5Hz
    → /goal_pose → PoseStampedInputComponent
    → /robotperf/benchmark/goal_pose → rrt_benchmark_node
    → /global_plan → PathOutputComponent

6-tracepoint chain:
  pose_stamped_input_cb_init/fini → msg_received_1 → msg_published_1
    → path_output_cb_init/fini
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    go2_share = get_package_share_directory('go2_config')
    map_yaml = os.path.join(go2_share, 'maps_manual', 'square_map', 'square_map.yaml')

    trace = Trace(
        session_name='e4_rrt_planner',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::planning::PoseStampedInputComponent',
        name='pose_stamped_input_component',
        parameters=[{
            'source_topic_name': '/goal_pose',
            'input_topic_name': '/robotperf/benchmark/goal_pose',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::planning::PathOutputComponent',
        name='path_output_component',
        parameters=[{'output_topic_name': '/global_plan'}],
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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
    )
    configure_map = TimerAction(period=2.0, actions=[ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', 'map_server', 'configure'],
        output='screen')])
    activate_map = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', 'map_server', 'activate'],
        output='screen')])

    kernel = Node(
        package='e4_latentros',
        executable='rrt_benchmark_node',
        output='screen',
    )

    # 5Hz goals — RRT-Connect takes ~10-50ms per plan
    goal_publisher = TimerAction(period=6.0, actions=[Node(
        package='e4_latentros',
        executable='goal_pose_publisher.py',
        name='goal_pose_publisher',
        output='screen',
        parameters=[{'rate': 5.0, 'goal_x': 2.0, 'goal_y': 0.0}],
    )])

    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([disable_cyclone_shm,
        trace, container,
        map_server, configure_map, activate_map,
        kernel, goal_publisher,
    ])
