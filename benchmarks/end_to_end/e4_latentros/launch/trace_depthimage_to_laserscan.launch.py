"""
Benchmark: Depth Image -> LaserScan  (depthimage_to_laserscan)
Kernel:    depthimage_to_laserscan::DepthImageToLaserScanROS
Session:   e4_depthimage_to_laserscan

Flow:
  rosbag d455_1_depth_image -> E4ImageInputComponent
    -> /benchmark/depth_image -> DepthImageToLaserScanROS
    -> /benchmark/scan -> LaserscanInputComponent -> /scan
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/perception/r2b_cafe'


def generate_launch_description():
    trace = Trace(
        session_name='e4_depthimage_to_laserscan',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--remap',
             'd455_1_depth_image:=/camera/depth/image_raw',
             'd455_1_depth_camera_info:=/camera/depth/camera_info'],
        output='screen',
    )

    # E4 input: plain rclcpp, reliable sub from rosbag, SensorDataQoS pub
    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::E4ImageInputComponent',
        name='image_input_component',
        parameters=[{
            'source_topic_name': '/camera/depth/image_raw',
            'input_topic_name': '/benchmark/depth_image',
            'camera_info_in': '/camera/depth/camera_info',
            'camera_info_out': '/benchmark/depth_camera_info',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Output: e1 LaserscanInputComponent observes kernel scan output
    output_component = ComposableNode(
        package='e1_autonomous_quadruped',
        namespace='robotperf',
        plugin='robotperf::perception::LaserscanInputComponent',
        name='laserscan_output_component',
        parameters=[{
            'input_topic_name': '/benchmark/scan',
            'output_topic_name': '/scan',
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

    # Kernel: DepthImageToLaserScan as separate process
    # Subscribes to depth (image_transport) and publishes scan
    kernel = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'output_frame_id': 'd455_1_depth_optical_frame',
        }],
        remappings=[
            ('depth', '/benchmark/depth_image'),
            ('depth_camera_info', '/benchmark/depth_camera_info'),
            ('scan', '/benchmark/scan'),
        ],
    )

    # Dummy subscriber so LaserscanInputComponent's fini tracepoint fires
    dummy_sub = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/scan',
             'sensor_msgs/msg/LaserScan', '--no-daemon'],
        output='log',
    )

    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    # Delay kernel start so input component loads first and publishes
    # camera_info with corrected distortion_model before kernel receives it
    delayed_kernel = TimerAction(period=3.0, actions=[kernel])

    return LaunchDescription([
        disable_cyclone_shm, trace, rosbag_play,
        container, delayed_kernel, dummy_sub,
    ])
