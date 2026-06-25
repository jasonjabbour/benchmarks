"""
Benchmark: Image -> Resized Image  (image_proc ResizeNode)
Kernel:    image_proc::ResizeNode
Session:   e4_image_resize

E4 image components (plain rclcpp) handle image + camera_info with matched timestamps.
ResizeNode as separate process with image_transport remappings.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/perception/r2b_cafe'


def generate_launch_description():
    trace = Trace(
        session_name='e4_image_resize',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--remap',
             'd455_1_rgb_image:=/camera/image_raw',
             'd455_1_rgb_camera_info:=/camera/camera_info'],
        output='screen',
    )

    # E4 input: subscribes to /camera/image_raw, publishes to /benchmark/image
    # Also forwards camera_info with matching timestamp
    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::E4ImageInputComponent',
        name='image_input_component',
        parameters=[{
            'source_topic_name': '/camera/image_raw',
            'input_topic_name': '/benchmark/image',
            'camera_info_in': '/camera/camera_info',
            'camera_info_out': '/benchmark/camera_info',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Kernel: ResizeNode as separate process
    kernel = Node(
        package='image_proc',
        executable='resize_node',
        name='resize_node',
        output='screen',
        parameters=[{
            'scale_height': 0.5,
            'scale_width': 0.5,
        }],
        remappings=[
            ('image/image_raw', '/benchmark/image'),
            ('image/camera_info', '/benchmark/camera_info'),
            ('resize/image_raw', '/benchmark/resize_image'),
            ('resize/camera_info', '/benchmark/resize_camera_info'),
        ],
    )

    # E4 output: subscribes to /benchmark/resize_image
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::E4ImageOutputComponent',
        name='image_output_component',
        parameters=[{
            'output_topic_name': '/benchmark/resize_image',
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

    disable_cyclone_shm = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '</Domain></CycloneDDS>'
    )

    return LaunchDescription([
        disable_cyclone_shm, trace, rosbag_play,
        container, kernel,
    ])
