"""
Benchmark: Stereo Images -> Disparity  (stereo_image_proc DisparityNode)
Kernel:    stereo_image_proc::DisparityNode (forked — sync bypassed)
Session:   e4_stereo_disparity

Kernel forked to bypass message_filters sync and use direct subscriptions.
Left image goes through E4ImageInputComponent (with key stamping).
Right image + camera_infos come from bag via direct subscriptions in kernel.
All RELIABLE QoS.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

ROSBAG = '/tmp/benchmark_ws/src/rosbags/perception/r2b_cafe'


def generate_launch_description():
    trace = Trace(
        session_name='e4_stereo_disparity',
        events_ust=['robotperf_benchmarks:*', 'ros2:*'] + DEFAULT_EVENTS_ROS,
        context_fields={
            'kernel': [],
            'userspace': ['vpid', 'vtid', 'procname'],
        },
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', ROSBAG, '--loop',
             '--remap',
             'hawk_0_left_rgb_image:=/stereo/left/image_raw',
             'hawk_0_left_rgb_camera_info:=/stereo/left/camera_info',
             'hawk_0_right_rgb_image:=/stereo/right/image_raw',
             'hawk_0_right_rgb_camera_info:=/stereo/right/camera_info'],
        output='screen',
    )

    # Input: intercept left image with key stamping
    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::E4ImageInputComponent',
        name='image_input_component',
        parameters=[{
            'source_topic_name': '/stereo/left/image_raw',
            'input_topic_name': '/benchmark/left/image_raw',
            'camera_info_in': '/stereo/left/camera_info',
            'camera_info_out': '/benchmark/left/camera_info',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Output: observe disparity
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::DisparityOutputComponent',
        name='disparity_output_component',
        parameters=[{'output_topic_name': '/disparity'}],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Kernel as composable node in same container
    # use_intra_process_comms=False → still goes through DDS
    kernel = ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::DisparityNode',
        namespace='',
        name='disparity_node',
        remappings=[
            ('left/image_rect', '/benchmark/left/image_raw'),
            ('left/camera_info', '/benchmark/left/camera_info'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    container = ComposableNodeContainer(
        name='benchmark_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[input_component, kernel, output_component],
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
        container,
    ])
