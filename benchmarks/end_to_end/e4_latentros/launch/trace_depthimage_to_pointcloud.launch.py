"""
Benchmark: DepthImage + RGB -> PointCloud2  (depth_image_proc PointCloudXyzrgbNode)
Kernel:    depth_image_proc::PointCloudXyzrgbNode (forked — sync bypassed)
Session:   e4_depthimage_to_pointcloud

Kernel forked to bypass message_filters sync. Depth image goes through
E4ImageInputComponent (with key stamping). RGB + camera_info come from
bag via direct subscriptions in kernel. All RELIABLE QoS.
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
        session_name='e4_depthimage_to_pointcloud',
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
             'd455_1_depth_camera_info:=/camera/depth/camera_info',
             'd455_1_rgb_image:=/camera/rgb/image_raw',
             'd455_1_rgb_camera_info:=/camera/rgb/camera_info'],
        output='screen',
    )

    # Input: intercept depth image with key stamping
    input_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::E4ImageInputComponent',
        name='image_input_component',
        parameters=[{
            'source_topic_name': '/camera/depth/image_raw',
            'input_topic_name': '/benchmark/depth/image_raw',
            'camera_info_in': '/camera/depth/camera_info',
            'camera_info_out': '/benchmark/depth/camera_info',
        }],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Kernel: forked PointCloudXyzrgbNode (sync bypassed, direct subs, all reliable)
    kernel = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzrgbNode',
        namespace='',
        name='point_cloud_xyzrgb_node',
        remappings=[
            ('depth_registered/image_rect', '/benchmark/depth/image_raw'),
            ('rgb/image_rect_color', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('points', '/benchmark/points'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Output: observe point cloud
    output_component = ComposableNode(
        package='e4_latentros',
        namespace='robotperf',
        plugin='robotperf::perception::PointCloudOutputComponent',
        name='pointcloud_output_component',
        parameters=[{'output_topic_name': '/benchmark/points'}],
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

    return LaunchDescription([disable_cyclone_shm, trace, rosbag_play, container])
