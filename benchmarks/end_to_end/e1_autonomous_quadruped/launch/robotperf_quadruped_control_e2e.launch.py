import os

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT

def generate_launch_description():

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_config"
    ).find("champ_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_description"
    ).find("champ_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "urdf/champ.urdf.xacro")

    # Launch arguments
    declare_description_path = DeclareLaunchArgument(
        name="description_path",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true"
    )
    declare_joints_map_path = DeclareLaunchArgument(
        name="joints_map_path",
        default_value=joints_config,
        description="Absolute path to joints map file"
    )
    declare_links_map_path = DeclareLaunchArgument(
        name="links_map_path",
        default_value=links_config,
        description="Absolute path to links map file"
    )
    declare_gait_config_path = DeclareLaunchArgument(
        name="gait_config_path",
        default_value=gait_config,
        description="Absolute path to gait config file"
    )
    declare_gazebo = DeclareLaunchArgument(
        "gazebo", 
        default_value="false", 
        description="If running in gazebo"
    )
    declare_joint_controller_topic = DeclareLaunchArgument(
        "joint_controller_topic",
        default_value="joint_group_effort_controller/joint_trajectory",
        description="Joint controller topic"
    )
    declare_publish_joint_control = DeclareLaunchArgument(
        "publish_joint_control",
        default_value="true",
        description="Publish joint control"
    )
    declare_publish_joint_states = DeclareLaunchArgument(
        "publish_joint_states",
        default_value="true",
        description="Publish joint states"
    )
    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts",
        default_value="true",
        description="Publish foot contacts"
    )

    declare_twist_type = DeclareLaunchArgument(
        "twist_type", 
        default_value="Twist", 
        description="Choose between Twist or TwistStamped")
    
    declare_control_mode = DeclareLaunchArgument(
        "control_mode",
        default_value="event_based", 
        description="Choose between event_based or frequency_based")
    
    declare_loop_rate = DeclareLaunchArgument(
        "loop_rate",
        default_value="200.0", 
        description="Loop rate for frequency-based control")


    trace = Trace(
        session_name= 'quadruped_control_test',
        events_ust=[
            "robotperf_benchmarks:*",
            "ros2:*",
            "robotcore_power:*",
            # "lttng_ust_cyg_profile*",
            # "lttng_ust_statedump*",
            # "liblttng-ust-libc-wrapper",
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        # events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
    )
    

    # Place Input Tracepoint for Twist
    twist_input_node = ComposableNode(
        package="e1_autonomous_quadruped",
        namespace='robotperf',
        plugin="robotperf::control::TwistInputComponent",
        name="twist_input_component",
        parameters=[
            {"output_topic_name": "/robotperf/benchmark/cmd_vel"}, #publishing to this
            {"twist_type": LaunchConfiguration("twist_type")}, #Twist or TwistStamped
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel"), #subscribing to this
        ],
        # extra_arguments=[{'use_intra_process_comms': True}],
    )

    quadruped_controller_node = Node(
        package="e1_autonomous_quadruped",
        executable="robotperf_quadruped_controller_node",
        output="screen",
        namespace='robotperf/benchmark',
        parameters=[
            {"twist_type": LaunchConfiguration("twist_type")},
            {"control_mode": LaunchConfiguration("control_mode")},
            {"loop_rate": LaunchConfiguration("loop_rate")},
            {"use_sim_time": True},
            {"gazebo": LaunchConfiguration("gazebo")},
            {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
            {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
            {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
            {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
            {"urdf": Command(['xacro ', LaunchConfiguration('description_path')])},
            LaunchConfiguration('joints_map_path'),
            LaunchConfiguration('links_map_path'),
            LaunchConfiguration('gait_config_path'),
        ],
        remappings=[("cmd_vel/smooth", "/robotperf/benchmark/cmd_vel"), 
                    ("joint_group_effort_controller/joint_trajectory", "/robotperf/benchmark/joint_group_effort_controller/joint_trajectory")], #subbing to this
        # extra_arguments=[{'use_intra_process_comms': True}],

    )

    # Record Final Tracepoint once trajectory message is produced
    trajectory_input_node = ComposableNode(
        package="e1_autonomous_quadruped",
        namespace='robotperf',
        plugin="robotperf::control::JointTrajectoryInputComponent",
        name="joint_trajectory_output_component",
        parameters=[
            {'input_topic_name': '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory'}, # subbing to this
            {'output_topic_name': '/joint_group_effort_controller/joint_trajectory'}, # outputting this

        ],
        # remappings=[
        #     ('joint_trajectory', '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory'), 
        # ],
        # extra_arguments=[{'use_intra_process_comms': True}],
    )
       
    # Put your two composable nodes (input and output) into a container
    composable_node_container = ComposableNodeContainer(
        name="end2end_container",
        package="rclcpp_components",
        namespace='',
        executable="component_container_mt",
        composable_node_descriptions=[
            twist_input_node,
            trajectory_input_node
        ],
        output="screen"
    )

    # Build the final LaunchDescription
    ld = LaunchDescription()

    # Declare all arguments
    ld.add_action(declare_description_path)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_joints_map_path)
    ld.add_action(declare_links_map_path)
    ld.add_action(declare_gait_config_path)
    ld.add_action(declare_gazebo)
    ld.add_action(declare_joint_controller_topic)
    ld.add_action(declare_publish_joint_control)
    ld.add_action(declare_publish_joint_states)
    ld.add_action(declare_publish_foot_contacts)
    ld.add_action(declare_twist_type)
    ld.add_action(declare_control_mode)
    ld.add_action(declare_loop_rate)

    # Add the container (input & output) and the controller node
    ld.add_action(trace)
    ld.add_action(composable_node_container)
    ld.add_action(quadruped_controller_node)

    return ld
