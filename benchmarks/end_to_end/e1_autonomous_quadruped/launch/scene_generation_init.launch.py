import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='e1_autonomous_quadruped',
            executable='scene_generation.py',
            output='screen'
        ),
    ])
