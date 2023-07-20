#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@@@@@&@@@@@@@@@@
#    @@@@@@@@@@@@@@@@@@@@
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    container = ComposableNodeContainer(
        name='imu_transformer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='imu_transformer',
                plugin='imu_transformer::ImuTransformer',
                name='imu_transformer_node',
                # remappings=[
                #     ('imu_in', 'imu_ned'),
                #     ('imu_out', 'imu_enu'),
                #     ('mag_in', 'mag_ned'),
                #     ('mag_out', 'mag_enu')
                # ],

                
            )
            # ComposableNode(
            #     package="a1_perception_2nodes",
            #     plugin="robotperf::perception::ImageInputComponent",
            #     name="image_input_component",
            #     namespace="robotperf",
            #     parameters=[
            #         {"input_topic_name":"/robotperf/input/left_input/left_image_raw"}
            #     ],
            #     remappings=[
            #         ("image", "/left_camera/image_raw"),
            #         ("camera_info", "/left_camera/camera_info"),
            #     ],
            #     extra_arguments=[{'use_intra_process_comms': True}],
            # ),
        ],
        output='screen',
    )

    ld.add_action(container)

    return ld