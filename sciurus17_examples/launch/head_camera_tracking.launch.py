# Copyright 2023 RT Corporation
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    container = ComposableNodeContainer(
        name='tracking_container',
        namespace='head_camera_tracking',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::ColorDetection',
                name='color_detection',
                namespace='head_camera_tracking',
                remappings=[
                    ('/image_raw', '/head_camera/color/image_raw')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::ObjectTracker',
                name='object_tracker',
                namespace='head_camera_tracking',
                remappings=[
                    ('/controller_state', '/neck_controller/controller_state')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::NeckJtControl',
                name='neck_jt_control',
                namespace='head_camera_tracking',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        container
    ])
