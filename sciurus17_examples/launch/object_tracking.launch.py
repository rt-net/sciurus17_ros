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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='tracking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::ColorDetection',
                name='color_detection',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::ObjectTracker',
                name='object_tracker',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::NeckJtControl',
                name='neck_jt_control',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
