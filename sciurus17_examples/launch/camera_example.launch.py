# Copyright 2024 RT Corporation
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
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from sciurus17_description.robot_description_loader import (
    RobotDescriptionLoader,
)


def generate_launch_description():
    declare_example_name = DeclareLaunchArgument(
        'example',
        default_value='point_cloud_detection',
        description=(
            'Set an example executable name: '
            '[aruco_detection, color_detection, point_cloud_detection]'
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=('Set true when using the gazebo simulator.'),
    )

    description_loader = RobotDescriptionLoader()

    moveit_config = MoveItConfigsBuilder('sciurus17').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    picking_node = Node(
        name='pick_and_place_tf',
        package='sciurus17_examples',
        executable='pick_and_place_tf',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    detection_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='sciurus17_examples',
        executable=LaunchConfiguration('example'),
        output='screen',
    )

    return LaunchDescription(
        [
            declare_example_name,
            declare_use_sim_time,
            SetParameter(
                name='use_sim_time', value=LaunchConfiguration('use_sim_time')
            ),
            picking_node,
            detection_node,
        ]
    )
