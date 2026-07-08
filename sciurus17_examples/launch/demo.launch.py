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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    declare_use_head_camera = DeclareLaunchArgument(
        'use_head_camera', default_value='true', description='Use head camera.'
    )

    declare_use_chest_camera = DeclareLaunchArgument(
        'use_chest_camera', default_value='true', description='Use chest camera.'
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('sciurus17_moveit_config'),
                '/launch/run_move_group.launch.py',
            ]
        ),
    )

    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('sciurus17_control'),
                '/launch/sciurus17_control.launch.py',
            ]
        ),
    )

    head_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('sciurus17_vision'), '/launch/head_camera.launch.py']
        ),
        condition=IfCondition(LaunchConfiguration('use_head_camera')),
    )

    chest_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('sciurus17_vision'), '/launch/chest_camera.launch.py']
        ),
        condition=IfCondition(LaunchConfiguration('use_chest_camera')),
    )

    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            declare_use_head_camera,
            declare_use_chest_camera,
            move_group,
            control_node,
            head_camera_node,
            chest_camera_node,
        ]
    )
