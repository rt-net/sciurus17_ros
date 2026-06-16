# Copyright 2026 RT Corporation
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_head_camera = DeclareLaunchArgument(
        'use_head_camera', default_value='true', description='Use head camera.'
    )

    declare_use_chest_camera = DeclareLaunchArgument(
        'use_chest_camera', default_value='true', description='Use chest camera.'
    )

    world_file = os.path.join(
        get_package_share_directory('sciurus17_gazebo'),
        'worlds',
        'table_with_blue_cube.sdf',
    )
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('sciurus17_gazebo'),
                '/launch/sciurus17_gazebo.launch.py',
            ]
        ),
        launch_arguments={
            'world_name': world_file,
            'use_head_camera': LaunchConfiguration('use_head_camera'),
            'use_chest_camera': LaunchConfiguration('use_chest_camera'),
        }.items(),
    )

    return LaunchDescription([declare_use_head_camera, declare_use_chest_camera, world_launch])
