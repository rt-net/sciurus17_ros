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

import os

from ament_index_python.packages import get_package_share_directory
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/sciurus17spine',
        description='Set port name.'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='3000000',
        description='Set baudrate.'
    )

    config_file_path = os.path.join(
        get_package_share_directory('sciurus17_control'),
        'config',
        'manipulator_config.yaml'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description_loader.baudrate = LaunchConfiguration('baudrate')
    description_loader.timeout_seconds = '1.0'
    description_loader.manipulator_config_file_path = config_file_path

    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sciurus17_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={
                'loaded_description': description
            }.items()
        )

    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sciurus17_control'),
                '/launch/sciurus17_control.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    return LaunchDescription([
        declare_port_name,
        declare_baudrate,
        move_group,
        control_node
    ])
