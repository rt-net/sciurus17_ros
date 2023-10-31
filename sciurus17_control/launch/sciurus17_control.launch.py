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
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('sciurus17_control'),
        'config',
        'manipulator_config.yaml'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = '/dev/sciurus17spine'
    description_loader.baudrate = '3000000'
    description_loader.timeout_seconds = '1.0'
    description_loader.manipulator_config_file_path = config_file_path

    declare_loaded_description = DeclareLaunchArgument(
        'loaded_description',
        default_value=description_loader.load(),
        description='Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in sciurus17_description.'
    )

    sciurus17_controllers = os.path.join(
        get_package_share_directory('sciurus17_control'),
        'config',
        'sciurus17_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': LaunchConfiguration('loaded_description')},
                    sciurus17_controllers],
        output='screen',
        )

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_neck_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner neck_controller'],
                shell=True,
                output='screen',
            )

    spawn_waist_yaw_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner waist_yaw_controller'],
                shell=True,
                output='screen',
            )

    return LaunchDescription([
      declare_loaded_description,
      controller_manager,
      spawn_joint_state_broadcaster,
      spawn_neck_controller,
      spawn_waist_yaw_controller
    ])
