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
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from sciurus17_description.robot_description_loader import (
    RobotDescriptionLoader,
)


def generate_launch_description():
    declare_example_name = DeclareLaunchArgument(
        'example',
        default_value='gripper_control',
        choices=[
            'gripper_control',
            'pose_groupstate',
            'joint_values',
            'cartesian_path',
            'neck_control',
            'waist_control',
            'pick_and_place_right_arm_waist',
            'pick_and_place_left_arm',
        ],
        description='Set an example executable name.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description=('Set true when using the gazebo simulator.'),
    )

    description_loader = RobotDescriptionLoader()

    moveit_config = MoveItConfigsBuilder('sciurus17').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='sciurus17_examples',
        executable=LaunchConfiguration('example'),
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            declare_example_name,
            declare_use_sim_time,
            SetParameter(
                name='use_sim_time', value=LaunchConfiguration('use_sim_time')
            ),
            example_node,
        ]
    )
