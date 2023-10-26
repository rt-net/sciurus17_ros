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
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('sciurus17_description'))}
    world_file = os.path.join(
        get_package_share_directory('sciurus17_gazebo'), 'worlds', 'table.sdf')
    gui_config = os.path.join(
        get_package_share_directory('sciurus17_gazebo'), 'gui', 'gui.config')
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'sciurus17',
                   '-z', '1.02',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.gz_control_config_package = 'sciurus17_control'
    description_loader.gz_control_config_file_path = 'config/sciurus17_controllers.yaml'
    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sciurus17_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_right_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner right_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_right_hand_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner right_hand_controller'],
                shell=True,
                output='screen',
            )

    spawn_left_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner left_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_left_hand_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner left_hand_controller'],
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

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ign_gazebo,
        ignition_spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_right_arm_controller,
        spawn_right_hand_controller,
        spawn_left_arm_controller,
        spawn_left_hand_controller,
        spawn_neck_controller,
        spawn_waist_yaw_controller,
        move_group,
        bridge
    ])
