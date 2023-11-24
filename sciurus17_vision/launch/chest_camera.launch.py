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
from launch_ros.actions import Node


def generate_launch_description():
    camera_info_file = 'file://' + get_package_share_directory(
        'sciurus17_vision') + '/config/chest_camera_info.yaml'
    usb_cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='chest_camera',
            parameters=[
                {'video_device': '/dev/chestcamera'},
                {'frame_id': 'chest_camera_link'},
                {'image_width': 1280},
                {'image_height': 720},
                {'framerate': 30.0},
                {'camera_name': 'chest_camera'},
                {'camera_info_url': camera_info_file},
                {'pixel_format': 'mjpeg2rgb'}
            ],
        )

    return LaunchDescription([
        usb_cam_node
    ])
