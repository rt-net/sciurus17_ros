from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sciurus17_examples_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,
                      'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RT Corporation',
    maintainer_email='shop@rt-net.jp',
    description='python examples of Sciurus17 ROS package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_control = sciurus17_examples_py.gripper_control:main',
            'pose_groupstate = sciurus17_examples_py.pose_groupstate:main',
            'joint_values = sciurus17_examples_py.joint_values:main',
            'neck_control = sciurus17_examples_py.neck_control:main',
            'waist_control = sciurus17_examples_py.waist_control:main',
            'pick_and_place_right_arm_waist = \
            sciurus17_examples_py.pick_and_place_right_arm_waist:main',
            'pick_and_place_left_arm = sciurus17_examples_py.pick_and_place_left_arm:main',
            'pick_and_place_tf = sciurus17_examples_py.pick_and_place_tf:main',
            'aruco_detection = sciurus17_examples_py.aruco_detection:main',
            'color_detection = sciurus17_examples_py.color_detection:main',
        ],
    },
)
