from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_rsp_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from sciurus17_description.robot_description_loader import RobotDescriptionLoader


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    declare_robot_description = DeclareLaunchArgument(
        'loaded_description',
        default_value=description_loader.load(),
        description='Set robot_description text.  \
                        It is recommended to use RobotDescriptionLoader() \
                            in sciurus17_description.',
    )

    moveit_config = (
        MoveItConfigsBuilder('sciurus17')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        'robot_description': LaunchConfiguration('loaded_description')
    }

    return LaunchDescription(
        [
            declare_robot_description,
            generate_move_group_launch(moveit_config),
            generate_moveit_rviz_launch(moveit_config),
            generate_static_virtual_joint_tfs_launch(moveit_config),
            generate_rsp_launch(moveit_config),
        ]
    )
