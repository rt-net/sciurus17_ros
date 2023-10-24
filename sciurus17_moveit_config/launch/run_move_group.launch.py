from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from moveit_configs_utils.launches import generate_rsp_launch
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    description_loader = RobotDescriptionLoader()

    ld.add_action(
        DeclareLaunchArgument(
            'loaded_description',
            default_value=description_loader.load(),
            description='Set robot_description text.  \
                        It is recommended to use RobotDescriptionLoader() in sciurus17_description.'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=get_package_share_directory(
                'sciurus17_moveit_config') + '/config/moveit.rviz',
            description='Set the path to rviz configuration file.'
        )
    )

    moveit_config = MoveItConfigsBuilder("sciurus17").to_moveit_configs()
    moveit_config.robot_description = {'robot_description': LaunchConfiguration('loaded_description')}

    # Move group
    ld.add_entity(generate_move_group_launch(moveit_config))

    # RViz
    ld.add_entity(generate_moveit_rviz_launch(moveit_config))

    # Static TF
    ld.add_entity(generate_static_virtual_joint_tfs_launch(moveit_config))

    # Publish TF
    ld.add_entity(generate_rsp_launch(moveit_config))

    return ld
