from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    declare_loaded_description = DeclareLaunchArgument(
        'loaded_description',
        default_value=description_loader.load(),
        description='Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in sciurus17_description.'
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=get_package_share_directory(
            'sciurus17_moveit_config') + '/config/moveit.rviz',
        description='Set the path to rviz configuration file.'
    )

    moveit_config = (
        MoveItConfigsBuilder("sciurus17")
        .trajectory_execution(file_path=get_package_share_directory('sciurus17_control') + '/config/sciurus17_controllers.yaml')
        .to_moveit_configs()
    )
    moveit_config.robot_description = {'robot_description': LaunchConfiguration('loaded_description')}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sciurus17_moveit_config'),
                '/launch/static_virtual_joint_tfs.launch.py'])
        )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([declare_loaded_description,
                              declare_rviz_config_file,
                              run_move_group_node,
                              rviz_node,
                              static_tf,
                              robot_state_publisher,
                              ])
