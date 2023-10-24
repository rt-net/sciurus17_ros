from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("sciurus17")
        .trajectory_execution(file_path=get_package_share_directory('sciurus17_control') + '/config/sciurus17_controllers.yaml')
        .to_moveit_configs()
    )
    return generate_static_virtual_joint_tfs_launch(moveit_config)
