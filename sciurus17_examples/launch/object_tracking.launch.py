import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='tracking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::ColorDetection',
                name='color_detection',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::ObjectTracker',
                name='object_tracker',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='sciurus17_examples',
                plugin='sciurus17_examples::NeckJtControl',
                name='neck_jt_control',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])