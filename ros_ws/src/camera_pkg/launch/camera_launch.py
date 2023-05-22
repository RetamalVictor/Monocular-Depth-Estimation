# image_pipeline_launch.py
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='image_pipeline',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_pkg',
                    plugin='camera_composition::CameraComponent',
                    name='camera'),
                ComposableNode(
                    package='camera_pkg',
                    plugin='camera_composition::DisplayComponent',
                    name='display'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])