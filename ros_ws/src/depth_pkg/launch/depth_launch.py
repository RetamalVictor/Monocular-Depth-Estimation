import launch
from launch_ros.actions import ComposableNodeContainer, Node
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

    midas_node = Node(
            package='depth_pkg',
            namespace='midas',
            executable='midas_node',
            name='midas_node',
    )

    return launch.LaunchDescription([midas_node, container])
