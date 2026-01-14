import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""

    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index'
    )

    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='MiDaS_small',
        description='MiDaS model type (MiDaS_small, DPT_Large, DPT_Hybrid)'
    )

    # Camera and display container
    container = ComposableNodeContainer(
        name='image_pipeline',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_pkg',
                plugin='camera_composition::CameraComponent',
                name='camera',
                parameters=[{
                    'camera_index': LaunchConfiguration('camera_index'),
                    'frame_width': 640,
                    'frame_height': 480,
                }]
            ),
            ComposableNode(
                package='camera_pkg',
                plugin='camera_composition::DisplayComponent',
                name='display'
            ),
        ],
        output='screen',
    )

    # MiDaS depth estimation node
    midas_node = Node(
        package='depth_pkg',
        namespace='',
        executable='midas_node',
        name='midas_node',
        parameters=[{
            'model_type': LaunchConfiguration('model_type'),
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        camera_index_arg,
        model_type_arg,
        container,
        midas_node,
    ])
