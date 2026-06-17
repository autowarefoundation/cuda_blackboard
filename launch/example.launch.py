import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("image_path", "image.png")

    container = ComposableNodeContainer(
        name="cuda_blackboard_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="cuda_blackboard",
                plugin="cuda_blackboard::CudaBlackboardPublisherNode",
                name="cuda_blackboard_publisher_example",
                parameters=[{"image_path": LaunchConfiguration("image_path")}],
            ),
            ComposableNode(
                package="cuda_blackboard",
                plugin="cuda_blackboard::CudaBlackboardSubscriberNode",
                name="cuda_blackboard_subscriber_example",
            ),
        ],
        output="both",
    )

    return launch.LaunchDescription(launch_arguments + [container])
