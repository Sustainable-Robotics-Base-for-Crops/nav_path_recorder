import launch
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = '/auto'

    package_dir = FindPackageShare('nav_path_recorder')

    yaml_path = PathJoinSubstitution(
        [package_dir, 'config', 'default.yaml'])

    container = ComposableNodeContainer(
        name='nav_path_recorder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='nav_path_recorder',
                plugin='nav_path_recorder::PathRecorder',
                name='recorder',
                parameters=[yaml_path])
        ],
        output='screen',
    )

    container_with_ns = GroupAction([
        PushRosNamespace(ns),
        container
    ])

    return launch.LaunchDescription([container_with_ns])
