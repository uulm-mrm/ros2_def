from launch import LaunchDescription
from launch_ros.actions import Node
from orchestrator_dummy_nodes.orchestrator_lib.name_utils import intercepted_name
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("INFO")),
            description="Logging level"
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='forwarding_node',
            name='forwarding',
            remappings=[("input", "o"), ("output", "i")],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='test',
            name='test',
            remappings=[("input", "i"), ("output", "o")],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])
