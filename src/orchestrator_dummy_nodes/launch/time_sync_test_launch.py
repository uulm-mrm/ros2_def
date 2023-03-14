from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("info")),
            description="Logging level"
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                {"mode": "time_sync"},
            ],
            arguments=['--ros-args', '--log-level', ['l:=', LaunchConfiguration('log_level')]]
        ),
        *generate_remappings_from_config("orchestrator_dummy_nodes", "time_sync_test_launch_config.json"),
        Node(
            package='orchestrator_dummy_nodes',
            executable='camera_input_node',
            name='camera_input'
        )
    ])
