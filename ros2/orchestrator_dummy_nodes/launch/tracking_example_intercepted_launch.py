from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config_file


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestrator_dummy_nodes',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                {"mode": "tracking"},
            ],
            on_exit=Shutdown(),
        ),
        *generate_remappings_from_config_file("orchestrator_dummy_nodes", "tracking_example_launch_config.json"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orchestrator_dummy_nodes'),
                    'launch/tracking_example_launch.py'
                ])
            ]),
            launch_arguments={"log_level": "debug"}.items()
        )
    ])
