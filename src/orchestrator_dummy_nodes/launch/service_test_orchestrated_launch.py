from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

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
                {"mode": "service"},
            ],
            arguments=['--ros-args', '--log-level',
                       ['l:=', LaunchConfiguration('log_level')]]
        ),
        *generate_remappings_from_config("orchestrator_dummy_nodes",
                                         "service_test_launch_config.json"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orchestrator_dummy_nodes'),
                    'launch/service_test_launch.py'
                ])
            ]),
            launch_arguments={"log_level": "warn"}.items()
        )
    ])
