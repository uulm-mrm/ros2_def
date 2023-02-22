from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config


def generate_launch_description():
    return LaunchDescription([
        *generate_remappings_from_config("orchestrator_dummy_nodes", "tracking_example_launch_config.json"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orchestrator_dummy_nodes'),
                    'tracking_example_launch.py'
                ])
            ])
        )
    ])
