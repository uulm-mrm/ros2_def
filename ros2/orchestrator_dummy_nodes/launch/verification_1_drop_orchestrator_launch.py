from launch import LaunchDescription
from launch.actions import Shutdown, GroupAction
from launch_ros.actions import Node

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config_file


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestrator_dummy_nodes',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                {"mode": "verification_1_drop"},
            ],
            arguments=['--ros-args', '--log-level', 'l:=warn', '--log-level', 'orchestrator:=info'],
            on_exit=Shutdown(),
        ),
        *generate_remappings_from_config_file("orchestrator_dummy_nodes", "verification_1_drop_launch_config.json"),
        GroupAction([
            Node(
                package='orchestrator_dummy_nodes',
                executable='detector',
                name='detector',
                parameters=[
                    {"processing_time": 1.0},
                    {"processing_time_range": 0.3},
                    {"queue_size": 3}
                ],
                remappings=[("input", "t")],
                on_exit=Shutdown(),
            ),
        ])
    ])
