from launch import LaunchDescription
from launch.actions import Shutdown, GroupAction
from launch_ros.actions import Node

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestrator_dummy_nodes',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                {"mode": "verification_4_service"},
            ],
            arguments=['--ros-args', '--log-level', 'l:=warn', '--log-level', 'orchestrator:=info'],
            on_exit=Shutdown(),
        ),
        *generate_remappings_from_config("orchestrator_dummy_nodes",
                                         "verification_4_service_launch_config.json"),
        GroupAction([
            Node(
                package="orchestrator_dummy_nodes",
                executable="service_provider_node",
                name="SP",
                exec_name="SP",
                remappings=[
                    ("input", "T"),
                    ("service", "S")
                ],
                on_exit=Shutdown(),
            ),
            Node(
                package="orchestrator_dummy_nodes",
                executable="service_caller_node",
                name="N1",
                exec_name="N1",
                remappings=[
                    ("input", "T"),
                    ("service", "S")
                ],
                on_exit=Shutdown(),
            ),
            Node(
                package="orchestrator_dummy_nodes",
                executable="service_caller_node",
                name="N2",
                exec_name="N2",
                remappings=[
                    ("input", "T"),
                    ("service", "S")
                ],
                on_exit=Shutdown(),
            ),
        ])
    ])
