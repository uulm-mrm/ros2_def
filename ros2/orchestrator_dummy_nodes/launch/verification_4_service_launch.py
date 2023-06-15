from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="orchestrator_dummy_nodes",
            executable="simple_timer_publisher",
            name="S",
            exec_name="S",
            parameters=[
                {"timer_period_s": 1.0},
            ],
            remappings=[("output", "T")],
            on_exit=Shutdown(),
        ),
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
