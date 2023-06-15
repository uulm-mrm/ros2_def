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
            remappings=[("output", "M")],
            on_exit=Shutdown(),
        ),
        Node(
            package="orchestrator_dummy_nodes",
            executable="detector",
            name="P1",
            exec_name="P1",
            parameters=[
                {"processing_time": 0.4},
                {"processing_time_range": 0.1},
                {"queue_size": 3}
            ],
            remappings=[
                ("input", "M"),
                ("output", "D1")
            ],
            on_exit=Shutdown(),
        ),
        Node(
            package="orchestrator_dummy_nodes",
            executable="detector",
            name="P2",
            exec_name="P2",
            parameters=[
                {"processing_time": 0.4},
                {"processing_time_range": 0.1},
                {"queue_size": 3}
            ],
            remappings=[
                ("input", "M"),
                ("output", "D2")
            ],
            on_exit=Shutdown(),
        ),
        Node(
            package="orchestrator_dummy_nodes",
            executable="verification_t_subscriber",
            name="T",
            exec_name="T",
            parameters=[
                {"processing_time": 0.2},
            ],
            remappings=[
                ("input1", "D1"),
                ("input2", "D2")
            ],
            on_exit=Shutdown(),
        ),
    ])
