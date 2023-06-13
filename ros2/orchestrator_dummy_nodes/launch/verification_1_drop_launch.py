from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestrator_dummy_nodes',
            executable='simple_timer_publisher',
            name='simple_timer_publisher',
            parameters=[
                {"timer_period_s": 0.2},
            ],
            remappings=[("output", "t")],
            on_exit=Shutdown(),
        ),
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
