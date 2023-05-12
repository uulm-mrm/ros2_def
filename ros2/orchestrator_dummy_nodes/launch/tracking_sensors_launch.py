from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    logger = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),

        # RADAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='radar_sensor',
            exec_name='radar_sensor',
            parameters=[
                {"timer_period_s": 1.0},
                {"timer_uncertainty_s": 0.2},
            ],
            remappings=[("output", "meas/radar"), ],
            arguments=['--ros-args', '--log-level', ['radar_sensor:=', logger]],
            on_exit=Shutdown(),
        ),
        # LIDAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='lidar_sensor',
            exec_name='lidar_sensor',
            parameters=[
                {"timer_period_s": 1.0},
                {"timer_uncertainty_s": 0.2},
            ],
            remappings=[("output", "meas/lidar"), ],
            arguments=['--ros-args', '--log-level', ['lidar_sensor:=', logger]],
            on_exit=Shutdown(),
        ),
        # CAMERA
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='camera_sensor',
            exec_name='camera_sensor',
            parameters=[
                {"timer_period_s": 1.0},
                {"timer_uncertainty_s": 0.2},
            ],
            remappings=[("output", "meas/camera"), ],
            arguments=['--ros-args', '--log-level', ['camera_sensor:=', logger]],
            on_exit=Shutdown(),
        ),

    ])
