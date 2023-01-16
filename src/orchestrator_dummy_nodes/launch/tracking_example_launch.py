from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # RADAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='radar',
            parameters=[
                {"timer_period_s": 0.1},
                {"timer_uncertainty_s": 0.005},
            ],
            remappings=[("/output", "/meas/radar")],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_radar',
            parameters=[
                {"processing_time": 0.0},
            ],
            remappings=[("input", "/meas/radar"),
                        ("output", "/detections/radar")],
        ),

        # CAMERA
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='camera',
            parameters=[
                {"timer_period_s": 0.1},
                {"timer_uncertainty_s": 0.0},
            ],
            remappings=[(f"/output", "/meas/camera")],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_camera',
            parameters=[
                {"processing_time": 0.0},
            ],
            remappings=[("input", "/meas/camera"),
                        ("output", "/detections/camera")],
        ),

        # LIDAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='lidar',
            parameters=[
                {"timer_period_s": 0.1},
                {"timer_uncertainty_s": 0.0},
            ],
            remappings=[(f"/output", "/meas/lidar")],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_lidar',
            parameters=[
                {"processing_time": 0.0},
            ],
            remappings=[("input", "/meas/lidar"),
                        ("output", "/detections/lidar")],
        ),

        # TRACKING
        Node(
            package='orchestrator_dummy_nodes',
            executable='tracking_subscriber',
            name='tracking',
            remappings=[("input_radar", "/detections/radar"),
                        ("input_lidar", "/detections/lidar"),
                        ("input_camera", "/detections/camera")]
        ),

        # GRIDMAP
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='gridmap',
            parameters=[
                {"processing_time": 0.03},
            ],
            remappings=[("input", "/meas/radar"),
                        ("output", "occupancy_grid")],
        ),
    ])
