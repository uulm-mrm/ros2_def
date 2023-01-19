from launch import LaunchDescription
from launch_ros.actions import Node
from collections.abc import Callable


def get_tracking_nodes(remapping_fn: Callable[[str, str], str]):
    """
    remapping_fn: node name x topic -> new topic name
    """

    time_scale = 10

    return LaunchDescription([
        # RADAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='radar',
            parameters=[
                {"timer_period_s": 0.1*time_scale},
                {"timer_uncertainty_s": 0.005*time_scale},
            ],
            remappings=[("output", "meas/radar")],
        ),  # type: ignore
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_radar',
            parameters=[
                {"processing_time": 0.0*time_scale},
            ],
            remappings=[("input", remapping_fn("detector_radar", "meas/radar")),
                        ("output", "detections/radar")],
        ),

        # CAMERA
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='camera',
            parameters=[
                {"timer_period_s": 0.1*time_scale},
                {"timer_uncertainty_s": 0.0*time_scale},
            ],
            remappings=[("output", "meas/camera")],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_camera',
            parameters=[
                {"processing_time": 0.0*time_scale},
            ],
            remappings=[("input", remapping_fn("detector_camera", "meas/camera")),
                        ("output", "detections/camera")],
        ),

        # LIDAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='timed_sensor_publisher',
            name='lidar',
            parameters=[
                {"timer_period_s": 0.1*time_scale},
                {"timer_uncertainty_s": 0.0*time_scale},
            ],
            remappings=[("output", "meas/lidar")],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_lidar',
            parameters=[
                {"processing_time": 0.0*time_scale},
            ],
            remappings=[("input", remapping_fn("detector_lidar", "meas/lidar")),
                        ("output", "detections/lidar")],
        ),

        # TRACKING
        Node(
            package='orchestrator_dummy_nodes',
            executable='tracking_subscriber',
            name='tracking',
            remappings=[("input_radar", remapping_fn("tracking", "detections/radar")),
                        ("input_lidar", remapping_fn("tracking", "detections/lidar")),
                        ("input_camera", remapping_fn("tracking", "detections/camera"))]
        ),

        # GRIDMAP
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='gridmap',
            parameters=[
                {"processing_time": 0.03*time_scale},
            ],
            remappings=[("input", remapping_fn("tracking", "meas/radar")),
                        ("output", "occupancy_grid")],
        ),
    ])
