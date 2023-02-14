from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from collections.abc import Callable


def get_tracking_nodes(remapping_fn: Callable[[str, str], str]):
    """
    remapping_fn: node name x topic -> new topic name
    """

    time_scale = 20
    logger = LaunchConfiguration("log_level")

    return [
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
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
            arguments=['--ros-args', '--log-level', ['radar:=', logger]],
        ),  # type: ignore
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_radar',
            parameters=[
                {"processing_time": 0.01*time_scale},
            ],
            remappings=[("input", remapping_fn("detector_radar", "meas/radar")),
                        ("output", "detections/radar")],
            arguments=['--ros-args', '--log-level', ['detector_radar:=', logger]],
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
            arguments=['--ros-args', '--log-level', ['camera:=', logger]],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_camera',
            parameters=[
                {"processing_time": 0.01*time_scale},
            ],
            remappings=[("input", remapping_fn("detector_camera", "meas/camera")),
                        ("output", "detections/camera")],
            arguments=['--ros-args', '--log-level', ['detector_camera:=', logger]],
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
            arguments=['--ros-args', '--log-level', ['lidar:=', logger]],
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_lidar',
            parameters=[
                {"processing_time": 0.01*time_scale},
            ],
            remappings=[("input", remapping_fn("detector_lidar", "meas/lidar")),
                        ("output", "detections/lidar")],
            arguments=['--ros-args', '--log-level', ['detector_lidar:=', logger]],
        ),

        # TRACKING
        Node(
            package='orchestrator_dummy_nodes',
            executable='tracking_subscriber',
            name='tracking',
            remappings=[("input_radar", remapping_fn("tracking", "detections/radar")),
                        ("input_lidar", remapping_fn("tracking", "detections/lidar")),
                        ("input_camera", remapping_fn("tracking", "detections/camera"))],
            arguments=['--ros-args', '--log-level', ['tracking:=', logger]],
        ),

        # GRIDMAP
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='gridmap',
            parameters=[
                {"processing_time": 0.03*time_scale},
            ],
            remappings=[("input", remapping_fn("gridmap", "meas/radar")),
                        ("output", "occupancy_grid")],
            arguments=['--ros-args', '--log-level', ['gridmap:=', logger]],
        ),

        # PLAUSIBILITY
        Node(
            package='orchestrator_dummy_nodes',
            executable='plausibility_node',
            name='plausibility',
            remappings=[("tracks_in", remapping_fn("plausibility", "tracks")),
                        ("gridmap", remapping_fn("plausibility", "occupancy_grid")),
                        ("tracks_out", "plausible_tracks"),
                        ("tracks_out_gridmap", "gridmap_tracks")],
            arguments=['--ros-args', '--log-level', ['plausibility:=', logger]],
        )
    ]
