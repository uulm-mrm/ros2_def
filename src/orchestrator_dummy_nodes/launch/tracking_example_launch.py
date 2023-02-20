from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    time_scale = 20
    logger = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),

        SetParameter(name="use_sim_time", value=True),

        # RADAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_radar',
            parameters=[
                {"processing_time": 0.01*time_scale},
            ],
            remappings=[("input", "meas/radar"),
                        ("output", "detections/radar")],
            arguments=['--ros-args', '--log-level', ['detector_radar:=', logger]],
        ),

        # CAMERA
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_camera',
            parameters=[
                {"processing_time": 0.01*time_scale},
            ],
            remappings=[("input", "meas/camera"),
                        ("output", "detections/camera")],
            arguments=['--ros-args', '--log-level', ['detector_camera:=', logger]],
        ),

        # LIDAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_lidar',
            parameters=[
                {"processing_time": 0.01*time_scale},
            ],
            remappings=[("input", "meas/lidar"),
                        ("output", "detections/lidar")],
            arguments=['--ros-args', '--log-level', ['detector_lidar:=', logger]],
        ),

        # TRACKING
        Node(
            package='orchestrator_dummy_nodes',
            executable='tracking_subscriber',
            name='tracking',
            remappings=[("input_radar", "detections/radar"),
                        ("input_lidar", "detections/lidar"),
                        ("input_camera", "detections/camera")],
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
            remappings=[("input", "meas/radar"),
                        ("output", "occupancy_grid")],
            arguments=['--ros-args', '--log-level', ['gridmap:=', logger]],
        ),

        # PLAUSIBILITY
        Node(
            package='orchestrator_dummy_nodes',
            executable='plausibility_node',
            name='plausibility',
            remappings=[("tracks_in", "tracks"),
                        ("gridmap", "occupancy_grid"),
                        ("tracks_out", "plausible_tracks"),
                        ("tracks_out_gridmap", "gridmap_tracks")],
            arguments=['--ros-args', '--log-level', ['plausibility:=', logger]],
        ),

        # PLANNING
        Node(
            package='orchestrator_dummy_nodes',
            executable='simple_timer_publisher',
            name='planning',
            remappings=[("output", "trajectory")],
            arguments=['--ros-args', '--log-level', ['planning:=', logger]],
        ),
    ])
