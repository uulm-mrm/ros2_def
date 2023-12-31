from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
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
        DeclareLaunchArgument(
            "sim_time",
            default_value="True",
        ),

        SetParameter(name="use_sim_time", value=LaunchConfiguration("sim_time")),

        # RADAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_radar',
            exec_name='detector_radar',
            parameters=[
                {"processing_time": 0.01 * time_scale},
            ],
            remappings=[("input", "meas/radar"),
                        ("output", "detections/radar")],
            arguments=['--ros-args', '--log-level', ['detector_radar:=', logger]],
            on_exit=Shutdown(),
        ),

        # CAMERA
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_camera',
            exec_name='detector_camera',
            parameters=[
                {"processing_time": 0.01 * time_scale},
            ],
            remappings=[("input", "meas/camera"),
                        ("output", "detections/camera")],
            arguments=['--ros-args', '--log-level', ['detector_camera:=', logger]],
            on_exit=Shutdown(),
        ),

        # LIDAR
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='detector_lidar',
            exec_name='detector_lidar',
            parameters=[
                {"processing_time": 0.01 * time_scale},
            ],
            remappings=[("input", "meas/lidar"),
                        ("output", "detections/lidar")],
            arguments=['--ros-args', '--log-level', ['detector_lidar:=', logger]],
            on_exit=Shutdown(),
        ),

        # TRACKING
        Node(
            package='orchestrator_dummy_nodes',
            executable='tracking_subscriber',
            name='tracking',
            exec_name='tracking',
            remappings=[("input_radar", "detections/radar"),
                        ("input_lidar", "detections/lidar"),
                        ("input_camera", "detections/camera")],
            arguments=['--ros-args', '--log-level', ['tracking:=', logger]],
            on_exit=Shutdown(),
        ),

        # GRIDMAP
        Node(
            package='orchestrator_dummy_nodes',
            executable='detector',
            name='gridmap',
            exec_name='gridmap',
            parameters=[
                {"processing_time": 0.03 * time_scale},
            ],
            remappings=[("input", "meas/radar"),
                        ("output", "occupancy_grid")],
            arguments=['--ros-args', '--log-level', ['gridmap:=', logger]],
            on_exit=Shutdown(),
        ),

        # PLAUSIBILITY
        Node(
            package='orchestrator_dummy_nodes',
            executable='plausibility_node',
            name='plausibility',
            exec_name='plausibility',
            remappings=[("tracks_in", "tracks"),
                        ("gridmap", "occupancy_grid"),
                        ("tracks_out", "plausible_tracks"),
                        ("tracks_out_gridmap", "gridmap_tracks")],
            arguments=['--ros-args', '--log-level', ['plausibility:=', logger]],
            on_exit=Shutdown(),
        ),

        # PLANNING
        Node(
            package='orchestrator_dummy_nodes',
            executable='simple_timer_publisher',
            name='planning',
            exec_name='planning',
            parameters=[
                {"timer_period_s": 0.3}
            ],
            remappings=[("output", "trajectory")],
            arguments=['--ros-args', '--log-level', ['planning:=', logger]],
            on_exit=Shutdown(),
        ),

        Node(
            package='orchestrator_dummy_nodes',
            executable='service_provider_node',
            name='egomotion_provider',
            exec_name='egomotion_provider',
            parameters=[
                {"timer_period_s": 0.3}
            ],
            remappings=[("service", "egomotion"),
                        ("input", "meas/camera")],
            arguments=['--ros-args', '--log-level', ['egomotion:=', logger]],
            on_exit=Shutdown(),
        ),
    ])
