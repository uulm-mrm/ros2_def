from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config_file


def generate_launch_description():
    config_package = "orchestrator_dummy_nodes"
    config_file = "time_sync_test_launch_config.json"

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_uri",
            default_value=TextSubstitution(text="/home/gja38/sandbox_otto/rosbag2_2023_03_15-14_01_00"),
            description="Bag file path"
        ),
        DeclareLaunchArgument(
            "no_wait",
            default_value=TextSubstitution(text="true"),
            description="Skip time whenever possible"
        ),
        Node(
            package="orchestrator_rosbag_player",
            executable="rosbag_player",
            parameters=[
                {"bag_uri": LaunchConfiguration("bag_uri")},
                {"rate": 1.0},
                {"no_wait": LaunchConfiguration("no_wait")},
                {"launch_config_package": config_package},
                {"launch_config_file": config_file}],
            on_exit=[Shutdown(reason="Player done.")]
        ),
        *generate_remappings_from_config_file(config_package, config_file),
        Node(
            package='orchestrator_dummy_nodes',
            executable='camera_input_node',
            name='camera_input'
        )
    ])
