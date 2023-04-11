from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("info")),
            description="Logging level"
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                {"mode": "double_timer"},
            ],
            arguments=['--ros-args', '--log-level', ['l:=', LaunchConfiguration('log_level')]]
        ),
        *generate_remappings_from_config("orchestrator_dummy_nodes", "double_timer_test_launch_config.json"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package='orchestrator_dummy_nodes',
            executable='double_timer_publisher',
            name='timer'
        ),
    ])
