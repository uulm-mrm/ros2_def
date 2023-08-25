from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, Shutdown
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config_file


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
                {"mode": "reconfiguration"},
            ],
            arguments=['--ros-args', '--log-level', ['l:=', LaunchConfiguration('log_level')]],
            on_exit=Shutdown(),
        ),
        *generate_remappings_from_config_file("orchestrator_dummy_nodes",
                                              "reconfiguration_test_before_launch_config.json"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package='orchestrator_dummy_nodes',
            executable='reconfigurator',
            name='reconfigurator',
            exec_name='reconfigurator',
            on_exit=Shutdown(),
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='configurable_forwarding_node',
            name='A',
            exec_name='A',
            remappings=[("output", "T1")],
            on_exit=Shutdown(),
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='configurable_forwarding_node',
            name='B',
            exec_name='B',
            remappings=[("input", "T1")],
            on_exit=Shutdown(),
        ),
    ])
