from launch import LaunchDescription
from launch_ros.actions import Node
from orchestrator_dummy_nodes.topic_remapping import intercepted_name
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

nr_publishers = 5


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("INFO")),
            description="Logging level"
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='parallel_undeterministic_publisher',
            name='publisher',
            parameters=[
                {"in_order": False},
                {"use_timer": False},
                {"nr_publishers": nr_publishers}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='multi_subscriber',
            name='subscriber',
            parameters=[
                {"nr_publishers": nr_publishers},
            ],
            remappings=[(f"/topic_{i}", intercepted_name("subscriber", f"topic_{i}")) for i in range(0, nr_publishers)],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])
