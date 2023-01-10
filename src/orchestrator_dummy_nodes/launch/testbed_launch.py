from launch import LaunchDescription
from launch_ros.actions import Node
from orchestrator_dummy_nodes.topic_remapping import intercepted_name

nr_publishers = 5


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestrator_dummy_nodes',
            executable='parallel_undeterministic_publisher',
            name='publisher',
            parameters=[
                {"in_order": False},
                {"use_timer": True},
                {"nr_publishers": nr_publishers}
            ]
        ),
        Node(
            package='orchestrator_dummy_nodes',
            executable='multi_subscriber',
            name='subscriber',
            parameters=[
                {"nr_publishers": nr_publishers},
            ],
        ),
    ])
