from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="robot_control",
                executable="robot_command_server",
                name="robot_command_server",
                output="screen",
            ),
            Node(
                package="robot_control",
                executable="voice_command_dispatcher",
                name="voice_command_dispatcher_node",
                output="screen",
            ),
        ]
    )
