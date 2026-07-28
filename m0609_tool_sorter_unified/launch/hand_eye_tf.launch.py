from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("m0609_tool_sorter_unified"),
                        "config",
                        "timestamped_rgbd_transform.yaml",
                    ]
                ),
            ),
            Node(
                package="m0609_tool_sorter_unified",
                executable="tcp_pose_tf_broadcaster",
                name="m0609_tcp_pose_tf_broadcaster",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="m0609_tool_sorter_unified",
                executable="hand_eye_tf_broadcaster",
                name="m0609_hand_eye_tf_broadcaster",
                output="screen",
                parameters=[config],
            ),
        ]
    )
