from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    parent_frame = LaunchConfiguration("parent_frame")
    child_frame = LaunchConfiguration("child_frame")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    base_frame = LaunchConfiguration("base_frame")
    publish_topic = LaunchConfiguration("publish_topic")

    arguments = [
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
        ),
        DeclareLaunchArgument(
            "parent_frame",
            default_value="link_6",
        ),
        DeclareLaunchArgument(
            "child_frame",
            default_value="camera_link",
        ),
        DeclareLaunchArgument(
            "camera_x",
            default_value="0.02",
            description="Camera X offset from link_6 [m]",
        ),
        DeclareLaunchArgument(
            "camera_y",
            default_value="0.075",
            description="Camera Y offset from link_6 [m]",
        ),
        DeclareLaunchArgument(
            "camera_z",
            default_value="0.04",
            description="Camera Z offset from link_6 [m]",
        ),
        DeclareLaunchArgument(
            "camera_roll",
            default_value="-1.5708",
            description="Camera roll from link_6 [rad]",
        ),
        DeclareLaunchArgument(
            "camera_pitch",
            default_value="-1.5708",
            description="Camera pitch from link_6 [rad]",
        ),
        DeclareLaunchArgument(
            "camera_yaw",
            default_value="0.0",
            description="Camera yaw from link_6 [rad]",
        ),
        DeclareLaunchArgument(
            "publish_topic",
            default_value="/pointcloud/merged",
        ),
    ]

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="link6_to_camera_tf",
        output="screen",
        arguments=[
            "--x",
            camera_x,
            "--y",
            camera_y,
            "--z",
            camera_z,
            "--roll",
            camera_roll,
            "--pitch",
            camera_pitch,
            "--yaw",
            camera_yaw,
            "--frame-id",
            parent_frame,
            "--child-frame-id",
            child_frame,
        ],
    )

    merger_node = Node(
        package="pointcloud",
        executable="merger_node",
        name="pointcloud_merger",
        output="screen",
        parameters=[
            {
                "base_frame": base_frame,
                "camera_frame": child_frame,
                "publish_topic": publish_topic,
            }
        ],
    )

    return LaunchDescription(arguments + [camera_mount_tf, merger_node])
