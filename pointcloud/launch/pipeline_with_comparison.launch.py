from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    object_type = LaunchConfiguration("object_type")
    multitap_reference_path = LaunchConfiguration("multitap_reference_path")
    bolt_reference_path = LaunchConfiguration("bolt_reference_path")
    input_topic = LaunchConfiguration("input_topic")
    save_frame = LaunchConfiguration("save_frame")

    arguments = [
        DeclareLaunchArgument("object_type", default_value="multitap"),
        DeclareLaunchArgument("multitap_reference_path", default_value=""),
        DeclareLaunchArgument("bolt_reference_path", default_value=""),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/camera/camera/depth/color/points",
        ),
        DeclareLaunchArgument("save_frame", default_value="base_link"),
    ]

    pipeline_node = Node(
        package="pointcloud",
        executable="pipeline_node",
        name="pointcloud_pipeline",
        output="screen",
        parameters=[
            {
                "object_type": object_type,
                "input_topic": input_topic,
                "save_frame": save_frame,
                "trigger_comparison_on_finalize": True,
                "comparison_service": "/pointcloud_comparison/compare",
            }
        ],
    )

    comparison_node = Node(
        package="pointcloud",
        executable="comparison_node",
        name="pointcloud_comparison",
        output="screen",
        parameters=[
            {
                "object_type": object_type,
                "multitap_reference_path": multitap_reference_path,
                "bolt_reference_path": bolt_reference_path,
            }
        ],
    )

    return LaunchDescription(arguments + [pipeline_node, comparison_node])
