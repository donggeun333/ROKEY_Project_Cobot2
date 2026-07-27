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
                "trigger_comparison_on_finalize": False,
                "comparison_service": "/pointcloud_comparison/compare",
            }
        ],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="pointcloud_camera_static_tf",
        output="screen",
        arguments=[
            "--x", "0.02",
            "--y", "0.075",
            "--z", "0.04",
            "--roll", "-1.5708",
            "--pitch", "-1.5708",
            "--yaw", "0.0",
            "--frame-id", "link_6",
            "--child-frame-id", "camera_link",
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

    return LaunchDescription(arguments + [static_tf_node, pipeline_node, comparison_node])
