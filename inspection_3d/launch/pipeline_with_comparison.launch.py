"""inspection_3d 검사 파이프라인 표준 launch.

이 launch는 프로젝트에서 3D 검사에 필요한 최소 노드 셋을 함께 띄운다.

- ``pipeline_node``: PointCloud2 캡처, 누적 ICP 병합, finalize 저장
- ``comparison_node``: 기준 PCD와 비교
- ``static_transform_publisher``: 카메라 프레임 고정 TF

검사 결과 저장 위치는 HMI 패키지 위치를 따라간다. 현재는
``src/cobot2_ws/operator_ui/pointclouds``를 검사 결과 저장 루트로 사용한다.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_operator_ui_pointcloud_dir(workspace_dir: Path) -> Path:
    """워크스페이스 내 operator_ui 패키지의 pointcloud 저장 루트를 찾는다."""

    candidates = [
        workspace_dir / "src" / "cobot2_ws" / "operator_ui" / "pointclouds",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return candidates[0].resolve()


def generate_launch_description() -> LaunchDescription:
    workspace_dir = Path(__file__).resolve().parents[4]
    operator_ui_pointcloud_dir = resolve_operator_ui_pointcloud_dir(workspace_dir)
    default_bolt_capture_dir = str(operator_ui_pointcloud_dir / "bolt" / "captures")
    default_outlet_capture_dir = str(operator_ui_pointcloud_dir / "outlet" / "captures")

    object_type = LaunchConfiguration("object_type")
    multitap_reference_path = LaunchConfiguration("multitap_reference_path")
    bolt_reference_path = LaunchConfiguration("bolt_reference_path")
    input_topic = LaunchConfiguration("input_topic")
    save_frame = LaunchConfiguration("save_frame")
    bolt_capture_dir = LaunchConfiguration("bolt_capture_dir")
    outlet_capture_dir = LaunchConfiguration("outlet_capture_dir")

    arguments = [
        DeclareLaunchArgument("object_type", default_value="multitap"),
        DeclareLaunchArgument("multitap_reference_path", default_value=""),
        DeclareLaunchArgument("bolt_reference_path", default_value=""),
        DeclareLaunchArgument(
            "bolt_capture_dir",
            default_value=default_bolt_capture_dir,
        ),
        DeclareLaunchArgument(
            "outlet_capture_dir",
            default_value=default_outlet_capture_dir,
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/camera/camera/depth/color/points",
        ),
        DeclareLaunchArgument("save_frame", default_value="base_link"),
    ]

    pipeline_node = Node(
        package="inspection_3d",
        executable="pipeline_node",
        name="pointcloud_pipeline",
        output="screen",
        parameters=[
            {
                "object_type": object_type,
                "input_topic": input_topic,
                "save_frame": save_frame,
                "filtered_dir": outlet_capture_dir,
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
        package="inspection_3d",
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
