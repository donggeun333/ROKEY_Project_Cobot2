from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sorter_config = LaunchConfiguration("sorter_config")
    handover_config = LaunchConfiguration("handover_config")
    transform_config = LaunchConfiguration("transform_config")
    use_gui = LaunchConfiguration("use_gui")
    calibration_path = PathJoinSubstitution(
        [
            FindPackageShare("tool_sorter_core"),
            "models",
            "T_gripper2camera.npy",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sorter_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tool_sorter_core"),
                        "config",
                        "tool_sorter.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "handover_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tool_sorter_handover"),
                        "config",
                        "handover.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument(
                "transform_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tool_sorter_core"),
                        "config",
                        "timestamped_rgbd_transform.yaml",
                    ]
                ),
            ),
            Node(
                package="tool_sorter_core",
                executable="tcp_pose_tf_broadcaster",
                name="m0609_tcp_pose_tf_broadcaster",
                output="screen",
                parameters=[transform_config],
            ),
            Node(
                package="tool_sorter_core",
                executable="hand_eye_tf_broadcaster",
                name="m0609_hand_eye_tf_broadcaster",
                output="screen",
                parameters=[
                    transform_config,
                    {"transform_path": calibration_path},
                ],
            ),
            Node(
                package="tool_sorter_core",
                executable="perception_node",
                name="tool_sorter_perception",
                output="screen",
                parameters=[sorter_config],
            ),
            Node(
                package="tool_sorter_handover",
                executable="handover_task_manager",
                name="tool_sorter_task_manager",
                output="screen",
                parameters=[sorter_config, handover_config],
            ),
            # 전달 전용 dashboard. organize dashboard를 상속해 영상·검출
            # 테이블은 그대로 쓰고, 요청 버튼과 "지금 할 일" 안내를 앞에 둔다.
            Node(
                package="tool_sorter_handover",
                executable="handover_dashboard",
                name="tool_sorter_dashboard",
                output="screen",
                parameters=[
                    sorter_config,
                    {
                        # 버튼 글자가 곧 발행 문자열이다. 아래 다섯 낱말은
                        # tool_request.py의 TOOL_ALIASES에 그대로 있으므로
                        # 외부 제어부가 보내는 키워드와 같은
                        # 화이트리스트를 지난다.
                        "tool_request_topic": (
                            "/tool_sorter/handover/request"
                        ),
                        "tool_request_names": [
                            "망치",
                            "드라이버",
                            "렌치",
                            "몽키렌치",
                            "바이스",
                        ],
                    },
                ],
                condition=IfCondition(use_gui),
            ),
        ]
    )
