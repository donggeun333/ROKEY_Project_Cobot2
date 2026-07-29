from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sorter_config = LaunchConfiguration("sorter_config")
    automation_config = LaunchConfiguration("automation_config")
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
                "automation_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(
                            "tool_sorter_cleanup"
                        ),
                        "config",
                        "autonomous_tool_sorter.yaml",
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
                package="tool_sorter_cleanup",
                executable="autonomous_task_manager",
                name="tool_sorter_task_manager",
                output="screen",
                parameters=[sorter_config, automation_config],
            ),
            # 자동 정리 전용 dashboard. 시작/정지와 5종 진행 체크리스트만
            # 보면 되므로 use_gui:=false로 완전히 headless 실행도 가능하다.
            Node(
                package="tool_sorter_cleanup",
                executable="autonomous_dashboard",
                name="tool_sorter_dashboard",
                output="screen",
                parameters=[sorter_config, automation_config],
                condition=IfCondition(use_gui),
            ),
        ]
    )
