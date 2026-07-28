"""공구 정리(TOOL_CLEANUP) / 공구 전달(TOOL_FETCH)용 노드 스택.

`robot_command_server`가 이 노드들의 Trigger 서비스를 부른다. 서버 자신은
`voice_command_stack.launch.py`가 띄우므로 여기엔 없다.

## 왜 각 패키지의 launch를 그냥 두 개 켜지 않는가

`autonomous_tool_sorter.launch.py`와 `handover.launch.py`를 동시에 켜면
`perception_node`와 TF broadcaster 2개가 **두 벌씩** 뜬다. 같은 노드 이름으로
같은 카메라 토픽을 두 프로세스가 구독하게 되어 노드 이름이 충돌하고, GPU에서
YOLO가 두 번 돈다.

그래서 여기서는 공유 노드(TF 2개 + perception)를 한 번만 띄우고, 그 위에
task manager 둘을 이름만 분리해서 붙인다. 실제로 움직이는 것은 항상 하나뿐이다
- 두 task manager 모두 자체 재진입 방지가 있고, 액션 서버가 goal을 하나씩
순차 실행하기 때문이다.

GUI(dashboard)는 띄우지 않는다. 이 구성에서 조작 주체는 HMI와 음성이다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sorter_config = LaunchConfiguration("sorter_config")
    automation_config = LaunchConfiguration("automation_config")
    handover_config = LaunchConfiguration("handover_config")
    transform_config = LaunchConfiguration("transform_config")
    calibration_path = PathJoinSubstitution(
        [
            FindPackageShare("m0609_tool_sorter_unified"),
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
                        FindPackageShare("m0609_tool_sorter_unified"),
                        "config",
                        "tool_sorter.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "automation_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("m0609_tool_sorter_autonomous"),
                        "config",
                        "autonomous_tool_sorter.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "handover_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("m0609_tool_sorter_handover"),
                        "config",
                        "handover.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "transform_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("m0609_tool_sorter_unified"),
                        "config",
                        "timestamped_rgbd_transform.yaml",
                    ]
                ),
            ),
            # ---- 공유 노드: 한 번만 ----
            Node(
                package="m0609_tool_sorter_unified",
                executable="tcp_pose_tf_broadcaster",
                name="m0609_tcp_pose_tf_broadcaster",
                output="screen",
                parameters=[transform_config],
            ),
            Node(
                package="m0609_tool_sorter_unified",
                executable="hand_eye_tf_broadcaster",
                name="m0609_hand_eye_tf_broadcaster",
                output="screen",
                parameters=[
                    transform_config,
                    {"transform_path": calibration_path},
                ],
            ),
            Node(
                package="m0609_tool_sorter_unified",
                executable="perception_node",
                name="tool_sorter_perception",
                output="screen",
                parameters=[sorter_config],
            ),
            # ---- 기능별 task manager: 이름을 분리해서 동시에 띄운다 ----
            Node(
                package="m0609_tool_sorter_autonomous",
                executable="autonomous_task_manager",
                name="tool_sorter_autonomous_task_manager",
                output="screen",
                parameters=[sorter_config, automation_config],
            ),
            Node(
                package="m0609_tool_sorter_handover",
                executable="handover_task_manager",
                name="tool_sorter_handover_task_manager",
                output="screen",
                parameters=[sorter_config, handover_config],
            ),
        ]
    )
