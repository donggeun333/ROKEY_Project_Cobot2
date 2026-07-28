"""robot_control 음성 명령 액션 스택.

현재 실사용 최소 구성은 ``robot_command_server`` 하나면 충분하다. 다만 옛 음성
스택(``voice_processing``의 ``/get_keyword`` 서비스)을 함께 써야 하는 환경에서는
이 launch가 ``voice_command_dispatcher``까지 같이 올려 준다.

정리:
- HMI(app_node) 기반 음성만 쓸 때: ``ros2 run robot_control robot_command_server``
- 레거시 ``/get_keyword`` 경로도 같이 쓸 때: 이 launch 사용
"""

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
