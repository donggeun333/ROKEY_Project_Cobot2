"""플러그 삽입 단독 실행용.

통합 경로(음성 → robot_command_server)로 쓸 때는 이 launch가 필요 없다.
``robot_command_server``가
`run_plug_insert()`를 직접 부른다.

이 launch는 삽입 시퀀스만 따로 돌려볼 때 쓴다 — 미리보기 창이 뜬다.
카메라와 로봇 드라이버(`bringup_camera.launch.py`)는 미리 떠 있어야 한다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="outlet_assembly",
            executable="plug_insert_standalone",
            name="plug_insert_node",
            namespace="dsr01",
            output="screen",
        ),
    ])
