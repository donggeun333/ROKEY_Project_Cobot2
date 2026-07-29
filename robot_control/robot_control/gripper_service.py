"""OnRobot RG2 서비스 클라이언트.

실행 스택 전체에서 그리퍼 소켓 소유권을 ``onrobot_rg_control`` 드라이버 하나로
통일하기 위해, 직접 Modbus 대신 ``/onrobot/sendCommand`` 서비스만 사용한다.
볼트 체결, inspection_3d 검사 전 개방, 플러그 삽입, 공구 정리/전달이 모두 이
방식을 공유한다.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from robot_control.task_config import (
    GRIPPER_CALL_TIMEOUT_SEC,
    GRIPPER_SERVICE_NAME,
    GRIPPER_SERVICE_WAIT_SEC,
)


class GripperServiceError(RuntimeError):
    """그리퍼 서비스가 없거나 명령이 거절/타임아웃 됐을 때 던진다."""


class OnRobotServiceGripper:
    """RG2를 `onrobot_rg_control` 드라이버의 서비스로 여닫는다.

    `onrobot.RG`(직접 Modbus)를 쓰지 않는 이유는 Compute Box 소켓의 소유자를
    드라이버 하나로 통일하기 위해서다. 볼트 체결이 자기 소켓을 따로 열면 공구
    정리/전달이 쓰는 드라이버와 소유권이 갈려, 두 기능을 한 세션에서 쓸 수
    없게 된다.

    드라이버가 없을 때 직접 Modbus로 되돌아가는 폴백은 의도적으로 두지 않았다.
    폴백이 있으면 소켓이 조용히 둘로 갈리면서 원래 문제로 되돌아간다.

    힘(force)은 드라이버가 들고 있는 값을 그대로 쓴다. `"o"`/`"c"`는
    `rgfr`을 건드리지 않고, 드라이버는 이 값을 RG2 최대치인 400(40N)으로
    초기화한다. 직접 Modbus 시절의 `open_gripper()`/`close_gripper()` 기본값
    `force_val=400`과 같은 값이라 파지력은 달라지지 않는다.
    """

    def __init__(
        self,
        node: Node,
        service_name: str = GRIPPER_SERVICE_NAME,
        wait_sec: float = GRIPPER_SERVICE_WAIT_SEC,
        call_timeout_sec: float = GRIPPER_CALL_TIMEOUT_SEC,
    ) -> None:
        # onrobot_rg_msgs가 없는 환경에서 robot_command_server 전체가 죽지
        # 않도록 여기서 import한다. 그 경우 볼트만 실패하고 공구 작업은 산다.
        from onrobot_rg_msgs.srv import SetCommand

        self._node = node
        self._service_type = SetCommand
        self._service_name = service_name
        self._call_timeout_sec = float(call_timeout_sec)
        self._client = node.create_client(SetCommand, service_name)

        if not self._client.wait_for_service(timeout_sec=float(wait_sec)):
            raise GripperServiceError(
                f"그리퍼 서비스를 찾을 수 없습니다: {service_name}. "
                "bringup의 OnRobotRGControllerServer가 떠 있는지 확인한다."
            )
        node.get_logger().info(f"그리퍼 서비스 연결: {service_name}")

    def send_command(self, command: str) -> None:
        """드라이버 명령 문자열을 그대로 보낸다 (`o`, `c`, 또는 0.1mm 단위 정수)."""
        request = self._service_type.Request()
        request.command = command

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=self._call_timeout_sec
        )

        if not future.done():
            future.cancel()
            raise GripperServiceError(
                f"그리퍼 명령 '{command}' 응답 시간 초과 "
                f"({self._call_timeout_sec}s, {self._service_name})"
            )

        response = future.result()
        if response is None:
            raise GripperServiceError(f"그리퍼 명령 '{command}' 응답이 비었습니다.")
        if not response.success:
            raise GripperServiceError(
                f"그리퍼 명령 '{command}'이(가) 거절됐습니다: {response.message}"
            )

    def open_gripper(self) -> None:
        """그리퍼를 최대 폭으로 연다. 직접 Modbus의 write와 마찬가지로 동작
        완료(busy flag)를 기다리지 않고 바로 반환하므로, 호출부가 이동 전에
        기존과 같은 대기 시간을 둬야 한다."""
        self.send_command("o")

    def close_gripper(self) -> None:
        """그리퍼를 닫는다. `open_gripper`와 같이 비동기다."""
        self.send_command("c")
