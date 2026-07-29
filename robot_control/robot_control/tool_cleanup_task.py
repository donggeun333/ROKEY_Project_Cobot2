"""공구 정리(TOOL_CLEANUP) task.

작업장에 놓인 공구를 클래스별 지정 위치로 되돌린다. 인식, 좌표 변환, 로봇
동작은 전부 `tool_sorter_cleanup` 노드가 수행하고, 여기서는 그 노드를
시작시킨 뒤 완료를 판정하는 일만 한다.

실제 시퀀스(Bird view 이동 -> depth 순 Pick & Place -> 매 Pick 후 재촬영)는
그 패키지의 README를 참고한다. 5종이 다 없어도 되고, 못 본 공구 이름은 완료
메시지에 남는다.
"""

from __future__ import annotations

from rclpy.node import Node

from robot_control.task_config import (
    TOOL_CLEANUP_PREFIX,
    TOOL_CLEANUP_START_SERVICE,
    TOOL_CLEANUP_TIMEOUT_SEC,
    TOOL_SERVICE_CONNECT_TIMEOUT_SEC,
    TOOL_SERVICE_RESPONSE_TIMEOUT_SEC,
)
from robot_control.tool_sorter_status import (
    FAILED_STATES,
    FREE_STATES,
    call_trigger,
    get_status_tracker,
)

# organize 접수 직후 STARTING이 발행되므로 곧바로 관측된다. 그래도 DDS 전파와
# spin 타이밍을 감안해 여유를 둔다.
_BUSY_TIMEOUT_SEC = 10.0


def run_tool_cleanup(node: Node) -> tuple[bool, str]:
    """공구 정리를 실행하고 `(success, message)`를 반환한다."""

    status = get_status_tracker(node, TOOL_CLEANUP_PREFIX)

    accepted, message = call_trigger(
        node,
        TOOL_CLEANUP_START_SERVICE,
        TOOL_SERVICE_CONNECT_TIMEOUT_SEC,
        TOOL_SERVICE_RESPONSE_TIMEOUT_SEC,
    )
    if not accepted:
        return False, f"공구 정리 시작 실패: {message}"

    # 접수됐다고 곧바로 완료를 기다리면 안 된다. 상태 토픽은 transient-local
    # 이라 직전 작업의 COMPLETE가 아직 남아 있을 수 있고, 그러면 시작하자마자
    # "이미 끝났다"로 오판한다. 먼저 실제로 바빠지는 것을 확인한다.
    if not status.wait_until_not(FREE_STATES, _BUSY_TIMEOUT_SEC):
        return False, (
            "공구 정리가 접수됐지만 시작되지 않았습니다 "
            f"(state={status.state})"
        )

    node.get_logger().info("공구 정리 진행 중...")
    final_state = status.wait_for(FREE_STATES, TOOL_CLEANUP_TIMEOUT_SEC)
    if final_state is None:
        return False, (
            f"공구 정리 완료 대기 타임아웃({TOOL_CLEANUP_TIMEOUT_SEC:.0f}s), "
            f"state={status.state}"
        )
    if final_state in FAILED_STATES:
        return False, f"공구 정리 중단({final_state}): {status.message}"
    return True, status.message or "공구 정리 완료"
