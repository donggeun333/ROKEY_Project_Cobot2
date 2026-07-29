"""공구 전달(TOOL_FETCH) task.

요청받은 공구를 공구함에서 찾아 집어 작업자에게 건네준다. 인식, 좌표 변환,
파지, 힘 기반 전달 판정은 전부 `tool_sorter_handover` 노드가 수행하고,
여기서는 그 노드를 구동하고 완료를 판정하는 일만 한다.

## 왜 start와 request가 따로인가

handover 노드는 세션 개념을 쓴다. `start`로 세션을 열면 공구함 관측 자세로
이동한 뒤 `WAITING_REQUEST`가 되고, **그 상태일 때만** 공구 요청을 받는다.
세션이 열리기 전에 발행된 요청은 조용히 버려지므로(`handover_task_manager`의
`_on_request`) 반드시 start -> WAITING_REQUEST 확인 -> request 순서를 지켜야
한다.
"""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import String

from tool_sorter_handover.tool_request import (
    SPOKEN_TOOL_NAMES,
    normalize_tool_request,
    spoken_tool_name,
)
from robot_control.task_config import (
    TOOL_HANDOVER_PICKUP_TIMEOUT_SEC,
    TOOL_HANDOVER_PREFIX,
    TOOL_HANDOVER_READY_TIMEOUT_SEC,
    TOOL_HANDOVER_REQUEST_TOPIC,
    TOOL_HANDOVER_START_SERVICE,
    TOOL_HANDOVER_STOP_SERVICE,
    TOOL_HANDOVER_STOP_TIMEOUT_SEC,
    TOOL_HANDOVER_TIMEOUT_SEC,
    TOOL_SERVICE_CONNECT_TIMEOUT_SEC,
    TOOL_SERVICE_RESPONSE_TIMEOUT_SEC,
)
from robot_control.tool_sorter_status import (
    FAILED_STATES,
    FREE_STATES,
    call_trigger,
    get_status_tracker,
)

WAITING_REQUEST = "WAITING_REQUEST"


def resolve_tool_name(tools: list[str]) -> str | None:
    """음성에서 온 공구 이름들을 handover가 아는 이름 하나로 정규화한다.

    HMI는 인식된 낱말을 그대로 실어 보내므로 "망치"일 수도 "hammer"일 수도
    있다. 별칭 표와 자모 퍼지 매칭은 handover 패키지가 이미 갖고 있으므로 그
    구현을 그대로 쓴다 - 여기서 표를 다시 만들면 두 벌이 어긋난다.
    """

    for candidate in tools:
        resolved = normalize_tool_request(candidate)
        if resolved is not None:
            return resolved
    return None


def _get_request_publisher(node: Node):
    publisher = getattr(node, "_tool_handover_request_pub", None)
    if publisher is None:
        publisher = node.create_publisher(String, TOOL_HANDOVER_REQUEST_TOPIC, 10)
        setattr(node, "_tool_handover_request_pub", publisher)
    return publisher


def _stop_session(node: Node, status) -> None:
    """열린 채 남은 세션을 닫는다.

    공구를 못 찾으면 handover는 전달을 안 했으므로 COMPLETE를 내지 않고
    WAITING_REQUEST로 돌아가 다른 선택을 기다린다. 그 세션을 닫지 않으면 이
    task가 끝난 뒤에도 로봇이 요청 대기 상태로 붙잡혀 있다.
    """

    stopped, message = call_trigger(
        node,
        TOOL_HANDOVER_STOP_SERVICE,
        TOOL_SERVICE_CONNECT_TIMEOUT_SEC,
        TOOL_SERVICE_RESPONSE_TIMEOUT_SEC,
    )
    if not stopped:
        node.get_logger().error(f"handover 세션 종료 실패: {message}")
        return
    if status.wait_for(FREE_STATES, TOOL_HANDOVER_STOP_TIMEOUT_SEC) is None:
        node.get_logger().error(
            f"handover가 stop 이후에도 정지하지 않았습니다 (state={status.state})"
        )


def run_tool_handover(node: Node, tools: list[str]) -> tuple[bool, str]:
    """공구 전달을 실행하고 `(success, message)`를 반환한다."""

    tool_name = resolve_tool_name(tools)
    if tool_name is None:
        return False, (
            f"어떤 공구인지 알 수 없습니다: {tools}. "
            f"전달 가능한 공구: {', '.join(sorted(SPOKEN_TOOL_NAMES.values()))}"
        )
    spoken = spoken_tool_name(tool_name)
    status = get_status_tracker(node, TOOL_HANDOVER_PREFIX)

    accepted, message = call_trigger(
        node,
        TOOL_HANDOVER_START_SERVICE,
        TOOL_SERVICE_CONNECT_TIMEOUT_SEC,
        TOOL_SERVICE_RESPONSE_TIMEOUT_SEC,
    )
    if not accepted:
        return False, f"공구 전달 시작 실패: {message}"

    # 관측 자세로 이동을 마쳐야 요청을 받는다. 그 전에 발행하면 버려진다.
    if status.wait_for({WAITING_REQUEST}, TOOL_HANDOVER_READY_TIMEOUT_SEC) is None:
        _stop_session(node, status)
        return False, (
            "handover가 요청 대기 상태가 되지 않았습니다 "
            f"(state={status.state}, {status.message})"
        )

    request = String()
    request.data = spoken
    _get_request_publisher(node).publish(request)
    node.get_logger().info(f"'{spoken}' 전달 요청 전송")

    # 요청을 실제로 집어들었는지부터 본다. 이걸 건너뛰면 아래 대기가 "아직
    # 시작도 안 한 WAITING_REQUEST"를 곧바로 종료로 오인한다.
    if not status.wait_until_not({WAITING_REQUEST}, TOOL_HANDOVER_PICKUP_TIMEOUT_SEC):
        _stop_session(node, status)
        return False, f"handover가 '{spoken}' 요청을 받지 않았습니다"

    # 전달을 마치면 COMPLETE로 세션이 닫힌다. 공구를 못 찾은 경우에만
    # WAITING_REQUEST로 되돌아오며, 그때는 세션이 열린 채로 남는다.
    final_state = status.wait_for(
        FREE_STATES | {WAITING_REQUEST},
        TOOL_HANDOVER_TIMEOUT_SEC,
    )
    if final_state is None:
        _stop_session(node, status)
        return False, (
            f"공구 전달 완료 대기 타임아웃({TOOL_HANDOVER_TIMEOUT_SEC:.0f}s), "
            f"state={status.state}"
        )

    if final_state == WAITING_REQUEST:
        # 미검출로 돌아온 것이다. 사유는 handover가 상태 메시지에 담아뒀다.
        reason = status.message or f"{spoken}을(를) 찾지 못했습니다"
        _stop_session(node, status)
        return False, f"공구 전달 실패: {reason}"

    if final_state in FAILED_STATES:
        # 이미 자유 상태이므로 stop을 부르지 않는다. 부르면 원인이 담긴
        # ERROR/BLOCKED가 STOPPED로 덮여 사라진다.
        return False, f"공구 전달 중단({final_state}): {status.message}"

    return True, status.message or f"{spoken} 전달 완료"
