"""공구 정리/전달 노드를 구동하고 완료를 판정하는 공통 클라이언트.

`tool_sorter_cleanup`와 `tool_sorter_handover`는 둘 다 같은
방식으로 외부에 노출되어 있다.

- 시작/중지는 `std_srvs/Trigger` 서비스
- 진행 상황은 transient-local JSON 상태 토픽

서비스 응답은 "요청을 접수했는가"만 즉시 알려주고, 실제 작업은 수십 초에서
수 분이 걸린다. 그래서 완료 판정은 반드시 상태 토픽으로 해야 한다.

이 모듈은 그 두 가지(Trigger 호출, 상태 대기)만 제공하고 작업 내용은 모른다.
실제 시퀀스는 `tool_handover_task.py`와 `tool_cleanup_task.py`가 담당한다.

## 왜 spin_once로 직접 돌리는가

`robot_command_server`는 액션 서버 노드만 `rclpy.spin()`으로 돌리고, task용
runtime 노드는 executor에 넣지 않는다(`pointcloud_inspector_task`가
`spin_until_future_complete(node, ...)`를 직접 부르는 것과 같은 구조다).
executor에 없는 노드는 아무도 대신 돌려주지 않으므로 상태 토픽 콜백이 영영
불리지 않는다. 그래서 대기 루프가 매 회 `spin_once`로 직접 콜백을 돌린다.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

# 상태 토픽은 transient-local로 발행된다. 늦게 붙는 구독자도 마지막 상태를
# 받아야 하기 때문인데, 구독 쪽 QoS가 다르면 아예 매칭이 되지 않는다.
STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# 이 상태들이면 해당 노드가 일을 놓고 쉬고 있는 것으로 본다.
FREE_STATES = frozenset(
    {"IDLE", "COMPLETE", "STOPPED", "ERROR", "BLOCKED", "SAFETY_STOP"}
)

# 작업이 실패로 끝난 상태. 성공/실패를 가르는 데 쓴다.
FAILED_STATES = frozenset({"ERROR", "BLOCKED", "SAFETY_STOP", "STOPPED"})

_SPIN_TIMEOUT_SEC = 0.1


class ToolSorterStatus:
    """한 tool sorter 노드의 상태 토픽을 따라가는 구독자.

    runtime 노드 하나에 여러 개(정리용/전달용)가 붙을 수 있으므로 노드에
    prefix별로 캐시해 재사용한다. 액션 goal이 올 때마다 새로 구독하면 같은
    토픽에 구독자가 계속 쌓인다.
    """

    def __init__(self, node: Node, prefix: str) -> None:
        self._node = node
        self._prefix = prefix
        self._state = "IDLE"
        self._message = ""
        node.create_subscription(
            String,
            f"{prefix}/status",
            self._on_status,
            STATUS_QOS,
        )

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        self._state = str(payload.get("state", self._state))
        self._message = str(payload.get("message", ""))

    @property
    def state(self) -> str:
        return self._state

    @property
    def message(self) -> str:
        return self._message

    def wait_for(self, target_states, timeout_sec: float) -> str | None:
        """`target_states` 중 처음 관측된 상태를 반환. 타임아웃이면 None.

        어느 상태로 끝났는지에 따라 분기하는 호출자는 반드시 이 반환값을 써야
        한다. 대기가 끝난 뒤 다시 `state`를 읽으면 그 사이에 상태가 또 넘어가
        있을 수 있다.
        """

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=_SPIN_TIMEOUT_SEC)
            if self._state in target_states:
                return self._state
        return None

    def wait_until_not(self, held_states, timeout_sec: float) -> bool:
        """`held_states`를 벗어날 때까지 대기."""

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=_SPIN_TIMEOUT_SEC)
            if self._state not in held_states:
                return True
        return False


def get_status_tracker(node: Node, prefix: str) -> ToolSorterStatus:
    """노드에 캐시된 상태 구독자를 돌려주고, 없으면 만든다."""

    trackers = getattr(node, "_tool_sorter_status_trackers", None)
    if trackers is None:
        trackers = {}
        setattr(node, "_tool_sorter_status_trackers", trackers)
    if prefix not in trackers:
        trackers[prefix] = ToolSorterStatus(node, prefix)
    return trackers[prefix]


def get_trigger_client(node: Node, service_name: str):
    """노드에 캐시된 Trigger 클라이언트를 돌려주고, 없으면 만든다."""

    clients = getattr(node, "_tool_sorter_trigger_clients", None)
    if clients is None:
        clients = {}
        setattr(node, "_tool_sorter_trigger_clients", clients)
    if service_name not in clients:
        clients[service_name] = node.create_client(Trigger, service_name)
    return clients[service_name]


def call_trigger(
    node: Node,
    service_name: str,
    connect_timeout_sec: float,
    response_timeout_sec: float,
) -> tuple[bool, str]:
    """Trigger 서비스를 부르고 `(success, message)`를 반환한다.

    접수 실패와 연결 실패를 구분하지 않는다. 호출자 입장에서는 둘 다 "작업이
    시작되지 않았다"로 같고, 사유는 message에 담겨 나간다.
    """

    client = get_trigger_client(node, service_name)
    if not client.wait_for_service(timeout_sec=connect_timeout_sec):
        return False, f"서비스 연결 실패: {service_name}"

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=response_timeout_sec)
    if not future.done():
        future.cancel()
        return False, f"서비스 응답 타임아웃: {service_name}"
    response = future.result()
    if response is None:
        return False, f"서비스 응답 없음: {service_name}"
    return bool(response.success), str(response.message)
