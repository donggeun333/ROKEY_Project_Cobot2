"""GUI for the tool handover flow.

Handover is a dialogue: the arm asks, the user answers with one tool, then the
user has to physically pull the tool out of the gripper. So this window puts
the current instruction and the five request buttons first, and drops the Bird
Scan button, which would drive the arm to the organize Bird view instead of the
``tool_pickup_scan_pose`` this package observes from.
"""

from __future__ import annotations

from tool_sorter_core.dashboard import (
    DashboardWindow,
    run_dashboard,
)


# 상태별 "지금 사람이 할 일". 값은 (문구, 색, 강조 여부)다.
# DEFAULT_PROMPT은 팔이 움직이는 중이라고 말하므로, 정지 상태는 여기에 반드시
# 항목이 있어야 한다. 없으면 멈춰 선 로봇을 두고 물러서라고 안내하게 된다.
PROMPTS = {
    "WAITING_REQUEST": ("필요한 공구를 선택하세요", "#93c5fd", True),
    "SEARCHING": ("요청한 공구를 찾는 중입니다", "#93c5fd", False),
    "WAITING_PULL": ("공구를 잡아당기세요", "#34d399", True),
    "RELEASING": ("그리퍼를 여는 중입니다. 공구를 받으세요", "#34d399", True),
    "GRIPPING": ("공구를 파지하는 중입니다", "#fbbf24", False),
    "IDLE": ("작업 시작을 누르면 공구함 관측 자세로 이동합니다", "#9ca3af", False),
    "BIRD_READY": ("관측 자세에서 대기 중입니다. 작업 시작을 누르세요", "#9ca3af", False),
    "COMPLETE": ("작업이 끝났습니다. 작업 시작으로 다시 진행하세요", "#34d399", False),
    "WAITING_TRANSFORM": ("좌표 변환 대기 중입니다. 잠시 기다리세요", "#fbbf24", False),
    "OFFLINE": ("task manager 연결 대기", "#9ca3af", False),
    "PAUSED": ("일시정지됨. 작업 시작으로 다시 진행하세요", "#fbbf24", False),
    "STOPPED": ("중지됨. 로봇 상태를 확인한 뒤 다시 시작하세요", "#f87171", False),
    "ERROR": ("오류. 로봇을 확인하고 task manager를 재시작하세요", "#f87171", True),
    "SAFETY_STOP": ("안전 정지. 현장을 확인하세요", "#f87171", True),
    "BLOCKED": ("설정 문제로 시작할 수 없습니다. 상태 메시지를 확인하세요", "#f87171", False),
}

DEFAULT_PROMPT = ("작업 진행 중입니다. 로봇에서 떨어져 계세요", "#60a5fa", False)


def handover_prompt(state: str, target: str = "") -> tuple[str, str, bool]:
    """Return (문구, 색, 강조) telling the operator what to do right now.

    ``target`` stays the canonical class name (``monkey_wrench``, not
    "몽키렌치") on purpose: it is the same string the detection table, the
    perception scene, and the status topic's ``target`` field show, so the
    operator can match this line against them without a mental translation.
    """

    text, color, emphasize = PROMPTS.get(str(state), DEFAULT_PROMPT)
    if state in {"WAITING_PULL", "RELEASING", "SEARCHING"} and target:
        text = f"[{target}] {text}"
    return text, color, emphasize


def request_hint(state: str) -> str:
    """Explain why the tool buttons are or are not clickable."""

    if state == "WAITING_REQUEST":
        return "버튼을 누르면 해당 공구를 찾아 Home 자세로 가져옵니다"
    # 정지 상태는 전부 "작업 시작을 누르라"로 모은다. START_STATES와 같은
    # 집합이라, 시작 버튼이 눌리는 상태와 안내가 어긋나지 않는다.
    if state in {"IDLE", "OFFLINE", "BIRD_READY", "COMPLETE", "STOPPED", "PAUSED"}:
        return (
            "먼저 작업 시작을 누르세요. 시작 전 요청은 무시됩니다"
        )
    return "현재 요청을 처리하는 중입니다. 요청 대기 상태에서 선택하세요"


class HandoverDashboardWindow(DashboardWindow):
    WINDOW_TITLE = "M0609 공구 전달"
    HEADER_TEXT = "M0609 · 공구 전달"
    # 전달은 tool_pickup_scan_pose에서 관측한다. Bird Scan은 다른 자세로 가므로
    # 버튼을 두지 않는다. 안내 문구도 그 버튼을 가리키면 안 된다.
    SHOW_BIRD_BUTTON = False
    RESCAN_HINT = "홈 복귀를 누른 뒤 다시 시작하세요"
    START_LABELS = {
        "pick_place": "전달 시작",
        "point": "포인팅 시작",
        "inspect": "좌표 계산 시작",
    }

    def _build_extra_panel(self, outer) -> None:
        from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout

        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        caption = QLabel("지금 할 일")
        caption.setObjectName("muted")
        self.prompt_label = QLabel("task manager 연결 대기")
        self.prompt_label.setWordWrap(True)
        self.request_hint_label = QLabel("")
        self.request_hint_label.setObjectName("muted")
        self.request_hint_label.setWordWrap(True)
        layout.addWidget(caption)
        layout.addWidget(self.prompt_label)
        layout.addWidget(self.request_hint_label)
        outer.addWidget(card)

    def _refresh_extra(self, status: dict, scene: dict | None) -> None:
        state = str(status.get("state", "OFFLINE"))
        target = str(status.get("current_target") or "")
        text, color, emphasize = handover_prompt(state, target)
        self.prompt_label.setText(text)
        self.prompt_label.setStyleSheet(
            f"color:{color};font-weight:700;"
            f"font-size:{'26px' if emphasize else '18px'};"
        )
        self.request_hint_label.setText(request_hint(state))


def main(args=None) -> None:
    run_dashboard(HandoverDashboardWindow, args=args)


if __name__ == "__main__":
    main()
