"""GUI for the five-tool autonomous sorting job.

The organize dashboard answers "무엇이 보이는가"; this one answers "무엇이 아직
남았는가". One press runs the whole job, so the screen is built around the five
required classes and their placement progress instead of a per-pick decision.
"""

from __future__ import annotations

from tool_sorter_core.dashboard import (
    DashboardWindow,
    run_dashboard,
)

from .autonomous_task_manager import REQUIRED_TOOL_NAMES


TOOL_DISPLAY_NAMES = {
    "hammer": "망치",
    "screwdriver": "드라이버",
    "wrench": "렌치",
    "monkey_wrench": "몽키렌치",
    "vise": "바이스",
}


def tool_progress_rows(
    completed_names: list[str] | None,
) -> list[tuple[str, str, bool]]:
    """Return (class, 표시 이름, 정리 완료) for every required tool.

    The order is fixed to ``REQUIRED_TOOL_NAMES`` so a row never moves while
    the job runs.
    """

    completed = {str(value) for value in (completed_names or [])}
    return [
        (name, TOOL_DISPLAY_NAMES.get(name, name), name in completed)
        for name in REQUIRED_TOOL_NAMES
    ]


def progress_summary(completed_names: list[str] | None) -> str:
    rows = tool_progress_rows(completed_names)
    done = sum(1 for _name, _label, finished in rows if finished)
    return f"{done} / {len(rows)}"


def remaining_tool_labels(completed_names: list[str] | None) -> list[str]:
    return [
        label
        for _name, label, finished in tool_progress_rows(completed_names)
        if not finished
    ]


class AutonomousDashboardWindow(DashboardWindow):
    WINDOW_TITLE = "M0609 자동 공구 정리"
    HEADER_TEXT = "M0609 · 공구 자동 정리"
    # 자동 정리는 시작하면 스스로 Home → Bird로 올라가지만, 시작 전에 셀을 눈으로
    # 확인하는 용도로 Bird Scan은 남겨둔다.
    SHOW_BIRD_BUTTON = True
    START_LABELS = {
        "pick_place": "자동 정리 시작",
        "point": "포인팅 시작",
        "inspect": "좌표 계산 시작",
    }

    def _build_extra_panel(self, outer) -> None:
        from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)

        header = QHBoxLayout()
        title = QLabel("공구 5종 정리 진행")
        title.setObjectName("muted")
        self.progress_value = QLabel("0 / 5")
        self.progress_value.setObjectName("metric")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.progress_value)
        layout.addLayout(header)

        chips = QHBoxLayout()
        self.tool_chips = {}
        for name, label, _finished in tool_progress_rows([]):
            chip = QLabel(f"⬜ {label}")
            chip.setStyleSheet(_chip_style(False))
            chips.addWidget(chip)
            self.tool_chips[name] = (chip, label)
        chips.addStretch()
        layout.addLayout(chips)

        self.remaining_label = QLabel("남은 공구: -")
        self.remaining_label.setObjectName("muted")
        self.remaining_label.setWordWrap(True)
        layout.addWidget(self.remaining_label)

        outer.addWidget(card)

    def _refresh_extra(self, status: dict, scene: dict | None) -> None:
        completed = status.get("completed_names") or []
        self.progress_value.setText(progress_summary(completed))
        for name, _label, finished in tool_progress_rows(completed):
            chip, label = self.tool_chips[name]
            chip.setText(f"{'✅' if finished else '⬜'} {label}")
            chip.setStyleSheet(_chip_style(finished))
        remaining = remaining_tool_labels(completed)
        self.remaining_label.setText(
            "남은 공구: " + (", ".join(remaining) if remaining else "없음")
        )


def _chip_style(finished: bool) -> str:
    color = "#34d399" if finished else "#9ca3af"
    background = "#064e3b" if finished else "#1f2937"
    return (
        f"color:{color};background:{background};border:1px solid #374151;"
        "border-radius:14px;padding:5px 12px;font-weight:700;"
    )


def main(args=None) -> None:
    run_dashboard(AutonomousDashboardWindow, args=args)


if __name__ == "__main__":
    main()
