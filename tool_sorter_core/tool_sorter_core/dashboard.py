"""공구 정리/전달 상태를 시각화하는 공통 GUI 로직.

autonomous/handover 대시보드는 서로 다른 문구와 버튼을 쓰지만, 상태 토픽 구독,
최신 Scene 미리보기, 알림 배지 계산은 같은 규칙을 공유한다. 그 공통층이 이
모듈이다.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import numpy as np

from .grasp_selector import SORTABLE_TOOL_NAMES, is_grasp_ready
from .image_messages import image_message_to_bgr8


def _scene_stamp(scene: dict | None) -> float | None:
    if not scene:
        return None
    sec = scene.get("stamp_sec")
    nanosec = scene.get("stamp_nanosec")
    if sec is not None and nanosec is not None:
        return float(sec) + float(nanosec) * 1.0e-9
    stamp = scene.get("stamp")
    return float(stamp) if stamp is not None else None


def scene_overlay_is_fresh(
    scene: dict | None,
    image_stamp: float | None,
    max_age_s: float,
) -> bool:
    """Only draw detections close to the displayed raw camera frame."""

    scene_stamp = _scene_stamp(scene)
    if scene_stamp is None or image_stamp is None:
        return False
    return abs(float(image_stamp) - scene_stamp) <= max(
        float(max_age_s),
        0.0,
    )


def draw_scene_overlay(
    image: np.ndarray,
    scene: dict | None,
    lan_report: dict | None = None,
) -> np.ndarray:
    """Draw the latest lightweight scene geometry on a live raw frame."""

    import cv2

    output = image.copy()
    if not scene:
        return output
    source_width = max(float(scene.get("image_width", output.shape[1])), 1.0)
    source_height = max(
        float(scene.get("image_height", output.shape[0])),
        1.0,
    )
    scale_x = float(output.shape[1]) / source_width
    scale_y = float(output.shape[0]) / source_height

    def point(values) -> tuple[int, int]:
        return (
            int(round(float(values[0]) * scale_x)),
            int(round(float(values[1]) * scale_y)),
        )

    for detection in scene.get("detections", []):
        bbox = detection.get("bbox")
        if not bbox or len(bbox) < 4:
            continue
        x1, y1 = point(bbox[:2])
        x2, y2 = point(bbox[2:4])
        valid = bool(detection.get("grasp_ready")) and (
            detection.get("grasp_depth_mm") is not None
        )
        color = (52, 211, 153) if valid else (40, 40, 230)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        depth = detection.get("depth_mm")
        depth_text = (
            f"{float(depth):.0f}mm" if depth is not None else "DEPTH HOLD"
        )
        part = detection.get("part_name")
        if part is None:
            expected = detection.get("expected_part_name")
            part = f"missing:{expected}" if expected else "unsupported"
        label = (
            f"#{int(detection.get('rank', 0))} "
            f"{detection.get('name', '?')} "
            f"{float(detection.get('confidence', 0.0)):.2f} "
            f"{depth_text} [{part}]"
        )
        cv2.putText(
            output,
            label,
            (x1, max(20, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
            cv2.LINE_AA,
        )

        axis_box = detection.get("axis_box")
        center = detection.get("center_px")
        if not axis_box or not center:
            continue
        scaled_box = np.asarray(
            [point(value) for value in axis_box],
            dtype=np.int32,
        )
        cv2.polylines(output, [scaled_box], True, (0, 215, 255), 2)
        center_x, center_y = point(center)
        angle = math.radians(float(detection.get("axis_angle_deg") or 0.0))
        delta_x = int(round(55.0 * scale_x * math.cos(angle)))
        delta_y = int(round(55.0 * scale_y * math.sin(angle)))
        cv2.line(
            output,
            (center_x - delta_x, center_y - delta_y),
            (center_x + delta_x, center_y + delta_y),
            (255, 80, 230),
            2,
        )
        cv2.circle(output, (center_x, center_y), 4, (255, 80, 230), -1)

    report = lan_report or {}
    report_detections = {
        int(item["detection_id"]): item
        for item in report.get("detections", [])
        if "detection_id" in item
    }
    for pair in report.get("pairs", []):
        cable = report_detections.get(int(pair["cable_detection_id"]))
        port = report_detections.get(int(pair["port_detection_id"]))
        if cable is None or port is None:
            continue
        color = (0, 220, 0) if pair.get("matched") else (0, 0, 255)
        cv2.line(
            output,
            point(cable["center"]),
            point(port["center"]),
            color,
            3,
            cv2.LINE_AA,
        )
    status = str(report.get("status", "UNKNOWN"))
    status_colors = {
        "OK": (0, 220, 0),
        "NG": (0, 0, 255),
        "UNKNOWN": (0, 220, 255),
    }
    cv2.rectangle(output, (8, 8), (500, 46), (20, 20, 20), -1)
    cv2.putText(
        output,
        f"LAN {status}: {report.get('reason', '')}",
        (18, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        status_colors.get(status, status_colors["UNKNOWN"]),
        2,
        cv2.LINE_AA,
    )
    return output


def actionable_target_count(
    scene: dict | None,
    target_names: list[str] | None = None,
) -> int:
    """Count detections that task_manager._eligible_detections would accept.

    ``target_names`` mirrors the task manager parameter of the same name so the
    GUI does not advertise a tool the session is configured to skip.
    """

    if not scene:
        return 0
    allowed = set(SORTABLE_TOOL_NAMES)
    if target_names:
        allowed.intersection_update(str(value) for value in target_names)
    return sum(
        1
        for detection in scene.get("detections", [])
        if detection.get("name") in allowed
        and detection.get("grasp_ready")
        and is_grasp_ready(
            str(detection.get("name")),
            detection.get("part_name"),
        )
        and detection.get("grasp_depth_mm") is not None
        and detection.get("center_px") is not None
        and detection.get("axis_angle_deg") is not None
        and detection.get("grip_width_px") is not None
    )


def scan_input_ready(scene: dict | None, video_connected: bool) -> bool:
    """Return whether live perception input is available."""

    return video_connected and scene is not None


def bird_scan_button_enabled(
    state: str,
    service_ready: bool,
    inputs_ready: bool,
) -> bool:
    """Enable physical Bird motion only from a known stationary state."""

    return (
        service_ready
        and inputs_ready
        and state in {"IDLE", "BIRD_READY"}
    )


GO_HOME_STATES = {
    "IDLE",
    "BIRD_READY",
    "BLOCKED",
    "COMPLETE",
    "PAUSED",
    "STOPPED",
}


def tool_request_button_enabled(state: str) -> bool:
    """Enable tool buttons only while the session is actually asking.

    The request topic is dropped whenever the task manager is not waiting for
    one, so a button that stays clickable outside ``WAITING_REQUEST`` looks
    broken: the press is silently ignored. The usual reason for not being in
    that state is that no session was started at all.
    """

    return state == "WAITING_REQUEST"


def go_home_button_enabled(state: str, service_ready: bool) -> bool:
    """Enable Home 복귀 only while the robot is stationary and recoverable."""

    return service_ready and state in GO_HOME_STATES


START_STATES = {
    "IDLE",
    "BIRD_READY",
    "BLOCKED",
    "COMPLETE",
    "PAUSED",
    "STOPPED",
}

EXECUTION_MODE_LABELS = {
    "pick_place": "공구 집기 시작",
    "point": "포인팅 시작",
    "inspect": "좌표 계산 시작",
}


def live_view_is_work_view(scan_scope: str, auto_scan_motion: bool) -> bool:
    """Whether the displayed frame is the frame the session will act on.

    With ``auto_scan_motion`` the session first drives to Home/Bird and takes a
    new capture, so the current count says nothing about what it will find
    there — unless the robot already sits at an observation pose
    (``scan_scope`` other than ``none``), which is what Bird Scan and the
    Home-first scan leave behind.
    """

    return (not auto_scan_motion) or scan_scope != "none"


def start_button_enabled(
    state: str,
    service_ready: bool,
    inputs_ready: bool,
    actionable: int,
    work_view: bool,
) -> bool:
    """Enable 시작 only when the session can actually act on something.

    When the live frame is the work frame, zero actionable targets means the
    session would immediately finish with "정리할 공구가 없습니다", so the
    button stays disabled instead of promising motion that never happens.
    """

    if not (service_ready and inputs_ready):
        return False
    if state not in START_STATES:
        return False
    return actionable > 0 or not work_view


class DashboardRosBridge:
    def __init__(self):
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.node = Node("tool_sorter_dashboard")
        self._lock = threading.Lock()
        self._image = None
        self._scene = None
        self._lan_report = {
            "status": "UNKNOWN",
            "reason": "not_processed",
            "pairs": [],
            "detections": [],
        }
        self._status = {
            "state": "OFFLINE",
            "message": "작업 관리자 연결 대기",
            "execution_mode": "-",
            "completed": 0,
            "current_target": "",
        }
        self._notice = ""
        self._notice_at = 0.0
        self._last_image_at = 0.0
        self._image_stamp = None
        self._image_generation = 0
        self._preview_fps = 0.0
        self._overlay_max_age_s = float(
            self.node.declare_parameter("overlay_max_age_s", 0.75).value
        )
        preview_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.node.create_subscription(
            Image,
            str(
                self.node.declare_parameter(
                    "color_topic", "/camera/camera/color/image_raw"
                ).value
            ),
            self._on_image,
            preview_qos,
        )
        self.node.create_subscription(
            String,
            "/tool_sorter/perception/scene",
            self._on_scene,
            1,
        )
        self.node.create_subscription(
            String,
            "/tool_sorter/inspection/lan_color",
            self._on_lan_report,
            1,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            String,
            "/tool_sorter/task/status",
            self._on_status,
            status_qos,
        )
        self._clients = {
            "start": self.node.create_client(
                Trigger, "/tool_sorter/start"
            ),
            "pause": self.node.create_client(
                Trigger, "/tool_sorter/pause"
            ),
            "stop": self.node.create_client(Trigger, "/tool_sorter/stop"),
            "bird_scan": self.node.create_client(
                Trigger, "/tool_sorter/bird_scan"
            ),
            "go_home": self.node.create_client(
                Trigger, "/tool_sorter/go_home"
            ),
        }
        self._trigger_type = Trigger
        self._string_type = String

        # task manager와 같은 target_names를 읽어, 세션이 건너뛸 공구를 GUI가
        # "작업 가능"으로 세지 않게 한다. 비어 있으면 정렬 대상 전체가 후보다.
        from rclpy.parameter import Parameter as _Parameter

        self.node.declare_parameter(
            "target_names", _Parameter.Type.STRING_ARRAY
        )
        try:
            configured_targets = self.node.get_parameter("target_names").value
        except Exception:
            configured_targets = None
        self.target_names = [
            str(value) for value in (configured_targets or [])
        ]

        # 전달(handover)용 공구 요청 버튼. 토픽이나 이름이 비어 있으면 버튼을
        # 만들지 않으므로 organize dashboard 화면은 그대로다. 여기서는 문자열을
        # 그대로 발행할 뿐이고, 어떤 이름이 유효한지는 요청을 받는 쪽이 정한다.
        from rclpy.parameter import Parameter

        self.node.declare_parameter(
            "tool_request_names", Parameter.Type.STRING_ARRAY
        )
        try:
            declared = self.node.get_parameter("tool_request_names").value
        except Exception:
            declared = None
        self.tool_request_names = [str(value) for value in (declared or [])]
        request_topic = str(
            self.node.declare_parameter("tool_request_topic", "").value
        ).strip()
        self._tool_request_publisher = (
            self.node.create_publisher(String, request_topic, 10)
            if request_topic and self.tool_request_names
            else None
        )

    def tool_request_enabled(self) -> bool:
        return self._tool_request_publisher is not None

    def request_tool(self, name: str) -> None:
        publisher = self._tool_request_publisher
        if publisher is None:
            return
        publisher.publish(self._string_type(data=name))
        self._set_notice(f"공구 요청 발행: {name}")

    def _on_image(self, message) -> None:
        received_at = time.monotonic()
        try:
            image = image_message_to_bgr8(message)
        except Exception as error:
            self._set_notice(f"영상 변환 실패: {error}")
            return
        stamp = message.header.stamp
        image_stamp = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        with self._lock:
            if self._last_image_at > 0.0:
                elapsed = received_at - self._last_image_at
                if elapsed > 0.0:
                    instant_fps = 1.0 / elapsed
                    self._preview_fps = (
                        instant_fps
                        if self._preview_fps == 0.0
                        else 0.9 * self._preview_fps + 0.1 * instant_fps
                    )
            self._image = image
            self._image_stamp = image_stamp
            self._image_generation += 1
            self._last_image_at = received_at

    def _on_scene(self, message) -> None:
        try:
            scene = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._scene = scene

    def _on_lan_report(self, message) -> None:
        try:
            report = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._lan_report = report

    def _on_status(self, message) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._status = status

    def _set_notice(self, value: str) -> None:
        with self._lock:
            self._notice = value
            self._notice_at = time.monotonic()

    def call(self, name: str) -> None:
        client = self._clients[name]
        try:
            ready = client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            self._set_notice(f"{name} 서비스가 아직 준비되지 않았습니다")
            return
        future = client.call_async(self._trigger_type.Request())

        def complete(result_future):
            try:
                response = result_future.result()
                marker = "완료" if response.success else "거부"
                self._set_notice(f"{name} {marker}: {response.message}")
            except Exception as error:
                self._set_notice(f"{name} 호출 실패: {error}")

        future.add_done_callback(complete)

    def service_ready(self, name: str) -> bool:
        try:
            return bool(self._clients[name].service_is_ready())
        except Exception:
            return False

    def snapshot(self, after_image_generation: int = -1):
        with self._lock:
            image_changed = self._image_generation != after_image_generation
            return (
                (
                    self._image.copy()
                    if image_changed and self._image is not None
                    else None
                ),
                self._image_generation,
                self._image_stamp,
                self._preview_fps,
                None if self._scene is None else dict(self._scene),
                dict(self._status),
                dict(self._lan_report),
                self._notice,
                self._notice_at,
                self._last_image_at,
                self._overlay_max_age_s,
            )


def _stylesheet() -> str:
    return """
    QWidget {
        background: #111827;
        color: #e5e7eb;
        font-family: "Noto Sans CJK KR", "Noto Sans", sans-serif;
        font-size: 13px;
    }
    QFrame#card {
        background: #1f2937;
        border: 1px solid #374151;
        border-radius: 9px;
    }
    QLabel#title {
        color: #f9fafb;
        font-size: 22px;
        font-weight: 700;
    }
    QLabel#metric {
        color: #f9fafb;
        font-size: 18px;
        font-weight: 700;
    }
    QLabel#muted { color: #9ca3af; }
    QPushButton {
        min-height: 40px;
        padding: 0 18px;
        border: 0;
        border-radius: 7px;
        background: #2563eb;
        color: white;
        font-weight: 700;
    }
    QPushButton:hover { background: #3b82f6; }
    QPushButton#bird { background: #059669; }
    QPushButton#bird:hover { background: #10b981; }
    QPushButton#home { background: #4f46e5; }
    QPushButton#home:hover { background: #6366f1; }
    QPushButton#pause { background: #d97706; }
    QPushButton#stop { background: #dc2626; }
    QPushButton#request { background: #0f766e; min-height: 34px; }
    QPushButton#request:hover { background: #14b8a6; }
    QTableWidget {
        background: #172033;
        alternate-background-color: #1b263b;
        border: 1px solid #374151;
        border-radius: 7px;
        gridline-color: #374151;
    }
    QHeaderView::section {
        background: #253247;
        color: #d1d5db;
        border: 0;
        padding: 7px;
        font-weight: 700;
    }
    """


def _status_color(state: str) -> str:
    if state in {"ERROR", "SAFETY_STOP", "BLOCKED", "STOPPED"}:
        return "#f87171"
    if state in {"COMPLETE", "IDLE", "BIRD_READY"}:
        return "#34d399"
    if state in {"PAUSED", "WAITING_SCENE"}:
        return "#fbbf24"
    if state == "OFFLINE":
        return "#9ca3af"
    return "#60a5fa"


def _sanitize_qt_environment() -> None:
    """Remove OpenCV wheel paths that make PyQt load incompatible Qt plugins."""
    for name in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
        value = os.environ.get(name, "")
        if "/cv2/" in value.replace("\\", "/"):
            os.environ.pop(name, None)


class DashboardWindow:
    """Organize dashboard, and the shared base for the flow-specific ones.

    ``tool_sorter_cleanup`` and ``tool_sorter_handover`` ship
    their own window subclasses, so every difference between the three GUIs is
    expressed by the class attributes and hooks below rather than by copies of
    this layout.
    """

    WINDOW_TITLE = "M0609 Tool Sorter"
    HEADER_TEXT = "M0609 · RGB-D Tool Sorter"
    SHOW_BIRD_BUTTON = True
    START_LABELS = EXECUTION_MODE_LABELS
    # 시작 버튼이 잠겼을 때 관측 자세를 다시 잡는 방법. 창마다 쓸 수 있는
    # 버튼이 다르므로, 없는 버튼을 누르라고 안내하지 않도록 여기서 갈린다.
    RESCAN_HINT = "Bird Scan으로 관측 자세를 다시 잡으세요"

    def __init__(self, bridge: DashboardRosBridge):
        from PyQt5.QtCore import Qt, QTimer
        from PyQt5.QtWidgets import (
            QFrame,
            QGridLayout,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
        )

        self.bridge = bridge
        self.window = QMainWindow()
        self.window.setWindowTitle(self.WINDOW_TITLE)
        self.window.resize(1360, 820)
        self.window.setStyleSheet(_stylesheet())

        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(18, 16, 18, 16)
        outer.setSpacing(12)

        title_row = QHBoxLayout()
        title = QLabel(self.HEADER_TEXT)
        title.setObjectName("title")
        title_row.addWidget(title)
        title_row.addStretch()
        self.connection = QLabel("● 영상 대기")
        self.connection.setObjectName("muted")
        title_row.addWidget(self.connection)
        outer.addLayout(title_row)

        metrics = QHBoxLayout()
        self.state_value, state_card = self._card("작업 상태", "-")
        self.mode_value, mode_card = self._card("실행 모드", "-")
        self.fps_value, fps_card = self._card("추론 속도", "- FPS")
        self.count_value, count_card = self._card("검출 / 완료", "0 / 0")
        for card in (state_card, mode_card, fps_card, count_card):
            metrics.addWidget(card)
        outer.addLayout(metrics)

        body = QGridLayout()
        body.setColumnStretch(0, 3)
        body.setColumnStretch(1, 2)
        self.video = QLabel("검출 영상 수신 대기")
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(720, 480)
        self.video.setStyleSheet(
            "background:#030712;border:1px solid #374151;border-radius:9px;"
        )
        body.addWidget(self.video, 0, 0, 2, 1)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ["순서", "클래스", "신뢰도", "Depth", "파지부", "겹침", "상태"]
        )
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        body.addWidget(self.table, 0, 1)

        status_card = QFrame()
        status_card.setObjectName("card")
        status_layout = QVBoxLayout(status_card)
        status_label = QLabel("현재 작업")
        status_label.setObjectName("muted")
        self.message = QLabel("작업 관리자 연결 대기")
        self.message.setWordWrap(True)
        self.message.setObjectName("metric")
        self.target = QLabel("대상: -")
        self.target.setObjectName("muted")
        self.notice = QLabel("")
        self.notice.setWordWrap(True)
        self.notice.setStyleSheet("color:#93c5fd;")
        status_layout.addWidget(status_label)
        status_layout.addWidget(self.message)
        status_layout.addWidget(self.target)
        status_layout.addStretch()
        status_layout.addWidget(self.notice)
        body.addWidget(status_card, 1, 1)
        outer.addLayout(body, 1)

        self._build_extra_panel(outer)

        self.request_buttons = []
        if bridge.tool_request_enabled():
            requests = QHBoxLayout()
            request_label = QLabel("공구 요청")
            request_label.setObjectName("muted")
            requests.addWidget(request_label)
            for tool_name in bridge.tool_request_names:
                request_button = QPushButton(tool_name)
                request_button.setObjectName("request")
                # 기본 인자로 묶지 않으면 모든 버튼이 마지막 이름을 보낸다.
                request_button.clicked.connect(
                    lambda _checked=False, value=tool_name: bridge.request_tool(
                        value
                    )
                )
                requests.addWidget(request_button)
                self.request_buttons.append(request_button)
            requests.addStretch()
            outer.addLayout(requests)

        buttons = QHBoxLayout()
        self._message_box_type = QMessageBox
        self.bird_button = (
            QPushButton("Bird Scan") if self.SHOW_BIRD_BUTTON else None
        )
        self.start_button = QPushButton("작업 시작")
        self.home_button = QPushButton("홈 복귀")
        self.home_button.setObjectName("home")
        pause = QPushButton("일시정지")
        pause.setObjectName("pause")
        stop = QPushButton("동작 중지")
        stop.setObjectName("stop")
        if self.bird_button is not None:
            self.bird_button.setObjectName("bird")
            self.bird_button.clicked.connect(self._confirm_bird_scan)
            buttons.addWidget(self.bird_button)
        self.start_button.clicked.connect(lambda: bridge.call("start"))
        self.home_button.clicked.connect(self._confirm_go_home)
        pause.clicked.connect(lambda: bridge.call("pause"))
        stop.clicked.connect(lambda: bridge.call("stop"))
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.home_button)
        buttons.addWidget(pause)
        buttons.addWidget(stop)
        buttons.addStretch()
        warning = QLabel(
            "※ GUI 중지는 안전 인증 E-stop이 아닙니다. 비상 시 물리 E-stop을 사용하세요."
        )
        warning.setStyleSheet("color:#fca5a5;")
        buttons.addWidget(warning)
        outer.addLayout(buttons)

        self.window.setCentralWidget(root)
        self._timer = QTimer(self.window)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(33)
        self._last_sequence = None
        self._last_image_generation = -1
        self._table_item_type = QTableWidgetItem

    def _build_extra_panel(self, outer) -> None:
        """Add a flow-specific panel between the body and the button row."""

    def _refresh_extra(self, status: dict, scene: dict | None) -> None:
        """Update whatever ``_build_extra_panel`` created, once per frame."""

    def _confirm_bird_scan(self) -> None:
        box = self._message_box_type
        answer = box.question(
            self.window,
            "Bird Scan 실제 이동 확인",
            "설정된 Home → Bird 경로로 실제 로봇이 이동합니다.\n\n"
            "티칭 펜던트의 비상정지가 해제되어 있고, Home 관절 경로와 "
            "Base +Z 상승 경로에 사람·공구·장애물이 없는지 확인했습니까?",
            box.Yes | box.Cancel,
            box.Cancel,
        )
        if answer == box.Yes:
            self.bridge.call("bird_scan")

    def _confirm_go_home(self) -> None:
        box = self._message_box_type
        answer = box.question(
            self.window,
            "Home 복귀 실제 이동 확인",
            "현재 자세에서 설정된 Home 관절 자세로 실제 로봇이 이동합니다.\n\n"
            "그리퍼가 공구를 잡고 있다면 그대로 이동합니다. 티칭 펜던트의 "
            "비상정지가 해제되어 있고, 상승·Home 경로에 사람·공구·장애물이 "
            "없는지 확인했습니까?",
            box.Yes | box.Cancel,
            box.Cancel,
        )
        if answer == box.Yes:
            self.bridge.call("go_home")

    @staticmethod
    def _card(label_text: str, value_text: str):
        from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout

        frame = QFrame()
        frame.setObjectName("card")
        layout = QVBoxLayout(frame)
        label = QLabel(label_text)
        label.setObjectName("muted")
        value = QLabel(value_text)
        value.setObjectName("metric")
        layout.addWidget(label)
        layout.addWidget(value)
        return value, frame

    def refresh(self) -> None:
        from PyQt5.QtCore import Qt
        from PyQt5.QtGui import QImage, QPixmap

        (
            image,
            image_generation,
            image_stamp,
            preview_fps,
            scene,
            status,
            lan_report,
            notice,
            notice_at,
            image_at,
            overlay_max_age_s,
        ) = self.bridge.snapshot(self._last_image_generation)
        connected = image is not None and time.monotonic() - image_at < 2.0
        if image is None:
            connected = image_at > 0.0 and time.monotonic() - image_at < 2.0
        self.connection.setText(
            f"● 원본 영상 {preview_fps:.1f} FPS"
            if connected
            else "● 영상 끊김"
        )
        self.connection.setStyleSheet(
            f"color:{'#34d399' if connected else '#f87171'};"
        )

        if image is not None:
            if scene_overlay_is_fresh(
                scene,
                image_stamp,
                overlay_max_age_s,
            ):
                image = draw_scene_overlay(image, scene, lan_report)
            rgb = image[:, :, ::-1].copy()
            height, width, channels = rgb.shape
            qimage = QImage(
                rgb.data,
                width,
                height,
                channels * width,
                QImage.Format_RGB888,
            ).copy()
            pixmap = QPixmap.fromImage(qimage).scaled(
                self.video.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
            self.video.setPixmap(pixmap)
            self._last_image_generation = image_generation

        state = str(status.get("state", "OFFLINE"))
        self.state_value.setText(state)
        self.state_value.setStyleSheet(f"color:{_status_color(state)};")
        mode = str(status.get("execution_mode", "-"))
        scan_scope = str(status.get("scan_scope", "none"))
        self.mode_value.setText(
            mode if scan_scope == "none" else f"{mode} · {scan_scope}"
        )
        self.message.setText(str(status.get("message", "")))
        target = status.get("current_target") or "-"
        self.target.setText(f"대상: {target}")

        actionable = actionable_target_count(
            scene,
            getattr(self.bridge, "target_names", None),
        )
        start_service_ready = self.bridge.service_ready("start")
        bird_service_ready = self.bridge.service_ready("bird_scan")
        home_service_ready = self.bridge.service_ready("go_home")
        inputs_ready = scan_input_ready(scene, connected)
        startable_state = state in START_STATES
        work_view = live_view_is_work_view(
            scan_scope,
            bool(status.get("auto_scan_motion", False)),
        )
        if self.bird_button is not None:
            self.bird_button.setEnabled(
                bird_scan_button_enabled(
                    state,
                    bird_service_ready,
                    inputs_ready,
                )
            )
            if not bird_service_ready:
                self.bird_button.setToolTip(
                    "task manager Bird Scan 서비스 연결 대기"
                )
            elif not inputs_ready:
                self.bird_button.setToolTip(
                    "실시간 RGB와 perception scene 수신 대기"
                )
            elif state not in {"IDLE", "BIRD_READY"}:
                self.bird_button.setToolTip(
                    "Bird Scan은 IDLE/BIRD_READY에서만 가능합니다. "
                    "ERROR/SAFETY_STOP 후에는 로봇 확인 및 재시작이 필요합니다"
                )
            else:
                self.bird_button.setToolTip(
                    "Home으로 이동한 뒤 Base +Z Bird view에서 새 장면 촬영"
                )
        request_ready = tool_request_button_enabled(state)
        for request_button in self.request_buttons:
            request_button.setEnabled(request_ready)
            request_button.setToolTip(
                f"{request_button.text()} 전달 요청"
                if request_ready
                else (
                    "작업 시작 후 요청 대기(WAITING_REQUEST) 상태에서만 "
                    "선택할 수 있습니다"
                )
            )

        self.home_button.setEnabled(
            go_home_button_enabled(state, home_service_ready)
        )
        if not home_service_ready:
            self.home_button.setToolTip(
                "task manager Home 복귀 서비스 연결 대기"
            )
        elif state not in GO_HOME_STATES:
            self.home_button.setToolTip(
                "Home 복귀는 로봇이 정지한 상태에서만 가능합니다. "
                "ERROR/SAFETY_STOP 후에는 로봇 확인 및 재시작이 필요합니다"
            )
        else:
            self.home_button.setToolTip(
                "설정된 home_joint_pose 관절 자세로 복귀합니다"
            )
        self.start_button.setEnabled(
            start_button_enabled(
                state,
                start_service_ready,
                inputs_ready,
                actionable,
                work_view,
            )
        )
        mode_label = self.START_LABELS.get(mode, "작업 시작")
        self.start_button.setText(f"{mode_label} (가능 {actionable})")
        if not start_service_ready:
            self.start_button.setToolTip(
                "task manager 시작 서비스 연결 대기"
            )
        elif not inputs_ready:
            self.start_button.setToolTip(
                "실시간 RGB와 perception scene 수신 대기"
            )
        elif not startable_state:
            self.start_button.setToolTip("현재 작업이 진행 중입니다")
        elif work_view and actionable == 0:
            self.start_button.setToolTip(
                "지금 화면에 파지 가능한 공구가 없습니다. 공구를 놓거나 "
                f"{self.RESCAN_HINT}"
            )
        elif not work_view:
            self.start_button.setToolTip(
                "시작하면 Home/Bird view로 이동해 새로 촬영한 장면에서 "
                "대상을 선택합니다 (지금 화면의 검출과 다를 수 있음)"
            )
        elif mode == "pick_place":
            self.start_button.setToolTip(
                f"파지 가능한 대상 {actionable}개를 우선순위 순서로 계속 "
                "집어 옮깁니다 (대상 소진 또는 max_picks까지 반복)"
            )
        elif mode == "point":
            self.start_button.setToolTip(
                f"파지 가능한 대상 {actionable}개 중 우선순위 1개 위로 "
                "이동만 합니다 (파지 없음)"
            )
        else:
            self.start_button.setToolTip(
                f"파지 가능한 대상 {actionable}개 중 우선순위 1개의 좌표만 "
                "계산합니다. 실제로 집으려면 execution_mode=pick_place가 "
                "필요합니다"
            )

        if scene is not None:
            sequence = scene.get("sequence")
            detections = scene.get("detections", [])
            self.fps_value.setText(f"{scene.get('fps', 0):.1f} FPS")
            self.count_value.setText(
                f"{len(detections)} / {status.get('completed', 0)}"
            )
            if sequence != self._last_sequence:
                self._update_table(detections)
                self._last_sequence = sequence

        self.notice.setText(
            notice if time.monotonic() - notice_at < 8.0 else ""
        )
        self._refresh_extra(status, scene)

    def _update_table(self, detections: list[dict]) -> None:
        self.table.setRowCount(len(detections))
        for row, detection in enumerate(detections):
            depth = detection.get("depth_mm")
            values = [
                str(detection.get("rank", "-")),
                str(detection.get("name", "")),
                f"{float(detection.get('confidence', 0.0)):.2f}",
                f"{depth:.0f} mm" if depth is not None else "보류",
                str(
                    detection.get("part_name")
                    or f"{detection.get('expected_part_name') or '-'} 누락"
                ),
                str(len(detection.get("overlaps_with", []))),
                (
                    "가능"
                    if detection.get("grasp_ready")
                    and detection.get("grasp_depth_mm") is not None
                    else "파지부/Depth 부족"
                ),
            ]
            for column, value in enumerate(values):
                item = self._table_item_type(value)
                self.table.setItem(row, column, item)
        self.table.resizeColumnsToContents()


def run_dashboard(window_type=None, args=None) -> None:
    """Run one dashboard window against a fresh ROS bridge.

    The autonomous and handover packages call this with their own window
    subclass so the Qt/ROS lifecycle stays defined in exactly one place.
    """

    _sanitize_qt_environment()

    import rclpy
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication
    from rclpy.executors import MultiThreadedExecutor

    # Qt owns the main thread, so construct the application before ROS workers.
    application = QApplication([])
    rclpy.init(args=args)
    bridge = DashboardRosBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge.node)
    spin_thread = threading.Thread(
        target=executor.spin, name="dashboard-ros-executor", daemon=True
    )
    spin_thread.start()

    dashboard = (window_type or DashboardWindow)(bridge)
    dashboard.window.show()
    shutdown_timer = QTimer(dashboard.window)
    shutdown_timer.timeout.connect(
        lambda: application.quit() if not rclpy.ok() else None
    )
    shutdown_timer.start(100)
    try:
        application.exec_()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        bridge.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    run_dashboard(DashboardWindow, args=args)


if __name__ == "__main__":
    main()
