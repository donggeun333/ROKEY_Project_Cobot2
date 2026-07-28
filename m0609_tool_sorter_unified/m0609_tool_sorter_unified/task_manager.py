"""공구 정리/전달 공통 로봇 작업기.

autonomous/handover 두 패키지가 공통으로 쓰는 핵심 런타임이다. perception에서
계산한 Scene를 받아, 스캔 자세 이동, 파지 후보 선택, RG2 제어, 힘 기반 도착/
전달 판정을 일관된 규칙으로 수행한다.
"""

from __future__ import annotations

import math
import threading
import time

from .geometry import orient_with_yaw
from .grasp_selector import SORTABLE_TOOL_NAMES, is_grasp_ready
from .motion import (
    DoosanMotion,
    ForceLimitExceeded,
    MotionCancelled,
    MotionError,
    OnRobotGripper,
    RG2_MAX_WIDTH_MM,
    ServiceCallError,
    TargetUnreachable,
)
from .scan_strategy import (
    bird_pose_from_home_tcp,
    camera_centered_tcp_pose,
)
from .schema import Detection, Scene, TaskStatus


class SceneTransformUnavailable(RuntimeError):
    pass


BIRD_SCAN_REQUEST_STATES = {"IDLE", "BIRD_READY"}
GO_HOME_REQUEST_STATES = {
    "IDLE",
    "BIRD_READY",
    "BLOCKED",
    "COMPLETE",
    "PAUSED",
    "STOPPED",
}


def bird_scan_request_allowed(state: str, active: bool) -> bool:
    """Allow a manual Bird scan only from a known stationary state."""

    return not active and state in BIRD_SCAN_REQUEST_STATES


def go_home_request_allowed(state: str, active: bool) -> bool:
    """Allow a manual Home return only while no other command is running."""

    return not active and state in GO_HOME_REQUEST_STATES


def apply_grasp_z_offset(
    raw_z_mm: float,
    offset_mm: float,
    table_z_mm: float,
    clearance_mm: float,
) -> float:
    """Apply a measured Base-Z bias without crossing the table clearance."""

    values = [raw_z_mm, offset_mm, table_z_mm, clearance_mm]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("grasp Z values must be finite")
    if float(clearance_mm) < 0.0:
        raise ValueError("minimum_table_clearance_mm cannot be negative")
    return max(
        float(raw_z_mm) + float(offset_mm),
        float(table_z_mm) + float(clearance_mm),
    )


def _handle_long_axis_mm(detection: Detection, mm_per_px: float) -> float:
    """Return the grasp rect's long edge in millimetres, 0.0 when unknown."""

    box = detection.axis_box
    if not box or len(box) != 4:
        return 0.0
    edges = [
        math.dist(
            [float(box[index][0]), float(box[index][1])],
            [
                float(box[(index + 1) % 4][0]),
                float(box[(index + 1) % 4][1]),
            ],
        )
        for index in range(4)
    ]
    return max(edges) * float(mm_per_px)


def select_axial_grasp_offset(
    *,
    handle_center_px: list[float],
    tool_center_px: list[float],
    axis_px: list[float],
    axis_base_xy: list[float],
    grasp_xy_mm: list[float],
    offset_mm: float,
    max_offset_mm: float,
) -> float:
    """Return how far to slide the grasp point along the +axis direction.

    MoveJointx sends the current IK solution space with the target, so a pose
    that branch cannot solve is rejected outright (controller alarm 1206,
    "Pose(...) is NOT REACHABLE"). A tool lying at the far edge of the toolbox
    produces exactly that. Sliding the grasp point down the handle toward the
    robot keeps the transfer inside the branch that already holds the
    observation pose.

    Sliding is only safe in one direction, though: toward the free end of the
    handle. The other way lies the head, and the vision grip width was measured
    across the handle, not the head. So the slide happens only when the free
    end is *also* the side nearer the robot. When it is not, the handle centre
    is kept and the caller is left to fail loudly rather than close the gripper
    on a hammer head.

    The return value is signed along ``+axis_base_xy``; ``0.0`` means keep the
    handle centre.
    """

    offset = min(abs(float(offset_mm)), abs(float(max_offset_mm)))
    if not math.isfinite(offset) or offset <= 0.0:
        return 0.0

    # 어느 쪽이 머리인가: 공구 마스크 중심에서 손잡이 마스크 중심으로 가는
    # 방향이 머리 반대쪽이다. 머리가 부피를 차지해 공구 중심을 자기 쪽으로
    # 끌어당기기 때문이다.
    away_u = float(handle_center_px[0]) - float(tool_center_px[0])
    away_v = float(handle_center_px[1]) - float(tool_center_px[1])
    projection = float(axis_px[0]) * away_u + float(axis_px[1]) * away_v
    if not math.isfinite(projection) or projection == 0.0:
        # 두 중심이 겹치거나 축이 그 방향과 직교하면 머리 쪽을 정할 근거가
        # 없다. 찍어서 반대로 가느니 중앙을 유지한다.
        return 0.0
    sign = 1.0 if projection > 0.0 else -1.0

    axis_x = float(axis_base_xy[0])
    axis_y = float(axis_base_xy[1])
    length = math.hypot(axis_x, axis_y)
    if not math.isfinite(length) or length <= 0.0:
        return 0.0

    grasp_x = float(grasp_xy_mm[0])
    grasp_y = float(grasp_xy_mm[1])
    if not (math.isfinite(grasp_x) and math.isfinite(grasp_y)):
        return 0.0
    shifted_x = grasp_x + sign * axis_x / length * offset
    shifted_y = grasp_y + sign * axis_y / length * offset
    if math.hypot(shifted_x, shifted_y) >= math.hypot(grasp_x, grasp_y):
        # 자유단이 로봇에서 멀어지는 쪽이다. 옮기면 도달 문제가 나빠진다.
        return 0.0
    return sign * offset


def select_gripper_target_width(
    measured_width_mm: float | None,
    *,
    preopen_width_mm: float,
    margin_mm: float,
) -> tuple[float | None, str]:
    """Return a vision target, or reject a width before robot approach."""

    preopen = float(preopen_width_mm)
    margin = float(margin_mm)
    if (
        not math.isfinite(preopen)
        or preopen <= 0.0
        or preopen > RG2_MAX_WIDTH_MM
    ):
        raise ValueError(
            f"gripper_preopen_width_mm must be in "
            f"(0, {RG2_MAX_WIDTH_MM:g}]"
        )
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("gripper_width_margin_mm cannot be negative")

    if measured_width_mm is not None:
        try:
            measured = float(measured_width_mm)
        except (TypeError, ValueError):
            measured = math.nan
        if math.isfinite(measured) and 0.0 < measured <= preopen:
            target = max(0.0, min(preopen, measured - margin))
            return target, "vision"
    return None, "invalid"


class ToolSortTaskManager:
    def __init__(self):
        import tf2_ros
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.node = Node("tool_sorter_task_manager")
        self._declare_parameters()
        self.execution_mode = str(self._parameter("execution_mode"))
        if self.execution_mode not in {"inspect", "point", "pick_place"}:
            raise ValueError(
                "execution_mode must be inspect, point, or pick_place"
            )

        self._scene_lock = threading.Condition()
        self._latest_scene: Scene | None = None
        self._scene_received_at = 0.0
        self._camera_info = None
        self._active = threading.Event()
        self._shutdown = threading.Event()
        self._stop_requested = threading.Event()
        self._command_lock = threading.Lock()
        self._pending_command: str | None = None
        self._bird_pose: list[float] | None = None
        # 마지막으로 Home에 섰을 때의 TCP. 공구함 이송의 안전 높이로 쓴다.
        self._home_tcp: list[float] | None = None
        self._observation_orientation: list[float] | None = None
        self._scan_scope = "none"
        self._local_focus_xy: tuple[float, float] | None = None
        self._scan_ready_stamp_ns = 0
        self._completed = 0

        self._tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(
                seconds=float(self._parameter("tf_cache_s"))
            )
        )
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self.node,
        )
        from .transform.tf_projector import (
            TimestampedRgbdProjector,
        )

        self._projector = TimestampedRgbdProjector(
            tf_buffer=self._tf_buffer,
            base_frame=str(self._parameter("base_frame")),
            projection_frame=str(self._parameter("projection_frame")),
            tf_timeout_sec=float(self._parameter("tf_timeout_s")),
        )

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.node.create_publisher(
            String, "/tool_sorter/task/status", status_qos
        )
        self.node.create_subscription(
            String,
            "/tool_sorter/perception/scene",
            self._on_scene,
            1,
        )
        self.node.create_subscription(
            CameraInfo,
            "/tool_sorter/perception/camera_info",
            self._on_camera_info,
            1,
        )
        self.node.create_service(
            Trigger, "/tool_sorter/start", self._on_start
        )
        self.node.create_service(
            Trigger, "/tool_sorter/pause", self._on_pause
        )
        self.node.create_service(
            Trigger, "/tool_sorter/stop", self._on_stop
        )
        self.node.create_service(
            Trigger, "/tool_sorter/bird_scan", self._on_bird_scan
        )
        self.node.create_service(
            Trigger, "/tool_sorter/go_home", self._on_go_home
        )

        self.motion = DoosanMotion(
            self.node,
            namespace=str(self._parameter("robot_namespace")),
            tcp_name=str(self._parameter("tcp_name")),
            tool_name=str(self._parameter("tool_name")),
            force_limit_n=float(self._parameter("force_limit_n")),
            force_poll_hz=float(self._parameter("force_poll_hz")),
            force_debounce_samples=int(
                self._parameter("force_debounce_samples")
            ),
            position_tolerance_mm=float(
                self._parameter("motion_position_tolerance_mm")
            ),
        )
        self.gripper = OnRobotGripper(
            self.node,
            service_name=str(self._parameter("gripper_service")),
            settle_s=float(self._parameter("gripper_settle_s")),
        )

        self._status = TaskStatus(execution_mode=self.execution_mode)
        self._last_logged_status: tuple[str, str] | None = None
        self._publish_status("IDLE", "시작 명령 대기")
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="tool-sorter-task-worker",
            daemon=True,
        )
        self._worker.start()

    def _declare_parameters(self) -> None:
        defaults = {
            "execution_mode": "inspect",
            "robot_namespace": "dsr01",
            "tcp_name": "GripperDA_v1",
            "tool_name": "Tool Weight",
            "enforce_tool_preset": False,
            "gripper_service": "/onrobot/sendCommand",
            "gripper_preopen_width_mm": 50.0,
            "gripper_width_margin_mm": 5.0,
            "base_frame": "base_link",
            "tcp_frame": "GripperDA_v1",
            "projection_frame": "calibrated_camera_color_optical_frame",
            "tf_cache_s": 20.0,
            "tf_timeout_s": 0.3,
            "table_z_mm": -0.6,
            "grasp_depth_ratio": 0.5,
            "grasp_z_offset_mm": -15.0,
            # 손잡이 축을 따라 로봇 쪽으로 파지점을 당기는 양. 0이면 손잡이
            # 중앙을 그대로 쓴다(기본). 자유단이 로봇 쪽일 때만 적용되고,
            # 손잡이 장축의 1/4을 넘지 않도록 잘린다.
            "grasp_axial_offset_mm": 0.0,
            "finger_offset_deg": 0.0,
            "hover_mm": 80.0,
            "lift_mm": 150.0,
            "minimum_table_clearance_mm": 2.0,
            "travel_velocity": 60.0,
            "travel_acceleration": 120.0,
            "approach_velocity": 20.0,
            "approach_acceleration": 40.0,
            "force_limit_n": 30.0,
            "force_poll_hz": 20.0,
            "force_debounce_samples": 2,
            # 이송 구간(안전 높이 수평·수직 이동) 충돌 감시. 정지 상태
            # 기준값 대비 상승분이며, 0 이하로 두면 감시를 끈다.
            # MoveJointx는 끝점만 고정하고 팔꿈치·손목이 지나는 부피는
            # 직선이 아니므로, 그 구간을 지키는 유일한 소프트웨어 감시다.
            "transfer_force_rise_n": 20.0,
            "transfer_force_baseline_samples": 3,
            "motion_position_tolerance_mm": 3.0,
            "gripper_settle_s": 1.3,
            "scene_max_age_s": 1.0,
            "camera_settle_s": 0.7,
            "service_wait_s": 12.0,
            "empty_scene_confirmations": 3,
            "local_empty_confirmations": 3,
            "max_picks": 20,
            "target_names": list(SORTABLE_TOOL_NAMES),
            "place_pose": [],
            "place_offset_xy_mm": [150.0, 0.0],
            # Physical scan motion is opt-in because the correct heights are
            # cell-specific and must never be guessed by software.
            "auto_scan_motion": False,
            "home_joint_pose": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
            "home_joint_velocity": 20.0,
            "home_joint_acceleration": 40.0,
            "home_motion_timeout_s": 45.0,
            "safe_jointx_velocity": 20.0,
            "safe_jointx_acceleration": 40.0,
            "home_direct_pick_enabled": True,
            "bird_raise_mm": 200.0,
            "bird_max_tilt_deg": 15.0,
            "local_rescan_enabled": True,
            "local_scan_standoff_mm": 0.0,
            "local_scan_radius_mm": 150.0,
        }
        for name, value in defaults.items():
            self.node.declare_parameter(name, value)

    def _parameter(self, name: str):
        return self.node.get_parameter(name).value

    def _list_parameter(self, name: str) -> list:
        """Return a ROS array parameter, normalizing every empty representation.

        ROS 2 Humble may keep a YAML ``[]`` parameter in the declared-but-not-
        initialized state. Accessing ``Parameter.value`` then raises instead of
        returning ``None``. Empty arrays are intentional for parameters such as
        ``place_pose``, so normalize both forms to an empty Python list.
        """

        from rclpy.exceptions import ParameterUninitializedException

        try:
            value = self._parameter(name)
        except ParameterUninitializedException:
            return []
        return [] if value is None else list(value)

    def _on_scene(self, message) -> None:
        try:
            scene = Scene.from_json(message.data)
        except Exception as error:
            self.node.get_logger().warning(f"Invalid scene message: {error}")
            return
        with self._scene_lock:
            self._latest_scene = scene
            self._scene_received_at = time.monotonic()
            self._scene_lock.notify_all()

    def _on_camera_info(self, message) -> None:
        if len(message.k) != 9 or float(message.k[0]) <= 0.0:
            self.node.get_logger().warning("Invalid CameraInfo received")
            return
        first = self._camera_info is None
        self._camera_info = message
        if first:
            self.node.get_logger().info(
                "Camera geometry ready; capture-time TF projection enabled"
            )

    def _on_start(self, _request, response):
        if self._status.state in {"ERROR", "SAFETY_STOP"}:
            response.success = False
            response.message = (
                "오류/안전 정지 후에는 로봇 상태를 확인하고 "
                "task manager를 재시작해야 합니다"
            )
            return response
        try:
            problem = self._validate_start()
        except Exception as error:
            problem = f"시작 설정 확인 실패: {error}"
            self.node.get_logger().error(problem)
        if problem:
            response.success = False
            response.message = problem
            self._publish_status("BLOCKED", problem)
            return response
        self._stop_requested.clear()
        self.motion.clear_stop()
        if not self._queue_command("session"):
            response.success = False
            response.message = "다른 작업 또는 Bird Scan이 진행 중입니다"
            return response
        with self._scene_lock:
            self._scene_lock.notify_all()
        response.success = True
        response.message = f"{self.execution_mode} mode started"
        self._publish_status("STARTING", "작업 시작")
        return response

    def _on_bird_scan(self, _request, response):
        state = self._status.state
        if not bird_scan_request_allowed(state, self._active.is_set()):
            response.success = False
            if state in {"ERROR", "SAFETY_STOP"}:
                response.message = (
                    "오류/안전 정지 후에는 로봇 상태를 확인하고 "
                    "task manager를 재시작해야 합니다"
                )
            else:
                response.message = (
                    f"현재 상태({state})에서는 Bird Scan을 시작할 수 없습니다"
                )
            return response
        try:
            problem = self._validate_bird_scan()
        except Exception as error:
            problem = f"Bird Scan 설정 확인 실패: {error}"
            self.node.get_logger().error(problem)
        if problem:
            response.success = False
            response.message = problem
            self._publish_status("BLOCKED", problem)
            return response
        self._stop_requested.clear()
        self.motion.clear_stop()
        if not self._queue_command("bird_scan"):
            response.success = False
            response.message = "다른 작업 또는 Bird Scan이 진행 중입니다"
            return response
        with self._scene_lock:
            self._scene_lock.notify_all()
        response.success = True
        response.message = "Bird Scan started"
        self._publish_status(
            "BIRD_SCAN_STARTING",
            "독립 Bird Scan 시작",
            target="",
        )
        return response

    def _on_go_home(self, _request, response):
        state = self._status.state
        if not go_home_request_allowed(state, self._active.is_set()):
            response.success = False
            if state in {"ERROR", "SAFETY_STOP"}:
                response.message = (
                    "오류/안전 정지 후에는 로봇 상태를 확인하고 "
                    "task manager를 재시작해야 합니다"
                )
            else:
                response.message = (
                    f"현재 상태({state})에서는 Home 복귀를 시작할 수 없습니다"
                )
            return response
        try:
            problem = self._validate_go_home()
        except Exception as error:
            problem = f"Home 복귀 설정 확인 실패: {error}"
            self.node.get_logger().error(problem)
        if problem:
            response.success = False
            response.message = problem
            self._publish_status("BLOCKED", problem)
            return response
        self._stop_requested.clear()
        self.motion.clear_stop()
        if not self._queue_command("go_home"):
            response.success = False
            response.message = "다른 작업 또는 Bird Scan이 진행 중입니다"
            return response
        with self._scene_lock:
            self._scene_lock.notify_all()
        response.success = True
        response.message = "go home started"
        self._publish_status(
            "HOMING",
            "사용자 요청 Home 복귀 시작",
            target="",
        )
        return response

    def _on_pause(self, _request, response):
        self._clear_pending_command()
        self._active.clear()
        self.motion.request_stop(quick=False)
        response.success = True
        response.message = "pause requested"
        self._publish_status("PAUSED", "일시정지 요청됨")
        return response

    def _on_stop(self, _request, response):
        self._clear_pending_command()
        self._active.clear()
        self._stop_requested.set()
        self.motion.request_stop(quick=True)
        response.success = True
        response.message = "stop requested"
        self._publish_status("STOPPED", "사용자 중지 요청")
        return response

    def _queue_command(self, command: str) -> bool:
        with self._command_lock:
            if self._active.is_set() or self._pending_command is not None:
                return False
            self._pending_command = command
            self._active.set()
            return True

    def _take_pending_command(self) -> str | None:
        with self._command_lock:
            command = self._pending_command
            self._pending_command = None
            return command

    def _clear_pending_command(self) -> None:
        with self._command_lock:
            self._pending_command = None

    def _validate_start(self) -> str | None:
        if self._camera_info is None:
            return "camera_info가 아직 준비되지 않았습니다"
        if int(self._parameter("empty_scene_confirmations")) < 1:
            return "empty_scene_confirmations는 1 이상이어야 합니다"
        if int(self._parameter("local_empty_confirmations")) < 1:
            return "local_empty_confirmations는 1 이상이어야 합니다"
        if int(self._parameter("max_picks")) < 1:
            return "max_picks는 1 이상이어야 합니다"
        if float(self._parameter("camera_settle_s")) < 0.0:
            return "camera_settle_s는 음수일 수 없습니다"
        grasp_z_offset = float(self._parameter("grasp_z_offset_mm"))
        if not math.isfinite(grasp_z_offset):
            return "grasp_z_offset_mm에 NaN 또는 Inf를 사용할 수 없습니다"
        transfer_rise = float(self._parameter("transfer_force_rise_n"))
        if not math.isfinite(transfer_rise):
            return "transfer_force_rise_n에 NaN 또는 Inf를 사용할 수 없습니다"
        if transfer_rise > 0.0 and transfer_rise >= float(
            self._parameter("force_limit_n")
        ):
            # 이송 감시는 기준값 대비 상승분이므로 절대 중단 기준보다 먼저
            # 걸려야 한다. 크게 잡으면 이송 감시가 사실상 죽는다.
            return (
                f"transfer_force_rise_n({transfer_rise:.1f}N)은 "
                f"force_limit_n({float(self._parameter('force_limit_n')):.1f}N)"
                "보다 작아야 합니다"
            )
        if int(self._parameter("transfer_force_baseline_samples")) < 1:
            return "transfer_force_baseline_samples는 1 이상이어야 합니다"
        if self.execution_mode == "pick_place":
            try:
                select_gripper_target_width(
                    None,
                    preopen_width_mm=float(
                        self._parameter("gripper_preopen_width_mm")
                    ),
                    margin_mm=float(
                        self._parameter("gripper_width_margin_mm")
                    ),
                )
            except (TypeError, ValueError) as error:
                return f"그리퍼 폭 설정 오류: {error}"
            place_pose = self._list_parameter("place_pose")
            if len(place_pose) not in {0, 6}:
                return (
                    "place_pose는 비워두거나 X Y Z RX RY RZ 6값이어야 합니다"
                )
            if not all(math.isfinite(float(value)) for value in place_pose):
                return "place_pose에 NaN 또는 Inf를 사용할 수 없습니다"
            place_offset = self._list_parameter("place_offset_xy_mm")
            if len(place_offset) != 2:
                return "place_offset_xy_mm는 X/Y 2값이어야 합니다"
            if not all(
                math.isfinite(float(value)) for value in place_offset
            ):
                return "place_offset_xy_mm에 NaN 또는 Inf를 사용할 수 없습니다"
        if bool(self._parameter("auto_scan_motion")):
            home = self._list_parameter("home_joint_pose")
            if len(home) != 6:
                return "home_joint_pose는 J1~J6 6값이어야 합니다"
            if not all(math.isfinite(float(value)) for value in home):
                return "home_joint_pose에 NaN 또는 Inf를 사용할 수 없습니다"
            for name in (
                "safe_jointx_velocity",
                "safe_jointx_acceleration",
            ):
                value = float(self._parameter(name))
                if not math.isfinite(value) or value <= 0.0:
                    return f"{name}는 0보다 커야 합니다"
            bird_raise = float(self._parameter("bird_raise_mm"))
            if not math.isfinite(bird_raise) or bird_raise <= 0.0:
                return (
                    "auto_scan_motion을 사용하려면 실제 셀에서 검증한 "
                    "bird_raise_mm를 0보다 크게 설정해야 합니다"
                )
            max_tilt = float(self._parameter("bird_max_tilt_deg"))
            if (
                not math.isfinite(max_tilt)
                or max_tilt < 0.0
                or max_tilt >= 90.0
            ):
                return "bird_max_tilt_deg는 0 이상 90 미만이어야 합니다"
            local_enabled = (
                self.execution_mode == "pick_place"
                and bool(self._parameter("local_rescan_enabled"))
            )
            if local_enabled:
                local_standoff = float(
                    self._parameter("local_scan_standoff_mm")
                )
                if (
                    not math.isfinite(local_standoff)
                    or local_standoff <= 0.0
                ):
                    return (
                        "local_rescan_enabled를 사용하려면 실제 셀에서 검증한 "
                        "local_scan_standoff_mm를 0보다 크게 설정해야 합니다"
                    )
                local_radius = float(
                    self._parameter("local_scan_radius_mm")
                )
                if not math.isfinite(local_radius) or local_radius <= 0.0:
                    return "local_scan_radius_mm는 0보다 커야 합니다"
        return None

    def _validate_bird_scan(self) -> str | None:
        if self._camera_info is None:
            return "camera_info가 아직 준비되지 않았습니다"
        home = self._list_parameter("home_joint_pose")
        if len(home) != 6:
            return "home_joint_pose는 J1~J6 6값이어야 합니다"
        if not all(math.isfinite(float(value)) for value in home):
            return "home_joint_pose에 NaN 또는 Inf를 사용할 수 없습니다"
        bird_raise = float(self._parameter("bird_raise_mm"))
        if not math.isfinite(bird_raise) or bird_raise <= 0.0:
            return "bird_raise_mm는 현장에서 검증한 0보다 큰 값이어야 합니다"
        max_tilt = float(self._parameter("bird_max_tilt_deg"))
        if (
            not math.isfinite(max_tilt)
            or max_tilt < 0.0
            or max_tilt >= 90.0
        ):
            return "bird_max_tilt_deg는 0 이상 90 미만이어야 합니다"
        if float(self._parameter("camera_settle_s")) < 0.0:
            return "camera_settle_s는 음수일 수 없습니다"
        return None

    def _validate_go_home(self) -> str | None:
        home = self._list_parameter("home_joint_pose")
        if len(home) != 6:
            return "home_joint_pose는 J1~J6 6값이어야 합니다"
        if not all(math.isfinite(float(value)) for value in home):
            return "home_joint_pose에 NaN 또는 Inf를 사용할 수 없습니다"
        for name in (
            "home_joint_velocity",
            "home_joint_acceleration",
            "home_motion_timeout_s",
        ):
            value = float(self._parameter(name))
            if not math.isfinite(value) or value <= 0.0:
                return f"{name}는 0보다 커야 합니다"
        return None

    def _publish_status(
        self,
        state: str,
        message: str,
        target: str | None = None,
        scene_sequence: int | None = None,
    ) -> None:
        from std_msgs.msg import String

        self._status.state = state
        self._status.message = message
        self._status.completed = self._completed
        self._status.updated_at = time.time()
        self._status.scan_scope = self._scan_scope
        self._status.auto_scan_motion = bool(
            self._parameter("auto_scan_motion")
        )
        if target is not None:
            self._status.current_target = target
        if scene_sequence is not None:
            self._status.scene_sequence = scene_sequence
        output = String()
        output.data = self._status.to_json()
        self._status_publisher.publish(output)
        self._log_status_change(state, message)

    def _log_status_change(self, state: str, message: str) -> None:
        """Mirror the status topic to the console, once per change.

        Without a GUI the status topic is the only place the operator sees an
        instruction such as "공구를 잡아당기세요", and echoing a topic needs a
        second terminal. Scanning states republish on every fresh scene, so log
        only when the (state, message) pair actually changes; that still catches
        a message that changes while the state holds.
        """

        current = (state, message)
        if current == self._last_logged_status:
            return
        self._last_logged_status = current
        logger = self.node.get_logger()
        line = f"[{state}] {message}"
        if state in {"ERROR", "SAFETY_STOP"}:
            logger.error(line)
        elif state in {"BLOCKED", "PAUSED", "STOPPED"}:
            logger.warning(line)
        else:
            logger.info(line)

    def _worker_loop(self) -> None:
        motion_services_ready = False
        gripper_service_ready = False
        while not self._shutdown.is_set():
            if not self._active.wait(timeout=0.2):
                continue
            command = self._take_pending_command()
            if command is None:
                continue
            try:
                if not motion_services_ready:
                    self._publish_status("STARTING", "로봇 서비스를 확인하는 중")
                    self.motion.wait_ready(
                        float(self._parameter("service_wait_s"))
                    )
                    self.motion.configure_presets(
                        enforce_tool_preset=bool(
                            self._parameter("enforce_tool_preset")
                        )
                    )
                    self.node.get_logger().info(
                        "Controller presets selected and verified: "
                        f"TCP={self._parameter('tcp_name')}, "
                        f"Tool={self._parameter('tool_name')}"
                    )
                    motion_services_ready = True
                if command == "bird_scan":
                    self._run_bird_scan()
                    self._active.clear()
                    continue
                if command == "go_home":
                    self._run_go_home()
                    self._active.clear()
                    continue
                if (
                    self.execution_mode == "pick_place"
                    and not gripper_service_ready
                ):
                    self.gripper.wait_ready(
                        float(self._parameter("service_wait_s"))
                    )
                    gripper_service_ready = True
                self._run_session()
            except MotionCancelled:
                state = (
                    "PAUSED"
                    if not self._stop_requested.is_set()
                    else "STOPPED"
                )
                self._publish_status(
                    state,
                    "이동이 중지되었습니다. 로봇과 파지 상태를 직접 확인하세요",
                )
                self._active.clear()
            except ForceLimitExceeded as error:
                self._publish_status(
                    "SAFETY_STOP",
                    f"{error}. 자동 복귀하지 않습니다; 현장을 확인하세요",
                )
                self._active.clear()
            except TargetUnreachable as error:
                # 팔이 아예 안 움직인 경우라 위험 상태가 아니다. 현장에서
                # 바로 할 수 있는 조치를 말해주는 편이 낫다.
                self.node.get_logger().warning(str(error))
                self._publish_status(
                    "BLOCKED",
                    "공구가 팔이 닿는 범위 밖에 있습니다. 공구를 로봇 쪽으로 "
                    "옮긴 뒤 다시 시도하세요",
                )
                self._active.clear()
            except (MotionError, ServiceCallError, ValueError) as error:
                # _publish_status가 ERROR를 error 레벨로 찍으므로 같은 문구를
                # 따로 로그하지 않는다.
                self._publish_status("ERROR", str(error))
                self._active.clear()
            except Exception as error:
                import traceback

                self.node.get_logger().error(
                    f"Unexpected task manager error: {error}\n"
                    f"{traceback.format_exc()}"
                )
                self._publish_status("ERROR", str(error))
                self._active.clear()

    def _run_bird_scan(self) -> None:
        self.motion.verify_presets(
            enforce_tool_preset=bool(
                self._parameter("enforce_tool_preset")
            )
        )
        last_sequence = self._move_home_to_bird(
            "독립 Bird Scan을 위해 Home으로 이동"
        )
        scene = self._wait_for_fresh_scene(last_sequence)
        self._publish_status(
            "BIRD_READY",
            "Bird view 이동 및 새 장면 촬영 완료; 작업 시작 대기",
            target="",
            scene_sequence=scene.sequence,
        )

    def _run_go_home(self) -> None:
        self.motion.verify_presets(
            enforce_tool_preset=bool(
                self._parameter("enforce_tool_preset")
            )
        )
        self._move_to_home("사용자 요청으로 Home 자세 복귀")
        # Home 복귀 후에는 이전 Bird/로컬 관측 기준이 더 이상 유효하지 않습니다.
        self._bird_pose = None
        self._observation_orientation = None
        self._scan_scope = "none"
        self._local_focus_xy = None
        self._publish_status(
            "IDLE",
            "Home 복귀 완료; 시작 명령 대기",
            target="",
        )

    def _run_session(self) -> None:
        self.motion.verify_presets(
            enforce_tool_preset=bool(
                self._parameter("enforce_tool_preset")
            )
        )
        if bool(self._parameter("auto_scan_motion")):
            if bool(self._parameter("home_direct_pick_enabled")):
                last_sequence = self._move_home_for_initial_scan()
            else:
                last_sequence = self._move_home_to_bird(
                    "최초 전체 장면을 촬영하기 위해 Home으로 이동"
                )
        else:
            self._bird_pose = self.motion.current_pose()
            self._observation_orientation = self._bird_pose[3:]
            self._scan_scope = "global"
            self._local_focus_xy = None
            self.node.get_logger().warning(
                "auto_scan_motion=false: 현재 TCP 자세를 고정 관측 자세로 사용"
            )
            last_sequence = self._arm_fresh_scan(
                "현재 자세에서 새 전체 장면을 기다리는 중"
            )

        empty_count = 0
        while self._active.is_set() and not self._shutdown.is_set():
            scene = self._wait_for_fresh_scene(last_sequence)
            last_sequence = scene.sequence
            try:
                selection = self._select_target_and_grasp(scene)
            except SceneTransformUnavailable as error:
                self._publish_status(
                    "WAITING_TRANSFORM",
                    str(error),
                    scene_sequence=scene.sequence,
                )
                continue

            if selection is None:
                empty_count += 1
                if not self._empty_scene_confirmed(
                    empty_count,
                    scene.sequence,
                ):
                    continue
                if self._scan_scope == "local":
                    empty_count = 0
                    last_sequence = self._move_home_to_bird(
                        "로컬 재검출 실패; Home을 거쳐 Bird view로 복귀"
                    )
                    continue
                elif self._scan_scope == "home":
                    empty_count = 0
                    last_sequence = self._move_home_to_bird(
                        "Home에서 작업 가능한 공구가 없어 Bird view로 상승",
                        already_home=True,
                    )
                    continue
                else:
                    self._publish_status("COMPLETE", "정리할 공구가 없습니다")
                    self._active.clear()
                    return
            empty_count = 0
            target, grasp = selection

            summary = (
                f"{target.name}: X={grasp['x_mm']:.1f}, "
                f"Y={grasp['y_mm']:.1f}, Z={grasp['z_mm']:.1f}, "
                f"yaw={grasp['yaw_deg']:.1f} ({self._scan_scope})"
            )
            self._publish_status(
                "PLANNED",
                summary,
                target=target.name,
                scene_sequence=scene.sequence,
            )

            if self.execution_mode == "inspect":
                self._publish_status(
                    "COMPLETE",
                    "좌표 계산 완료; GUI에서 확인하세요",
                    target=target.name,
                    scene_sequence=scene.sequence,
                )
                self._active.clear()
                return
            if self.execution_mode == "point":
                self._execute_point(target, grasp)
                self._publish_status(
                    "COMPLETE",
                    "포인팅 이동 완료; 위치와 각도를 확인하세요",
                    target=target.name,
                    scene_sequence=scene.sequence,
                )
                self._active.clear()
                return

            self._execute_pick_place(target, grasp)
            self._completed += 1
            if self._completed >= int(self._parameter("max_picks")):
                self._publish_status(
                    "COMPLETE", "max_picks에 도달해 종료했습니다"
                )
                self._active.clear()
                return

            if (
                bool(self._parameter("auto_scan_motion"))
                and bool(self._parameter("local_rescan_enabled"))
            ):
                last_sequence = self._move_to_local_scan(grasp)
            elif bool(self._parameter("auto_scan_motion")):
                if bool(self._parameter("home_direct_pick_enabled")):
                    # 배치를 마친 팔은 이미 Home을 거쳐 나왔다. 여기서 Bird
                    # 높이까지 bird_raise_mm를 더 올렸다가 다음 공구를 집으러
                    # 도로 내려오면 왕복이 통째로 낭비다. Home 장면에서 바로
                    # 집을 수 있으면 집고, 비었을 때만 Bird로 올라간다 —
                    # 그 분기는 위쪽 _scan_scope == "home" 처리에 이미 있다.
                    last_sequence = self._move_home_for_initial_scan()
                else:
                    last_sequence = self._move_home_to_bird(
                        "로컬 재검출 비활성화; 다음 전체 장면 촬영"
                    )
            else:
                if self._bird_pose is None:
                    raise ValueError("observation pose is not set")
                self._travel(
                    self._bird_pose,
                    "RETURNING",
                    "고정 관측 자세로 복귀",
                )
                self._scan_scope = "global"
                self._local_focus_xy = None
                last_sequence = self._arm_fresh_scan(
                    "새 전체 장면을 기다리는 중"
                )

    def _empty_scene_confirmed(
        self,
        empty_count: int,
        scene_sequence: int,
    ) -> bool:
        """Report empty-scene progress and decide whether the scope is done.

        Subclasses may replace the frame counter with another rule, so this
        hook both publishes the progress message and returns the decision.
        """

        limit_name = (
            "local_empty_confirmations"
            if self._scan_scope == "local"
            else "empty_scene_confirmations"
        )
        limit = int(self._parameter(limit_name))
        self._publish_status(
            "WAITING_SCENE",
            f"{self._scan_scope} 대상 없음 확인 {empty_count}/{limit}",
            target="",
            scene_sequence=scene_sequence,
        )
        return empty_count >= limit

    def _arm_fresh_scan(self, message: str) -> int:
        """Settle, discard all prior frames, then require a newer capture."""

        self._publish_status("SETTLING", message)
        deadline = time.monotonic() + float(
            self._parameter("camera_settle_s")
        )
        while time.monotonic() < deadline:
            if not self._active.is_set() or self._shutdown.is_set():
                raise MotionCancelled("session is no longer active")
            time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))
        with self._scene_lock:
            barrier = (
                self._latest_scene.sequence
                if self._latest_scene is not None
                else -1
            )
        self._scan_ready_stamp_ns = self.node.get_clock().now().nanoseconds
        return barrier

    def _lookup_tcp_from_camera(self):
        from rclpy.duration import Duration
        from rclpy.time import Time
        from .transform.tf_projector import (
            transform_message_to_matrix,
        )

        try:
            stamped = self._tf_buffer.lookup_transform(
                str(self._parameter("tcp_frame")),
                str(self._parameter("projection_frame")),
                Time(),
                timeout=Duration(
                    seconds=float(self._parameter("tf_timeout_s"))
                ),
            )
        except Exception as error:
            raise ValueError(
                "Hand-Eye TF를 찾을 수 없습니다: "
                f"{self._parameter('tcp_frame')} <- "
                f"{self._parameter('projection_frame')}: {error}"
            ) from error
        return transform_message_to_matrix(stamped.transform)

    def _move_to_home(self, message: str) -> list[float]:
        if self._bird_pose is not None:
            self._raise_to_safe_scan_height(
                "Home 이동 전 안전 높이까지 후퇴"
            )
        self._publish_status("HOMING", message)
        self.motion.move_joint(
            [
                float(value)
                for value in self._list_parameter("home_joint_pose")
            ],
            velocity=float(self._parameter("home_joint_velocity")),
            acceleration=float(
                self._parameter("home_joint_acceleration")
            ),
            timeout_s=float(self._parameter("home_motion_timeout_s")),
        )
        self._home_tcp = self.motion.current_pose()
        return list(self._home_tcp)

    def _validate_observation_pose(
        self,
        pose: list[float],
        label: str,
    ) -> None:
        """Ensure an observation pose keeps the calibrated camera downward."""

        tcp_from_camera = self._lookup_tcp_from_camera()
        _unused_pose, optical_axis = camera_centered_tcp_pose(
            focus_base_mm=[0.0, 0.0, float(self._parameter("table_z_mm"))],
            tcp_orientation_deg=pose[3:],
            tcp_from_camera_m=tcp_from_camera,
            camera_standoff_mm=max(
                float(self._parameter("local_scan_standoff_mm")),
                1.0,
            ),
            max_tilt_deg=float(self._parameter("bird_max_tilt_deg")),
        )
        self.node.get_logger().info(
            f"{label} camera optical axis in Base: "
            + ", ".join(f"{value:.4f}" for value in optical_axis)
        )

    def _move_home_for_initial_scan(self) -> int:
        """Scan from Home, before spending a Bird climb on the next pick.

        Used at session start and again after every place, so "initial" means
        first-in-the-cycle rather than once-per-session. Leaves ``_scan_scope``
        at "home"; the session loop rises to Bird only when that scene turns
        out to be empty.
        """

        home_tcp = self._move_to_home(
            "Home에서 작업 가능한 공구를 먼저 확인하기 위해 이동"
        )
        self._validate_observation_pose(home_tcp, "Home")
        # Until a Bird view is actually needed, Home is both the observation
        # pose and the validated safe-height reference for direct picking.
        self._bird_pose = home_tcp
        self._observation_orientation = home_tcp[3:]
        self._scan_scope = "home"
        self._local_focus_xy = None
        return self._arm_fresh_scan(
            "Home 자세에서 바로 작업 가능한 새 장면을 기다리는 중"
        )

    def _move_home_to_bird(
        self,
        message: str,
        *,
        already_home: bool = False,
    ) -> int:
        home_tcp = (
            self.motion.current_pose()
            if already_home
            else self._move_to_home(message)
        )
        bird = bird_pose_from_home_tcp(
            home_tcp,
            float(self._parameter("bird_raise_mm")),
        )

        # Catch an incorrect Hand-Eye direction or a Home pose that does not
        # face down before commanding the Base +Z Bird motion.
        self._validate_observation_pose(bird, "Bird")
        self._travel(
            bird,
            "MOVING_BIRD",
            "Home에서 Base +Z 방향으로 Bird view 상승",
        )
        self._bird_pose = bird
        self._observation_orientation = bird[3:]
        self._scan_scope = "global"
        self._local_focus_xy = None
        return self._arm_fresh_scan(
            "Bird view 안정화 후 새 전체 장면을 기다리는 중"
        )

    def _raise_to_safe_scan_height(self, message: str) -> None:
        if self._bird_pose is None:
            return
        current = self.motion.current_pose()
        safe_z = float(self._bird_pose[2])
        if current[2] >= safe_z - 1.0:
            return
        retreat = current.copy()
        retreat[2] = safe_z
        # 안전 높이 이송은 전 구간이 관절 보간이다. MoveLine으로 올리면
        # 컨트롤러가 특이점 영역에 진입해(알람 3205) 회피 모드가 J3를
        # 안전한계까지 감속시키고(알람 3210), 팔이 명령 경로를 따라오지
        # 못한다. 실기 로그에서 amovel 직후에만 관측된 현상이다.
        self._travel(
            retreat, "SAFE_RETREAT", message, joint_interpolated=True
        )

    def _move_via_safe_scan_height(
        self,
        target: list[float],
        *,
        state: str,
        message: str,
    ) -> None:
        if self._bird_pose is None:
            raise ValueError("bird pose is not set")
        self._raise_to_safe_scan_height("로컬 이동 전 수직 안전 후퇴")
        safe_z = max(float(self._bird_pose[2]), float(target[2]))
        target_high = target.copy()
        target_high[2] = safe_z
        self._travel(
            target_high,
            state,
            message + " (안전 높이 수평 이동)",
            joint_interpolated=True,
        )
        if abs(float(target[2]) - safe_z) > 1.0:
            # 하강도 관절 보간이다. 여기가 실기에서 특이점 알람이 실제로
            # 뜬 구간이다. 끝점은 MoveJointx가 그대로 고정하므로 도착
            # 자세는 같고, 중간 궤적만 직선이 아니게 된다. 그 부피는
            # transfer_force_rise_n 감시가 지킨다. 힘 감시로 내려가야 하는
            # 최종 접근(_approach)은 직선이어야 의미가 있어 그대로 둔다.
            self._travel(
                target,
                state,
                message + " (관측 높이 진입)",
                joint_interpolated=True,
            )

    def _move_to_local_scan(self, grasp: dict[str, float]) -> int:
        if self._observation_orientation is None:
            raise ValueError("observation orientation is not set")
        focus = [
            float(grasp["x_mm"]),
            float(grasp["y_mm"]),
            float(self._parameter("table_z_mm")),
        ]
        local_pose, optical_axis = camera_centered_tcp_pose(
            focus_base_mm=focus,
            tcp_orientation_deg=self._observation_orientation,
            tcp_from_camera_m=self._lookup_tcp_from_camera(),
            camera_standoff_mm=float(
                self._parameter("local_scan_standoff_mm")
            ),
            max_tilt_deg=float(self._parameter("bird_max_tilt_deg")),
        )
        self.node.get_logger().info(
            "Local scan focus Base XY="
            f"({focus[0]:.1f}, {focus[1]:.1f}), optical_axis="
            + ", ".join(f"{value:.4f}" for value in optical_axis)
        )
        self._move_via_safe_scan_height(
            local_pose,
            state="MOVING_LOCAL_SCAN",
            message="마지막 Pick 위치 위로 로컬 재검출 이동",
        )
        self._scan_scope = "local"
        self._local_focus_xy = (focus[0], focus[1])
        return self._arm_fresh_scan(
            "마지막 Pick 주변의 새 장면을 기다리는 중"
        )

    def _wait_for_fresh_scene(self, after_sequence: int) -> Scene:
        max_age = float(self._parameter("scene_max_age_s"))
        while self._active.is_set() and not self._shutdown.is_set():
            with self._scene_lock:
                scene = self._latest_scene
                age = time.monotonic() - self._scene_received_at
                if (
                    scene is not None
                    and scene.sequence > after_sequence
                    and age <= max_age
                    and scene.stamp_nanoseconds >= self._scan_ready_stamp_ns
                ):
                    return scene
                self._scene_lock.wait(timeout=0.2)
            self._publish_status("WAITING_SCENE", "최신 RGB-D 검출 대기")
        raise MotionCancelled("session is no longer active")

    def _eligible_detections(self, scene: Scene) -> list[Detection]:
        configured = set(
            str(value) for value in self._list_parameter("target_names")
        )
        allowed = set(SORTABLE_TOOL_NAMES)
        if configured:
            allowed.intersection_update(configured)
        eligible = []
        for detection in sorted(scene.detections, key=lambda item: item.rank):
            if detection.name not in allowed:
                continue
            if not detection.grasp_ready or not is_grasp_ready(
                detection.name, detection.part_name
            ):
                continue
            if (
                detection.grasp_depth_mm is None
                or detection.center_px is None
                or detection.axis_angle_deg is None
                or detection.grip_width_px is None
            ):
                continue
            eligible.append(detection)
        return eligible

    def _select_target_and_grasp(
        self,
        scene: Scene,
    ) -> tuple[Detection, dict[str, float]] | None:
        for detection in self._eligible_detections(scene):
            grasp = self._compute_grasp(detection, scene)
            if self._scan_scope == "local":
                if self._local_focus_xy is None:
                    raise ValueError("local scan focus is not set")
                distance = math.hypot(
                    grasp["x_mm"] - self._local_focus_xy[0],
                    grasp["y_mm"] - self._local_focus_xy[1],
                )
                if distance > float(
                    self._parameter("local_scan_radius_mm")
                ):
                    continue
            if getattr(self, "execution_mode", "inspect") == "pick_place":
                preopen_width = float(
                    self._parameter("gripper_preopen_width_mm")
                )
                target_width, _source = select_gripper_target_width(
                    grasp.get("grip_width_mm"),
                    preopen_width_mm=preopen_width,
                    margin_mm=float(
                        self._parameter("gripper_width_margin_mm")
                    ),
                )
                if target_width is None:
                    self.node.get_logger().warning(
                        f"{detection.name} 건너뜀: 비전 파지 폭 "
                        f"{grasp.get('grip_width_mm')!r} mm가 "
                        f"{preopen_width:.1f} mm 사전 개방 범위에서 "
                        "유효하지 않음"
                    )
                    continue
                grasp["gripper_target_width_mm"] = target_width
            return detection, grasp
        return None

    def _compute_grasp(
        self,
        detection: Detection,
        scene: Scene,
    ) -> dict[str, float]:
        from rclpy.time import Time
        from .transform.tf_projector import (
            TimestampedTransformUnavailable,
        )

        if self._camera_info is None:
            raise ValueError("camera projection is not ready")
        if (
            detection.center_px is None
            or detection.axis_angle_deg is None
            or detection.grip_width_px is None
            or detection.grasp_depth_mm is None
        ):
            raise ValueError("detection has incomplete grasp geometry")

        capture_stamp = Time(nanoseconds=scene.stamp_nanoseconds)
        depth_m = float(detection.grasp_depth_mm) / 1000.0
        u, v = [float(value) for value in detection.center_px]
        angle = math.radians(float(detection.axis_angle_deg))
        probe_px = 50.0
        try:
            center = self._projector.project_depth(
                center_px=[u, v],
                depth_m=depth_m,
                valid_depth_ratio=float(detection.valid_depth_ratio),
                camera_info=self._camera_info,
                capture_stamp=capture_stamp,
            )
            axis_point = self._projector.project_depth(
                center_px=[
                    u + probe_px * math.cos(angle),
                    v + probe_px * math.sin(angle),
                ],
                depth_m=depth_m,
                valid_depth_ratio=float(detection.valid_depth_ratio),
                camera_info=self._camera_info,
                capture_stamp=capture_stamp,
            )
        except TimestampedTransformUnavailable as error:
            raise SceneTransformUnavailable(
                f"촬영 시각 TF 대기: sequence={scene.sequence}: {error}"
            ) from error

        base_point = center.base_point_mm
        axis_base = axis_point.base_point_mm - base_point
        yaw = math.degrees(math.atan2(axis_base[1], axis_base[0])) % 180.0
        fx = float(self._camera_info.k[0])
        mm_per_px = depth_m * 1000.0 / fx
        axial_shift = select_axial_grasp_offset(
            handle_center_px=[u, v],
            tool_center_px=[
                (float(detection.bbox[0]) + float(detection.bbox[2])) / 2.0,
                (float(detection.bbox[1]) + float(detection.bbox[3])) / 2.0,
            ],
            axis_px=[math.cos(angle), math.sin(angle)],
            axis_base_xy=[float(axis_base[0]), float(axis_base[1])],
            grasp_xy_mm=[float(base_point[0]), float(base_point[1])],
            offset_mm=float(self._parameter("grasp_axial_offset_mm")),
            # 손잡이 밖으로 미끄러지면 파지 폭이 의미를 잃는다. 손잡이
            # 장축 길이의 1/4까지만 허용해 중앙에서 중하단까지로 묶는다.
            max_offset_mm=_handle_long_axis_mm(detection, mm_per_px) / 4.0,
        )
        if axial_shift != 0.0:
            axis_length = math.hypot(float(axis_base[0]), float(axis_base[1]))
            # Z는 원래 중심의 값을 그대로 쓴다. 손잡이는 축을 따라 두께가
            # 거의 일정하고, 옮긴 픽셀의 깊이를 다시 재는 것보다 이미
            # 유효 깊이 비율을 통과한 값을 유지하는 편이 안전하다.
            base_point = [
                float(base_point[0])
                + float(axis_base[0]) / axis_length * axial_shift,
                float(base_point[1])
                + float(axis_base[1]) / axis_length * axial_shift,
                float(base_point[2]),
            ]
            self.node.get_logger().info(
                f"Grasp slid {axial_shift:+.1f} mm along the {detection.name} "
                "handle toward the base: "
                f"XY=({float(base_point[0]):.1f}, {float(base_point[1]):.1f})"
            )
        table_z = float(self._parameter("table_z_mm"))
        thickness = max(float(base_point[2]) - table_z, 0.0)
        raw_grasp_z = (
            table_z
            + thickness * float(self._parameter("grasp_depth_ratio"))
        )
        grasp_z_offset = float(self._parameter("grasp_z_offset_mm"))
        grasp_z = apply_grasp_z_offset(
            raw_z_mm=raw_grasp_z,
            offset_mm=grasp_z_offset,
            table_z_mm=table_z,
            clearance_mm=float(
                self._parameter("minimum_table_clearance_mm")
            ),
        )
        self.node.get_logger().info(
            "Grasp Z calculation: "
            f"top={float(base_point[2]):.2f} mm, "
            f"raw={raw_grasp_z:.2f} mm, "
            f"offset={grasp_z_offset:+.2f} mm, "
            f"final={grasp_z:.2f} mm"
        )
        width_mm = float(detection.grip_width_px) * mm_per_px
        return {
            "x_mm": round(float(base_point[0]), 2),
            "y_mm": round(float(base_point[1]), 2),
            "z_mm": round(float(grasp_z), 2),
            "raw_z_mm": round(float(raw_grasp_z), 2),
            "z_offset_mm": round(float(grasp_z_offset), 2),
            "top_z_mm": round(float(base_point[2]), 2),
            "yaw_deg": round(float(yaw), 2),
            "thickness_mm": round(float(thickness), 2),
            "grip_width_mm": round(float(width_mm), 2),
        }

    def _grasp_targets(self, grasp: dict[str, float]):
        if self._observation_orientation is None:
            raise ValueError("observation orientation is not set")
        table_z = float(self._parameter("table_z_mm"))
        clearance = float(self._parameter("minimum_table_clearance_mm"))
        grasp_z = max(grasp["z_mm"], table_z + clearance)
        orientation = orient_with_yaw(
            self._observation_orientation,
            grasp["yaw_deg"],
            float(self._parameter("finger_offset_deg")),
        )
        landing = [grasp["x_mm"], grasp["y_mm"], grasp_z] + orientation
        hover = landing.copy()
        hover[2] += float(self._parameter("hover_mm"))
        return landing, hover

    def _transfer_force_rise_n(self) -> float | None:
        """Return the transfer collision threshold, or None when disabled."""

        value = float(self._parameter("transfer_force_rise_n"))
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value

    def _travel(
        self,
        target: list[float],
        state: str,
        message: str,
        *,
        joint_interpolated: bool = False,
    ) -> None:
        self._publish_status(state, message)
        # 이송 구간은 절대값이 아니라 정지 상태 기준값 대비 상승분으로 감시한다.
        # 공구를 쥐고 있으면 자중이 이미 절대값에 얹혀 있어 접근용
        # force_limit_n을 그대로 쓸 수 없다.
        force_rise_n = self._transfer_force_rise_n()
        baseline_samples = int(
            self._parameter("transfer_force_baseline_samples")
        )
        if joint_interpolated:
            self.motion.move_jointx(
                target,
                velocity=float(self._parameter("safe_jointx_velocity")),
                acceleration=float(
                    self._parameter("safe_jointx_acceleration")
                ),
                timeout_s=float(self._parameter("home_motion_timeout_s")),
                force_rise_n=force_rise_n,
                force_rise_baseline_samples=baseline_samples,
            )
        else:
            self.motion.move_line(
                target,
                velocity=float(self._parameter("travel_velocity")),
                acceleration=float(self._parameter("travel_acceleration")),
                force_rise_n=force_rise_n,
                force_rise_baseline_samples=baseline_samples,
            )

    def _approach(self, target: list[float], state: str, message: str) -> None:
        self._publish_status(state, message)
        self.motion.move_line(
            target,
            velocity=float(self._parameter("approach_velocity")),
            acceleration=float(self._parameter("approach_acceleration")),
            monitor_force=True,
        )

    def _execute_point(
        self, detection: Detection, grasp: dict[str, float]
    ) -> None:
        _landing, hover = self._grasp_targets(grasp)
        self._move_via_safe_scan_height(
            hover,
            state="POINTING",
            message=f"{detection.name} 위로 이동",
        )
        if self._bird_pose is not None:
            self._move_via_safe_scan_height(
                self._bird_pose,
                state="RETURNING",
                message="Bird 관측 자세로 복귀",
            )

    def _execute_pick_place(
        self, detection: Detection, grasp: dict[str, float]
    ) -> None:
        landing, hover = self._grasp_targets(grasp)
        configured_place = [
            float(value) for value in self._list_parameter("place_pose")
        ]
        if configured_place:
            place = configured_place
            place_message = "설정된 공구함 pose로 이동"
        else:
            # 공구함 pose가 아직 정해지지 않은 초기 단계에서는, 집은 위치의
            # base +X/+Y 옆을 임시 배치점으로 사용한다. 실제 공구함을 놓을
            # 위치가 정해지면 place_pose 6값을 넣어 이 fallback을 대체한다.
            offset_x, offset_y = [
                float(value)
                for value in self._list_parameter("place_offset_xy_mm")
            ]
            place = [
                landing[0] + offset_x,
                landing[1] + offset_y,
                landing[2],
                landing[3],
                landing[4],
                landing[5],
            ]
            place_message = (
                f"집은 위치 옆 임시 배치점(+X {offset_x:.0f}, "
                f"+Y {offset_y:.0f} mm)으로 이동"
            )
        place_hover = place.copy()
        place_hover[2] += float(self._parameter("hover_mm"))
        lifted = landing.copy()
        lifted[2] += float(self._parameter("lift_mm"))

        preopen_width = float(
            self._parameter("gripper_preopen_width_mm")
        )
        self._publish_status(
            "GRIPPER",
            f"그리퍼 {preopen_width:.1f} mm 사전 개방",
            target=detection.name,
        )
        self.gripper.open(width_mm=preopen_width)
        self._move_via_safe_scan_height(
            hover,
            state="APPROACHING",
            message=f"{detection.name} hover 이동",
        )
        self._approach(
            landing,
            "DESCENDING",
            "연속 하강 중 (힘 실시간 감시)",
        )
        measured_width = grasp.get("grip_width_mm")
        target_width = grasp.get("gripper_target_width_mm")
        if target_width is None:
            target_width, _source = select_gripper_target_width(
                measured_width,
                preopen_width_mm=preopen_width,
                margin_mm=float(
                    self._parameter("gripper_width_margin_mm")
                ),
            )
        if target_width is None:
            raise ValueError(
                "비전 파지 폭이 유효하지 않아 로봇 접근 전에 제외해야 합니다"
            )
        margin = float(self._parameter("gripper_width_margin_mm"))
        width_detail = (
            f"비전 {float(measured_width):.1f} mm"
            f" - 여유 {margin:.1f} mm"
        )
        self._publish_status(
            "GRIPPING",
            (
                f"{detection.name} 파지: RG2 {target_width:.1f} mm "
                f"({width_detail})"
            ),
        )
        self.gripper.close(width_mm=target_width)
        self._travel(lifted, "LIFTING", f"{detection.name} 들어올림")
        # 공구함 수평 이동은 MoveJointx이고, MoveJointx는 현재 IK solution
        # space를 그대로 쓴다. Pick 위치의 분기는 공구 위치마다 달라 공구함
        # pose에 해가 없을 수 있으므로(컨트롤러가 NOT REACHABLE만 내고 정지),
        # 분기가 항상 같은 Home 관절 자세를 먼저 경유한다. Pick 쪽 이동이
        # 안정적인 이유도 항상 Home에서 출발하기 때문이다.
        #
        # 그 경유 구간의 안전 높이는 Bird가 아니라 Home이다. Bird 높이는
        # 장면을 넓게 보려고 Home보다 bird_raise_mm만큼 더 올린 관측용
        # 높이인데, 공구함은 Home보다 낮아서 이송하는 데 그 높이가 필요
        # 없다. 그대로 두면 Home 경유 직전과 직후에 같은 높이까지 두 번
        # 올라간다. Pick 구간은 관측 높이에서 출발해야 하므로 건드리지 않고,
        # 배치 구간에서만 낮춘 뒤 되돌린다.
        observation_pose = self._bird_pose
        if self._home_tcp is not None:
            self._bird_pose = list(self._home_tcp)
        try:
            self._move_to_home(
                f"{detection.name} 파지 후 공구함 이동 전 Home 경유"
            )
            self._move_via_safe_scan_height(
                place_hover,
                state="PLACING",
                message=place_message,
            )
            self._approach(
                place,
                "PLACE_DESCENT",
                "공구함으로 연속 하강 (힘 실시간 감시)",
            )
            self._publish_status("RELEASING", f"{detection.name} 놓기")
            self.gripper.open(width_mm=preopen_width)
            self._travel(place_hover, "RETREATING", "공구함에서 후퇴")
        finally:
            self._bird_pose = observation_pose

    def shutdown(self) -> None:
        self._shutdown.set()
        self._active.clear()
        with self._scene_lock:
            self._scene_lock.notify_all()
        self._worker.join(timeout=3.0)
        self.node.destroy_node()


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import (
        ExternalShutdownException,
        MultiThreadedExecutor,
    )

    rclpy.init(args=args)
    manager = ToolSortTaskManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(manager.node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        manager.shutdown()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
