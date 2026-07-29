"""공구 전달 전용 task manager.

사용자 요청 공구를 하나만 집어 전달 위치까지 옮기고, 사용자가 잡아당겼는지를
힘 변화로 판정해 release까지 마무리한다. ``robot_control.TOOL_FETCH``는 이
노드와 request 토픽을 통해 세션을 구동한다.
"""

from __future__ import annotations

import math
import threading
import time

from tool_sorter_core.motion import MotionCancelled, MotionError
from tool_sorter_core.schema import Detection
from tool_sorter_core.task_manager import (
    SceneTransformUnavailable,
    ToolSortTaskManager,
    select_gripper_target_width,
)

from .tool_request import (  # noqa: F401  (re-exported for callers/tests)
    TOOL_ALIASES,
    normalize_tool_request,
    spoken_tool_name,
)


HANDOVER_PREFIX = "/tool_sorter/handover"


class HandoverTaskManager(ToolSortTaskManager):
    """Hand a requested tool to the user and release it when they pull.

    The request arrives as one keyword on ``<prefix>/request``. Producing that
    keyword is somebody else's job: a GUI button, an external speech stack, or
    a plain ``ros2 topic pub``. This node owns no audio device and speaks
    nothing, so every user-facing message goes out on the status topic.
    """

    def __init__(self) -> None:
        self._request_lock = threading.Lock()
        self._requested_name: str | None = None
        self._request_event = threading.Event()
        self._accepting_requests = threading.Event()
        self._handover_status_publisher = None
        super().__init__()

        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._handover_status_publisher = self.node.create_publisher(
            String,
            f"{HANDOVER_PREFIX}/status",
            status_qos,
        )
        self.node.create_subscription(
            String,
            f"{HANDOVER_PREFIX}/request",
            self._on_request,
            10,
        )
        self.node.create_service(
            Trigger, f"{HANDOVER_PREFIX}/start", self._on_start
        )
        self.node.create_service(
            Trigger, f"{HANDOVER_PREFIX}/stop", self._on_stop
        )
        self._publish_handover_status()

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        from rclpy.parameter import Parameter

        super()._declare_parameters()
        # 공구함 위 실측 절대 관측 자세 [X Y Z RX RY RZ]. 비워두면 organize와
        # 같은 Bird view(Home TCP + bird_raise_mm)를 쓴다. 빈 배열을 기본값으로
        # 주면 Humble이 타입을 추론하지 못하므로 타입을 명시한다.
        self.node.declare_parameter(
            "tool_pickup_scan_pose", Parameter.Type.DOUBLE_ARRAY
        )
        defaults = {
            "release_force_n": 12.0,
            "release_timeout_s": 30.0,
            "release_baseline_samples": 5,
            "search_timeout_s": 10.0,
            # 정확 매칭이 실패했을 때만 쓰는 발음 유사도 문턱값. 올리면
            # 되묻는 일이 늘고, 내리면 엉뚱한 공구를 집을 수 있다.
            "request_fuzzy_threshold": 0.78,
        }
        for name, value in defaults.items():
            self.node.declare_parameter(name, value)

    def _tool_pickup_scan_pose(self) -> list[float]:
        return [
            float(value)
            for value in self._list_parameter("tool_pickup_scan_pose")
        ]

    def _validate_start(self) -> str | None:
        problem = super()._validate_start()
        if problem:
            return problem
        if self.execution_mode != "pick_place":
            return "전달 패키지는 execution_mode=pick_place만 허용합니다"
        if not bool(self._parameter("auto_scan_motion")):
            # 관측 자세로 가려면 어느 경로든 Home을 먼저 거쳐야 한다. 이 값이
            # 꺼져 있으면 상위가 home_joint_pose와 safe_jointx_* 를 검증하지
            # 않아, 검증되지 않은 자세와 속도로 팔이 움직인다.
            return "전달 패키지는 auto_scan_motion=true가 필요합니다"
        pose = self._tool_pickup_scan_pose()
        if pose and len(pose) != 6:
            return (
                "tool_pickup_scan_pose는 비워두거나 X Y Z RX RY RZ "
                "6값이어야 합니다"
            )
        if not all(math.isfinite(value) for value in pose):
            return "tool_pickup_scan_pose에 NaN 또는 Inf를 쓸 수 없습니다"
        release_force = float(self._parameter("release_force_n"))
        if not math.isfinite(release_force) or release_force <= 0.0:
            return "release_force_n은 0보다 큰 값이어야 합니다"
        force_limit = float(self._parameter("force_limit_n"))
        if release_force >= force_limit:
            return (
                f"release_force_n({release_force:.1f}N)은 안전 중단 기준인 "
                f"force_limit_n({force_limit:.1f}N)보다 작아야 합니다"
            )
        for name in ("release_timeout_s", "search_timeout_s"):
            value = float(self._parameter(name))
            if not math.isfinite(value) or value <= 0.0:
                return f"{name}는 0보다 큰 값이어야 합니다"
        if int(self._parameter("release_baseline_samples")) < 1:
            return "release_baseline_samples는 1 이상이어야 합니다"
        fuzzy = float(self._parameter("request_fuzzy_threshold"))
        if not 0.0 < fuzzy <= 1.0:
            return "request_fuzzy_threshold는 0 초과 1 이하여야 합니다"
        return None

    # ------------------------------------------------------------------
    # Keyword request handling
    # ------------------------------------------------------------------

    def _on_request(self, message) -> None:
        raw = str(message.data)
        tool_name = normalize_tool_request(
            raw,
            threshold=float(self._parameter("request_fuzzy_threshold")),
        )
        if tool_name is None:
            self.node.get_logger().warning(
                f"Unrecognized tool request: {raw!r}"
            )
            if self._accepting_requests.is_set():
                # 팔이 요청을 기다리며 멈춰 있는 동안에만 상태를 덮는다.
                # 이동 중에 덮으면 dashboard가 현재 동작을 잃는다.
                self._publish_status(
                    "WAITING_REQUEST",
                    f"'{raw}'는 아는 공구가 아닙니다. 다시 선택하세요",
                    target="",
                )
            return
        if not self._accepting_requests.is_set():
            # 세션이 아예 시작되지 않은 경우와 앞 요청을 처리 중인 경우를
            # 구분한다. 둘 다 요청을 버리지만 사용자가 할 일이 다르다.
            if not self._active.is_set():
                self.node.get_logger().warning(
                    f"Ignoring request before the session starts: {tool_name}. "
                    "Call /tool_sorter/handover/start (or the dashboard "
                    "작업 시작 button) first; Bird Scan alone does not start one"
                )
                self._publish_status(
                    "IDLE",
                    f"{tool_name} 요청을 받았으나 전달 작업이 시작되지 "
                    "않았습니다. 작업 시작을 먼저 실행하세요",
                )
                return
            # 이동 중에는 상태를 덮지 않고 로그만 남긴다.
            self.node.get_logger().info(
                f"Ignoring request while busy: {tool_name}"
            )
            return
        with self._request_lock:
            self._requested_name = tool_name
        self._request_event.set()

    def _wait_for_request(self) -> str | None:
        """Accept one keyword request while the arm rests at the toolbox."""

        self._request_event.clear()
        with self._request_lock:
            self._requested_name = None
        self._accepting_requests.set()
        # 상태 문구는 입력 방식과 무관하게 읽히도록 둔다. 요청을 만드는 쪽이
        # GUI 버튼일 수도, 외부 제어부일 수도 있다.
        self._publish_status(
            "WAITING_REQUEST",
            "필요한 공구를 선택하세요",
            target="",
        )
        try:
            while self._active.is_set() and not self._shutdown.is_set():
                if self._request_event.wait(timeout=0.2):
                    with self._request_lock:
                        return self._requested_name
            raise MotionCancelled("handover session is no longer active")
        finally:
            self._accepting_requests.clear()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def _move_to_observation(
        self,
        message: str,
        *,
        already_home: bool = False,
    ) -> int:
        """Reach the observation pose without leaving the Home IK branch.

        MoveJointx keeps the current solution space, so every safe-height
        transfer in this package departs from Home. Arriving here *by*
        MoveJointx means the branch at the observation pose is still Home's,
        which is what keeps the later observation->object transfer solvable.
        Reaching the same pose with MoveLine would both risk a wrist
        singularity on the straight path and leave the branch unverified.
        """

        pose = self._tool_pickup_scan_pose()
        if not pose:
            return self._move_home_to_bird(message, already_home=already_home)

        # 이동 전에 카메라가 실제로 아래를 보는지 확인한다. Bird 경로가
        # _move_home_to_bird 안에서 하는 것과 같은 검사다.
        self._validate_observation_pose(pose, "ToolPickupScan")
        home_tcp = (
            self.motion.current_pose()
            if already_home
            else self._move_to_home("공구함 관측 자세 이동 전 Home 경유")
        )
        # 안전 높이는 Home과 관측 자세 중 높은 쪽. 이 기준을 넘겨주면
        # _move_via_safe_scan_height가 수직 상승 → movejx 수평 → 수직 하강
        # 순서로 쪼개 준다. 수평 구간만 관절 보간이라는 점이 핵심이다.
        reference = list(home_tcp)
        reference[2] = max(float(home_tcp[2]), float(pose[2]))
        self._bird_pose = reference
        self._observation_orientation = pose[3:]
        self._move_via_safe_scan_height(
            pose,
            state="MOVING_OBSERVATION",
            message=message,
        )
        # 도착한 뒤에는 관측 자세 자체가 이후 안전 후퇴 높이의 기준이 된다.
        self._bird_pose = pose
        self._observation_orientation = pose[3:]
        self._scan_scope = "global"
        self._local_focus_xy = None
        return self._arm_fresh_scan(
            "공구함 관측 자세 안정화 후 새 장면을 기다리는 중"
        )

    # ------------------------------------------------------------------
    # Session
    # ------------------------------------------------------------------

    def _run_session(self) -> None:
        self.motion.verify_presets(
            enforce_tool_preset=bool(
                self._parameter("enforce_tool_preset")
            )
        )
        last_sequence = self._move_to_observation(
            "전달 시작; Home을 거쳐 공구함 관측 자세로 이동"
        )

        while self._active.is_set() and not self._shutdown.is_set():
            tool_name = self._wait_for_request()
            if tool_name is None:
                continue
            spoken = spoken_tool_name(tool_name)
            found = self._find_requested(tool_name, last_sequence)
            if found is None:
                last_sequence = self._arm_fresh_scan(
                    f"{spoken} 미검출; 다시 선택하거나 공구 위치를 확인하세요"
                )
                continue
            detection, grasp, last_sequence = found
            self._publish_status(
                "PLANNED",
                f"{tool_name}: X={grasp['x_mm']:.1f}, "
                f"Y={grasp['y_mm']:.1f}, Z={grasp['z_mm']:.1f}, "
                f"yaw={grasp['yaw_deg']:.1f}",
                target=tool_name,
            )
            self._execute_handover(detection, grasp)
            self._completed += 1
            # 전달을 마친 팔은 Home에 서 있으므로 다시 Home으로 갈 필요가 없다.
            # Home에서 출발한다는 movejx 전제도 그대로 만족한다. 사람이 공구를
            # 받아간 자리에 팔을 세워두지 않도록 공구함 위로 물러난 뒤 끝낸다.
            self._move_to_observation(
                "전달 완료; 공구함 관측 자세로 복귀",
                already_home=True,
            )
            # 한 번의 start는 한 번의 전달로 끝난다. 세션을 열어둔 채
            # WAITING_REQUEST로 돌아가면 이 노드는 COMPLETE를 영영 내지 않고,
            # 호출한 쪽은 작업이 끝났다는 것을 판정할 수 없다.
            self._publish_status(
                "COMPLETE",
                f"{spoken} 전달 완료",
                target=tool_name,
            )
            self._active.clear()
            return

    def _find_requested(
        self,
        tool_name: str,
        after_sequence: int,
    ) -> tuple[Detection, dict[str, float], int] | None:
        """Search fresh scenes for the requested class until the search ends."""

        deadline = time.monotonic() + float(
            self._parameter("search_timeout_s")
        )
        sequence = after_sequence
        while self._active.is_set() and not self._shutdown.is_set():
            scene = self._wait_for_fresh_scene(sequence)
            sequence = scene.sequence
            # _eligible_detections는 이미 겹침을 반영한 rank 순서다. 같은 공구가
            # 두 개 놓이는 일이 없으므로 그 순서에서 요청 클래스를 그대로 집는다.
            detection = next(
                (
                    item
                    for item in self._eligible_detections(scene)
                    if item.name == tool_name
                ),
                None,
            )
            if detection is not None:
                try:
                    grasp = self._compute_grasp(detection, scene)
                except SceneTransformUnavailable as error:
                    self._publish_status(
                        "WAITING_TRANSFORM",
                        str(error),
                        scene_sequence=scene.sequence,
                    )
                    continue
                return detection, grasp, sequence
            if time.monotonic() > deadline:
                return None
            self._publish_status(
                "SEARCHING",
                f"{tool_name} 위치를 확인하는 중",
                target=tool_name,
                scene_sequence=scene.sequence,
            )
        raise MotionCancelled("handover session is no longer active")

    def _execute_handover(
        self,
        detection: Detection,
        grasp: dict[str, float],
    ) -> None:
        landing, hover = self._grasp_targets(grasp)
        lifted = landing.copy()
        lifted[2] += float(self._parameter("lift_mm"))
        preopen_width = float(
            self._parameter("gripper_preopen_width_mm")
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
                "비전 파지 폭이 유효하지 않아 공구에 접근할 수 없습니다"
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
        self._publish_status(
            "GRIPPING",
            f"{detection.name} 파지: RG2 {target_width:.1f} mm",
        )
        self.gripper.close(width_mm=target_width)
        self._travel(lifted, "LIFTING", f"{detection.name} 들어올림")
        # Home이 곧 전달 자세다. _move_to_home이 Bird 높이까지 수직 후퇴를
        # 먼저 수행하므로 별도 안전 후퇴를 붙이지 않는다.
        self._move_to_home(f"{detection.name} 전달 자세(Home)로 이동")
        self._release_on_pull(detection.name)

    def _release_on_pull(self, tool_name: str) -> None:
        """Hold the tool still and open only once the user pulls it.

        The gripper must not open on a timeout: nobody is holding the tool
        then, so it would drop. Keep holding and re-post the prompt instead, and
        let the operator end the job with the stop service.
        """

        spoken = spoken_tool_name(tool_name)
        self._publish_status(
            "WAITING_PULL",
            f"{spoken} 전달 대기; 공구를 잡아당기세요",
            target=tool_name,
        )
        while True:
            try:
                rise_n = self.motion.wait_for_external_force(
                    threshold_n=float(self._parameter("release_force_n")),
                    timeout_s=float(self._parameter("release_timeout_s")),
                    baseline_samples=int(
                        self._parameter("release_baseline_samples")
                    ),
                )
                break
            except MotionCancelled:
                raise
            except MotionError as error:
                if not self._active.is_set() or self._shutdown.is_set():
                    raise
                self.node.get_logger().warning(str(error))
                self._publish_status(
                    "WAITING_PULL",
                    f"{spoken}를 아직 가져가지 않았습니다; 계속 대기 중이니 "
                    "공구를 잡아당기세요",
                    target=tool_name,
                )

        self.node.get_logger().info(
            f"Handover pull confirmed: +{rise_n:.1f}N over baseline"
        )
        self._publish_status(
            "RELEASING", f"{spoken} 그리퍼 열기", target=tool_name
        )
        self.gripper.open(
            width_mm=float(self._parameter("gripper_preopen_width_mm"))
        )

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_handover_status(self) -> None:
        publisher = self._handover_status_publisher
        if publisher is None:
            return
        from std_msgs.msg import String

        output = String()
        output.data = self._status.to_json()
        publisher.publish(output)

    def _publish_status(
        self,
        state: str,
        message: str,
        target: str | None = None,
        scene_sequence: int | None = None,
    ) -> None:
        super()._publish_status(
            state,
            message,
            target=target,
            scene_sequence=scene_sequence,
        )
        self._publish_handover_status()


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import (
        ExternalShutdownException,
        MultiThreadedExecutor,
    )

    rclpy.init(args=args)
    manager = HandoverTaskManager()
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
