"""Doosan + RG2 공통 모션 유틸리티.

공구 정리/전달 경로에서 실제 로봇 서비스 호출과 안전 검사를 담당한다. 이 파일의
예외 타입은 "왜 실패했는지"를 task manager가 상위 상태 문자열로 바꿀 수 있게
정교하게 나뉘어 있다.
"""

from __future__ import annotations

import math
import threading
import time


class MotionError(RuntimeError):
    pass


class MotionCancelled(MotionError):
    pass


class ForceLimitExceeded(MotionError):
    pass


class TargetUnreachable(MotionError):
    """The controller refused a pose instead of moving toward it.

    MoveJointx carries a fixed IK solution space, so a pose that branch cannot
    solve is rejected outright (controller alarm 1206, "Pose(...) is NOT
    REACHABLE"). The command is asynchronous and still reports success, and
    check_motion returns to IDLE immediately, so the only evidence visible from
    here is that the arm never left its starting pose.
    """


class ServiceCallError(MotionError):
    pass


RG2_MAX_WIDTH_MM = 110.0


def rg2_width_mm_to_command(width_mm: float) -> str:
    """Convert an RG2 opening in millimetres to its 0.1 mm command unit."""

    width = float(width_mm)
    if not math.isfinite(width):
        raise ValueError("RG2 width must be finite")
    if width < 0.0 or width > RG2_MAX_WIDTH_MM:
        raise ValueError(
            f"RG2 width must be between 0 and {RG2_MAX_WIDTH_MM:g} mm"
        )
    # Use round-half-up instead of Python's bankers rounding. The driver
    # accepts an integer string where one count is 0.1 mm.
    tenths_mm = int(math.floor(width * 10.0 + 0.5))
    return str(tenths_mm)


class ServiceCaller:
    """Wait for a future while another executor thread spins the node."""

    @staticmethod
    def call(client, request, timeout_s: float, label: str):
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout_s):
            future.cancel()
            raise ServiceCallError(
                f"{label} service timeout ({timeout_s:.1f}s)"
            )
        error = future.exception()
        if error is not None:
            raise ServiceCallError(f"{label} service failed: {error}")
        response = future.result()
        if response is None:
            raise ServiceCallError(f"{label} returned no response")
        return response


class ForceRiseDetector:
    """Confirm a sustained force rise over a measured resting baseline.

    A handover cannot use an absolute threshold: the held tool already loads
    the sensor with its own weight, and that resting load differs per tool and
    per pose. Only the rise above the measured baseline means somebody pulled.
    """

    def __init__(
        self,
        baseline_n: float,
        threshold_n: float,
        debounce_samples: int,
    ):
        if not math.isfinite(float(baseline_n)):
            raise ValueError("baseline_n must be finite")
        if not math.isfinite(float(threshold_n)) or float(threshold_n) <= 0.0:
            raise ValueError("threshold_n must be greater than zero")
        self.baseline_n = float(baseline_n)
        self.threshold_n = float(threshold_n)
        self.debounce_samples = max(int(debounce_samples), 1)
        self.last_rise_n = 0.0
        self._hits = 0

    def update(self, force_n: float) -> bool:
        """Feed one reading and report whether a pull is confirmed."""

        self.last_rise_n = float(force_n) - self.baseline_n
        self._hits = (
            self._hits + 1 if self.last_rise_n >= self.threshold_n else 0
        )
        return self._hits >= self.debounce_samples


class DoosanMotion:
    """Non-blocking Doosan line motion with continuous force monitoring."""

    DR_BASE = 0
    DR_COND_NONE = -10000.0
    DR_STATE_IDLE = 0
    DR_STATE_BUSY = 2
    DR_QSTOP = 1
    DR_SSTOP = 2

    def __init__(
        self,
        node,
        namespace: str,
        tcp_name: str,
        tool_name: str,
        force_limit_n: float,
        force_poll_hz: float,
        force_debounce_samples: int,
        position_tolerance_mm: float,
    ):
        from dsr_msgs2.srv import (
            CheckMotion,
            GetCurrentPosx,
            GetCurrentTcp,
            GetCurrentTool,
            GetToolForce,
            MoveJoint,
            MoveJointx,
            MoveLine,
            MoveStop,
            SetCurrentTcp,
            SetCurrentTool,
        )

        self.node = node
        prefix = "/" + namespace.strip("/")
        self._types = {
            "check": CheckMotion,
            "pose": GetCurrentPosx,
            "get_tcp": GetCurrentTcp,
            "get_tool": GetCurrentTool,
            "set_tcp": SetCurrentTcp,
            "set_tool": SetCurrentTool,
            "force": GetToolForce,
            "move_joint": MoveJoint,
            "move_jointx": MoveJointx,
            "move": MoveLine,
            "stop": MoveStop,
        }
        self._clients = {
            "check": node.create_client(
                CheckMotion, f"{prefix}/motion/check_motion"
            ),
            "pose": node.create_client(
                GetCurrentPosx, f"{prefix}/aux_control/get_current_posx"
            ),
            "get_tcp": node.create_client(
                GetCurrentTcp, f"{prefix}/tcp/get_current_tcp"
            ),
            "get_tool": node.create_client(
                GetCurrentTool, f"{prefix}/tool/get_current_tool"
            ),
            "set_tcp": node.create_client(
                SetCurrentTcp, f"{prefix}/tcp/set_current_tcp"
            ),
            "set_tool": node.create_client(
                SetCurrentTool, f"{prefix}/tool/set_current_tool"
            ),
            "force": node.create_client(
                GetToolForce, f"{prefix}/aux_control/get_tool_force"
            ),
            "move_joint": node.create_client(
                MoveJoint, f"{prefix}/motion/move_joint"
            ),
            "move_jointx": node.create_client(
                MoveJointx, f"{prefix}/motion/move_jointx"
            ),
            "move": node.create_client(MoveLine, f"{prefix}/motion/move_line"),
            "stop": node.create_client(MoveStop, f"{prefix}/motion/move_stop"),
        }
        self.force_limit_n = float(force_limit_n)
        self.tcp_name = str(tcp_name).strip()
        self.tool_name = str(tool_name).strip()
        if not self.tcp_name or not self.tool_name:
            raise ValueError("tcp_name and tool_name cannot be empty")
        self.force_poll_period = 1.0 / max(float(force_poll_hz), 1.0)
        self.force_debounce_samples = max(int(force_debounce_samples), 1)
        self.position_tolerance_mm = float(position_tolerance_mm)
        if (
            not math.isfinite(self.position_tolerance_mm)
            or self.position_tolerance_mm <= 0.0
        ):
            raise ValueError("position_tolerance_mm must be greater than zero")
        self._cancel = threading.Event()
        self._quick_stop = threading.Event()

    def wait_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        for name, client in self._clients.items():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(remaining):
                raise ServiceCallError(f"Doosan service unavailable: {name}")

    def request_stop(self, quick: bool = False) -> None:
        if quick:
            self._quick_stop.set()
        self._cancel.set()
        # Fire immediately; the worker also stops when it sees the event.
        if self._clients["stop"].service_is_ready():
            request = self._types["stop"].Request()
            request.stop_mode = self.DR_QSTOP if quick else self.DR_SSTOP
            self._clients["stop"].call_async(request)

    def clear_stop(self) -> None:
        self._cancel.clear()
        self._quick_stop.clear()

    def current_pose(self) -> list[float]:
        values = self._current_pose_values()
        return [float(value) for value in values[:6]]

    def current_solution_space(self) -> int:
        """Return the controller IK branch for a subsequent MoveJointx."""

        values = self._current_pose_values()
        if len(values) < 7:
            raise ServiceCallError(
                "get_current_posx pose has no solution-space value"
            )
        solution_space = int(values[6])
        if not 0 <= solution_space <= 7:
            raise ServiceCallError(
                "get_current_posx returned invalid solution space: "
                f"{solution_space}"
            )
        return solution_space

    def _current_pose_values(self) -> list[float]:
        request = self._types["pose"].Request()
        request.ref = self.DR_BASE
        response = ServiceCaller.call(
            self._clients["pose"], request, 2.0, "get_current_posx"
        )
        if not response.success or not response.task_pos_info:
            raise ServiceCallError("get_current_posx returned an invalid pose")
        values = list(response.task_pos_info[0].data)
        if len(values) < 6:
            raise ServiceCallError(
                "get_current_posx pose has fewer than 6 values"
            )
        return [float(value) for value in values]

    def _set_named_preset(self, key: str, name: str) -> None:
        request = self._types[key].Request()
        request.name = name
        response = ServiceCaller.call(
            self._clients[key], request, 2.0, key
        )
        if not response.success:
            raise ServiceCallError(f"{key} rejected preset: {name}")

    def _get_named_preset(self, key: str) -> str:
        response = ServiceCaller.call(
            self._clients[key], self._types[key].Request(), 2.0, key
        )
        if not response.success:
            raise ServiceCallError(f"{key} reported failure")
        return str(response.info)

    def verify_presets(self, enforce_tool_preset: bool = True) -> None:
        actual_tool = self._get_named_preset("get_tool")
        actual_tcp = self._get_named_preset("get_tcp")
        if actual_tool != self.tool_name:
            message = (
                f"Active Tool Weight mismatch: expected={self.tool_name!r}, "
                f"actual={actual_tool!r}. TCP coordinates are unaffected, "
                "but force compensation depends on the active Tool settings"
            )
            if enforce_tool_preset:
                raise ServiceCallError(message)
            self.node.get_logger().warning(message)
        if actual_tcp != self.tcp_name:
            raise ServiceCallError(
                f"Active TCP mismatch: expected={self.tcp_name!r}, "
                f"actual={actual_tcp!r}"
            )

    def configure_presets(self, enforce_tool_preset: bool = True) -> None:
        # Avoid set_current_* calls when the calibrated presets are already
        # active. Some controller modes allow querying but reject redundant
        # selection calls.
        actual_tool = self._get_named_preset("get_tool")
        actual_tcp = self._get_named_preset("get_tcp")
        if enforce_tool_preset and actual_tool != self.tool_name:
            self._set_named_preset("set_tool", self.tool_name)
        if actual_tcp != self.tcp_name:
            self._set_named_preset("set_tcp", self.tcp_name)
        self.verify_presets(enforce_tool_preset=enforce_tool_preset)

    def tool_force_magnitude(self) -> float:
        request = self._types["force"].Request()
        request.ref = self.DR_BASE
        response = ServiceCaller.call(
            self._clients["force"], request, 1.0, "get_tool_force"
        )
        if not response.success:
            raise ServiceCallError("get_tool_force reported failure")
        force = response.tool_force
        return math.sqrt(sum(float(force[index]) ** 2 for index in range(3)))

    def measure_force_baseline(self, samples: int = 5) -> float:
        """Average the resting tool force while the robot holds still."""

        count = max(int(samples), 1)
        total = 0.0
        for index in range(count):
            total += self.tool_force_magnitude()
            if index + 1 < count:
                time.sleep(self.force_poll_period)
        return total / count

    def wait_for_external_force(
        self,
        threshold_n: float,
        timeout_s: float,
        baseline_n: float | None = None,
        baseline_samples: int = 5,
    ) -> float:
        """Hold position until the tool force rises by ``threshold_n``.

        This is the opposite of ``monitor_force`` during a move: here an
        external force is the expected handover signal rather than a collision,
        so the robot is never stopped and ``ForceLimitExceeded`` is never
        raised. Returns the confirmed rise in newtons.
        """

        if baseline_n is None:
            baseline_n = self.measure_force_baseline(baseline_samples)
        detector = ForceRiseDetector(
            baseline_n=baseline_n,
            threshold_n=threshold_n,
            debounce_samples=self.force_debounce_samples,
        )
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if self._cancel.is_set():
                raise MotionCancelled("wait for external force cancelled")
            if detector.update(self.tool_force_magnitude()):
                return detector.last_rise_n
            if time.monotonic() > deadline:
                raise MotionError(
                    f"no external force within {float(timeout_s):.1f}s "
                    f"(baseline={baseline_n:.1f}N, "
                    f"last={detector.last_rise_n:+.1f}N)"
                )
            time.sleep(self.force_poll_period)

    def stop(self, quick: bool = False) -> None:
        request = self._types["stop"].Request()
        request.stop_mode = self.DR_QSTOP if quick else self.DR_SSTOP
        response = ServiceCaller.call(
            self._clients["stop"], request, 2.0, "move_stop"
        )
        if not response.success:
            raise ServiceCallError("move_stop reported failure")

    def _motion_status(self) -> int:
        request = self._types["check"].Request()
        response = ServiceCaller.call(
            self._clients["check"], request, 1.0, "check_motion"
        )
        if not response.success:
            raise ServiceCallError("check_motion reported failure")
        return int(response.status)

    def transfer_force_detector(
        self,
        threshold_n: float | None,
        baseline_samples: int,
    ) -> ForceRiseDetector | None:
        """Sample the resting load before a transfer that carries a tool.

        A transfer cannot use the absolute approach threshold: a held tool
        already loads the sensor with its own weight, and that load differs per
        tool and per pose. Call this while the arm is still stationary, before
        the async move command is issued.
        """

        if threshold_n is None:
            return None
        threshold = float(threshold_n)
        if not math.isfinite(threshold) or threshold <= 0.0:
            return None
        return ForceRiseDetector(
            baseline_n=self.measure_force_baseline(baseline_samples),
            threshold_n=threshold,
            debounce_samples=self.force_debounce_samples,
        )

    def _wait_motion_complete(
        self,
        *,
        timeout_s: float,
        monitor_force: bool,
        force_rise: ForceRiseDetector | None = None,
    ) -> None:
        started = time.monotonic()
        force_hits = 0
        while True:
            if self._cancel.is_set():
                self.stop(quick=self._quick_stop.is_set())
                raise MotionCancelled("motion cancelled by operator")

            if force_rise is not None:
                # 이송 구간은 기준값 대비 상승분으로 본다. 쥔 공구의 자중이
                # 이미 절대값에 얹혀 있어 접근용 문턱값을 그대로 쓸 수 없다.
                force = self.tool_force_magnitude()
                if force_rise.update(force):
                    self.stop(quick=False)
                    raise ForceLimitExceeded(
                        f"external force rose "
                        f"{force_rise.last_rise_n:.1f}N over the "
                        f"{force_rise.baseline_n:.1f}N resting baseline "
                        f"(limit {force_rise.threshold_n:.1f}N)"
                    )
            elif monitor_force:
                force = self.tool_force_magnitude()
                force_hits = (
                    force_hits + 1
                    if force > self.force_limit_n
                    else 0
                )
                if force_hits >= self.force_debounce_samples:
                    self.stop(quick=False)
                    raise ForceLimitExceeded(
                        f"external force {force:.1f}N exceeded "
                        f"{self.force_limit_n:.1f}N"
                    )

            status = self._motion_status()
            elapsed = time.monotonic() - started
            # Avoid the short IDLE race between accepting an async command and
            # the controller changing to INIT/BUSY.
            if status == self.DR_STATE_IDLE and elapsed >= 0.5:
                return
            if elapsed > timeout_s:
                self.stop(quick=False)
                raise MotionError(
                    f"motion timed out after {timeout_s:.1f}s"
                )
            time.sleep(self.force_poll_period)

    def _verify_position_target(
        self,
        target: list[float],
        motion_name: str = "motion",
        start: list[float] | None = None,
    ) -> None:
        """Reject an async Cartesian target that ended IDLE without arrival."""

        actual = self.current_pose()
        error_mm = math.dist(actual[:3], target[:3])
        if error_mm <= self.position_tolerance_mm:
            return
        expected_xyz = ", ".join(f"{value:.2f}" for value in target[:3])
        actual_xyz = ", ".join(f"{value:.2f}" for value in actual[:3])
        if (
            start is not None
            and math.dist(actual[:3], start[:3])
            <= self.position_tolerance_mm
        ):
            # 팔이 출발점을 벗어나지도 못했다. 감속이나 충돌이 아니라
            # 컨트롤러가 자세 자체를 거부한 것이다.
            raise TargetUnreachable(
                f"{motion_name} target is NOT REACHABLE in the current IK "
                f"solution space: target_xyz=[{expected_xyz}] mm, "
                f"the arm never left [{actual_xyz}] mm"
            )
        raise MotionError(
            f"{motion_name} target was not reached: "
            f"target_xyz=[{expected_xyz}] mm, "
            f"actual_xyz=[{actual_xyz}] mm, "
            f"error={error_mm:.2f} mm > "
            f"tolerance={self.position_tolerance_mm:.2f} mm"
        )

    def move_joint(
        self,
        target: list[float],
        velocity: float,
        acceleration: float,
        timeout_s: float = 45.0,
    ) -> None:
        if len(target) != 6:
            raise ValueError("move_joint target must have six values")
        request = self._types["move_joint"].Request()
        request.pos = [float(value) for value in target]
        request.vel = float(velocity)
        request.acc = float(acceleration)
        request.time = 0.0
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 1
        response = ServiceCaller.call(
            self._clients["move_joint"], request, 3.0, "move_joint"
        )
        if not response.success:
            raise ServiceCallError("move_joint was rejected")
        self._wait_motion_complete(
            timeout_s=float(timeout_s),
            monitor_force=False,
        )

    def move_jointx(
        self,
        target: list[float],
        velocity: float,
        acceleration: float,
        timeout_s: float = 45.0,
        force_rise_n: float | None = None,
        force_rise_baseline_samples: int = 5,
    ) -> None:
        """Move to a Cartesian pose with joint interpolation.

        The current IK solution space is retained so a safe-height transfer
        does not unexpectedly flip shoulder, elbow, or wrist configuration.
        Callers must therefore start from a pose whose branch can also hold the
        target; every transfer in this package departs from Home for that
        reason.

        Joint interpolation only pins the endpoints: the elbow and wrist sweep
        a volume that is not a straight Cartesian line. ``force_rise_n`` guards
        that unpinned volume against a collision.
        """

        if len(target) != 6:
            raise ValueError("move_jointx target must have six values")
        start = self.current_pose()
        # 기준값은 명령을 보내기 전, 팔이 아직 멈춰 있을 때 재야 한다.
        force_rise = self.transfer_force_detector(
            force_rise_n, force_rise_baseline_samples
        )
        request = self._types["move_jointx"].Request()
        request.pos = [float(value) for value in target]
        request.vel = float(velocity)
        request.acc = float(acceleration)
        request.time = 0.0
        request.radius = 0.0
        request.ref = self.DR_BASE
        request.mode = 0
        request.blend_type = 0
        request.sol = self.current_solution_space()
        request.sync_type = 1
        response = ServiceCaller.call(
            self._clients["move_jointx"],
            request,
            3.0,
            "move_jointx",
        )
        if not response.success:
            raise ServiceCallError("move_jointx was rejected")
        self._wait_motion_complete(
            timeout_s=float(timeout_s),
            monitor_force=False,
            force_rise=force_rise,
        )
        self._verify_position_target(target, "move_jointx", start=start)

    def move_line(
        self,
        target: list[float],
        velocity: float,
        acceleration: float,
        monitor_force: bool = False,
        timeout_s: float | None = None,
        force_rise_n: float | None = None,
        force_rise_baseline_samples: int = 5,
    ) -> None:
        """Move on a straight Base-frame line.

        ``monitor_force`` compares the absolute reading against
        ``force_limit_n`` and suits an approach that starts from an empty
        gripper. ``force_rise_n`` instead compares against a baseline sampled
        here and suits a transfer that may be carrying a tool. Passing both is
        rejected: two thresholds on one motion hide which one stopped it.
        """

        if len(target) != 6:
            raise ValueError("move_line target must have six values")
        if monitor_force and force_rise_n is not None:
            raise ValueError(
                "use either monitor_force or force_rise_n, not both"
            )
        start = self.current_pose()
        distance = math.dist(start[:3], target[:3])
        if timeout_s is None:
            timeout_s = max(8.0, distance / max(velocity, 1.0) * 4.0 + 5.0)

        # 기준값은 명령을 보내기 전, 팔이 아직 멈춰 있을 때 재야 한다.
        force_rise = self.transfer_force_detector(
            force_rise_n, force_rise_baseline_samples
        )
        request = self._types["move"].Request()
        request.pos = [float(value) for value in target]
        request.vel = [float(velocity), self.DR_COND_NONE]
        request.acc = [float(acceleration), self.DR_COND_NONE]
        request.time = 0.0
        request.radius = 0.0
        request.ref = self.DR_BASE
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 1  # ASYNC: one uninterrupted controller trajectory
        response = ServiceCaller.call(
            self._clients["move"], request, 3.0, "move_line"
        )
        if not response.success:
            raise ServiceCallError("move_line was rejected")
        self._wait_motion_complete(
            timeout_s=float(timeout_s),
            monitor_force=monitor_force,
            force_rise=force_rise,
        )
        self._verify_position_target(target, "move_line")


class OnRobotGripper:
    def __init__(self, node, service_name: str, settle_s: float):
        from onrobot_rg_msgs.srv import SetCommand

        self._type = SetCommand
        self._client = node.create_client(SetCommand, service_name)
        self.settle_s = float(settle_s)

    def wait_ready(self, timeout_s: float) -> None:
        if not self._client.wait_for_service(timeout_s):
            raise ServiceCallError("OnRobot gripper service unavailable")

    def command(self, value: str) -> None:
        request = self._type.Request()
        request.command = value
        response = ServiceCaller.call(
            self._client, request, 3.0, f"gripper({value})"
        )
        if not response.success:
            raise ServiceCallError(
                f"gripper command rejected: {response.message}"
            )
        time.sleep(self.settle_s)

    def open(self, width_mm: float | None = None) -> None:
        if width_mm is None:
            self.command("o")
            return
        self.command(rg2_width_mm_to_command(width_mm))

    def close(self, width_mm: float | None = None) -> None:
        if width_mm is None:
            self.command("c")
            return
        self.command(rg2_width_mm_to_command(width_mm))
