"""공구 정리 전용 task manager.

``tool_sorter_core.task_manager``를 기반으로, 다섯 공구를 각 고정 위치로
되돌리는 organize 시퀀스만 특화해서 구현한다. ``robot_control.TOOL_CLEANUP``은
이 노드의 start/stop 서비스와 상태 토픽을 통해서만 상호작용한다.
"""

from __future__ import annotations

import math
import time
from collections.abc import Iterable, Mapping, Sequence

from tool_sorter_core.grasp_selector import SORTABLE_TOOL_NAMES
from tool_sorter_core.perception_core import comparable_depths
from tool_sorter_core.schema import Detection
from tool_sorter_core.task_manager import ToolSortTaskManager


REQUIRED_TOOL_NAMES = tuple(SORTABLE_TOOL_NAMES)
INTEGRATION_SERVICE_PREFIX = "/integration/tool_sorter"


def place_pose_parameter(tool_name: str) -> str:
    """Return the ROS parameter containing one tool's absolute Base pose."""

    if tool_name not in REQUIRED_TOOL_NAMES:
        raise ValueError(f"Unsupported tool name: {tool_name}")
    return f"place_pose_{tool_name}"


def validate_absolute_place_poses(
    poses: Mapping[str, Sequence[float]],
) -> list[str]:
    """Return validation errors for five class-specific Base-frame poses."""

    errors: list[str] = []
    for tool_name in REQUIRED_TOOL_NAMES:
        values = list(poses.get(tool_name, []))
        if len(values) != 6:
            errors.append(
                f"{place_pose_parameter(tool_name)} must contain "
                "X Y Z RX RY RZ"
            )
            continue
        if not all(math.isfinite(float(value)) for value in values):
            errors.append(
                f"{place_pose_parameter(tool_name)} contains NaN or Inf"
            )
    return errors


def unsorted_tool_names(completed_names: Iterable[str]) -> list[str]:
    """Return required tool classes this job has not placed yet."""

    completed = set(completed_names)
    return [name for name in REQUIRED_TOOL_NAMES if name not in completed]


#: A stacked tool sits roughly one tool thickness above the one beneath it.
#: Anything closer than this is depth noise, not a real height difference, so
#: it must not decide which of two overlapping tools is on top.
OCCLUSION_DEPTH_MARGIN_MM = 5.0


def _depth_fields(item: Detection) -> dict[str, float | None]:
    """Expose one detection's depth sources to ``comparable_depths``."""

    return {
        "depth_mm": item.depth_mm,
        "grasp_depth_mm": item.grasp_depth_mm,
    }


def camera_depth_order(
    detections: Iterable[Detection],
    completed_names: Iterable[str] = (),
) -> list[Detection]:
    """Order remaining tools from nearest to farthest from the camera.

    Ordering is keyed on ``grasp_depth_mm`` alone. ``depth_mm`` is the median
    over the whole tool mask and ``grasp_depth_mm`` the median over the handle
    only; they differ by the tool thickness, so falling back from one to the
    other reorders tools whenever a reflective tool loses its whole-mask depth
    for a frame. Every detection reaching this point has passed the
    ``grasp_depth_mm is None`` filter in ``_eligible_detections``.
    """

    completed = set(completed_names)

    def depth(item: Detection) -> tuple[bool, float, int]:
        value = item.grasp_depth_mm
        return (
            value is None,
            float(value) if value is not None else math.inf,
            int(item.detection_id),
        )

    return sorted(
        (
            item
            for item in detections
            if item.name in REQUIRED_TOOL_NAMES
            and item.name not in completed
        ),
        key=depth,
    )


def buried_last(
    ordered: Iterable[Detection],
    scene_detections: Iterable[Detection],
    margin_mm: float = OCCLUSION_DEPTH_MARGIN_MM,
) -> list[Detection]:
    """Send tools that something nearer is lying on top of to the back.

    ``camera_depth_order`` only ranks graspable tools against each other, but a
    tool pinned under a *non*-graspable one (handle hidden, so no part match)
    stays at the front of that ranking and gets pulled out from underneath.
    Overlap partners are therefore re-checked against the full scene.

    This demotes rather than drops: if every remaining tool is buried the job
    still makes progress instead of stalling until ``empty_scene_timeout_s``.
    """

    by_id = {
        int(item.detection_id): item for item in scene_detections
    }

    def is_buried(item: Detection) -> bool:
        for other_id in item.overlaps_with:
            other = by_id.get(int(other_id))
            if other is None:
                continue
            depths = comparable_depths(
                _depth_fields(item), _depth_fields(other)
            )
            if depths is None:
                continue
            own_depth, other_depth = depths
            if own_depth - other_depth > margin_mm:
                return True
        return False

    free: list[Detection] = []
    buried: list[Detection] = []
    for item in ordered:
        (buried if is_buried(item) else free).append(item)
    return free + buried


class AutonomousToolSortManager(ToolSortTaskManager):
    """Sort whichever of the five tools are present, GUI or not.

    The job is driven by one start request, so it also runs headless with
    ``use_gui:=false`` for an external node manager.
    """

    def __init__(self) -> None:
        self._picked_names: set[str] = set()
        self._empty_streak_started_at = 0.0
        self._selected_place_pose: list[float] | None = None
        self._integration_status_publisher = None
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
        self._integration_status_publisher = self.node.create_publisher(
            String,
            f"{INTEGRATION_SERVICE_PREFIX}/status",
            status_qos,
        )
        self.node.create_service(
            Trigger,
            f"{INTEGRATION_SERVICE_PREFIX}/organize",
            self._on_start,
        )
        self.node.create_service(
            Trigger,
            f"{INTEGRATION_SERVICE_PREFIX}/pause",
            self._on_pause,
        )
        self.node.create_service(
            Trigger,
            f"{INTEGRATION_SERVICE_PREFIX}/stop",
            self._on_stop,
        )
        self._publish_integration_status()

    def _declare_parameters(self) -> None:
        from rclpy.parameter import Parameter

        super()._declare_parameters()
        self.node.declare_parameter("empty_scene_timeout_s", 3.0)
        for tool_name in REQUIRED_TOOL_NAMES:
            self.node.declare_parameter(
                place_pose_parameter(tool_name),
                Parameter.Type.DOUBLE_ARRAY,
            )

    def _absolute_place_poses(self) -> dict[str, list[float]]:
        return {
            name: [
                float(value)
                for value in self._list_parameter(
                    place_pose_parameter(name)
                )
            ]
            for name in REQUIRED_TOOL_NAMES
        }

    def _validate_start(self) -> str | None:
        problem = super()._validate_start()
        if problem:
            return problem
        if self.execution_mode != "pick_place":
            return (
                "자동 정리 패키지는 execution_mode=pick_place만 "
                "허용합니다"
            )
        if not bool(self._parameter("auto_scan_motion")):
            return "자동 정리는 auto_scan_motion=true가 필요합니다"
        if bool(self._parameter("local_rescan_enabled")):
            return (
                "depth 재정렬을 위해 local_rescan_enabled=false로 설정하고 "
                "매 Pick 후 Bird view로 복귀해야 합니다"
            )
        timeout_s = float(self._parameter("empty_scene_timeout_s"))
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            return "empty_scene_timeout_s는 0보다 큰 값이어야 합니다"
        configured = [
            str(value) for value in self._list_parameter("target_names")
        ]
        if set(configured) != set(REQUIRED_TOOL_NAMES):
            return (
                "target_names에는 지정된 공구 5종이 정확히 "
                "들어가야 합니다"
            )
        if int(self._parameter("max_picks")) != len(REQUIRED_TOOL_NAMES):
            return f"max_picks는 {len(REQUIRED_TOOL_NAMES)}여야 합니다"
        errors = validate_absolute_place_poses(
            self._absolute_place_poses()
        )
        if errors:
            return (
                "절대 배치 좌표를 실측해 YAML에 입력해야 합니다: "
                + "; ".join(errors)
            )
        return None

    def _queue_command(self, command: str) -> bool:
        """Reset one autonomous job before exposing it to the worker."""

        with self._command_lock:
            if self._active.is_set() or self._pending_command is not None:
                return False
            if command == "session":
                self._completed = 0
                self._picked_names.clear()
                self._empty_streak_started_at = 0.0
            self._pending_command = command
            self._active.set()
            return True

    def _eligible_detections(self, scene):
        eligible = super()._eligible_detections(scene)
        ordered = camera_depth_order(eligible, self._picked_names)
        return buried_last(ordered, scene.detections)

    def _empty_scene_confirmed(
        self,
        empty_count: int,
        scene_sequence: int,
    ) -> bool:
        """Confirm an empty cell by elapsed time instead of frame count.

        Metal tools lose depth for a few frames at a time, so a frame counter
        can end the job while a tool is still on the table. Hold the Bird view
        for a measured interval and finish only if nothing becomes graspable.
        """

        timeout_s = float(self._parameter("empty_scene_timeout_s"))
        now = time.monotonic()
        if empty_count <= 1:
            self._empty_streak_started_at = now
        elapsed = now - self._empty_streak_started_at
        self._publish_status(
            "WAITING_SCENE",
            "파지 가능한 공구 없음; "
            f"{elapsed:.1f}/{timeout_s:.1f}초 재확인 중 (미정리: "
            + ", ".join(unsorted_tool_names(self._picked_names))
            + ")",
            target="",
            scene_sequence=scene_sequence,
        )
        return elapsed >= timeout_s

    def _list_parameter(self, name: str) -> list:
        if name == "place_pose" and self._selected_place_pose is not None:
            return list(self._selected_place_pose)
        return super()._list_parameter(name)

    def _execute_pick_place(
        self,
        detection: Detection,
        grasp: dict[str, float],
    ) -> None:
        poses = self._absolute_place_poses()
        self._selected_place_pose = poses[detection.name]
        try:
            super()._execute_pick_place(detection, grasp)
        finally:
            self._selected_place_pose = None

        self._picked_names.add(detection.name)
        if set(self._picked_names) == set(REQUIRED_TOOL_NAMES):
            self._move_home_to_bird(
                "다섯 공구 배치 완료; 최종 Bird view로 복귀"
            )

    def _publish_integration_status(self) -> None:
        publisher = self._integration_status_publisher
        if publisher is None:
            return
        from std_msgs.msg import String

        output = String()
        output.data = self._status.to_json()
        publisher.publish(output)

    def _completion_message(self) -> str:
        """Describe a finished job, naming tools never seen in the cell."""

        missing = unsorted_tool_names(self._picked_names)
        if not missing:
            return f"{len(REQUIRED_TOOL_NAMES)}종 모두 정리 완료"
        return (
            f"{len(self._picked_names)}종 정리 완료; 작업장에서 확인되지 "
            "않은 공구: " + ", ".join(missing)
        )

    def _publish_status(
        self,
        state: str,
        message: str,
        target: str | None = None,
        scene_sequence: int | None = None,
    ) -> None:
        if state == "COMPLETE":
            # 작업장에 없던 공구는 실패가 아니다. 있던 공구를 모두 정리했으면
            # COMPLETE로 끝내고, 못 본 공구 이름만 메시지에 남긴다.
            message = self._completion_message()
        # GUI 체크리스트가 메시지 문자열을 파싱하지 않도록 진행 상황을 그대로
        # 싣는다. REQUIRED_TOOL_NAMES 순서를 유지해 화면 순서가 흔들리지 않는다.
        self._status.completed_names = [
            name for name in REQUIRED_TOOL_NAMES if name in self._picked_names
        ]
        super()._publish_status(
            state,
            message,
            target=target,
            scene_sequence=scene_sequence,
        )
        self._publish_integration_status()


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import (
        ExternalShutdownException,
        MultiThreadedExecutor,
    )

    rclpy.init(args=args)
    manager = AutonomousToolSortManager()
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
