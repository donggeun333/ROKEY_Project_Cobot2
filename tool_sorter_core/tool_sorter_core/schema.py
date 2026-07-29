"""공구 정리/전달 패키지들이 공유하는 직렬화 스키마.

Scene, Detection, TaskStatus를 dataclass로 고정해 두면 perception, dashboard,
task manager가 같은 필드 이름과 의미를 공유할 수 있다. 제출물 검토 시 데이터
계약을 이해하기 가장 좋은 진입점이기도 하다.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class Detection:
    detection_id: int
    name: str
    confidence: float
    bbox: list[float]
    depth_mm: float | None
    grasp_depth_mm: float | None
    valid_depth_ratio: float
    overlaps_with: list[int] = field(default_factory=list)
    rank: int = 0
    part_name: str | None = None
    part_confidence: float | None = None
    part_detection_id: int | None = None
    part_containment: float | None = None
    expected_part_name: str | None = None
    grasp_ready: bool = False
    center_px: list[float] | None = None
    axis_angle_deg: float | None = None
    grip_width_px: float | None = None
    axis_box: list[list[int]] | None = None

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Detection":
        fields = cls.__dataclass_fields__
        return cls(**{key: val for key, val in value.items() if key in fields})


@dataclass
class Scene:
    sequence: int
    stamp: float
    frame_id: str
    image_width: int
    image_height: int
    inference_ms: float
    fps: float
    detections: list[Detection] = field(default_factory=list)
    stamp_sec: int | None = None
    stamp_nanosec: int | None = None

    @property
    def stamp_nanoseconds(self) -> int:
        """Return the original timestamp without float precision loss."""

        if self.stamp_sec is not None and self.stamp_nanosec is not None:
            return int(self.stamp_sec) * 1_000_000_000 + int(
                self.stamp_nanosec
            )
        # Backward compatibility for scene publishers from before the exact
        # sec/nanosec fields were introduced.
        return int(round(float(self.stamp) * 1_000_000_000))

    def to_json(self) -> str:
        return json.dumps(
            asdict(self),
            ensure_ascii=False,
            separators=(",", ":"),
        )

    @classmethod
    def from_json(cls, value: str) -> "Scene":
        raw = json.loads(value)
        raw["detections"] = [
            Detection.from_dict(item) for item in raw.get("detections", [])
        ]
        return cls(**raw)


@dataclass
class TaskStatus:
    state: str = "IDLE"
    message: str = "대기 중"
    execution_mode: str = "point"
    completed: int = 0
    current_target: str = ""
    scene_sequence: int = -1
    scan_scope: str = "none"
    # 세션이 시작하면서 관측 자세로 다시 이동/촬영하는지 여부. GUI는 이 값으로
    # 지금 화면의 검출이 곧 작업 대상인지 판단한다.
    auto_scan_motion: bool = False
    # 이번 작업에서 배치까지 끝낸 공구 클래스. 자동 정리 GUI가 진행 체크리스트를
    # 문자열 파싱 없이 그리기 위해 사용한다.
    completed_names: list[str] = field(default_factory=list)
    updated_at: float = 0.0

    def to_json(self) -> str:
        return json.dumps(
            asdict(self),
            ensure_ascii=False,
            separators=(",", ":"),
        )
