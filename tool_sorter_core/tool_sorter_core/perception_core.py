"""공구 검출 결과를 로봇이 쓸 수 있는 기하 정보로 바꾸는 순수 계산층.

마스크, 깊이, 파트 라벨을 입력으로 받아 grasp-ready 여부, 중심점, 깊이 유효성,
LAN 색상 검사 결과를 계산한다. 가능하면 ROS 메시지 의존성을 배제해 테스트하기
쉽게 유지한다.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict

import cv2
import numpy as np

from .grasp_selector import (
    DEFAULT_GRASP_PARTS,
    expected_grasp_part,
    match_grasp_parts,
)

from .lan_color import (
    LanColorConfig,
    draw_lan_color_report,
    inspect_lan_colors,
)
from .schema import Detection


def robust_mask_depth(
    mask: np.ndarray,
    depth_mm: np.ndarray,
    min_valid_ratio: float,
    erode_pixels: int = 2,
) -> tuple[float | None, float]:
    binary = mask.astype(np.uint8)
    if erode_pixels > 0:
        size = erode_pixels * 2 + 1
        eroded = cv2.erode(binary, np.ones((size, size), np.uint8))
        if np.count_nonzero(eroded) >= 25:
            binary = eroded
    selected = depth_mm[binary > 0]
    if selected.size == 0:
        return None, 0.0
    valid = selected[np.isfinite(selected) & (selected > 0.0)]
    ratio = float(valid.size / selected.size)
    if ratio < min_valid_ratio or valid.size == 0:
        return None, ratio
    return float(np.median(valid)), ratio


def grasp_axis(mask: np.ndarray) -> dict | None:
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(contour)
    (center_x, center_y), (width, height), angle = rect
    if width < height:
        angle += 90.0
        width, height = height, width
    return {
        "center_px": [float(center_x), float(center_y)],
        "angle_deg": float(angle % 180.0),
        "grip_width_px": float(height),
        "box": cv2.boxPoints(rect).astype(int).tolist(),
    }


def _overlap_fraction(mask_a: np.ndarray, mask_b: np.ndarray) -> float:
    a = mask_a > 0
    b = mask_b > 0
    intersection = int(np.logical_and(a, b).sum())
    if intersection == 0:
        return 0.0
    smaller_area = min(int(a.sum()), int(b.sum()))
    return intersection / smaller_area if smaller_area else 0.0


#: Depth fields usable for ordering, most trustworthy first. A pair is only
#: ever compared through one of these, never across two of them: the whole-tool
#: median and the handle median measure different parts of the same object and
#: differ by the tool thickness, so mixing them inverts the order.
_ORDERING_DEPTH_KEYS = ("depth_mm", "grasp_depth_mm")


def comparable_depths(
    left: Mapping, right: Mapping
) -> tuple[float, float] | None:
    """Return one depth pair for ``left``/``right`` measured the same way.

    Metal tools lose their whole-mask depth for a few frames at a time, so the
    handle median is kept as a second source. Both sides must come from the
    same source or the comparison is meaningless and ``None`` is returned.
    """

    for key in _ORDERING_DEPTH_KEYS:
        left_depth = left.get(key)
        right_depth = right.get(key)
        if left_depth is not None and right_depth is not None:
            return float(left_depth), float(right_depth)
    return None


def _any_depth(item: Mapping) -> float | None:
    for key in _ORDERING_DEPTH_KEYS:
        depth = item.get(key)
        if depth is not None:
            return float(depth)
    return None


def topological_depth_order(
    items: list[dict], overlap_threshold: float
) -> list[dict]:
    """Respect top-before-bottom overlap constraints; use depth for stable ties."""
    count = len(items)
    outgoing: dict[int, set[int]] = {index: set() for index in range(count)}
    indegree = [0] * count

    for left in range(count):
        for right in range(left + 1, count):
            overlap = _overlap_fraction(items[left]["mask"], items[right]["mask"])
            if overlap < overlap_threshold:
                continue
            items[left]["overlaps_with"].append(items[right]["detection_id"])
            items[right]["overlaps_with"].append(items[left]["detection_id"])
            depths = comparable_depths(items[left], items[right])
            if depths is None:
                continue
            left_depth, right_depth = depths
            upper, lower = (
                (left, right) if left_depth <= right_depth else (right, left)
            )
            if lower not in outgoing[upper]:
                outgoing[upper].add(lower)
                indegree[lower] += 1

    def priority(index: int) -> tuple[bool, float, int]:
        # Only breaks ties between items no overlap edge constrains, so the
        # source mixing that ``comparable_depths`` guards against cannot
        # reorder a stacked pair here.
        depth = _any_depth(items[index])
        return depth is None, depth if depth is not None else math.inf, index

    ready = sorted(
        (index for index, degree in enumerate(indegree) if degree == 0),
        key=priority,
    )
    ordered_indices: list[int] = []
    while ready:
        current = ready.pop(0)
        ordered_indices.append(current)
        for child in outgoing[current]:
            indegree[child] -= 1
            if indegree[child] == 0:
                ready.append(child)
                ready.sort(key=priority)

    if len(ordered_indices) != count:
        remaining = set(range(count)) - set(ordered_indices)
        ordered_indices.extend(sorted(remaining, key=priority))
    return [items[index] for index in ordered_indices]


class ToolDetector:
    def __init__(
        self,
        model,
        confidence: float = 0.4,
        part_confidence: float = 0.15,
        min_valid_depth_ratio: float = 0.3,
        overlap_threshold: float = 0.05,
        part_min_containment: float = 0.5,
        grasp_parts: dict[str, str] | None = None,
        device: str = "",
        image_size: int = 640,
        lan_color_enabled: bool = True,
        lan_color_confidence: float = 0.15,
        lan_color_config: LanColorConfig | None = None,
    ):
        self.model = model
        self.confidence = confidence
        self.part_confidence = part_confidence
        self.min_valid_depth_ratio = min_valid_depth_ratio
        self.overlap_threshold = overlap_threshold
        self.part_min_containment = part_min_containment
        self.grasp_parts = (
            DEFAULT_GRASP_PARTS if grasp_parts is None else grasp_parts
        )
        self.part_classes = set(self.grasp_parts.values())
        self.device = device or None
        self.image_size = image_size
        self.lan_color_enabled = lan_color_enabled
        self.lan_color_confidence = lan_color_confidence
        self.lan_color_config = lan_color_config or LanColorConfig()
        self.last_lan_color_report = {
            "status": "UNKNOWN",
            "ok": None,
            "reason": "not_processed",
            "pairs": [],
            "detections": [],
        }

    def detect(
        self, color: np.ndarray, depth_mm: np.ndarray
    ) -> tuple[list[Detection], np.ndarray]:
        prediction = self.model.predict(
            color,
            conf=min(self.confidence, self.part_confidence),
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )[0]
        if prediction.masks is None:
            self.last_lan_color_report = {
                "status": "UNKNOWN",
                "ok": None,
                "reason": "segmentation_not_detected",
                "pairs": [],
                "detections": [],
            }
            return [], color.copy()

        height, width = depth_mm.shape
        raw_masks = prediction.masks.data.cpu().numpy()
        items: list[dict] = []
        parts: list[dict] = []
        lan_color_items: list[dict] = []

        for index, raw_mask in enumerate(raw_masks):
            box = prediction.boxes[index]
            class_id = int(box.cls.item())
            name = str(self.model.names[class_id])
            confidence = float(box.conf.item())
            is_part = name in self.part_classes
            threshold = self.part_confidence if is_part else self.confidence
            regular_candidate = confidence >= threshold
            lan_color_candidate = (
                self.lan_color_enabled
                and name in {"lan_cable", "lan_port"}
                and confidence >= self.lan_color_confidence
            )
            if not regular_candidate and not lan_color_candidate:
                continue
            mask = cv2.resize(
                raw_mask, (width, height), interpolation=cv2.INTER_NEAREST
            )
            mask = (mask > 0.5).astype(np.uint8)
            item = {
                "detection_id": index,
                "name": name,
                "confidence": confidence,
                "bbox": [float(value) for value in box.xyxy[0].cpu().tolist()],
                "mask": mask,
            }
            if lan_color_candidate:
                lan_color_items.append(item)
            if not regular_candidate:
                continue
            if is_part:
                parts.append(item)
                continue
            depth, ratio = robust_mask_depth(
                mask, depth_mm, self.min_valid_depth_ratio
            )
            item.update(
                {
                    "depth_mm": depth,
                    "valid_depth_ratio": ratio,
                    "overlaps_with": [],
                    "part_name": None,
                    "part_confidence": None,
                    "part_detection_id": None,
                    "part_containment": None,
                    "expected_part_name": expected_grasp_part(
                        name, self.grasp_parts
                    ),
                    "grasp_ready": False,
                    "grasp_mask": mask,
                }
            )
            items.append(item)

        if self.lan_color_enabled:
            self.last_lan_color_report = inspect_lan_colors(
                color,
                lan_color_items,
                self.lan_color_config,
            )
        else:
            self.last_lan_color_report = {
                "status": "UNKNOWN",
                "ok": None,
                "reason": "disabled",
                "pairs": [],
                "detections": [],
            }

        self._match_grasp_parts(items, parts)
        # The handle depth is the fallback ordering source, so it has to exist
        # before the overlap order is decided, not after.
        for item in items:
            grasp_depth = None
            if item["grasp_ready"]:
                grasp_depth, _ = robust_mask_depth(
                    item["grasp_mask"], depth_mm, self.min_valid_depth_ratio
                )
            item["grasp_depth_mm"] = grasp_depth
        items = topological_depth_order(items, self.overlap_threshold)

        detections: list[Detection] = []
        for rank, item in enumerate(items, start=1):
            axis = grasp_axis(item["grasp_mask"]) if item["grasp_ready"] else None
            grasp_depth = item["grasp_depth_mm"]
            detections.append(
                Detection(
                    detection_id=item["detection_id"],
                    name=item["name"],
                    confidence=round(item["confidence"], 4),
                    bbox=[round(value, 1) for value in item["bbox"]],
                    depth_mm=(
                        round(item["depth_mm"], 2)
                        if item["depth_mm"] is not None
                        else None
                    ),
                    grasp_depth_mm=(
                        round(grasp_depth, 2)
                        if grasp_depth is not None
                        else None
                    ),
                    valid_depth_ratio=round(item["valid_depth_ratio"], 3),
                    overlaps_with=item["overlaps_with"],
                    rank=rank,
                    part_name=item["part_name"],
                    part_confidence=item["part_confidence"],
                    part_detection_id=item["part_detection_id"],
                    part_containment=item["part_containment"],
                    expected_part_name=item["expected_part_name"],
                    grasp_ready=item["grasp_ready"],
                    center_px=axis["center_px"] if axis else None,
                    axis_angle_deg=(
                        round(axis["angle_deg"], 2) if axis else None
                    ),
                    grip_width_px=(
                        round(axis["grip_width_px"], 2) if axis else None
                    ),
                    axis_box=axis["box"] if axis else None,
                )
            )
        annotated = draw_detections(color, detections)
        if self.lan_color_enabled:
            annotated = draw_lan_color_report(
                annotated, self.last_lan_color_report
            )
        return detections, annotated

    def _match_grasp_parts(self, items: list[dict], parts: list[dict]) -> None:
        matches = match_grasp_parts(
            items,
            parts,
            min_containment=self.part_min_containment,
            grasp_parts=self.grasp_parts,
        )
        parts_by_id = {int(part["detection_id"]): part for part in parts}
        for target in items:
            match = matches.get(int(target["detection_id"]))
            if match is None:
                continue
            part = parts_by_id[match.part_detection_id]
            target["part_name"] = match.part_name
            target["part_confidence"] = match.part_confidence
            target["part_detection_id"] = match.part_detection_id
            target["part_containment"] = match.containment
            target["grasp_ready"] = True
            target["grasp_mask"] = part["mask"]


def draw_detections(
    image: np.ndarray, detections: list[Detection]
) -> np.ndarray:
    output = image.copy()
    for detection in detections:
        x1, y1, x2, y2 = map(int, detection.bbox)
        valid = detection.grasp_ready and detection.grasp_depth_mm is not None
        color = (52, 211, 153) if valid else (40, 40, 230)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        depth = (
            f"{detection.depth_mm:.0f}mm"
            if detection.depth_mm is not None
            else "DEPTH HOLD"
        )
        part = detection.part_name
        if part is None:
            part = (
                f"missing:{detection.expected_part_name}"
                if detection.expected_part_name
                else "unsupported"
            )
        label = (
            f"#{detection.rank} {detection.name} "
            f"{detection.confidence:.2f} {depth} [{part}]"
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
        if not detection.axis_box or not detection.center_px:
            continue
        box = np.asarray(detection.axis_box, dtype=np.int32)
        cv2.polylines(output, [box], True, (0, 215, 255), 2)
        center_x, center_y = map(int, detection.center_px)
        angle = math.radians(detection.axis_angle_deg or 0.0)
        delta_x = int(55 * math.cos(angle))
        delta_y = int(55 * math.sin(angle))
        cv2.line(
            output,
            (center_x - delta_x, center_y - delta_y),
            (center_x + delta_x, center_y + delta_y),
            (255, 80, 230),
            2,
        )
        cv2.circle(output, (center_x, center_y), 4, (255, 80, 230), -1)
    return output


def detections_as_dicts(detections: list[Detection]) -> list[dict]:
    return [asdict(detection) for detection in detections]
