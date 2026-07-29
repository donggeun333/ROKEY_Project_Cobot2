"""LAN 케이블 테이프와 LAN 포트의 색상 비교를 위한 순수 OpenCV 코어.

ROS2나 Ultralytics에 의존하지 않으므로 단위 테스트와 오프라인 튜닝에 사용할 수
있다. OpenCV HSV 범위(H: 0~179, S/V: 0~255)를 기준으로 한다.
"""

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Optional

import cv2
import numpy as np


HUE_PERIOD = 180.0


@dataclass(frozen=True)
class DominantHSV:
    """마스크 안의 고채도 픽셀에서 얻은 대표 색상과 신뢰도."""

    hue: float
    saturation: float
    value: float
    object_pixels: int
    candidate_pixels: int
    dominant_pixels: int
    concentration: float

    def to_dict(self) -> dict:
        result = asdict(self)
        result["color_name"] = hue_to_korean_name(self.hue)
        return result


@dataclass(frozen=True)
class HSVComparison:
    """두 대표색의 거리와 최종 일치 판정."""

    matched: bool
    score: float
    hue_distance: float
    saturation_distance: float
    value_distance: float

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class LanColorConfig:
    """LAN 색상 검사 임계값."""

    min_saturation: int = 45
    min_value: int = 25
    max_value: int = 245
    cable_tip_fraction: float = 0.35
    max_hue_distance: float = 12.0
    max_saturation_distance: float = 130.0
    max_value_distance: float = 190.0
    max_pair_gap_ratio: float = 0.04
    duplicate_iou: float = 0.55


def circular_hue_distance(first: float, second: float) -> float:
    """OpenCV Hue의 0/179 경계를 고려한 최단 거리."""

    direct = abs(float(first) - float(second)) % HUE_PERIOD
    return min(direct, HUE_PERIOD - direct)


def _circular_hue_mean(hues: np.ndarray) -> float:
    angles = hues.astype(np.float64) / HUE_PERIOD * (2.0 * math.pi)
    angle = math.atan2(np.sin(angles).mean(), np.cos(angles).mean())
    return float((angle % (2.0 * math.pi)) * HUE_PERIOD / (2.0 * math.pi))


def _smooth_circular_histogram(histogram: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return histogram.astype(np.float64)
    kernel_size = radius * 2 + 1
    padded = np.concatenate((histogram[-radius:], histogram, histogram[:radius]))
    return np.convolve(padded, np.ones(kernel_size), mode="valid")


def extract_dominant_hsv(
    bgr: np.ndarray,
    mask: np.ndarray,
    *,
    min_saturation: int = 45,
    min_value: int = 25,
    max_value: int = 245,
    saturation_percentile: float = 60.0,
    hue_window: int = 8,
    histogram_radius: int = 4,
    erode_pixels: int = 1,
    min_candidate_pixels: int = 20,
    min_concentration: float = 0.35,
) -> Optional[DominantHSV]:
    """마스크 내부에서 테이프/포트 플라스틱의 대표 HSV를 추출한다.

    회색 케이블과 흰색 공유기 외장은 채도가 낮으므로 제외하고, 남은 픽셀 중
    채도가 높은 쪽을 사용한다. Hue 히스토그램의 가장 큰 봉우리 주변만 다시
    모아 배경이나 반사광의 영향을 줄인다.

    색 픽셀이 부족하거나 Hue가 한 군집으로 모이지 않으면 ``None``을 반환한다.
    이런 경우 공정 NG로 단정하지 말고 재촬영/조명 확인 대상으로 처리해야 한다.
    """

    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr은 HxWx3 컬러 이미지여야 합니다.")
    if mask is None or mask.shape != bgr.shape[:2]:
        raise ValueError("mask 크기는 bgr 이미지의 HxW와 같아야 합니다.")
    if not 0.0 <= saturation_percentile <= 100.0:
        raise ValueError("saturation_percentile은 0~100 범위여야 합니다.")

    work_mask = (mask > 0).astype(np.uint8)
    if erode_pixels > 0:
        kernel_size = erode_pixels * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        eroded = cv2.erode(work_mask, kernel)
        # 가느다란 케이블 마스크가 전부 사라지면 원본 마스크를 사용한다.
        if np.count_nonzero(eroded) >= min_candidate_pixels:
            work_mask = eroded

    object_pixels = int(np.count_nonzero(work_mask))
    if object_pixels < min_candidate_pixels:
        return None

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    pixels = hsv[work_mask > 0]
    valid = pixels[
        (pixels[:, 1] >= min_saturation)
        & (pixels[:, 2] >= min_value)
        & (pixels[:, 2] <= max_value)
    ]
    if len(valid) < min_candidate_pixels:
        return None

    saturation_cutoff = np.percentile(valid[:, 1], saturation_percentile)
    candidates = valid[valid[:, 1] >= saturation_cutoff]
    if len(candidates) < min_candidate_pixels:
        return None

    histogram = np.bincount(
        candidates[:, 0],
        weights=candidates[:, 1].astype(np.float64),
        minlength=int(HUE_PERIOD),
    )
    histogram = _smooth_circular_histogram(histogram, histogram_radius)
    peak_hue = int(np.argmax(histogram))

    distances = np.array(
        [circular_hue_distance(hue, peak_hue) for hue in candidates[:, 0]],
        dtype=np.float32,
    )
    dominant = candidates[distances <= hue_window]
    concentration = float(len(dominant) / len(candidates))
    if len(dominant) < min_candidate_pixels or concentration < min_concentration:
        return None

    return DominantHSV(
        hue=round(_circular_hue_mean(dominant[:, 0]), 2),
        saturation=round(float(np.median(dominant[:, 1])), 2),
        value=round(float(np.median(dominant[:, 2])), 2),
        object_pixels=object_pixels,
        candidate_pixels=int(len(candidates)),
        dominant_pixels=int(len(dominant)),
        concentration=round(concentration, 3),
    )


def compare_hsv(
    first: DominantHSV,
    second: DominantHSV,
    *,
    max_hue_distance: float = 12.0,
    max_saturation_distance: float = 100.0,
    max_value_distance: float = 160.0,
) -> HSVComparison:
    """두 색을 비교한다.

    조명 변화에 민감한 V와 어느 정도 변하는 S보다 Hue를 가장 강하게 본다.
    기본 Hue 허용치 12는 OpenCV 스케일에서 실제 색상환 약 24도에 해당한다.
    """

    if min(max_hue_distance, max_saturation_distance, max_value_distance) <= 0:
        raise ValueError("HSV 허용 거리는 모두 0보다 커야 합니다.")

    hue_distance = circular_hue_distance(first.hue, second.hue)
    saturation_distance = abs(first.saturation - second.saturation)
    value_distance = abs(first.value - second.value)
    score = math.sqrt(
        (hue_distance / max_hue_distance) ** 2
        + 0.20 * (saturation_distance / max_saturation_distance) ** 2
        + 0.05 * (value_distance / max_value_distance) ** 2
    )
    matched = (
        hue_distance <= max_hue_distance
        and saturation_distance <= max_saturation_distance
        and value_distance <= max_value_distance
    )
    return HSVComparison(
        matched=matched,
        score=round(float(score), 3),
        hue_distance=round(float(hue_distance), 2),
        saturation_distance=round(float(saturation_distance), 2),
        value_distance=round(float(value_distance), 2),
    )


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    first_bool = first > 0
    second_bool = second > 0
    union = np.logical_or(first_bool, second_bool).sum()
    if union == 0:
        return 0.0
    return float(np.logical_and(first_bool, second_bool).sum() / union)


def minimum_mask_gap(first: np.ndarray, second: np.ndarray) -> float:
    """두 마스크 사이의 최소 픽셀 거리. 접촉하거나 겹치면 0."""

    first_bool = first > 0
    second_bool = second > 0
    if first_bool.shape != second_bool.shape:
        raise ValueError("두 마스크의 크기가 같아야 합니다.")
    if not first_bool.any() or not second_bool.any():
        return math.inf
    if np.logical_and(first_bool, second_bool).any():
        return 0.0

    distance_to_second = cv2.distanceTransform(
        (~second_bool).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    )
    return float(distance_to_second[first_bool].min())


def nearest_mask_region(
    object_mask: np.ndarray,
    reference_mask: np.ndarray,
    *,
    fraction: float,
    min_pixels: int = 20,
) -> np.ndarray:
    """객체 마스크 중 기준 마스크에 가장 가까운 일부 영역만 반환한다.

    LAN 케이블에서는 포트에 가장 가까운 끝부분이 RJ45 팁과 바로 뒤의 색 테이프
    영역이다. 전체 케이블의 다른 색이나 배경 누출이 대표색을 차지하지 않도록
    이 영역만 HSV 추출에 사용한다.
    """

    raw_object_bool = object_mask > 0
    reference_bool = reference_mask > 0
    if raw_object_bool.shape != reference_bool.shape:
        raise ValueError("object_mask와 reference_mask의 크기가 같아야 합니다.")
    if not 0.0 < fraction <= 1.0:
        raise ValueError("fraction은 0보다 크고 1 이하여야 합니다.")

    # 겹친 예측 픽셀은 포트 자체의 색이 케이블 색으로 섞여 항상 일치하는
    # false OK를 만들 수 있으므로 케이블 ROI에서 제외한다.
    object_bool = np.logical_and(raw_object_bool, ~reference_bool)
    result = np.zeros(object_bool.shape, dtype=np.uint8)
    object_indices = np.flatnonzero(object_bool)
    if object_indices.size == 0 or not reference_bool.any():
        return result

    distance_to_reference = cv2.distanceTransform(
        (~reference_bool).astype(np.uint8),
        cv2.DIST_L2,
        cv2.DIST_MASK_PRECISE,
    )
    target_pixels = min(
        int(object_indices.size),
        max(int(min_pixels), int(math.ceil(object_indices.size * fraction))),
    )
    if target_pixels == object_indices.size:
        result.flat[object_indices] = 1
        return result

    distances = distance_to_reference.flat[object_indices]
    nearest_indices = np.argpartition(distances, target_pixels - 1)[:target_pixels]
    result.flat[object_indices[nearest_indices]] = 1
    return result


def pair_by_minimum_gap(
    cables: Iterable[dict],
    ports: Iterable[dict],
    *,
    max_gap_pixels: float,
) -> tuple[list[tuple[dict, dict, float]], list[dict], list[dict]]:
    """케이블과 포트를 최소 마스크 거리로 일대일 매칭한다.

    각 dict에는 ``mask``가 있어야 한다. 포트가 여러 개여도 실제 케이블 끝과
    가장 가까운 포트를 선택하므로, 단순히 같은 색 포트를 찾아 OK 처리하는
    잘못을 방지한다.
    """

    cables = list(cables)
    ports = list(ports)
    candidates = []
    for cable_index, cable in enumerate(cables):
        for port_index, port in enumerate(ports):
            gap = minimum_mask_gap(cable["mask"], port["mask"])
            candidates.append((gap, cable_index, port_index))
    candidates.sort(key=lambda item: item[0])

    used_cables = set()
    used_ports = set()
    pairs = []
    for gap, cable_index, port_index in candidates:
        if gap > max_gap_pixels:
            break
        if cable_index in used_cables or port_index in used_ports:
            continue
        used_cables.add(cable_index)
        used_ports.add(port_index)
        pairs.append((cables[cable_index], ports[port_index], round(float(gap), 2)))

    unpaired_cables = [
        cable for index, cable in enumerate(cables) if index not in used_cables
    ]
    unpaired_ports = [
        port for index, port in enumerate(ports) if index not in used_ports
    ]
    return pairs, unpaired_cables, unpaired_ports


def hue_to_korean_name(hue: float) -> str:
    """로그와 TTS 설명용의 거친 색상명. 실제 판정은 연속 HSV 거리로 한다."""

    hue = float(hue) % HUE_PERIOD
    if hue < 5 or hue >= 170:
        return "빨강"
    if hue < 15:
        return "주황"
    if hue < 35:
        return "노랑"
    if hue < 85:
        return "초록"
    if hue < 100:
        return "청록"
    if hue < 130:
        return "파랑"
    if hue < 155:
        return "보라"
    return "자홍"


def _mask_center(mask: np.ndarray) -> tuple[int, int]:
    moments = cv2.moments(mask.astype(np.uint8), binaryImage=True)
    if moments["m00"] > 0:
        return (
            int(round(moments["m10"] / moments["m00"])),
            int(round(moments["m01"] / moments["m00"])),
        )
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return (0, 0)
    return (int(round(xs.mean())), int(round(ys.mean())))


def _deduplicate(detections: list[dict], iou_threshold: float) -> list[dict]:
    kept = []
    for detection in sorted(
        detections, key=lambda item: item["confidence"], reverse=True
    ):
        duplicate = any(
            detection["name"] == previous["name"]
            and mask_iou(detection["mask"], previous["mask"]) >= iou_threshold
            for previous in kept
        )
        if not duplicate:
            kept.append(detection)
    return kept


def _public_detection(detection: dict) -> dict:
    feature = detection.get("feature")
    return {
        "detection_id": int(detection["detection_id"]),
        "name": detection["name"],
        "confidence": round(float(detection["confidence"]), 3),
        "bbox": [round(float(value), 1) for value in detection["bbox"]],
        "center": list(detection["center"]),
        "color_region": detection.get("color_region", "full_mask"),
        "color_roi_pixels": int(detection.get("color_roi_pixels", 0)),
        "hsv": feature.to_dict() if feature is not None else None,
    }


def _unknown_report(reason: str, detections: list[dict]) -> dict:
    return {
        "status": "UNKNOWN",
        "ok": None,
        "reason": reason,
        "pairs": [],
        "detections": [_public_detection(item) for item in detections],
    }


def _extract_configured_hsv(
    bgr: np.ndarray,
    mask: np.ndarray,
    config: LanColorConfig,
) -> Optional[DominantHSV]:
    return extract_dominant_hsv(
        bgr,
        mask,
        min_saturation=config.min_saturation,
        min_value=config.min_value,
        max_value=config.max_value,
    )


def inspect_lan_colors(
    bgr: np.ndarray,
    detections: Iterable[dict],
    config: LanColorConfig | None = None,
) -> dict:
    """이미 수행된 YOLO 검출 마스크를 재사용해 LAN 공정 색상을 판정한다.

    입력 detection에는 ``detection_id``, ``name``, ``confidence``, ``bbox``,
    ``mask``가 있어야 한다. ``lan_cable``과 ``lan_port``만 사용한다.
    """

    config = config or LanColorConfig()
    prepared = []
    for source in detections:
        if source["name"] not in {"lan_cable", "lan_port"}:
            continue
        detection = dict(source)
        detection["mask"] = (source["mask"] > 0).astype(np.uint8)
        detection["center"] = _mask_center(detection["mask"])
        is_port = detection["name"] == "lan_port"
        detection["feature"] = (
            _extract_configured_hsv(bgr, detection["mask"], config)
            if is_port
            else None
        )
        detection["color_region"] = "full_mask" if is_port else "awaiting_pair"
        detection["color_roi_pixels"] = (
            int(np.count_nonzero(detection["mask"])) if is_port else 0
        )
        prepared.append(detection)
    prepared = _deduplicate(prepared, config.duplicate_iou)

    cables = [item for item in prepared if item["name"] == "lan_cable"]
    ports = [item for item in prepared if item["name"] == "lan_port"]
    if not cables:
        return _unknown_report("lan_cable_not_detected", prepared)
    if not ports:
        return _unknown_report("lan_port_not_detected", prepared)

    height, width = bgr.shape[:2]
    max_gap = math.hypot(width, height) * config.max_pair_gap_ratio
    pairs, unpaired_cables, _unpaired_ports = pair_by_minimum_gap(
        cables,
        ports,
        max_gap_pixels=max_gap,
    )
    if not pairs:
        return _unknown_report("cable_port_relation_uncertain", prepared)
    if unpaired_cables:
        return _unknown_report("unpaired_cable_detected", prepared)

    for cable, port, _gap in pairs:
        cable_tip_mask = nearest_mask_region(
            cable["mask"],
            port["mask"],
            fraction=config.cable_tip_fraction,
        )
        cable["feature"] = _extract_configured_hsv(
            bgr,
            cable_tip_mask,
            config,
        )
        cable["color_region"] = "tip_nearest_port"
        cable["color_roi_pixels"] = int(np.count_nonzero(cable_tip_mask))

    if any(
        cable["feature"] is None or port["feature"] is None
        for cable, port, _gap in pairs
    ):
        # 공간적으로 가장 가까운 실제 상대의 색을 못 읽었을 때 다른 포트의
        # 색으로 갈아타면 오판이므로 보류한다.
        return _unknown_report("paired_color_unavailable", prepared)

    pair_reports = []
    for cable, port, gap in pairs:
        comparison = compare_hsv(
            cable["feature"],
            port["feature"],
            max_hue_distance=config.max_hue_distance,
            max_saturation_distance=config.max_saturation_distance,
            max_value_distance=config.max_value_distance,
        )
        pair_reports.append(
            {
                "cable_detection_id": int(cable["detection_id"]),
                "port_detection_id": int(port["detection_id"]),
                "gap_pixels": gap,
                "matched": comparison.matched,
                "cable_color_region": cable["color_region"],
                "cable_color_roi_pixels": cable["color_roi_pixels"],
                "comparison": comparison.to_dict(),
                "thresholds": {
                    "max_hue_distance": config.max_hue_distance,
                    "max_saturation_distance": config.max_saturation_distance,
                    "max_value_distance": config.max_value_distance,
                },
                "cable_hsv": cable["feature"].to_dict(),
                "port_hsv": port["feature"].to_dict(),
            }
        )

    ok = all(pair["matched"] for pair in pair_reports)
    return {
        "status": "OK" if ok else "NG",
        "ok": ok,
        "reason": "all_pairs_match" if ok else "color_mismatch",
        "pairs": pair_reports,
        "detections": [_public_detection(item) for item in prepared],
    }


def format_lan_color_log(report: dict) -> str:
    """ROS logger에서 바로 읽을 수 있는 간단한 LAN 판정 문자열."""

    status = str(report.get("status", "UNKNOWN"))
    reason = str(report.get("reason", ""))
    parts = [f"status={status}", f"reason={reason}"]
    for pair in report.get("pairs", []):
        cable = pair["cable_hsv"]
        port = pair["port_hsv"]
        comparison = pair["comparison"]
        thresholds = pair["thresholds"]
        parts.append(
            "cable#{cable_id} {cable_name}"
            "(H={cable_hue:.1f},S={cable_sat:.1f},V={cable_val:.1f}) -> "
            "port#{port_id} {port_name}"
            "(H={port_hue:.1f},S={port_sat:.1f},V={port_val:.1f}), "
            "dH={hue_distance:.1f},dS={sat_distance:.1f},"
            "dV={val_distance:.1f}, "
            "limits(dH<={max_hue:.1f},dS<={max_sat:.1f},dV<={max_val:.1f}), "
            "match={matched}, gap={gap:.1f}px".format(
                cable_id=pair["cable_detection_id"],
                cable_name=cable["color_name"],
                cable_hue=float(cable["hue"]),
                cable_sat=float(cable["saturation"]),
                cable_val=float(cable["value"]),
                port_id=pair["port_detection_id"],
                port_name=port["color_name"],
                port_hue=float(port["hue"]),
                port_sat=float(port["saturation"]),
                port_val=float(port["value"]),
                hue_distance=float(comparison["hue_distance"]),
                sat_distance=float(comparison["saturation_distance"]),
                val_distance=float(comparison["value_distance"]),
                max_hue=float(thresholds["max_hue_distance"]),
                max_sat=float(thresholds["max_saturation_distance"]),
                max_val=float(thresholds["max_value_distance"]),
                matched=bool(pair["matched"]),
                gap=float(pair["gap_pixels"]),
            )
        )
    return " | ".join(parts)


def draw_lan_color_report(image: np.ndarray, report: dict) -> np.ndarray:
    """기존 sorter annotated 영상 위에 HSV와 케이블–포트 연결선을 추가."""

    output = image.copy()
    detections = {
        int(item["detection_id"]): item for item in report.get("detections", [])
    }
    for detection in detections.values():
        x1, y1, _x2, _y2 = map(int, detection["bbox"])
        hsv = detection["hsv"]
        if hsv is None:
            text = f"{detection['name']} HSV:unknown"
        else:
            text = (
                f"{detection['name']} {hsv['color_name']} "
                f"H{hsv['hue']:.0f} S{hsv['saturation']:.0f} V{hsv['value']:.0f}"
            )
        cv2.putText(
            output,
            text,
            (x1, max(38, y1 + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 220, 0),
            1,
            cv2.LINE_AA,
        )

    for pair in report.get("pairs", []):
        cable = detections[pair["cable_detection_id"]]
        port = detections[pair["port_detection_id"]]
        color = (0, 220, 0) if pair["matched"] else (0, 0, 255)
        cv2.line(
            output,
            tuple(cable["center"]),
            tuple(port["center"]),
            color,
            3,
            cv2.LINE_AA,
        )

    status = report.get("status", "UNKNOWN")
    colors = {
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
        colors.get(status, colors["UNKNOWN"]),
        2,
        cv2.LINE_AA,
    )
    return output


class VerdictStabilizer:
    """연속 프레임 판정과 검사 사이 UNKNOWN 구간을 관리한다."""

    def __init__(self, stable_frames: int = 3, reset_after_seconds: float = 2.0):
        if stable_frames < 1:
            raise ValueError("stable_frames는 1 이상이어야 합니다.")
        self.stable_frames = stable_frames
        self.reset_after_seconds = max(float(reset_after_seconds), 0.0)
        self.candidate_status: str | None = None
        self.candidate_count = 0
        self.last_emitted_status: str | None = None
        self.unknown_since: float | None = None

    def update(self, status: str, now: float) -> dict:
        if status not in {"OK", "NG"}:
            self.candidate_status = None
            self.candidate_count = 0
            if self.unknown_since is None:
                self.unknown_since = now
            elif now - self.unknown_since >= self.reset_after_seconds:
                self.last_emitted_status = None
            return {"stable": False, "changed": False, "frames": 0}

        self.unknown_since = None
        if status == self.candidate_status:
            self.candidate_count += 1
        else:
            self.candidate_status = status
            self.candidate_count = 1
        stable = self.candidate_count >= self.stable_frames
        changed = stable and status != self.last_emitted_status
        if changed:
            self.last_emitted_status = status
        return {
            "stable": stable,
            "changed": changed,
            "frames": self.candidate_count,
        }


def speech_for_status(status: str) -> str | None:
    if status == "OK":
        return "랜선과 랜포트 색상이 일치합니다. 올바른 공정입니다."
    if status == "NG":
        return "랜선과 랜포트 색상이 일치하지 않습니다. 잘못된 공정입니다."
    return None
