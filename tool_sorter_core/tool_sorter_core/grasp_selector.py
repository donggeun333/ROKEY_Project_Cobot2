"""공구별 파지 가능 조건과 그립 대상 파트 규칙.

perception은 "무엇이 보였는가"를 계산하고, 이 모듈은 "지금 집어도 되는가"를
판정한다. 공구 종류별 기대 파트와 포함 관계 규칙을 한곳에 모아, 정책 변경 시
task manager까지 파급되지 않게 한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np


SORTABLE_TOOL_NAMES = (
    "hammer",
    "screwdriver",
    "wrench",
    "monkey_wrench",
    "vise",
)


DEFAULT_GRASP_PARTS = MappingProxyType(
    {
        "monkey_wrench": "handle",
        "wrench": "handle",
        "screwdriver": "handle",
        "screw_driver": "handle",
        "hammer": "handle",
        "vise": "vise_head",
    }
)


@dataclass(frozen=True)
class GraspPartMatch:
    """A generic part detection associated with one concrete tool detection."""

    tool_detection_id: int
    tool_name: str
    part_detection_id: int
    part_name: str
    containment: float
    part_confidence: float


def expected_grasp_part(
    tool_name: str,
    grasp_parts: Mapping[str, str] | None = None,
) -> str | None:
    mapping = DEFAULT_GRASP_PARTS if grasp_parts is None else grasp_parts
    return mapping.get(tool_name)


def is_grasp_ready(
    tool_name: str,
    matched_part_name: str | None,
    grasp_parts: Mapping[str, str] | None = None,
) -> bool:
    required = expected_grasp_part(tool_name, grasp_parts)
    return required is not None and matched_part_name == required


def match_grasp_parts(
    tools: Sequence[Mapping],
    parts: Sequence[Mapping],
    min_containment: float = 0.5,
    grasp_parts: Mapping[str, str] | None = None,
) -> dict[int, GraspPartMatch]:
    """Match part masks to compatible tool masks using one-to-one containment.

    Each candidate score is the fraction of the part mask contained by the tool
    mask. Candidates below ``min_containment`` are rejected. Greedy assignment
    in descending containment order prevents one part or tool detection from
    being assigned more than once.
    """

    if not 0.0 <= min_containment <= 1.0:
        raise ValueError("min_containment must be between 0.0 and 1.0")

    mapping = DEFAULT_GRASP_PARTS if grasp_parts is None else grasp_parts
    candidates: list[tuple[float, float, int, int, Mapping, Mapping]] = []

    for part_index, part in enumerate(parts):
        part_pixels = np.asarray(part["mask"]) > 0
        part_area = int(part_pixels.sum())
        if part_area == 0:
            continue

        for tool_index, tool in enumerate(tools):
            if mapping.get(str(tool["name"])) != str(part["name"]):
                continue
            tool_pixels = np.asarray(tool["mask"]) > 0
            if tool_pixels.shape != part_pixels.shape:
                raise ValueError("tool and part masks must have identical shapes")
            containment = float(
                np.logical_and(part_pixels, tool_pixels).sum() / part_area
            )
            if containment < min_containment:
                continue
            candidates.append(
                (
                    containment,
                    float(part["confidence"]),
                    tool_index,
                    part_index,
                    tool,
                    part,
                )
            )

    candidates.sort(key=lambda value: (value[0], value[1]), reverse=True)
    used_tools: set[int] = set()
    used_parts: set[int] = set()
    matches: dict[int, GraspPartMatch] = {}

    for containment, confidence, tool_index, part_index, tool, part in candidates:
        if tool_index in used_tools or part_index in used_parts:
            continue
        tool_detection_id = int(tool["detection_id"])
        matches[tool_detection_id] = GraspPartMatch(
            tool_detection_id=tool_detection_id,
            tool_name=str(tool["name"]),
            part_detection_id=int(part["detection_id"]),
            part_name=str(part["name"]),
            containment=round(containment, 4),
            part_confidence=round(confidence, 4),
        )
        used_tools.add(tool_index)
        used_parts.add(part_index)

    return matches
