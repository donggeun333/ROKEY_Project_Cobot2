from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from .geometry import robot_pose_to_matrix


def _pose6(values: Sequence[float], label: str) -> list[float]:
    if len(values) != 6:
        raise ValueError(f"{label} must contain exactly six values")
    pose = [float(value) for value in values]
    if not np.isfinite(pose).all():
        raise ValueError(f"{label} contains NaN or Inf")
    return pose


def bird_pose_from_home_tcp(
    home_tcp_pose: Sequence[float],
    raise_mm: float,
) -> list[float]:
    """Keep Base X/Y and TCP orientation, changing only Base Z."""

    pose = _pose6(home_tcp_pose, "home_tcp_pose")
    distance = float(raise_mm)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("bird_raise_mm must be greater than zero")
    pose[2] += distance
    return pose


def camera_centered_tcp_pose(
    *,
    focus_base_mm: Sequence[float],
    tcp_orientation_deg: Sequence[float],
    tcp_from_camera_m: np.ndarray,
    camera_standoff_mm: float,
    max_tilt_deg: float,
) -> tuple[list[float], list[float]]:
    """Place the calibrated camera center ray on a Base-frame focus point.

    ``tcp_from_camera_m`` maps camera optical-frame points into the active TCP
    frame. The returned TCP pose keeps the supplied orientation and chooses its
    translation so the camera optical +Z ray reaches ``focus_base_mm`` after
    ``camera_standoff_mm``.
    """

    focus = np.asarray(focus_base_mm, dtype=np.float64).reshape(-1)
    orientation = np.asarray(
        tcp_orientation_deg,
        dtype=np.float64,
    ).reshape(-1)
    transform = np.asarray(tcp_from_camera_m, dtype=np.float64)
    if focus.size != 3 or not np.isfinite(focus).all():
        raise ValueError("focus_base_mm must contain three finite values")
    if orientation.size != 3 or not np.isfinite(orientation).all():
        raise ValueError("tcp_orientation_deg must contain three finite values")
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("tcp_from_camera_m must be a finite 4x4 matrix")
    homogeneous_row = [0.0, 0.0, 0.0, 1.0]
    if not np.allclose(transform[3], homogeneous_row, atol=1.0e-8):
        raise ValueError("tcp_from_camera_m is not a homogeneous transform")

    standoff = float(camera_standoff_mm)
    tilt_limit = float(max_tilt_deg)
    if not math.isfinite(standoff) or standoff <= 0.0:
        raise ValueError("local_scan_standoff_mm must be greater than zero")
    if not math.isfinite(tilt_limit) or not 0.0 <= tilt_limit < 90.0:
        raise ValueError("bird_max_tilt_deg must be in [0, 90)")

    base_from_tcp_rotation = robot_pose_to_matrix(
        [0.0, 0.0, 0.0, *orientation.tolist()]
    )[:3, :3]
    tcp_from_camera_rotation = transform[:3, :3]
    optical_axis_base = (
        base_from_tcp_rotation
        @ tcp_from_camera_rotation
        @ np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    )
    optical_axis_base /= np.linalg.norm(optical_axis_base)
    downward = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
    tilt_deg = math.degrees(
        math.acos(
            float(np.clip(np.dot(optical_axis_base, downward), -1.0, 1.0))
        )
    )
    if tilt_deg > tilt_limit:
        raise ValueError(
            f"camera optical axis tilt {tilt_deg:.1f}deg exceeds "
            f"bird_max_tilt_deg={tilt_limit:.1f}deg"
        )

    camera_origin_base_mm = focus - standoff * optical_axis_base
    tcp_to_camera_offset_base_mm = (
        base_from_tcp_rotation @ transform[:3, 3]
    ) * 1000.0
    tcp_origin_base_mm = (
        camera_origin_base_mm - tcp_to_camera_offset_base_mm
    )
    pose = [
        *[float(value) for value in tcp_origin_base_mm],
        *[float(value) for value in orientation],
    ]
    return pose, [float(value) for value in optical_axis_base]
