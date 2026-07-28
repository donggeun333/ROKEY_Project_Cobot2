from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class TransformConventionError(ValueError):
    """Raised when a transform direction or unit is not explicit and valid."""


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        values = np.asarray([self.fx, self.fy, self.cx, self.cy], dtype=np.float64)
        if not np.isfinite(values).all():
            raise ValueError("Camera intrinsics contain NaN or Inf")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Camera intrinsics fx/fy must be greater than zero")

    @classmethod
    def from_camera_matrix(cls, matrix: Sequence[float]) -> "CameraIntrinsics":
        if len(matrix) != 9:
            raise ValueError("Camera matrix K must contain exactly 9 values")
        return cls(
            fx=float(matrix[0]),
            fy=float(matrix[4]),
            cx=float(matrix[2]),
            cy=float(matrix[5]),
        )


@dataclass(frozen=True)
class DepthSamplingConfig:
    depth_scale_16u_to_m: float = 0.001
    depth_scale_float_to_m: float = 1.0
    erode_pixels: int = 2
    min_valid_ratio: float = 0.30
    min_valid_depth_m: float = 0.15
    max_valid_depth_m: float = 2.0
    min_valid_pixel_count: int = 25

    def __post_init__(self) -> None:
        if self.depth_scale_16u_to_m <= 0.0:
            raise ValueError("depth_scale_16u_to_m must be greater than zero")
        if self.depth_scale_float_to_m <= 0.0:
            raise ValueError("depth_scale_float_to_m must be greater than zero")
        if self.erode_pixels < 0:
            raise ValueError("erode_pixels cannot be negative")
        if not 0.0 <= self.min_valid_ratio <= 1.0:
            raise ValueError("min_valid_ratio must be between 0 and 1")
        if self.min_valid_depth_m < 0.0:
            raise ValueError("min_valid_depth_m cannot be negative")
        if self.max_valid_depth_m <= self.min_valid_depth_m:
            raise ValueError("max_valid_depth_m must exceed min_valid_depth_m")
        if self.min_valid_pixel_count < 1:
            raise ValueError("min_valid_pixel_count must be at least 1")


def robust_mask_depth_m(
    mask: np.ndarray,
    depth_image: np.ndarray,
    depth_encoding: str,
    config: DepthSamplingConfig | None = None,
) -> tuple[float | None, float]:
    """Return a robust metric depth from an eroded segmentation mask.

    The returned ratio is the number of valid, in-range depth pixels divided by
    the number of pixels selected by the final mask. Invalid or insufficient
    samples return ``(None, ratio)``.
    """

    settings = config or DepthSamplingConfig()
    binary = np.asarray(mask) > 0
    depth = np.asarray(depth_image)
    if binary.ndim != 2 or depth.ndim != 2:
        raise ValueError("mask and depth_image must both be 2-D")
    if binary.shape != depth.shape:
        raise ValueError("mask and depth_image must have identical shapes")

    final_mask = binary.astype(np.uint8)
    if settings.erode_pixels > 0:
        size = settings.erode_pixels * 2 + 1
        eroded = cv2.erode(
            final_mask,
            np.ones((size, size), dtype=np.uint8),
        )
        if int(np.count_nonzero(eroded)) >= settings.min_valid_pixel_count:
            final_mask = eroded

    selected = depth[final_mask > 0]
    if selected.size == 0:
        return None, 0.0

    encoding = str(depth_encoding).strip().lower()
    values_m = selected.astype(np.float64)
    if depth.dtype == np.uint16 or encoding in {"16uc1", "mono16"}:
        values_m *= settings.depth_scale_16u_to_m
    else:
        values_m *= settings.depth_scale_float_to_m

    valid = values_m[
        np.isfinite(values_m)
        & (values_m >= settings.min_valid_depth_m)
        & (values_m <= settings.max_valid_depth_m)
    ]
    ratio = float(valid.size / selected.size)
    if (
        valid.size < settings.min_valid_pixel_count
        or ratio < settings.min_valid_ratio
    ):
        return None, ratio
    return float(np.median(valid)), ratio


def deproject_pixel_to_camera(
    u: float,
    v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Deproject an aligned color pixel into the camera optical frame in meters."""

    if not np.isfinite([u, v, depth_m]).all() or depth_m <= 0.0:
        raise ValueError("u, v and depth_m must be finite and depth must be positive")
    return np.asarray(
        [
            (float(u) - intrinsics.cx) * float(depth_m) / intrinsics.fx,
            (float(v) - intrinsics.cy) * float(depth_m) / intrinsics.fy,
            float(depth_m),
        ],
        dtype=np.float64,
    )


def doosan_pose_to_matrix(pose: Sequence[float], *, translation_scale: float) -> np.ndarray:
    """Convert Doosan ``[X,Y,Z,RX,RY,RZ]`` ZYZ pose to a homogeneous matrix."""

    if len(pose) != 6:
        raise ValueError("Doosan pose must contain exactly 6 values")
    values = np.asarray(pose, dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("Doosan pose contains NaN or Inf")
    if translation_scale <= 0.0:
        raise ValueError("translation_scale must be greater than zero")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_euler(
        "ZYZ",
        values[3:6],
        degrees=True,
    ).as_matrix()
    matrix[:3, 3] = values[:3] * float(translation_scale)
    return matrix


def _validate_homogeneous_transform(matrix: np.ndarray) -> None:
    if matrix.shape != (4, 4):
        raise TransformConventionError("Transform matrix must be 4x4")
    if not np.isfinite(matrix).all():
        raise TransformConventionError("Transform matrix contains NaN or Inf")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-8):
        raise TransformConventionError(
            "Transform matrix last row must be [0, 0, 0, 1]"
        )
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-4):
        raise TransformConventionError("Transform rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-4):
        raise TransformConventionError("Transform rotation determinant is not +1")


def load_parent_from_child_transform(
    transform_path: str,
    *,
    stored_direction: str,
    translation_unit: str,
) -> np.ndarray:
    """Load a transform and return ``T_parent_child`` in meters.

    ``stored_direction`` must be one of:

    - ``parent_from_child``: stored matrix already maps child points to parent.
    - ``child_from_parent``: stored matrix maps parent points to child and is
      inverted exactly once.

    No direction or unit is inferred from the filename or value magnitude.
    """

    path = Path(transform_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Transform file not found: {path}")
    matrix = np.asarray(np.load(str(path)), dtype=np.float64)
    _validate_homogeneous_transform(matrix)

    direction = str(stored_direction).strip().lower()
    if direction == "child_from_parent":
        matrix = np.linalg.inv(matrix)
    elif direction != "parent_from_child":
        raise TransformConventionError(
            "stored_direction must be 'parent_from_child' or 'child_from_parent'"
        )

    unit = str(translation_unit).strip().lower()
    if unit == "mm":
        matrix[:3, 3] /= 1000.0
    elif unit != "m":
        raise TransformConventionError("translation_unit must be 'm' or 'mm'")

    _validate_homogeneous_transform(matrix)
    return matrix


def transform_point(matrix: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply a homogeneous transform to a 3-D point."""

    transform = np.asarray(matrix, dtype=np.float64)
    _validate_homogeneous_transform(transform)
    value = np.asarray(point, dtype=np.float64).reshape(3)
    if not np.isfinite(value).all():
        raise ValueError("Point contains NaN or Inf")
    output = transform @ np.append(value, 1.0)
    if abs(float(output[3])) < 1.0e-12:
        raise ValueError("Transformed homogeneous point has w=0")
    result = output[:3] / output[3]
    if not np.isfinite(result).all():
        raise ValueError("Transformed point contains NaN or Inf")
    return result
