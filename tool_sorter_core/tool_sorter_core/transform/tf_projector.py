from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import tf2_ros
from rclpy.duration import Duration

from .core import (
    CameraIntrinsics,
    DepthSamplingConfig,
    deproject_pixel_to_camera,
    robust_mask_depth_m,
    transform_point,
)


class TimestampedTransformUnavailable(RuntimeError):
    """Raised when the exact capture-time transform cannot be obtained."""


@dataclass(frozen=True)
class ProjectionResult:
    pixel: tuple[float, float]
    depth_m: float
    valid_depth_ratio: float
    camera_point_m: np.ndarray
    base_point_m: np.ndarray
    base_frame: str
    projection_frame: str

    @property
    def base_point_mm(self) -> np.ndarray:
        return self.base_point_m * 1000.0


def transform_message_to_matrix(transform) -> np.ndarray:
    """Convert geometry_msgs/Transform into a 4x4 parent-from-child matrix."""

    translation = transform.translation
    quaternion = transform.rotation
    values = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(values))
    if norm < 1.0e-12:
        raise ValueError("TF quaternion norm is zero")
    x, y, z, w = values / norm
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [
        float(translation.x),
        float(translation.y),
        float(translation.z),
    ]
    return matrix


class TimestampedRgbdProjector:
    """Project segmentation grasp points using only capture-time TF.

    ``projection_frame`` is normally the calibrated virtual camera frame
    published by this package. It deliberately differs from the RealSense URDF
    camera frame so two publishers never own the same TF child frame.
    """

    def __init__(
        self,
        *,
        tf_buffer: tf2_ros.Buffer,
        base_frame: str,
        projection_frame: str,
        tf_timeout_sec: float = 0.2,
        depth_config: DepthSamplingConfig | None = None,
    ) -> None:
        if not str(base_frame).strip():
            raise ValueError("base_frame cannot be empty")
        if not str(projection_frame).strip():
            raise ValueError("projection_frame cannot be empty")
        if tf_timeout_sec <= 0.0:
            raise ValueError("tf_timeout_sec must be greater than zero")
        self.tf_buffer = tf_buffer
        self.base_frame = str(base_frame)
        self.projection_frame = str(projection_frame)
        self.tf_timeout_sec = float(tf_timeout_sec)
        self.depth_config = depth_config or DepthSamplingConfig()

    def lookup_base_from_camera(self, capture_stamp) -> np.ndarray:
        """Return exact capture-time TF; never fall back to the latest transform."""

        try:
            stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.projection_frame,
                capture_stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TimeoutException,
        ) as error:
            raise TimestampedTransformUnavailable(
                f"TF unavailable at capture time: "
                f"{self.base_frame} <- {self.projection_frame}: {error}"
            ) from error
        return transform_message_to_matrix(stamped.transform)

    def project_mask(
        self,
        *,
        center_px: tuple[float, float] | list[float],
        mask: np.ndarray,
        depth_image: np.ndarray,
        depth_encoding: str,
        camera_info,
        capture_stamp,
    ) -> ProjectionResult | None:
        """Project one segmentation grasp mask into the robot base frame."""

        depth_m, ratio = robust_mask_depth_m(
            mask,
            depth_image,
            depth_encoding,
            self.depth_config,
        )
        if depth_m is None:
            return None
        return self.project_depth(
            center_px=center_px,
            depth_m=depth_m,
            valid_depth_ratio=ratio,
            camera_info=camera_info,
            capture_stamp=capture_stamp,
        )

    def project_depth(
        self,
        *,
        center_px: tuple[float, float] | list[float],
        depth_m: float,
        camera_info,
        capture_stamp,
        valid_depth_ratio: float = 1.0,
    ) -> ProjectionResult:
        """Project an already validated metric depth at a grasp center.

        This API fits pipelines such as ``tool_sorter_core`` where perception
        computes robust mask depth and sends only the representative depth to a
        task manager. The original capture timestamp must still accompany it.
        """

        if not 0.0 <= float(valid_depth_ratio) <= 1.0:
            raise ValueError("valid_depth_ratio must be between 0 and 1")
        intrinsics = CameraIntrinsics.from_camera_matrix(camera_info.k)
        u, v = float(center_px[0]), float(center_px[1])
        camera_point = deproject_pixel_to_camera(u, v, depth_m, intrinsics)
        base_from_camera = self.lookup_base_from_camera(capture_stamp)
        base_point = transform_point(base_from_camera, camera_point)
        return ProjectionResult(
            pixel=(u, v),
            depth_m=float(depth_m),
            valid_depth_ratio=float(valid_depth_ratio),
            camera_point_m=camera_point,
            base_point_m=base_point,
            base_frame=self.base_frame,
            projection_frame=self.projection_frame,
        )
