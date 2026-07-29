from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def robot_pose_to_matrix(pose: list[float] | tuple[float, ...]) -> np.ndarray:
    if len(pose) != 6:
        raise ValueError("Doosan pose must contain X, Y, Z, RX, RY, RZ")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_euler(
        "ZYZ", pose[3:6], degrees=True
    ).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def orient_with_yaw(
    observe_orientation_deg: list[float],
    object_yaw_deg: float,
    finger_offset_deg: float = 0.0,
) -> list[float]:
    """Rotate only around the approach axis and keep the tool tilt unchanged."""
    observed = Rotation.from_euler(
        "ZYZ", list(observe_orientation_deg), degrees=True
    )
    axis = observed.apply([0.0, 0.0, 1.0])
    if axis[2] < 0:
        axis = -axis

    desired = object_yaw_deg + 90.0 + finger_offset_deg
    target = observed
    for _ in range(4):
        finger_axis = target.apply([1.0, 0.0, 0.0])
        current = math.degrees(math.atan2(finger_axis[1], finger_axis[0]))
        delta = (desired - current + 90.0) % 180.0 - 90.0
        if abs(delta) < 1.0e-4:
            break
        target = Rotation.from_rotvec(np.radians(delta) * axis) * target

    import warnings

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        result = target.as_euler("ZYZ", degrees=True)
    return [round(float(value), 3) for value in result]


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    ppx: float
    ppy: float

    @classmethod
    def from_camera_info(cls, camera_info: dict[str, float]) -> "CameraIntrinsics":
        return cls(
            fx=float(camera_info["fx"]),
            fy=float(camera_info["fy"]),
            ppx=float(camera_info["ppx"]),
            ppy=float(camera_info["ppy"]),
        )


class GraspProjector:
    """Project a grasp measured in the aligned depth image into robot base."""

    def __init__(
        self,
        gripper_to_camera_path: str,
        intrinsics: CameraIntrinsics,
        table_z_mm: float,
    ):
        self.gripper_to_camera = np.load(gripper_to_camera_path)
        if self.gripper_to_camera.shape != (4, 4):
            raise ValueError("T_gripper2camera must be a 4x4 homogeneous matrix")
        if not np.isfinite(self.gripper_to_camera).all():
            raise ValueError("T_gripper2camera contains a non-finite value")
        self.intrinsics = intrinsics
        self.table_z_mm = float(table_z_mm)

    def pixel_to_camera(self, u: float, v: float, depth_mm: float) -> np.ndarray:
        camera_x = (u - self.intrinsics.ppx) * depth_mm / self.intrinsics.fx
        camera_y = (v - self.intrinsics.ppy) * depth_mm / self.intrinsics.fy
        return np.array([camera_x, camera_y, depth_mm, 1.0], dtype=np.float64)

    def pixel_to_base(
        self, u: float, v: float, depth_mm: float, robot_pose: list[float]
    ) -> np.ndarray:
        base_to_camera = (
            robot_pose_to_matrix(robot_pose) @ self.gripper_to_camera
        )
        return (base_to_camera @ self.pixel_to_camera(u, v, depth_mm))[:3]

    def object_yaw_in_base(
        self,
        center_u: float,
        center_v: float,
        image_angle_deg: float,
        depth_mm: float,
        robot_pose: list[float],
        probe_px: float = 50.0,
    ) -> float:
        radians = math.radians(image_angle_deg)
        point_a = self.pixel_to_base(
            center_u, center_v, depth_mm, robot_pose
        )
        point_b = self.pixel_to_base(
            center_u + probe_px * math.cos(radians),
            center_v + probe_px * math.sin(radians),
            depth_mm,
            robot_pose,
        )
        return math.degrees(
            math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
        ) % 180.0

    def compute(
        self,
        center_px: list[float],
        image_angle_deg: float,
        grip_width_px: float,
        depth_mm: float,
        robot_pose: list[float],
        grasp_depth_ratio: float,
    ) -> dict[str, float]:
        u, v = center_px
        base_point = self.pixel_to_base(u, v, depth_mm, robot_pose)
        thickness = max(float(base_point[2]) - self.table_z_mm, 0.0)
        grasp_z = self.table_z_mm + thickness * grasp_depth_ratio
        yaw = self.object_yaw_in_base(
            u, v, image_angle_deg, depth_mm, robot_pose
        )
        width_mm = grip_width_px * depth_mm / self.intrinsics.fx
        return {
            "x_mm": round(float(base_point[0]), 2),
            "y_mm": round(float(base_point[1]), 2),
            "z_mm": round(float(grasp_z), 2),
            "top_z_mm": round(float(base_point[2]), 2),
            "yaw_deg": round(float(yaw), 2),
            "thickness_mm": round(float(thickness), 2),
            "grip_width_mm": round(float(width_mm), 2),
        }
