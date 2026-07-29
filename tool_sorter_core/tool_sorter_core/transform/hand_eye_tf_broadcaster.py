from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .core import load_parent_from_child_transform


class HandEyeTfBroadcaster:
    """Publish an explicitly configured Hand-Eye matrix as a static TF."""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.node import Node
        from tf2_ros import StaticTransformBroadcaster

        self._rclpy = rclpy
        self.node = Node("m0609_hand_eye_tf_broadcaster")
        self.node.declare_parameter("transform_path", "")
        self.node.declare_parameter("parent_frame", "")
        self.node.declare_parameter(
            "child_frame",
            "calibrated_camera_color_optical_frame",
        )
        self.node.declare_parameter("stored_direction", "parent_from_child")
        self.node.declare_parameter("translation_unit", "mm")

        transform_path = str(self._parameter("transform_path")).strip()
        parent_frame = str(self._parameter("parent_frame")).strip()
        child_frame = str(self._parameter("child_frame")).strip()
        if not transform_path:
            raise ValueError("transform_path must be configured explicitly")
        if not parent_frame:
            raise ValueError(
                "parent_frame must be the exact Hand-Eye calibration frame"
            )
        if not child_frame:
            raise ValueError("child_frame cannot be empty")
        if parent_frame == child_frame:
            raise ValueError("parent_frame and child_frame must differ")
        if child_frame == "camera_color_optical_frame":
            raise ValueError(
                "Do not reuse the URDF-owned camera_color_optical_frame; "
                "use a separate calibrated camera child frame"
            )

        matrix = load_parent_from_child_transform(
            transform_path,
            stored_direction=str(self._parameter("stored_direction")),
            translation_unit=str(self._parameter("translation_unit")),
        )
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()

        message = TransformStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = parent_frame
        message.child_frame_id = child_frame
        message.transform.translation.x = float(matrix[0, 3])
        message.transform.translation.y = float(matrix[1, 3])
        message.transform.translation.z = float(matrix[2, 3])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])

        self._broadcaster = StaticTransformBroadcaster(self.node)
        self._broadcaster.sendTransform(message)
        self.node.get_logger().info(
            f"Published calibrated Hand-Eye TF: {parent_frame} -> {child_frame}; "
            f"stored_direction={self._parameter('stored_direction')}, "
            f"translation_unit={self._parameter('translation_unit')}"
        )

    def _parameter(self, name: str):
        return self.node.get_parameter(name).value


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    application = None
    try:
        application = HandEyeTfBroadcaster()
        rclpy.spin(application.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if application is not None:
            application.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
