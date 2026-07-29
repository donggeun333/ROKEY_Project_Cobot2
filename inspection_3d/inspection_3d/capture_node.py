from __future__ import annotations

from datetime import datetime
from pathlib import Path
import struct

from geometry_msgs.msg import TransformStamped
import numpy as np
import open3d as o3d
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
import tf2_ros


_DATATYPE_TO_STRUCT = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


def transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def rgb_float_to_colors(rgb_values: np.ndarray) -> np.ndarray:
    colors = np.ones((len(rgb_values), 3), dtype=np.float64)

    for index, value in enumerate(rgb_values):
        packed = struct.unpack("I", struct.pack("f", float(value)))[0]
        colors[index, 0] = ((packed >> 16) & 0xFF) / 255.0
        colors[index, 1] = ((packed >> 8) & 0xFF) / 255.0
        colors[index, 2] = (packed & 0xFF) / 255.0

    return colors


def pointcloud2_to_arrays(message: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
    field_map = {field.name: field for field in message.fields}
    required_fields = ["x", "y", "z"]
    for field_name in required_fields:
        if field_name not in field_map:
            raise ValueError(f"PointCloud2 field missing: {field_name}")

    point_count = message.width * message.height
    points = np.zeros((point_count, 3), dtype=np.float64)
    has_rgb = "rgb" in field_map
    rgb_values = np.zeros(point_count, dtype=np.float32) if has_rgb else None

    for index in range(point_count):
        base_offset = index * message.point_step
        for axis, field_name in enumerate(("x", "y", "z")):
            field = field_map[field_name]
            fmt, _ = _DATATYPE_TO_STRUCT[field.datatype]
            points[index, axis] = struct.unpack_from(
                fmt,
                message.data,
                base_offset + field.offset,
            )[0]

        if has_rgb:
            field = field_map["rgb"]
            fmt, _ = _DATATYPE_TO_STRUCT[field.datatype]
            rgb_values[index] = struct.unpack_from(
                fmt,
                message.data,
                base_offset + field.offset,
            )[0]

    valid = np.isfinite(points).all(axis=1)
    if has_rgb:
        colors = rgb_float_to_colors(rgb_values)
        colors = colors[valid]
    else:
        colors = np.ones((np.count_nonzero(valid), 3), dtype=np.float64)

    return points[valid], colors


class PointCloudCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_capture")

        self.declare_parameter("input_topic", "/camera/depth/color/points")
        self.declare_parameter("output_dir", "data/captures")
        self.declare_parameter("save_frame", "")
        self.declare_parameter("tf_timeout_sec", 1.0)
        self.declare_parameter("voxel_size", 0.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value).resolve()
        self.save_frame = self.get_parameter("save_frame").value
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.latest_cloud: PointCloud2 | None = None

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.handle_cloud,
            10,
        )
        self.capture_service = self.create_service(
            Trigger,
            "~/capture",
            self.handle_capture,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.get_logger().info(
            f"PointCloud capture ready. topic={self.input_topic}, "
            "service=~/capture"
        )

    def handle_cloud(self, message: PointCloud2) -> None:
        self.latest_cloud = message

    def maybe_transform_points(
        self,
        points: np.ndarray,
        source_frame: str,
        stamp
    ) -> np.ndarray:
        if not self.save_frame or self.save_frame == source_frame:
            return points

        transform = self.tf_buffer.lookup_transform(
            self.save_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        matrix = transform_to_matrix(transform)
        homogeneous = np.hstack((points, np.ones((len(points), 1), dtype=np.float64)))
        transformed = (matrix @ homogeneous.T).T
        return transformed[:, :3]

    def handle_capture(self, request, response):
        del request

        try:
            if self.latest_cloud is None:
                raise RuntimeError("No PointCloud2 message received yet.")

            source_frame = self.latest_cloud.header.frame_id
            stamp = self.latest_cloud.header.stamp

            points, colors = pointcloud2_to_arrays(self.latest_cloud)
            if len(points) == 0:
                raise RuntimeError("Received point cloud is empty.")

            points = self.maybe_transform_points(
                points, 
                source_frame,
                stamp)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            if self.voxel_size > 0.0:
                pcd = pcd.voxel_down_sample(self.voxel_size)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            frame_suffix = self.save_frame if self.save_frame else source_frame
            safe_suffix = frame_suffix.replace("/", "_")
            output_path = self.output_dir / f"capture_{timestamp}_{safe_suffix}.pcd"

            saved = o3d.io.write_point_cloud(str(output_path), pcd)
            if not saved:
                raise RuntimeError(f"Failed to save {output_path}")

            response.success = True
            response.message = str(output_path)
            self.get_logger().info(f"Saved capture: {output_path}")
        except Exception as error:
            response.success = False
            response.message = f"Capture failed: {error}"
            self.get_logger().error(response.message)

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
