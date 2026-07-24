from __future__ import annotations

from datetime import datetime
from pathlib import Path
import struct

from geometry_msgs.msg import TransformStamped
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import tf2_ros


def pack_rgb_as_float(rgb_colors: np.ndarray) -> np.ndarray:
    clipped = np.clip(rgb_colors * 255.0, 0.0, 255.0).astype(np.uint8)
    rgb_uint32 = (
        (clipped[:, 0].astype(np.uint32) << 16)
        | (clipped[:, 1].astype(np.uint32) << 8)
        | clipped[:, 2].astype(np.uint32)
    )
    return np.array(
        [struct.unpack("f", struct.pack("I", value))[0] for value in rgb_uint32],
        dtype=np.float32,
    )


def pointcloud2_from_arrays(
    points: np.ndarray,
    colors: np.ndarray,
    frame_id: str,
    stamp,
) -> PointCloud2:
    if len(points) == 0:
        return PointCloud2(
            header=Header(frame_id=frame_id, stamp=stamp),
            height=1,
            width=0,
            fields=[],
            is_bigendian=False,
            point_step=0,
            row_step=0,
            data=b"",
            is_dense=True,
        )

    rgb_column = pack_rgb_as_float(colors).reshape(-1, 1)
    cloud = np.hstack((points.astype(np.float32), rgb_column))

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    return PointCloud2(
        header=Header(frame_id=frame_id, stamp=stamp),
        height=1,
        width=cloud.shape[0],
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * cloud.shape[0],
        data=cloud.tobytes(),
        is_dense=True,
    )


def transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class RealSenseGrabber:
    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        min_depth: float,
        max_depth: float,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.min_depth = min_depth
        self.max_depth = max_depth

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, fps
        )
        self.config.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, fps
        )

        self.align = rs.align(rs.stream.color)
        self.pointcloud = rs.pointcloud()
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling_filter = rs.hole_filling_filter()
        self.started = False

    def start(self) -> None:
        if self.started:
            return

        self.pipeline.start(self.config)
        self.started = True

        try:
            for _ in range(30):
                self.pipeline.wait_for_frames()
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        if not self.started:
            return
        self.pipeline.stop()
        self.started = False

    def capture_pointcloud(self) -> o3d.geometry.PointCloud:
        if not self.started:
            self.start()

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("Failed to capture aligned RealSense frames.")

        filtered_depth = self.spatial_filter.process(depth_frame)
        filtered_depth = self.temporal_filter.process(filtered_depth)
        filtered_depth = self.hole_filling_filter.process(filtered_depth)

        self.pointcloud.map_to(color_frame)
        rs_points = self.pointcloud.calculate(filtered_depth)

        vertices = np.asarray(rs_points.get_vertices()).view(np.float32).reshape(-1, 3)
        uv = np.asarray(rs_points.get_texture_coordinates()).view(np.float32).reshape(
            -1, 2
        )
        color_image = np.asarray(color_frame.get_data())

        image_height, image_width, _ = color_image.shape
        cols = np.clip((uv[:, 0] * image_width).astype(np.int32), 0, image_width - 1)
        rows = np.clip((uv[:, 1] * image_height).astype(np.int32), 0, image_height - 1)
        rgb = color_image[rows, cols][:, ::-1] / 255.0

        valid = (
            np.isfinite(vertices).all(axis=1)
            & (vertices[:, 2] >= self.min_depth)
            & (vertices[:, 2] <= self.max_depth)
        )
        vertices = vertices[valid]
        rgb = rgb[valid]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(vertices)
        cloud.colors = o3d.utility.Vector3dVector(rgb)
        return cloud


class PointCloudMergerNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_merger")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("publish_topic", "/pointcloud/merged")
        self.declare_parameter("output_dir", "data/merged")
        self.declare_parameter("capture_dir", "data/captures")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("min_depth", 0.20)
        self.declare_parameter("max_depth", 1.20)
        self.declare_parameter("voxel_size", 0.003)
        self.declare_parameter("outlier_neighbors", 20)
        self.declare_parameter("outlier_std_ratio", 2.0)
        self.declare_parameter("tf_timeout_sec", 1.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value).resolve()
        self.capture_dir = Path(self.get_parameter("capture_dir").value).resolve()
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.outlier_neighbors = int(self.get_parameter("outlier_neighbors").value)
        self.outlier_std_ratio = float(
            self.get_parameter("outlier_std_ratio").value
        )
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.capture_dir.mkdir(parents=True, exist_ok=True)

        self.camera = RealSenseGrabber(
            width=int(self.get_parameter("width").value),
            height=int(self.get_parameter("height").value),
            fps=int(self.get_parameter("fps").value),
            min_depth=float(self.get_parameter("min_depth").value),
            max_depth=float(self.get_parameter("max_depth").value),
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.merged_cloud = o3d.geometry.PointCloud()

        self.publisher = self.create_publisher(PointCloud2, self.publish_topic, 10)
        self.capture_service = self.create_service(
            Trigger,
            "~/capture",
            self.handle_capture,
        )
        self.reset_service = self.create_service(
            Trigger,
            "~/reset",
            self.handle_reset,
        )
        self.save_service = self.create_service(
            Trigger,
            "~/save",
            self.handle_save,
        )
        self.publish_timer = self.create_timer(0.5, self.publish_merged_cloud)

        self.get_logger().info(
            "PointCloud merger ready. Services: "
            "~/capture, ~/reset, ~/save"
        )

    def destroy_node(self):
        self.camera.stop()
        return super().destroy_node()

    def lookup_base_to_camera(self) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        return transform_to_matrix(transform)

    def preprocess_cloud(
        self,
        cloud: o3d.geometry.PointCloud,
    ) -> o3d.geometry.PointCloud:
        if len(cloud.points) == 0:
            raise ValueError("Captured point cloud is empty.")

        filtered = cloud.voxel_down_sample(self.voxel_size)
        if len(filtered.points) >= self.outlier_neighbors:
            filtered, _ = filtered.remove_statistical_outlier(
                nb_neighbors=self.outlier_neighbors,
                std_ratio=self.outlier_std_ratio,
            )
        return filtered

    def merge_cloud(
        self,
        new_cloud: o3d.geometry.PointCloud,
    ) -> None:
        if len(self.merged_cloud.points) == 0:
            self.merged_cloud = o3d.geometry.PointCloud(new_cloud)
            return

        merged = o3d.geometry.PointCloud(self.merged_cloud)
        merged += new_cloud
        merged = merged.voxel_down_sample(self.voxel_size)
        if len(merged.points) >= self.outlier_neighbors:
            merged, _ = merged.remove_statistical_outlier(
                nb_neighbors=self.outlier_neighbors,
                std_ratio=self.outlier_std_ratio,
            )
        self.merged_cloud = merged

    def capture_base_cloud(self) -> o3d.geometry.PointCloud:
        last_error = None
        for _ in range(3):
            try:
                camera_cloud = self.camera.capture_pointcloud()
                break
            except Exception as error:
                last_error = error
                self.camera.stop()
        else:
            raise RuntimeError(
                "RealSense capture failed after 3 retries."
            ) from last_error

        camera_cloud = self.preprocess_cloud(camera_cloud)
        base_to_camera = self.lookup_base_to_camera()

        base_cloud = o3d.geometry.PointCloud(camera_cloud)
        base_cloud.transform(base_to_camera)
        return self.preprocess_cloud(base_cloud)

    def handle_capture(self, request, response):
        del request
        try:
            base_cloud = self.capture_base_cloud()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            capture_path = self.capture_dir / f"capture_{timestamp}.pcd"
            saved = o3d.io.write_point_cloud(str(capture_path), base_cloud)
            if not saved:
                raise RuntimeError(f"Failed to save {capture_path}")
            self.merge_cloud(base_cloud)
            point_count = len(self.merged_cloud.points)
            response.success = True
            response.message = (
                f"Captured, saved, and merged. "
                f"path={capture_path}, points={point_count}"
            )
            self.get_logger().info(response.message)
        except Exception as error:
            response.success = False
            response.message = f"Capture failed: {error}"
            self.get_logger().error(response.message)
        return response

    def handle_reset(self, request, response):
        del request
        self.merged_cloud = o3d.geometry.PointCloud()
        response.success = True
        response.message = "Merged point cloud reset."
        self.get_logger().info(response.message)
        return response

    def handle_save(self, request, response):
        del request
        try:
            if len(self.merged_cloud.points) == 0:
                raise RuntimeError("Merged point cloud is empty.")

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = self.output_dir / f"merged_{timestamp}.pcd"
            saved = o3d.io.write_point_cloud(str(output_path), self.merged_cloud)
            if not saved:
                raise RuntimeError(f"Failed to save {output_path}")

            response.success = True
            response.message = str(output_path)
            self.get_logger().info(f"Saved merged cloud: {output_path}")
        except Exception as error:
            response.success = False
            response.message = f"Save failed: {error}"
            self.get_logger().error(response.message)
        return response

    def publish_merged_cloud(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if len(self.merged_cloud.points) == 0:
            message = PointCloud2(
                header=Header(frame_id=self.base_frame, stamp=stamp),
                height=1,
                width=0,
                fields=[],
                is_bigendian=False,
                point_step=0,
                row_step=0,
                data=b"",
                is_dense=True,
            )
            self.publisher.publish(message)
            return

        points = np.asarray(self.merged_cloud.points)
        colors = np.asarray(self.merged_cloud.colors)
        if len(colors) != len(points):
            colors = np.ones((len(points), 3), dtype=np.float32)

        message = pointcloud2_from_arrays(
            points=points,
            colors=colors,
            frame_id=self.base_frame,
            stamp=stamp,
        )
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
