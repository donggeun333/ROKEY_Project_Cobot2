from __future__ import annotations

from datetime import datetime
from pathlib import Path
import copy
import struct
import threading

from geometry_msgs.msg import TransformStamped
import numpy as np
from od_msg.srv import SrvPointCloudCompare
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
    for field_name in ("x", "y", "z"):
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
        colors = rgb_float_to_colors(rgb_values)[valid]
    else:
        colors = np.ones((np.count_nonzero(valid), 3), dtype=np.float64)

    return points[valid], colors


def load_point_cloud(path: Path) -> o3d.geometry.PointCloud:
    cloud = o3d.io.read_point_cloud(str(path))
    if len(cloud.points) == 0:
        raise RuntimeError(f"Empty point cloud: {path}")
    return cloud


class PointCloudPipelineNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_pipeline")

        self.declare_parameter("object_type", "multitap")
        self.declare_parameter("input_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("save_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 1.0)
        self.declare_parameter("capture_dir", "data/pipeline/captures")
        self.declare_parameter("merged_dir", "data/pipeline/merged")
        self.declare_parameter("filtered_dir", "data/pipeline/filtered")
        self.declare_parameter("capture_voxel_size", 0.0)
        self.declare_parameter("icp_voxel_size", 0.002)
        self.declare_parameter("normal_radius", 0.008)
        self.declare_parameter("trigger_comparison_on_finalize", True)
        self.declare_parameter(
            "comparison_service",
            "/pointcloud_comparison/compare",
        )
        self.declare_parameter("max_correspondence_coarse", 0.03)
        self.declare_parameter("max_correspondence_fine", 0.01)
        self.declare_parameter("coarse_iterations", 60)
        self.declare_parameter("fine_iterations", 100)
        self.declare_parameter("min_fitness", 0.4)
        self.declare_parameter("max_rmse", 0.005)
        self.declare_parameter("outlier_nb_neighbors", 30)
        self.declare_parameter("outlier_std_ratio", 1.5)
        self.declare_parameter("multitap_roi_min", [0.28, 0.01, -0.03])
        self.declare_parameter("multitap_roi_max", [0.46, 0.20, 0.10])
        self.declare_parameter("bolt_roi_min", [0.28, -0.20, -0.03])
        self.declare_parameter("bolt_roi_max", [0.46, 0.00, 0.10])
        self.declare_parameter("dbscan_eps", 0.012)
        self.declare_parameter("dbscan_min_points", 20)

        self.object_type = str(self.get_parameter("object_type").value).strip().lower()
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.save_frame = str(self.get_parameter("save_frame").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.capture_dir = Path(self.get_parameter("capture_dir").value).resolve()
        self.merged_dir = Path(self.get_parameter("merged_dir").value).resolve()
        self.filtered_dir = Path(self.get_parameter("filtered_dir").value).resolve()
        self.capture_voxel_size = float(
            self.get_parameter("capture_voxel_size").value
        )
        self.icp_voxel_size = float(self.get_parameter("icp_voxel_size").value)
        self.normal_radius = float(self.get_parameter("normal_radius").value)
        self.trigger_comparison_on_finalize = bool(
            self.get_parameter("trigger_comparison_on_finalize").value
        )
        self.comparison_service_name = str(
            self.get_parameter("comparison_service").value
        )
        self.max_correspondence_coarse = float(
            self.get_parameter("max_correspondence_coarse").value
        )
        self.max_correspondence_fine = float(
            self.get_parameter("max_correspondence_fine").value
        )
        self.coarse_iterations = int(self.get_parameter("coarse_iterations").value)
        self.fine_iterations = int(self.get_parameter("fine_iterations").value)
        self.min_fitness = float(self.get_parameter("min_fitness").value)
        self.max_rmse = float(self.get_parameter("max_rmse").value)
        self.outlier_nb_neighbors = int(
            self.get_parameter("outlier_nb_neighbors").value
        )
        self.outlier_std_ratio = float(
            self.get_parameter("outlier_std_ratio").value
        )
        self.roi_by_object = {
            "multitap": (
                np.array(
                    self.get_parameter("multitap_roi_min").value, dtype=np.float64),
                np.array(self.get_parameter("multitap_roi_max").value, dtype=np.float64),
            ),
            
            "bolt": (
                np.array(self.get_parameter("bolt_roi_min").value, dtype=np.float64),
                np.array(self.get_parameter("bolt_roi_max").value, dtype=np.float64),
            ),
        }
        self.roi_min, self.roi_max = self.resolve_roi_bounds()
        self.dbscan_eps = float(self.get_parameter("dbscan_eps").value)
        self.dbscan_min_points = int(self.get_parameter("dbscan_min_points").value)

        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.merged_dir.mkdir(parents=True, exist_ok=True)
        self.filtered_dir.mkdir(parents=True, exist_ok=True)

        self.latest_cloud: PointCloud2 | None = None
        self.capture_paths: list[Path] = []
        self.last_merged_path: Path | None = None
        self.last_filtered_path: Path | None = None

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
        self.reset_service = self.create_service(
            Trigger,
            "~/reset",
            self.handle_reset,
        )
        self.finalize_service = self.create_service(
            Trigger,
            "~/finalize",
            self.handle_finalize,
        )
        self.comparison_client = self.create_client(
            SrvPointCloudCompare,
            self.comparison_service_name,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.get_logger().info(
            "PointCloud pipeline ready. "
            f"topic={self.input_topic}, frame={self.save_frame}, "
            f"object_type={self.object_type}, roi_min={self.roi_min.tolist()}, "
            f"roi_max={self.roi_max.tolist()}, services=~/reset, ~/capture, ~/finalize, "
            f"comparison_service={self.comparison_service_name}"
        )

    def handle_cloud(self, message: PointCloud2) -> None:
        self.latest_cloud = message

    def resolve_roi_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        if self.object_type not in self.roi_by_object:
            raise ValueError(
                "Invalid object_type. Expected one of: "
                f"{', '.join(self.roi_by_object)}."
            )

        return self.roi_by_object[self.object_type]


    def maybe_transform_points(
        self,
        points: np.ndarray,
        source_frame: str,
        stamp,
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

    def make_timestamp(self) -> str:
        return datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    def safe_frame_suffix(self) -> str:
        if not self.save_frame:
            return "source_frame"
        return self.save_frame.replace("/", "_")

    def cloud_from_latest_message(self) -> o3d.geometry.PointCloud:
        if self.latest_cloud is None:
            raise RuntimeError("No PointCloud2 message received yet.")

        source_frame = self.latest_cloud.header.frame_id
        stamp = self.latest_cloud.header.stamp
        points, colors = pointcloud2_to_arrays(self.latest_cloud)
        if len(points) == 0:
            raise RuntimeError("Received point cloud is empty.")

        points = self.maybe_transform_points(points, source_frame, stamp)

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        if self.capture_voxel_size > 0.0:
            cloud = cloud.voxel_down_sample(self.capture_voxel_size)
        if len(cloud.points) == 0:
            raise RuntimeError("Point cloud became empty after capture preprocessing.")
        return cloud

    def preprocess_for_icp(
        self,
        cloud: o3d.geometry.PointCloud,
    ) -> o3d.geometry.PointCloud:
        processed = cloud.voxel_down_sample(self.icp_voxel_size)
        if len(processed.points) == 0:
            return processed

        if len(processed.points) >= self.outlier_nb_neighbors:
            processed, _ = processed.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio,
            )

        if len(processed.points) == 0:
            return processed

        processed.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius,
                max_nn=50,
            )
        )
        return processed

    def run_icp(
        self,
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
    ) -> o3d.pipelines.registration.RegistrationResult:
        coarse_result = o3d.pipelines.registration.registration_icp(
            source=source,
            target=target,
            max_correspondence_distance=self.max_correspondence_coarse,
            init=np.eye(4),
            estimation_method=(
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            ),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=self.coarse_iterations,
            ),
        )

        return o3d.pipelines.registration.registration_icp(
            source=source,
            target=target,
            max_correspondence_distance=self.max_correspondence_fine,
            init=coarse_result.transformation,
            estimation_method=(
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            ),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-7,
                relative_rmse=1e-7,
                max_iteration=self.fine_iterations,
            ),
        )

    def merge_capture_paths(self) -> o3d.geometry.PointCloud:
        if len(self.capture_paths) < 2:
            raise RuntimeError("At least two captured point clouds are required.")

        merged_full = load_point_cloud(self.capture_paths[0])

        for index, source_path in enumerate(self.capture_paths[1:], start=1):
            source_full = load_point_cloud(source_path)
            source_down = self.preprocess_for_icp(source_full)
            target_down = self.preprocess_for_icp(merged_full)

            if len(source_down.points) < 3:
                raise RuntimeError(
                    f"Capture {index + 1} has fewer than 3 ICP points: {source_path}"
                )
            if len(target_down.points) < 3:
                raise RuntimeError("Merged target has fewer than 3 ICP points.")

            result = self.run_icp(source_down, target_down)
            if result.fitness < self.min_fitness:
                raise RuntimeError(
                    f"ICP fitness too low for {source_path.name}: {result.fitness:.4f}"
                )
            if result.inlier_rmse > self.max_rmse:
                raise RuntimeError(
                    f"ICP RMSE too high for {source_path.name}: {result.inlier_rmse:.6f}"
                )

            aligned_source = copy.deepcopy(source_full)
            aligned_source.transform(result.transformation)
            merged_full += aligned_source
            merged_full = merged_full.voxel_down_sample(self.icp_voxel_size)

            self.get_logger().info(
                f"ICP merged {index}/{len(self.capture_paths) - 1}: "
                f"{source_path.name}, fitness={result.fitness:.4f}, "
                f"rmse={result.inlier_rmse:.6f}"
            )

        return merged_full

    def postprocess_cloud(
        self,
        cloud: o3d.geometry.PointCloud,
    ) -> o3d.geometry.PointCloud:
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=self.roi_min,
            max_bound=self.roi_max,
        )
        roi_cloud = cloud.crop(bbox)
        if len(roi_cloud.points) == 0:
            raise RuntimeError("No points found inside ROI.")

        processed = roi_cloud.voxel_down_sample(self.icp_voxel_size)
        if len(processed.points) == 0:
            raise RuntimeError("No points remain after ROI downsampling.")

        if len(processed.points) >= self.outlier_nb_neighbors:
            processed, _ = processed.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio,
            )
        if len(processed.points) == 0:
            raise RuntimeError("No points remain after outlier removal.")

        labels = np.asarray(
            processed.cluster_dbscan(
                eps=self.dbscan_eps,
                min_points=self.dbscan_min_points,
                print_progress=False,
            )
        )
        if labels.size == 0:
            raise RuntimeError("DBSCAN returned no labels.")

        valid_labels = labels[labels >= 0]
        if valid_labels.size == 0:
            raise RuntimeError("All points were classified as DBSCAN noise.")

        cluster_ids, cluster_counts = np.unique(valid_labels, return_counts=True)
        largest_cluster_id = cluster_ids[np.argmax(cluster_counts)]
        largest_indices = np.where(labels == largest_cluster_id)[0]
        filtered = processed.select_by_index(largest_indices)
        if len(filtered.points) == 0:
            raise RuntimeError("Selected DBSCAN cluster is empty.")
        return filtered

    def handle_capture(self, request, response):
        del request

        try:
            cloud = self.cloud_from_latest_message()
            timestamp = self.make_timestamp()
            output_path = (
                self.capture_dir
                / f"capture_{timestamp}_{self.safe_frame_suffix()}.pcd"
            )
            saved = o3d.io.write_point_cloud(str(output_path), cloud)
            if not saved:
                raise RuntimeError(f"Failed to save {output_path}")

            self.capture_paths.append(output_path)
            response.success = True
            response.message = str(output_path)
            self.get_logger().info(
                f"Captured {len(self.capture_paths)} cloud(s): {output_path}"
            )
        except Exception as error:
            response.success = False
            response.message = f"Capture failed: {error}"
            self.get_logger().error(response.message)

        return response

    def handle_reset(self, request, response):
        del request

        self.capture_paths.clear()
        self.last_merged_path = None
        self.last_filtered_path = None

        response.success = True
        response.message = "Pipeline session reset."
        self.get_logger().info(response.message)
        return response

    def trigger_comparison_async(self) -> None:
        if not self.trigger_comparison_on_finalize:
            return

        def worker() -> None:
            if not self.comparison_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning(
                    "Comparison service is unavailable. "
                    f"service={self.comparison_service_name}"
                )
                return

            if self.last_filtered_path is None:
                self.get_logger().warning(
                    "Comparison trigger skipped because last_filtered_path is empty."
                )
                return

            request = SrvPointCloudCompare.Request()
            request.test_path = str(self.last_filtered_path)
            future = self.comparison_client.call_async(request)

            def log_result(done_future) -> None:
                try:
                    result = done_future.result()
                    if result is None:
                        self.get_logger().error(
                            "Comparison service returned no response."
                        )
                    elif result.success:
                        self.get_logger().info(
                            f"Comparison completed: {result.message}"
                        )
                    else:
                        self.get_logger().warning(
                            f"Comparison reported failure: {result.message}"
                        )
                except Exception as error:
                    self.get_logger().error(
                        f"Comparison request failed: {error}"
                    )

            future.add_done_callback(log_result)

        threading.Thread(target=worker, daemon=True).start()

    def handle_finalize(self, request, response):
        del request

        try:
            merged_cloud = self.merge_capture_paths()
            filtered_cloud = self.postprocess_cloud(merged_cloud)

            timestamp = self.make_timestamp()
            merged_path = self.merged_dir / f"merged_icp_{timestamp}.pcd"
            filtered_path = self.filtered_dir / f"filtered_dbscan_{timestamp}.pcd"

            merged_saved = o3d.io.write_point_cloud(str(merged_path), merged_cloud)
            filtered_saved = o3d.io.write_point_cloud(str(filtered_path), filtered_cloud)
            if not merged_saved:
                raise RuntimeError(f"Failed to save {merged_path}")
            if not filtered_saved:
                raise RuntimeError(f"Failed to save {filtered_path}")

            self.last_merged_path = merged_path
            self.last_filtered_path = filtered_path
            self.trigger_comparison_async()
            response.success = True
            response.message = (
                f"merged={merged_path}, filtered={filtered_path}, "
                f"captures={len(self.capture_paths)}, "
                f"comparison_triggered={self.trigger_comparison_on_finalize}"
            )
            self.get_logger().info(response.message)
        except Exception as error:
            response.success = False
            response.message = f"Finalize failed: {error}"
            self.get_logger().error(response.message)

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
