"""다중 시점 캡처와 ICP 누적 병합을 담당하는 ROS 노드.

``robot_control.pointcloud_inspector_task``가 이 노드의 서비스들을 호출한다.
각 capture 요청은 최신 PointCloud2를 받아 누적 점군에 즉시 ICP 병합하고,
finalize는 ROI crop/outlier 제거/DBSCAN 후 최종 PCD를 저장한다.
"""

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
from rcl_interfaces.msg import SetParametersResult
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

DEFAULT_OBJECT_TYPE = "multitap"
DEFAULT_INPUT_TOPIC = "/camera/camera/depth/color/points"
DEFAULT_SAVE_FRAME = "base_link"
DEFAULT_CAPTURE_DIR = "data/pipeline/captures"
DEFAULT_MERGED_DIR = "data/pipeline/merged"
DEFAULT_FILTERED_DIR = "data/pipeline/filtered"
DEFAULT_COMPARISON_SERVICE = "/pointcloud_comparison/compare"
DEFAULT_SAVE_CAPTURE = False
DEFAULT_SAVE_MERGED = False
DEFAULT_SAVE_FILTERED = True
OBJECT_ROI_BOUNDS = {
    "multitap": (
        np.array([0.31, 0.055, 0.015], dtype=np.float64),
        np.array([0.413, 0.159, 0.10], dtype=np.float64),
    ),
    "bolt": (
        np.array([0.308, -0.20, 0], dtype=np.float64),
        np.array([0.42, 0.00, 0.10], dtype=np.float64),
    ),
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

class PointCloudPipelineNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_pipeline")

        self.object_type = self.get_string_param("object_type", DEFAULT_OBJECT_TYPE)
        self.input_topic = self.get_string_param("input_topic", DEFAULT_INPUT_TOPIC)
        self.save_frame = self.get_string_param("save_frame", DEFAULT_SAVE_FRAME)
        self.tf_timeout_sec = self.get_float_param("tf_timeout_sec", 1.0)
        self.capture_dir = Path(
            self.get_string_param("capture_dir", DEFAULT_CAPTURE_DIR)
        ).resolve()
        self.merged_dir = Path(
            self.get_string_param("merged_dir", DEFAULT_MERGED_DIR)
        ).resolve()
        self.filtered_dir = Path(
            self.get_string_param("filtered_dir", DEFAULT_FILTERED_DIR)
        ).resolve()
        self.capture_voxel_size = self.get_float_param("capture_voxel_size", 0.0)
        self.save_capture = self.get_bool_param("save_capture", DEFAULT_SAVE_CAPTURE)
        self.save_merged = self.get_bool_param("save_merged", DEFAULT_SAVE_MERGED)
        self.save_filtered = self.get_bool_param("save_filtered", DEFAULT_SAVE_FILTERED)
        self.icp_voxel_size = self.get_float_param("icp_voxel_size", 0.001)
        self.normal_radius = self.get_float_param("normal_radius", 0.008)
        self.trigger_comparison_on_finalize = self.get_bool_param(
            "trigger_comparison_on_finalize",
            True,
        )
        self.comparison_service_name = self.get_string_param(
            "comparison_service",
            DEFAULT_COMPARISON_SERVICE,
        )
        self.max_correspondence_coarse = self.get_float_param(
            "max_correspondence_coarse",
            0.03,
        )
        self.max_correspondence_fine = self.get_float_param(
            "max_correspondence_fine",
            0.01,
        )
        self.coarse_iterations = self.get_int_param("coarse_iterations", 60)
        self.fine_iterations = self.get_int_param("fine_iterations", 100)
        # 멀티탭 스캔은 시야 변화가 커서 볼트보다 ICP 품질 편차가 크다.
        # 기본 허용치를 약간 완화해 7~8번째 캡처에서 과도하게 중단되지 않도록 한다.
        self.min_fitness = self.get_float_param("min_fitness", 0.3)
        self.max_rmse = self.get_float_param("max_rmse", 0.008)
        self.outlier_nb_neighbors = self.get_int_param("outlier_nb_neighbors", 10)
        self.outlier_std_ratio = self.get_float_param("outlier_std_ratio", 2.0)
        self.roi_by_object = self.build_roi_by_object()
        self.roi_min, self.roi_max = self.resolve_roi_bounds()
        self.dbscan_eps = self.get_float_param("dbscan_eps", 0.012)
        self.dbscan_min_points = self.get_int_param("dbscan_min_points", 20)

        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.merged_dir.mkdir(parents=True, exist_ok=True)
        self.filtered_dir.mkdir(parents=True, exist_ok=True)

        self.latest_cloud: PointCloud2 | None = None
        self.capture_count = 0
        self.merged_cloud: o3d.geometry.PointCloud | None = None
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
        self.add_on_set_parameters_callback(self.handle_parameter_update)

    def get_param(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def get_string_param(self, name: str, default: str) -> str:
        return str(self.get_param(name, default)).strip()

    def get_float_param(self, name: str, default: float) -> float:
        return float(self.get_param(name, default))

    def get_int_param(self, name: str, default: int) -> int:
        return int(self.get_param(name, default))

    def get_bool_param(self, name: str, default: bool) -> bool:
        return bool(self.get_param(name, default))

    def get_array_param(self, name: str, default: np.ndarray) -> np.ndarray:
        return np.array(self.get_param(name, default.tolist()), dtype=np.float64)

    def handle_parameter_update(self, parameters):
        updated_object_type = self.object_type
        updated_filtered_dir = self.filtered_dir

        for parameter in parameters:
            if parameter.name == "object_type":
                updated_object_type = str(parameter.value).strip()
            elif parameter.name == "filtered_dir":
                updated_filtered_dir = Path(str(parameter.value).strip()).resolve()

        if updated_object_type not in self.roi_by_object:
            return SetParametersResult(
                successful=False,
                reason=(
                    "Invalid object_type. Expected one of: "
                    f"{', '.join(self.roi_by_object)}."
                ),
            )

        self.object_type = updated_object_type
        self.filtered_dir = updated_filtered_dir
        self.filtered_dir.mkdir(parents=True, exist_ok=True)
        self.roi_min, self.roi_max = self.resolve_roi_bounds()

        self.get_logger().info(
            "Updated runtime parameters: "
            f"object_type={self.object_type}, filtered_dir={self.filtered_dir}"
        )
        return SetParametersResult(successful=True)

    def build_roi_by_object(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {
            object_type: (
                self.get_array_param(f"{object_type}_roi_min", roi_min),
                self.get_array_param(f"{object_type}_roi_max", roi_max),
            )
            for object_type, (roi_min, roi_max) in OBJECT_ROI_BOUNDS.items()
        }

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

        requested_time = Time.from_msg(stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.save_frame,
                source_frame,
                requested_time,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except tf2_ros.ExtrapolationException as error:
            self.get_logger().warn(
                "TF lookup at message stamp failed; retrying with latest transform. "
                f"source_frame={source_frame}, target_frame={self.save_frame}, error={error}"
            )
            transform = self.tf_buffer.lookup_transform(
                self.save_frame,
                source_frame,
                Time(),
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

    def merge_into_accumulated_cloud(
        self,
        source_full: o3d.geometry.PointCloud,
    ) -> None:
        if self.merged_cloud is None:
            self.merged_cloud = copy.deepcopy(source_full)
            self.capture_count = 1
            self.get_logger().info("Initialized merged cloud from first capture.")
            return

        source_down = self.preprocess_for_icp(source_full)
        target_down = self.preprocess_for_icp(self.merged_cloud)

        if len(source_down.points) < 3:
            raise RuntimeError("New capture has fewer than 3 ICP points.")
        if len(target_down.points) < 3:
            raise RuntimeError("Merged target has fewer than 3 ICP points.")

        result = self.run_icp(source_down, target_down)
        if result.fitness < self.min_fitness:
            raise RuntimeError(
                f"ICP fitness too low for capture {self.capture_count + 1}: {result.fitness:.4f}"
            )
        if result.inlier_rmse > self.max_rmse:
            raise RuntimeError(
                f"ICP RMSE too high for capture {self.capture_count + 1}: {result.inlier_rmse:.6f}"
            )

        aligned_source = copy.deepcopy(source_full)
        aligned_source.transform(result.transformation)
        self.merged_cloud += aligned_source
        self.merged_cloud = self.merged_cloud.voxel_down_sample(self.icp_voxel_size)
        self.capture_count += 1
        self.get_logger().info(
            f"ICP merged capture {self.capture_count}: "
            f"fitness={result.fitness:.4f}, rmse={result.inlier_rmse:.6f}"
        )

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
            self.merge_into_accumulated_cloud(cloud)
            output_path = None
            if self.save_capture:
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
            response.message = str(output_path) if output_path is not None else "captured_in_memory"
            capture_count = self.capture_count
            if output_path is not None:
                self.get_logger().info(
                    f"Captured {capture_count} cloud(s): {output_path}"
                )
            else:
                self.get_logger().info(
                    f"Captured {capture_count} cloud(s): saved_in_memory_only"
                )
        except Exception as error:
            response.success = False
            response.message = f"Capture failed: {error}"
            self.get_logger().error(response.message)

        return response

    def handle_reset(self, request, response):
        del request

        self.object_type = self.get_string_param("object_type", DEFAULT_OBJECT_TYPE)
        self.roi_min, self.roi_max = self.resolve_roi_bounds()
        self.capture_count = 0
        self.merged_cloud = None
        self.capture_paths.clear()
        self.last_merged_path = None
        self.last_filtered_path = None

        response.success = True
        response.message = f"Pipeline session reset. object_type={self.object_type}"
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
            if self.capture_count < 2 or self.merged_cloud is None:
                raise RuntimeError("At least two captured point clouds are required.")

            merged_cloud = copy.deepcopy(self.merged_cloud)
            filtered_cloud = self.postprocess_cloud(merged_cloud)

            timestamp = self.make_timestamp()
            merged_path = self.merged_dir / f"merged_icp_{timestamp}.pcd"
            filtered_path = self.filtered_dir / f"filtered_dbscan_{timestamp}.pcd"

            if self.save_merged:
                merged_saved = o3d.io.write_point_cloud(str(merged_path), merged_cloud)
                if not merged_saved:
                    raise RuntimeError(f"Failed to save {merged_path}")
                self.last_merged_path = merged_path
            else:
                self.last_merged_path = None

            if self.save_filtered:
                filtered_saved = o3d.io.write_point_cloud(str(filtered_path), filtered_cloud)
                if not filtered_saved:
                    raise RuntimeError(f"Failed to save {filtered_path}")
                self.last_filtered_path = filtered_path
            else:
                raise RuntimeError("save_filtered must remain enabled for comparison.")

            self.trigger_comparison_async()
            response.success = True
            response.message = (
                f"merged={merged_path if self.save_merged else 'not_saved'}, "
                f"filtered={filtered_path}, "
                f"captures={self.capture_count}, "
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
