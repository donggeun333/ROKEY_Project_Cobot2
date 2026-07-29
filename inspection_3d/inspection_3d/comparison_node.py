"""기준 PCD와 검사 PCD를 비교하는 ROS 노드.

``pointcloud_pipeline``이 finalize로 저장한 결과 파일을 입력으로 받아,
voxel 점유 기반 유사도/누락/추가 비율을 계산한다. HMI와 robot_control은 이
노드의 compare 서비스 결과 문자열을 요약 정보로 사용한다.
"""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
from od_msg.srv import SrvPointCloudCompare
import rclpy
from rclpy.node import Node

from .occupancy_compare import (
    ComparisonConfig,
    build_summary,
    compare_point_cloud_files,
    save_comparison_outputs,
)

POINTCLOUD_SHARE_DIR = Path(get_package_share_directory("inspection_3d")).resolve()
DEFAULT_OBJECT_TYPE = "multitap"
DEFAULT_FILTERED_DIR = "data/pipeline/filtered"
DEFAULT_OUTPUT_DIR = "data/pipeline/comparison"
DEFAULT_REFERENCE_BY_OBJECT = {
    "multitap": str(POINTCLOUD_SHARE_DIR / "resource" / "good_multitap.pcd"),
    "bolt": str(POINTCLOUD_SHARE_DIR / "resource" / "good_bolt.pcd"),
}
OBJECT_ROI_BOUNDS = {
    "multitap": (
        np.array([0.31, 0.055, 0.015], dtype=np.float64),
        np.array([0.413, 0.159, 0.10], dtype=np.float64),
    ),
    "bolt": (
        np.array([0.308, -0.20, 0.0], dtype=np.float64),
        np.array([0.42, 0.00, 0.10], dtype=np.float64),
    ),
}


class PointCloudComparisonNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_comparison")

        self.object_type = self.get_string_param("object_type", DEFAULT_OBJECT_TYPE)
        self.filtered_dir = Path(
            self.get_string_param("filtered_dir", DEFAULT_FILTERED_DIR)
        ).resolve()
        self.output_dir = Path(
            self.get_string_param("output_dir", DEFAULT_OUTPUT_DIR)
        ).resolve()
        self.reference_by_object = self.build_reference_by_object()
        self.roi_by_object = self.build_roi_by_object()
        self.config = ComparisonConfig(
            voxel_size=self.get_float_param("voxel_size", 0.003),
            neighbor_tolerance=self.get_int_param("neighbor_tolerance", 1),
            min_similarity=self.get_float_param("min_similarity", 0.85),
            max_missing_ratio=self.get_float_param("max_missing_ratio", 0.10),
            max_added_ratio=self.get_float_param("max_added_ratio", 0.10),
            use_roi=self.get_bool_param("use_roi", True),
            roi_min=self.resolve_roi_bounds()[0],
            roi_max=self.resolve_roi_bounds()[1],
        )

        self.filtered_dir.mkdir(parents=True, exist_ok=True)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.last_output_dir: Path | None = None

        self.compare_service = self.create_service(
            SrvPointCloudCompare,
            "~/compare",
            self.handle_compare,
        )
        self.get_logger().info(
            "PointCloud comparison ready. "
            f"object_type={self.object_type}, filtered_dir={self.filtered_dir}, "
            f"output_dir={self.output_dir}, service=~/compare"
        )

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

    def build_reference_by_object(self) -> dict[str, str]:
        return {
            object_type: self.get_string_param(
                f"{object_type}_reference_path",
                default_path,
            )
            for object_type, default_path in DEFAULT_REFERENCE_BY_OBJECT.items()
        }

    def build_roi_by_object(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {
            object_type: (
                self.get_array_param(f"{object_type}_roi_min", roi_min),
                self.get_array_param(f"{object_type}_roi_max", roi_max),
            )
            for object_type, (roi_min, roi_max) in OBJECT_ROI_BOUNDS.items()
        }

    def resolve_roi_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        if self.object_type not in self.roi_by_object:
            raise ValueError(
                "Invalid object_type. Expected one of: "
                f"{', '.join(self.roi_by_object)}."
            )
        return self.roi_by_object[self.object_type]

    def resolve_reference_path(self) -> Path:
        if self.object_type not in self.reference_by_object:
            raise ValueError(
                "Invalid object_type. Expected one of: "
                f"{', '.join(self.reference_by_object)}."
            )

        reference_path = self.reference_by_object[self.object_type]
        if not reference_path:
            raise ValueError(
                f"Reference path is not configured for object_type={self.object_type}."
            )
        return Path(reference_path).resolve()

    def make_output_dir(self, test_path: Path) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return self.output_dir / f"{test_path.stem}_{timestamp}"

    def handle_compare(self, request, response):
        try:
            self.object_type = self.get_string_param("object_type", DEFAULT_OBJECT_TYPE)
            self.reference_by_object = self.build_reference_by_object()
            self.roi_by_object = self.build_roi_by_object()
            roi_min, roi_max = self.resolve_roi_bounds()
            current_config = replace(self.config, roi_min=roi_min, roi_max=roi_max)
            reference_path = self.resolve_reference_path()
            test_path = Path(request.test_path).resolve()
            if not request.test_path.strip():
                raise ValueError("test_path is required.")
            output_dir = self.make_output_dir(test_path)
            result = compare_point_cloud_files(reference_path, test_path, current_config)
            save_comparison_outputs(
                output_dir=output_dir,
                reference_path=reference_path,
                test_path=test_path,
                config=current_config,
                result=result,
            )
            self.last_output_dir = output_dir
            metrics = result.metrics
            response.success = True
            response.message = build_summary(result, output_dir)
            response.output_dir = str(output_dir)
            response.result_text = metrics.result_text
            response.similarity = float(metrics.similarity)
            response.missing_ratio = float(metrics.missing_ratio)
            response.added_ratio = float(metrics.added_ratio)
            self.get_logger().info(response.message)
        except Exception as error:
            response.success = False
            response.message = f"Comparison failed: {error}"
            response.output_dir = ""
            response.result_text = ""
            response.similarity = 0.0
            response.missing_ratio = 0.0
            response.added_ratio = 0.0
            self.get_logger().error(response.message)

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
