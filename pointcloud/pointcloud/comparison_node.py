from __future__ import annotations

from datetime import datetime
from pathlib import Path

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


class PointCloudComparisonNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_comparison")

        self.declare_parameter("object_type", "multitap")
        self.declare_parameter("filtered_dir", "data/pipeline/filtered")
        self.declare_parameter("output_dir", "data/pipeline/comparison")
        self.declare_parameter("multitap_reference_path", "")
        self.declare_parameter("bolt_reference_path", "")
        self.declare_parameter("use_roi", True)
        self.declare_parameter("multitap_roi_min", [0.28, 0.01, -0.03])
        self.declare_parameter("multitap_roi_max", [0.46, 0.20, 0.10])
        self.declare_parameter("bolt_roi_min", [0.28, -0.20, -0.03])
        self.declare_parameter("bolt_roi_max", [0.46, 0.00, 0.10])
        self.declare_parameter("voxel_size", 0.003)
        self.declare_parameter("neighbor_tolerance", 1)
        self.declare_parameter("min_similarity", 0.85)
        self.declare_parameter("max_missing_ratio", 0.10)
        self.declare_parameter("max_added_ratio", 0.10)

        self.object_type = str(self.get_parameter("object_type").value).strip().lower()
        self.filtered_dir = Path(self.get_parameter("filtered_dir").value).resolve()
        self.output_dir = Path(self.get_parameter("output_dir").value).resolve()
        self.reference_by_object = {
            "multitap": str(self.get_parameter("multitap_reference_path").value).strip(),
            "bolt": str(self.get_parameter("bolt_reference_path").value).strip(),
        }
        self.roi_by_object = {
            "multitap": (
                np.array(self.get_parameter("multitap_roi_min").value, dtype=np.float64),
                np.array(self.get_parameter("multitap_roi_max").value, dtype=np.float64),
            ),
            "bolt": (
                np.array(self.get_parameter("bolt_roi_min").value, dtype=np.float64),
                np.array(self.get_parameter("bolt_roi_max").value, dtype=np.float64),
            ),
        }
        self.config = ComparisonConfig(
            voxel_size=float(self.get_parameter("voxel_size").value),
            neighbor_tolerance=int(self.get_parameter("neighbor_tolerance").value),
            min_similarity=float(self.get_parameter("min_similarity").value),
            max_missing_ratio=float(self.get_parameter("max_missing_ratio").value),
            max_added_ratio=float(self.get_parameter("max_added_ratio").value),
            use_roi=bool(self.get_parameter("use_roi").value),
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
            reference_path = self.resolve_reference_path()
            test_path = Path(request.test_path).resolve()
            if not request.test_path.strip():
                raise ValueError("test_path is required.")
            output_dir = self.make_output_dir(test_path)
            result = compare_point_cloud_files(reference_path, test_path, self.config)
            save_comparison_outputs(
                output_dir=output_dir,
                reference_path=reference_path,
                test_path=test_path,
                config=self.config,
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
