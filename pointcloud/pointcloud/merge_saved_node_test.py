from __future__ import annotations

from datetime import datetime
from pathlib import Path

import open3d as o3d
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MergeSavedNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_merge")

        self.declare_parameter("input_dir", "data/captures")
        self.declare_parameter(
            "glob_pattern",
            "capture_*_base_link.pcd",
        )
        self.declare_parameter("output_dir", "data/merged")
        self.declare_parameter("voxel_size", 0.003)
        self.declare_parameter("outlier_neighbors", 20)
        self.declare_parameter("outlier_std_ratio", 2.0)

        self.input_dir = Path(
            self.get_parameter("input_dir").value
        ).resolve()

        self.glob_pattern = self.get_parameter(
            "glob_pattern"
        ).value

        self.output_dir = Path(
            self.get_parameter("output_dir").value
        ).resolve()

        self.voxel_size = float(
            self.get_parameter("voxel_size").value
        )

        self.outlier_neighbors = int(
            self.get_parameter("outlier_neighbors").value
        )

        self.outlier_std_ratio = float(
            self.get_parameter("outlier_std_ratio").value
        )

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.merge_service = self.create_service(
            Trigger,
            "~/merge",
            self.handle_merge,
        )

        self.get_logger().info(
            f"PointCloud merge service ready. "
            f"service=~/merge, input={self.input_dir}, "
            f"pattern={self.glob_pattern}"
        )

    def preprocess(
        self,
        pcd: o3d.geometry.PointCloud,
    ) -> o3d.geometry.PointCloud:
        if len(pcd.points) == 0:
            raise ValueError("Point cloud is empty.")

        if self.voxel_size > 0.0:
            pcd = pcd.voxel_down_sample(
                self.voxel_size
            )

        return pcd

    def run(self) -> Path:
        if not self.input_dir.exists():
            raise FileNotFoundError(
                f"Input directory not found: {self.input_dir}"
            )

        capture_paths = sorted(
            self.input_dir.glob(self.glob_pattern)
        )

        if not capture_paths:
            raise FileNotFoundError(
                f"No PCD files found: "
                f"{self.input_dir / self.glob_pattern}"
            )

        merged = o3d.geometry.PointCloud()
        valid_count = 0

        for capture_path in capture_paths:
            self.get_logger().info(
                f"Loading {capture_path}"
            )

            current = o3d.io.read_point_cloud(
                str(capture_path)
            )

            if len(current.points) == 0:
                self.get_logger().warning(
                    f"Skip empty cloud: {capture_path}"
                )
                continue

            current = self.preprocess(current)
            merged += current
            valid_count += 1

            self.get_logger().info(
                f"Accumulated files={valid_count}, "
                f"points={len(merged.points)}"
            )

        if valid_count == 0 or len(merged.points) == 0:
            raise RuntimeError(
                "Merged point cloud is empty."
            )

        # 모든 PCD를 합친 뒤 최종 다운샘플링
        if self.voxel_size > 0.0:
            merged = merged.voxel_down_sample(
                self.voxel_size
            )

        # 최종 결과에만 이상치 제거
        if len(merged.points) >= self.outlier_neighbors:
            merged, _ = merged.remove_statistical_outlier(
                nb_neighbors=self.outlier_neighbors,
                std_ratio=self.outlier_std_ratio,
            )

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        output_path = (
            self.output_dir
            / f"merged_{timestamp}_base_link.pcd"
        )

        saved = o3d.io.write_point_cloud(
            str(output_path),
            merged,
        )

        if not saved:
            raise RuntimeError(
                f"Failed to save merged cloud: "
                f"{output_path}"
            )

        self.get_logger().info(
            f"Saved merged cloud: {output_path}"
        )
        self.get_logger().info(
            f"Merged files={valid_count}, "
            f"final points={len(merged.points)}"
        )

        return output_path

    def handle_merge(self, request, response):
        del request

        try:
            output_path = self.run()

            response.success = True
            response.message = str(output_path)

        except Exception as error:
            response.success = False
            response.message = f"Merge failed: {error}"

            self.get_logger().error(
                response.message
            )

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MergeSavedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()