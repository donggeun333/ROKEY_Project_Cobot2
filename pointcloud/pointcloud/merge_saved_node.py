from __future__ import annotations

from datetime import datetime
from pathlib import Path

import open3d as o3d
import rclpy
from rclpy.node import Node


class MergeSavedNode(Node):
    def __init__(self) -> None:
        super().__init__("merge_saved_node")

        self.declare_parameter("input_dir", "data/captures")
        self.declare_parameter("glob_pattern", "*.pcd")
        self.declare_parameter("output_dir", "data/merged")
        self.declare_parameter("voxel_size", 0.003)
        self.declare_parameter("outlier_neighbors", 20)
        self.declare_parameter("outlier_std_ratio", 2.0)

        self.input_dir = Path(self.get_parameter("input_dir").value).resolve()
        self.glob_pattern = self.get_parameter("glob_pattern").value
        self.output_dir = Path(self.get_parameter("output_dir").value).resolve()
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.outlier_neighbors = int(self.get_parameter("outlier_neighbors").value)
        self.outlier_std_ratio = float(
            self.get_parameter("outlier_std_ratio").value
        )

        self.output_dir.mkdir(parents=True, exist_ok=True)

    def preprocess(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        if len(pcd.points) == 0:
            raise ValueError("Point cloud is empty.")

        downsampled = pcd.voxel_down_sample(self.voxel_size)
        if len(downsampled.points) >= self.outlier_neighbors:
            downsampled, _ = downsampled.remove_statistical_outlier(
                nb_neighbors=self.outlier_neighbors,
                std_ratio=self.outlier_std_ratio,
            )
        return downsampled

    def run(self) -> Path:
        if not self.input_dir.exists():
            raise FileNotFoundError(f"Input directory not found: {self.input_dir}")

        capture_paths = sorted(self.input_dir.glob(self.glob_pattern))
        if not capture_paths:
            raise FileNotFoundError(
                f"No PCD files found: {self.input_dir / self.glob_pattern}"
            )

        merged = o3d.geometry.PointCloud()

        for capture_path in capture_paths:
            self.get_logger().info(f"Loading {capture_path}")
            current = o3d.io.read_point_cloud(str(capture_path))
            if len(current.points) == 0:
                self.get_logger().warn(f"Skip empty cloud: {capture_path}")
                continue

            current = self.preprocess(current)

            if len(merged.points) == 0:
                merged = o3d.geometry.PointCloud(current)
            else:
                merged += current
                merged = merged.voxel_down_sample(self.voxel_size)
                if len(merged.points) >= self.outlier_neighbors:
                    merged, _ = merged.remove_statistical_outlier(
                        nb_neighbors=self.outlier_neighbors,
                        std_ratio=self.outlier_std_ratio,
                    )

            self.get_logger().info(f"Merged points: {len(merged.points)}")

        if len(merged.points) == 0:
            raise RuntimeError("Merged point cloud is empty.")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = self.output_dir / f"merged_{timestamp}.pcd"
        saved = o3d.io.write_point_cloud(str(output_path), merged)
        if not saved:
            raise RuntimeError(f"Failed to save merged cloud: {output_path}")

        self.get_logger().info(f"Saved merged cloud: {output_path}")
        return output_path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MergeSavedNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
