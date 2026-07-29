"""PCD 파일을 PointCloud2로 발행해 RViz2에서 확인하는 유틸리티.

사용 예:

    python3 -m inspection_3d.view_pcd /path/to/file.pcd

RViz2에서는 Fixed Frame을 ``base_link``로 두고, ``/pcd_view`` 토픽의
PointCloud2 디스플레이를 추가하면 된다.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


PUBLISH_TOPIC = "/pcd_view"
DEFAULT_FRAME_ID = "base_link"


def build_cloud_message(
    points: np.ndarray,
    colors: np.ndarray | None,
    frame_id: str,
    stamp,
) -> PointCloud2:
    """numpy 점군 배열을 PointCloud2 메시지로 변환한다."""

    if colors is not None and len(colors) == len(points):
        rgb8 = np.clip(colors * 255.0, 0, 255).astype(np.uint8)
        rgb_packed = (
            (rgb8[:, 0].astype(np.uint32) << 16)
            | (rgb8[:, 1].astype(np.uint32) << 8)
            | rgb8[:, 2].astype(np.uint32)
        ).view(np.float32)
        cloud_data = [
            [float(point[0]), float(point[1]), float(point[2]), float(rgb)]
            for point, rgb in zip(points, rgb_packed)
        ]
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
    else:
        cloud_data = points.astype(np.float32).tolist()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    return point_cloud2.create_cloud(header, fields, cloud_data)


class PcdPublisher(Node):
    """지정한 PCD를 반복 발행하는 노드."""

    def __init__(self, pcd_path: str, frame_id: str = DEFAULT_FRAME_ID) -> None:
        super().__init__("pcd_view_publisher")
        self._publisher = self.create_publisher(PointCloud2, PUBLISH_TOPIC, 10)
        self._frame_id = frame_id
        self._pcd_path = Path(pcd_path).expanduser().resolve()

        cloud = o3d.io.read_point_cloud(str(self._pcd_path))
        if cloud.is_empty():
            raise RuntimeError(f"PCD가 비어 있습니다: {self._pcd_path}")

        self._points = np.asarray(cloud.points, dtype=np.float32)
        self._colors = (
            np.asarray(cloud.colors, dtype=np.float32)
            if cloud.has_colors()
            else None
        )

        self._timer = self.create_timer(1.0, self.publish_once)
        self.get_logger().info(
            f"Loaded PCD: {self._pcd_path} -> topic={PUBLISH_TOPIC}, "
            f"frame_id={self._frame_id}, points={len(self._points)}"
        )

    def publish_once(self) -> None:
        msg = build_cloud_message(
            self._points,
            self._colors,
            self._frame_id,
            self.get_clock().now().to_msg(),
        )
        self._publisher.publish(msg)


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv if argv is None else argv)
    if len(argv) < 2:
        raise SystemExit(
            "usage: python3 -m inspection_3d.view_pcd <pcd_path> [frame_id]"
        )

    pcd_path = argv[1]
    frame_id = argv[2] if len(argv) >= 3 else DEFAULT_FRAME_ID

    rclpy.init()
    node = PcdPublisher(pcd_path, frame_id=frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
