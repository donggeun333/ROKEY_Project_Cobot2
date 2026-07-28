"""공구 인식 결과를 ROS 토픽으로 발행하는 perception 노드.

카메라 영상, 깊이, YOLO 결과를 모아 ``Scene`` 스냅샷을 만들고 task manager가
소비할 수 있게 JSON/이미지 형태로 발행한다. 공구 정리와 전달은 이 노드가 만든
동일한 관측 모델을 공유한다.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque

import numpy as np

from .lan_color import (
    LanColorConfig,
    VerdictStabilizer,
    format_lan_color_log,
    speech_for_status,
)
from .image_messages import bgr8_to_image_message
from .paths import default_weights_path
from .perception_core import ToolDetector
from .schema import Scene


def _message_time(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _camera_qos(reliability_name: str):
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    reliability = (
        ReliabilityPolicy.BEST_EFFORT
        if reliability_name.lower() == "best_effort"
        else ReliabilityPolicy.RELIABLE
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


class PerceptionNode:
    """ROS facade with a latest-frame-only inference worker."""

    def __init__(self):
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Bool, String

        self.node = Node("tool_sorter_perception")
        self.bridge = CvBridge()
        self._declare_parameters()

        weights = self._parameter("weights_path")
        self.node.get_logger().info(f"YOLO segmentation model loading: {weights}")
        from ultralytics import YOLO

        model = YOLO(weights)
        if bool(self._parameter("lan_color_enabled")):
            required = {"lan_cable", "lan_port"}
            missing = required - set(model.names.values())
            if missing:
                raise ValueError(
                    "LAN color inspection requires model classes "
                    f"{sorted(missing)}; available={model.names}"
                )
        self.detector = ToolDetector(
            model=model,
            confidence=float(self._parameter("confidence")),
            part_confidence=float(self._parameter("part_confidence")),
            min_valid_depth_ratio=float(
                self._parameter("min_valid_depth_ratio")
            ),
            overlap_threshold=float(self._parameter("overlap_threshold")),
            part_min_containment=float(
                self._parameter("part_min_containment")
            ),
            device=str(self._parameter("device")),
            image_size=int(self._parameter("image_size")),
            lan_color_enabled=bool(self._parameter("lan_color_enabled")),
            lan_color_confidence=float(
                self._parameter("lan_color_confidence")
            ),
            lan_color_config=LanColorConfig(
                min_saturation=int(
                    self._parameter("lan_color_min_saturation")
                ),
                min_value=int(self._parameter("lan_color_min_value")),
                max_value=int(self._parameter("lan_color_max_value")),
                cable_tip_fraction=float(
                    self._parameter("lan_color_cable_tip_fraction")
                ),
                max_hue_distance=float(
                    self._parameter("lan_color_max_hue_distance")
                ),
                max_saturation_distance=float(
                    self._parameter("lan_color_max_saturation_distance")
                ),
                max_value_distance=float(
                    self._parameter("lan_color_max_value_distance")
                ),
                max_pair_gap_ratio=float(
                    self._parameter("lan_color_max_pair_gap_ratio")
                ),
                duplicate_iou=float(
                    self._parameter("lan_color_duplicate_iou")
                ),
            ),
        )
        self.scene_publisher = self.node.create_publisher(
            String, "/tool_sorter/perception/scene", 1
        )
        self.image_publisher = self.node.create_publisher(
            Image, "/tool_sorter/perception/annotated", 1
        )
        self.info_publisher = self.node.create_publisher(
            CameraInfo, "/tool_sorter/perception/camera_info", 1
        )
        self.lan_color_publisher = self.node.create_publisher(
            String, "/tool_sorter/inspection/lan_color", 10
        )
        self.lan_color_ok_publisher = self.node.create_publisher(
            Bool, "/tool_sorter/inspection/lan_color_ok", 10
        )
        self.speech_publisher = self.node.create_publisher(
            String, "/tool_sorter/speech", 10
        )
        self.lan_color_stabilizer = VerdictStabilizer(
            stable_frames=int(self._parameter("lan_color_stable_frames")),
            reset_after_seconds=float(
                self._parameter("lan_color_reset_after_seconds")
            ),
        )

        qos = _camera_qos(str(self._parameter("camera_qos")))
        self.color_subscriber = self.node.create_subscription(
            Image,
            str(self._parameter("color_topic")),
            self._on_color,
            qos,
        )
        self.depth_subscriber = self.node.create_subscription(
            Image,
            str(self._parameter("depth_topic")),
            self._on_depth,
            qos,
        )
        self.camera_info_subscriber = self.node.create_subscription(
            CameraInfo,
            str(self._parameter("camera_info_topic")),
            self._on_camera_info,
            qos,
        )

        self._sync_tolerance = float(self._parameter("sync_tolerance_ms")) / 1000.0
        self._depth_scale = float(self._parameter("depth_scale_to_mm"))
        self._color_buffer = deque(maxlen=6)
        self._depth_buffer = deque(maxlen=6)
        self._buffer_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_event = threading.Event()
        self._stop_event = threading.Event()
        self._sequence = 0
        self._fps = 0.0
        self._processed = 0
        self._color_received = 0
        self._depth_received = 0
        self._last_sync_delta_ms = None
        self._last_frame_at = time.monotonic()
        self._last_size_warning_at = 0.0
        self._last_lan_log_key = None
        self._worker = threading.Thread(
            target=self._inference_loop,
            name="tool-sorter-inference",
            daemon=True,
        )
        self._worker.start()
        self.node.create_timer(2.0, self._watchdog)
        self.node.get_logger().info(
            "Perception ready: synchronized RGB/depth, latest-frame-only queue"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "weights_path": default_weights_path(),
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "camera_qos": "reliable",
            "sync_tolerance_ms": 50.0,
            "depth_scale_to_mm": 1.0,
            "confidence": 0.4,
            "part_confidence": 0.15,
            "min_valid_depth_ratio": 0.3,
            "overlap_threshold": 0.05,
            "part_min_containment": 0.5,
            "image_size": 640,
            "device": "",
            "lan_color_enabled": True,
            "lan_color_confidence": 0.15,
            "lan_color_min_saturation": 45,
            "lan_color_min_value": 25,
            "lan_color_max_value": 245,
            "lan_color_cable_tip_fraction": 0.35,
            "lan_color_max_hue_distance": 12.0,
            "lan_color_max_saturation_distance": 130.0,
            "lan_color_max_value_distance": 190.0,
            "lan_color_max_pair_gap_ratio": 0.04,
            "lan_color_duplicate_iou": 0.55,
            "lan_color_stable_frames": 3,
            "lan_color_reset_after_seconds": 2.0,
        }
        for name, value in defaults.items():
            self.node.declare_parameter(name, value)

    def _parameter(self, name: str):
        return self.node.get_parameter(name).value

    def _on_color(self, message) -> None:
        self._color_received += 1
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.node.get_logger().warning(f"Color conversion failed: {error}")
            return
        with self._buffer_lock:
            self._color_buffer.append((_message_time(message), image, message.header))
            self._match_frame_locked()

    def _on_depth(self, message) -> None:
        self._depth_received += 1
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            depth_mm = np.asarray(depth, dtype=np.float32) * self._depth_scale
        except Exception as error:
            self.node.get_logger().warning(f"Depth conversion failed: {error}")
            return
        with self._buffer_lock:
            self._depth_buffer.append((_message_time(message), depth_mm))
            self._match_frame_locked()

    def _on_camera_info(self, message) -> None:
        self.info_publisher.publish(message)

    def _match_frame_locked(self) -> None:
        if not self._color_buffer or not self._depth_buffer:
            return
        color_time, color, header = self._color_buffer[-1]
        depth_index, depth_entry = min(
            enumerate(self._depth_buffer),
            key=lambda entry: abs(entry[1][0] - color_time),
        )
        depth_time, depth = depth_entry
        sync_delta = abs(depth_time - color_time)
        self._last_sync_delta_ms = sync_delta * 1000.0
        if sync_delta > self._sync_tolerance:
            return
        if color.shape[:2] != depth.shape[:2]:
            now = time.monotonic()
            if now - self._last_size_warning_at > 2.0:
                self.node.get_logger().warning(
                    "Color and aligned depth sizes differ: "
                    f"color={color.shape[:2]}, depth={depth.shape[:2]}; "
                    "check align_depth.enable"
                )
                self._last_size_warning_at = now
            return
        self._color_buffer.clear()
        self._depth_buffer.clear()
        with self._frame_lock:
            # Replacing this slot drops stale frames while inference is busy.
            self._latest_frame = (color_time, color.copy(), depth.copy(), header)
        self._frame_event.set()
        self._last_frame_at = time.monotonic()

    def _take_latest_frame(self):
        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None
            self._frame_event.clear()
        return frame

    def _inference_loop(self) -> None:
        from std_msgs.msg import Bool, String

        while not self._stop_event.is_set():
            if not self._frame_event.wait(timeout=0.2):
                continue
            frame = self._take_latest_frame()
            if frame is None:
                continue
            stamp, color, depth_mm, header = frame
            started = time.perf_counter()
            try:
                detections, annotated = self.detector.detect(color, depth_mm)
            except Exception as error:
                self.node.get_logger().error(f"Inference failed: {error}")
                time.sleep(0.1)
                continue
            elapsed = time.perf_counter() - started
            instant_fps = 1.0 / elapsed if elapsed > 0.0 else 0.0
            self._fps = (
                instant_fps
                if self._fps == 0.0
                else 0.85 * self._fps + 0.15 * instant_fps
            )
            self._sequence += 1
            scene = Scene(
                sequence=self._sequence,
                stamp=stamp,
                frame_id=header.frame_id,
                image_width=int(color.shape[1]),
                image_height=int(color.shape[0]),
                inference_ms=round(elapsed * 1000.0, 1),
                fps=round(self._fps, 1),
                detections=detections,
                stamp_sec=int(header.stamp.sec),
                stamp_nanosec=int(header.stamp.nanosec),
            )
            scene_message = String()
            scene_message.data = scene.to_json()
            self.scene_publisher.publish(scene_message)

            lan_report = dict(self.detector.last_lan_color_report)
            stability = self.lan_color_stabilizer.update(
                str(lan_report["status"]), time.monotonic()
            )
            lan_report.update(stability)
            lan_report.update(
                {
                    "sequence": self._sequence,
                    "stamp": stamp,
                    "stamp_sec": int(header.stamp.sec),
                    "stamp_nanosec": int(header.stamp.nanosec),
                    "frame_id": header.frame_id,
                }
            )
            lan_message = String()
            lan_message.data = json.dumps(
                lan_report,
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self.lan_color_publisher.publish(lan_message)

            log_key = (
                lan_report["status"],
                lan_report["reason"],
                tuple(
                    (
                        pair["cable_detection_id"],
                        pair["port_detection_id"],
                        pair["matched"],
                    )
                    for pair in lan_report.get("pairs", [])
                ),
            )
            log_text = format_lan_color_log(lan_report)
            if log_key != self._last_lan_log_key:
                self._last_lan_log_key = log_key
                if lan_report["status"] == "UNKNOWN":
                    self.node.get_logger().info(
                        f"LAN color inspection pending: {log_text}"
                    )
                elif not stability["changed"]:
                    self.node.get_logger().info(
                        "LAN color inspection candidate "
                        f"{stability['frames']}/"
                        f"{self.lan_color_stabilizer.stable_frames}: {log_text}"
                    )

            if stability["changed"]:
                self.lan_color_ok_publisher.publish(
                    Bool(data=bool(lan_report["ok"]))
                )
                speech = speech_for_status(str(lan_report["status"]))
                if speech:
                    self.speech_publisher.publish(String(data=speech))
                self.node.get_logger().info(
                    f"LAN color inspection confirmed: {log_text}"
                )

            try:
                image_message = bgr8_to_image_message(annotated, header)
                self.image_publisher.publish(image_message)
            except Exception as error:
                self.node.get_logger().error(
                    f"Annotated image publication failed: {error}"
                )
            self._processed += 1

    def _watchdog(self) -> None:
        age = time.monotonic() - self._last_frame_at
        if age > 3.0:
            delta = (
                "none"
                if self._last_sync_delta_ms is None
                else f"{self._last_sync_delta_ms:.3f}ms"
            )
            self.node.get_logger().warning(
                "No synchronized RGB/depth frame for "
                f"{age:.1f}s; callbacks color={self._color_received}, "
                f"depth={self._depth_received}, nearest_delta={delta}; "
                "verify topics, QoS and depth alignment"
            )

    def shutdown(self) -> None:
        self._stop_event.set()
        self._frame_event.set()
        self._worker.join(timeout=3.0)
        self.node.destroy_node()


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    application = PerceptionNode()
    try:
        rclpy.spin(application.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        application.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
