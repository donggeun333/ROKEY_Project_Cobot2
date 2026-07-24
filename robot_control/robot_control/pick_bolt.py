from __future__ import annotations

import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
import DR_init
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from ultralytics import YOLO

from robot_control.onrobot import RG


# 그리퍼 설정----------------------------
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
#--------------------------------------

# 로봇 기본 설정 -------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
MODEL_PATH = "/home/dg/cobot_ws/src/cobot2_ws/robot_control/resource/D-1.pt"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
DR_init.__dsr__node = None
# --------------------------------------


def transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class BoltDetector:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.bridge = CvBridge()
        self.color_frame: np.ndarray | None = None
        self.depth_frame: np.ndarray | None = None
        self.camera_info: CameraInfo | None = None
        self.color_stamp = None
        self.color_frame_id: str | None = None

        self.node.declare_parameter(
            "bolt_model_path",
            MODEL_PATH,
        )
        self.node.declare_parameter(
            "color_topic",
            "/camera/camera/color/image_raw",
        )
        self.node.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.node.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.node.declare_parameter("camera_link_frame", "camera_link")
        self.node.declare_parameter("base_frame", "base_link")
        self.node.declare_parameter("target_label", "")
        self.node.declare_parameter("detect_confidence", 0.25)
        self.node.declare_parameter("detect_timeout_sec", 5.0)
        self.node.declare_parameter("tf_timeout_sec", 1.0)
        self.node.declare_parameter("depth_unit_scale", 0.001)
        self.node.declare_parameter("show_detection_window", True)
        self.node.declare_parameter("detection_window_name", "bolt_detection")
        self.node.declare_parameter("monitor_detection_only", True)
        self.node.declare_parameter("log_interval_sec", 1.0)

        model_path = Path(self.node.get_parameter("bolt_model_path").value).resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        self.color_topic = str(self.node.get_parameter("color_topic").value)
        self.depth_topic = str(self.node.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.node.get_parameter("camera_info_topic").value)
        self.camera_link_frame = str(
            self.node.get_parameter("camera_link_frame").value
        )
        self.base_frame = str(self.node.get_parameter("base_frame").value)
        self.target_label = str(self.node.get_parameter("target_label").value).strip()
        self.detect_confidence = float(
            self.node.get_parameter("detect_confidence").value
        )
        self.detect_timeout_sec = float(
            self.node.get_parameter("detect_timeout_sec").value
        )
        self.tf_timeout_sec = float(self.node.get_parameter("tf_timeout_sec").value)
        self.depth_unit_scale = float(
            self.node.get_parameter("depth_unit_scale").value
        )
        self.show_detection_window = bool(
            self.node.get_parameter("show_detection_window").value
        )
        self.detection_window_name = str(
            self.node.get_parameter("detection_window_name").value
        )
        self.monitor_detection_only = bool(
            self.node.get_parameter("monitor_detection_only").value
        )
        self.log_interval_sec = float(
            self.node.get_parameter("log_interval_sec").value
        )
        self.last_logged_at = 0.0

        self.model = YOLO(str(model_path))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=True,
        )

        self.node.create_subscription(
            Image,
            self.color_topic,
            self.handle_color,
            10,
        )
        self.node.create_subscription(
            Image,
            self.depth_topic,
            self.handle_depth,
            10,
        )
        self.node.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.handle_camera_info,
            10,
        )

        self.node.get_logger().info(
            f"Bolt detector ready. model={model_path}, color_topic={self.color_topic}, "
            f"depth_topic={self.depth_topic}, camera_link_frame={self.camera_link_frame}, "
            f"base_frame={self.base_frame}"
        )

    def handle_color(self, msg: Image) -> None:
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.color_stamp = msg.header.stamp
        self.color_frame_id = msg.header.frame_id

    def handle_depth(self, msg: Image) -> None:
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def handle_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def wait_for_frames(self) -> bool:
        deadline = time.time() + self.detect_timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (
                self.color_frame is not None
                and self.depth_frame is not None
                and self.camera_info is not None
                and self.color_stamp is not None
                and self.color_frame_id is not None
            ):
                return True
        return False

    def detect(self) -> list[float] | None:
        deadline = None
        if not self.monitor_detection_only:
            deadline = time.time() + self.detect_timeout_sec

        while rclpy.ok():
            if deadline is not None and time.time() >= deadline:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if (
                self.color_frame is None
                or self.depth_frame is None
                or self.camera_info is None
                or self.color_stamp is None
                or self.color_frame_id is None
            ):
                continue

            result = self.model(self.color_frame, verbose=False)[0]
            self.show_detection_result(result)

            selected_box = self.select_detection(result)
            if selected_box is None:
                continue

            x1, y1, x2, y2 = selected_box.xyxy[0].tolist()
            cx = int(round((x1 + x2) / 2.0))
            cy = int(round((y1 + y2) / 2.0))

            depth_m = self.read_depth_meters(cx, cy)
            if depth_m is None:
                self.node.get_logger().warning("검출 중심점 depth를 읽지 못했습니다.")
                continue

            camera_point = self.pixel_to_camera_point(cx, cy, depth_m)
            link_point = self.color_optical_to_camera_link(camera_point)
            base_point = self.camera_link_to_base(link_point)
            if base_point is None:
                continue

            if self.should_log_detection():
                self.node.get_logger().info(
                    f"Bolt detected. pixel=({cx}, {cy}), depth={depth_m:.4f} m, "
                    f"base_link={np.round(base_point, 4).tolist()}"
                )

            if self.monitor_detection_only:
                continue

            self.close_detection_window()
            return base_point.tolist()

        if not self.monitor_detection_only:
            self.node.get_logger().warning("검출 시간 내에 유효한 볼트를 찾지 못했습니다.")
            self.close_detection_window()
        return None

    def select_detection(self, result):
        names = result.names
        candidates = []
        for box in result.boxes:
            score = float(box.conf[0])
            if score < self.detect_confidence:
                continue

            class_id = int(box.cls[0])
            class_name = names.get(class_id, str(class_id))
            if self.target_label and class_name != self.target_label:
                continue
            candidates.append(box)

        if not candidates:
            return None

        return max(candidates, key=lambda box: float(box.conf[0]))

    def show_detection_result(self, result) -> None:
        if not self.show_detection_window:
            return

        annotated = result.plot()
        cv2.imshow(self.detection_window_name, annotated)
        cv2.waitKey(1)

    def close_detection_window(self) -> None:
        if not self.show_detection_window:
            return

        cv2.destroyWindow(self.detection_window_name)

    def should_log_detection(self) -> bool:
        now = time.time()
        if now - self.last_logged_at < self.log_interval_sec:
            return False

        self.last_logged_at = now
        return True

    def read_depth_meters(self, x: int, y: int) -> float | None:
        if self.depth_frame is None:
            return None

        height, width = self.depth_frame.shape[:2]
        if x < 0 or x >= width or y < 0 or y >= height:
            return None

        search_radius = 3
        values = []
        for yy in range(max(0, y - search_radius), min(height, y + search_radius + 1)):
            for xx in range(max(0, x - search_radius), min(width, x + search_radius + 1)):
                depth_value = float(self.depth_frame[yy, xx])
                if depth_value > 0.0 and math.isfinite(depth_value):
                    values.append(depth_value)

        if not values:
            return None

        return float(np.median(values)) * self.depth_unit_scale

    def pixel_to_camera_point(self, x: int, y: int, depth_m: float) -> np.ndarray:
        if self.camera_info is None:
            raise RuntimeError("camera_info is not available.")

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        return np.array(
            [
                (x - cx) * depth_m / fx,
                (y - cy) * depth_m / fy,
                depth_m,
            ],
            dtype=np.float64,
        )

    def color_optical_to_camera_link(
        self,
        optical_point: np.ndarray,
    ) -> np.ndarray:
        x_optical, y_optical, z_optical = optical_point
        return np.array(
            [
                z_optical,
                -x_optical,
                -y_optical,
            ],
            dtype=np.float64,
        )

    def camera_link_to_base(self, camera_point: np.ndarray) -> np.ndarray | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_link_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as error:
            self.node.get_logger().error(f"TF lookup 실패: {error}")
            return None

        matrix = transform_to_matrix(transform)
        homogeneous = np.append(camera_point, 1.0)
        base_point = matrix @ homogeneous
        return base_point[:3]


def pick_bolt(gripper: RG, bolt_pose: list[float]) -> bool:
    """볼트 위치로 이동해 파지 작업을 수행한다."""
    del gripper
    del bolt_pose
    return False


def fasten_bolt(gripper: RG) -> bool:
    """체결 위치로 이동해 볼트 체결 작업을 수행한다."""
    del gripper
    return False


def detect_bolt(detector: BoltDetector) -> list[float] | None:
    """YOLO 검출 결과를 base_link 기준 볼트 pose로 반환한다."""
    return detector.detect()


def execute_bolt_task(detector: BoltDetector, gripper: RG) -> None:
    """볼트 탐지, 파지, 체결의 전체 흐름을 관리한다."""
    bolt_pose = detect_bolt(detector)
    if bolt_pose is None:
        return

    if not pick_bolt(gripper, bolt_pose):
        return

    fasten_bolt(gripper)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pick_bolt_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import set_tcp, set_tool
    except ImportError as error:
        node.get_logger().error(f"DSR_ROBOT2 import 실패: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        detector = BoltDetector(node)
        gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        set_tool("Tool Weight")
        set_tcp("GripperDA_v1")
        execute_bolt_task(detector, gripper)
    except KeyboardInterrupt:
        node.get_logger().warning("사용자에 의해 작업이 중단되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
