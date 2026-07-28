from __future__ import annotations

import threading
import time

from scipy.spatial.transform import Rotation

from .core import doosan_pose_to_matrix


class ServiceCallError(RuntimeError):
    pass


def call_service(client, request, timeout_sec: float, label: str):
    """Wait for a service future while the ROS node spins on another thread."""

    future = client.call_async(request)
    completed = threading.Event()
    future.add_done_callback(lambda _future: completed.set())
    if not completed.wait(timeout_sec):
        future.cancel()
        raise ServiceCallError(f"{label} service timeout ({timeout_sec:.2f}s)")
    error = future.exception()
    if error is not None:
        raise ServiceCallError(f"{label} service failed: {error}")
    response = future.result()
    if response is None:
        raise ServiceCallError(f"{label} returned no response")
    return response


class TcpPoseTfBroadcaster:
    """Select the calibrated presets and publish Base -> active TCP over time."""

    DR_BASE = 0

    def __init__(self) -> None:
        from dsr_msgs2.srv import (
            GetCurrentPosx,
            GetCurrentTcp,
            GetCurrentTool,
            SetCurrentTcp,
            SetCurrentTool,
        )
        from rclpy.node import Node
        from tf2_ros import TransformBroadcaster

        self.node = Node("m0609_tcp_pose_tf_broadcaster")
        self._declare_parameters()
        namespace = str(self._parameter("robot_namespace")).strip("/")
        prefix = f"/{namespace}" if namespace else ""

        self.base_frame = str(self._parameter("base_frame")).strip()
        self.tcp_frame = str(self._parameter("tcp_frame")).strip()
        self.tcp_name = str(self._parameter("tcp_name")).strip()
        self.tool_name = str(self._parameter("tool_name")).strip()
        self.publish_rate_hz = float(self._parameter("publish_rate_hz"))
        self.service_timeout_sec = float(self._parameter("service_timeout_sec"))
        self.max_sample_latency_sec = (
            float(self._parameter("max_sample_latency_ms")) / 1000.0
        )
        if not all(
            [self.base_frame, self.tcp_frame, self.tcp_name, self.tool_name]
        ):
            raise ValueError(
                "base_frame, tcp_frame, tcp_name and tool_name cannot be empty"
            )
        if self.base_frame == self.tcp_frame:
            raise ValueError("base_frame and tcp_frame must differ")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.service_timeout_sec <= 0.0:
            raise ValueError("service_timeout_sec must be greater than zero")
        if self.max_sample_latency_sec <= 0.0:
            raise ValueError("max_sample_latency_ms must be greater than zero")

        self._types = {
            "get_pose": GetCurrentPosx,
            "get_tcp": GetCurrentTcp,
            "get_tool": GetCurrentTool,
            "set_tcp": SetCurrentTcp,
            "set_tool": SetCurrentTool,
        }
        self._clients = {
            "get_pose": self.node.create_client(
                GetCurrentPosx,
                f"{prefix}/aux_control/get_current_posx",
            ),
            "get_tcp": self.node.create_client(
                GetCurrentTcp,
                f"{prefix}/tcp/get_current_tcp",
            ),
            "get_tool": self.node.create_client(
                GetCurrentTool,
                f"{prefix}/tool/get_current_tool",
            ),
            "set_tcp": self.node.create_client(
                SetCurrentTcp,
                f"{prefix}/tcp/set_current_tcp",
            ),
            "set_tool": self.node.create_client(
                SetCurrentTool,
                f"{prefix}/tool/set_current_tool",
            ),
        }
        self._broadcaster = TransformBroadcaster(self.node)
        self._shutdown = threading.Event()
        self._worker = threading.Thread(
            target=self._run,
            name="m0609-tcp-pose-tf",
            daemon=True,
        )
        self._worker.start()

    def _declare_parameters(self) -> None:
        defaults = {
            "robot_namespace": "dsr01",
            "base_frame": "base_link",
            "tcp_frame": "GripperDA_v1",
            "tcp_name": "GripperDA_v1",
            "tool_name": "Tool Weight",
            "configure_controller": False,
            "publish_rate_hz": 20.0,
            "service_wait_sec": 12.0,
            "service_timeout_sec": 1.0,
            "max_sample_latency_ms": 100.0,
        }
        for name, value in defaults.items():
            self.node.declare_parameter(name, value)

    def _parameter(self, name: str):
        return self.node.get_parameter(name).value

    def _wait_ready(self) -> None:
        deadline = time.monotonic() + float(self._parameter("service_wait_sec"))
        for name, client in self._clients.items():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(remaining):
                raise ServiceCallError(f"Doosan service unavailable: {name}")

    def _set_named_preset(self, key: str, name: str) -> None:
        request = self._types[key].Request()
        request.name = name
        response = call_service(
            self._clients[key],
            request,
            self.service_timeout_sec,
            key,
        )
        if not response.success:
            raise ServiceCallError(f"{key} rejected preset: {name}")

    def _get_named_preset(self, key: str) -> str:
        response = call_service(
            self._clients[key],
            self._types[key].Request(),
            self.service_timeout_sec,
            key,
        )
        if not response.success:
            raise ServiceCallError(f"{key} reported failure")
        return str(response.info)

    def _configure_and_verify(self) -> None:
        actual_tool = self._get_named_preset("get_tool")
        actual_tcp = self._get_named_preset("get_tcp")
        if bool(self._parameter("configure_controller")):
            if actual_tool != self.tool_name:
                self._set_named_preset("set_tool", self.tool_name)
            if actual_tcp != self.tcp_name:
                self._set_named_preset("set_tcp", self.tcp_name)
        actual_tool = self._get_named_preset("get_tool")
        actual_tcp = self._get_named_preset("get_tcp")
        if actual_tcp != self.tcp_name:
            raise ServiceCallError(
                f"Active TCP mismatch: expected={self.tcp_name!r}, "
                f"actual={actual_tcp!r}"
            )
        if actual_tool != self.tool_name:
            self.node.get_logger().warning(
                "Tool Weight mismatch does not affect TCP TF publication; "
                "task manager may continue with the active Tool, but force "
                "compensation accuracy must be verified: "
                f"expected={self.tool_name!r}, actual={actual_tool!r}"
            )
        self.node.get_logger().info(
            f"TCP preset verified for TF publication: {actual_tcp}"
        )

    def _sample_pose(self):
        request = self._types["get_pose"].Request()
        request.ref = self.DR_BASE
        start_ns = self.node.get_clock().now().nanoseconds
        started = time.monotonic()
        response = call_service(
            self._clients["get_pose"],
            request,
            self.service_timeout_sec,
            "get_current_posx",
        )
        latency = time.monotonic() - started
        end_ns = self.node.get_clock().now().nanoseconds
        if latency > self.max_sample_latency_sec:
            raise ServiceCallError(
                f"TCP pose sample latency {latency * 1000.0:.1f}ms exceeds "
                f"{self.max_sample_latency_sec * 1000.0:.1f}ms"
            )
        if not response.success or not response.task_pos_info:
            raise ServiceCallError("get_current_posx returned an invalid pose")
        values = list(response.task_pos_info[0].data)
        if len(values) < 6:
            raise ServiceCallError("get_current_posx pose has fewer than 6 values")
        return values[:6], (start_ns + end_ns) // 2

    def _publish_pose(self, pose, stamp_ns: int) -> None:
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time

        matrix = doosan_pose_to_matrix(pose, translation_scale=0.001)
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        message = TransformStamped()
        message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        message.header.frame_id = self.base_frame
        message.child_frame_id = self.tcp_frame
        message.transform.translation.x = float(matrix[0, 3])
        message.transform.translation.y = float(matrix[1, 3])
        message.transform.translation.z = float(matrix[2, 3])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self._broadcaster.sendTransform(message)

    def _run(self) -> None:
        try:
            self._wait_ready()
            self._configure_and_verify()
        except Exception as error:
            self.node.get_logger().error(str(error))
            return

        period = 1.0 / self.publish_rate_hz
        next_sample = time.monotonic()
        last_warning = 0.0
        while not self._shutdown.is_set():
            try:
                pose, stamp_ns = self._sample_pose()
                self._publish_pose(pose, stamp_ns)
            except Exception as error:
                now = time.monotonic()
                if now - last_warning >= 2.0:
                    self.node.get_logger().warning(str(error))
                    last_warning = now
            next_sample += period
            wait_sec = max(0.0, next_sample - time.monotonic())
            if self._shutdown.wait(wait_sec):
                break
            if time.monotonic() - next_sample > period:
                next_sample = time.monotonic()

    def shutdown(self) -> None:
        self._shutdown.set()
        self._worker.join(timeout=3.0)
        self.node.destroy_node()


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    application = TcpPoseTfBroadcaster()
    try:
        rclpy.spin(application.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        application.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
