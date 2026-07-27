from __future__ import annotations

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from robot_control.bolt_assemble_task import run_bolt_assemble
from robot_control.pointcloud_inspector_task import (
    OBJECT_TYPE_BOLT,
    OBJECT_TYPE_MULTITAP,
    run_pointcloud_inspection,
)
from robot_control.task_config import ROBOT_ID
from voice_interfaces.action import RobotCommand

SUPPORTED_INTENTS = {"BOLT_ASSEMBLE", "INSPECT_FASTEN", "CONNECTOR_INSPECT"}


class RobotCommandServerNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_command_server")
        self.bolt_task_node = rclpy.create_node(
            "bolt_assemble_runtime",
            namespace=ROBOT_ID,
        )
        self.pointcloud_task_node = rclpy.create_node(
            "pointcloud_inspection_runtime",
            namespace=ROBOT_ID,
        )
        self._action_server = ActionServer(
            self,
            RobotCommand,
            f"/{ROBOT_ID}/robot_command",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            "RobotCommandServer ready. supported_intents=[BOLT_ASSEMBLE, INSPECT_FASTEN, CONNECTOR_INSPECT]"
        )

    def goal_callback(self, goal_request: RobotCommand.Goal) -> GoalResponse:
        if goal_request.intent not in SUPPORTED_INTENTS:
            self.get_logger().warn(
                f"지원하지 않는 intent를 거절합니다: {goal_request.intent}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("취소 요청을 받았지만 현재 BOLT_ASSEMBLE은 중간 취소를 지원하지 않습니다.")
        return CancelResponse.REJECT

    def publish_feedback(self, goal_handle, status: str, progress: float) -> None:
        feedback = RobotCommand.Feedback()
        feedback.status = status
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)

    def execute_callback(self, goal_handle) -> RobotCommand.Result:
        intent = goal_handle.request.intent
        tools = list(goal_handle.request.tools)
        handler = self.resolve_handler(intent, tools)
        return handler(goal_handle)

    def resolve_handler(self, intent: str, tools: list[str]):
        if intent == "BOLT_ASSEMBLE":
            return self.execute_bolt_assemble
        if intent == "INSPECT_FASTEN":
            return self.execute_bolt_inspect
        if intent == "CONNECTOR_INSPECT":
            if "multitap" in tools or "outlet" in tools:
                return self.execute_multitap_inspect
            return lambda goal_handle: self.abort_result(
                goal_handle,
                "현재 CONNECTOR_INSPECT는 multitap/outlet 검사만 지원합니다.",
            )
        return lambda goal_handle: self.abort_result(
            goal_handle,
            f"지원하지 않는 intent입니다: {intent}",
        )

    def success_result(self, goal_handle, message: str) -> RobotCommand.Result:
        result = RobotCommand.Result()
        goal_handle.succeed()
        result.success = True
        result.message = message
        return result

    def abort_result(self, goal_handle, message: str) -> RobotCommand.Result:
        result = RobotCommand.Result()
        goal_handle.abort()
        result.success = False
        result.message = message
        return result

    def execute_bolt_assemble(self, goal_handle) -> RobotCommand.Result:
        self.publish_feedback(goal_handle, "볼트 체결 시퀀스 시작", 0.1)
        success = run_bolt_assemble(self.bolt_task_node)
        if not success:
            return self.abort_result(goal_handle, "BOLT_ASSEMBLE 실행 실패")
        self.publish_feedback(goal_handle, "볼트 체결 시퀀스 완료", 1.0)
        return self.success_result(goal_handle, "BOLT_ASSEMBLE 실행 완료")

    def execute_multitap_inspect(self, goal_handle) -> RobotCommand.Result:
        self.publish_feedback(goal_handle, "멀티탭 검사 시퀀스 시작", 0.1)
        success, message = run_pointcloud_inspection(
            self.pointcloud_task_node,
            OBJECT_TYPE_MULTITAP,
        )
        if not success:
            return self.abort_result(goal_handle, f"MULTITAP_INSPECT 실행 실패: {message}")
        self.publish_feedback(goal_handle, "멀티탭 검사 시퀀스 완료", 1.0)
        return self.success_result(goal_handle, f"MULTITAP_INSPECT 실행 완료: {message}")

    def execute_bolt_inspect(self, goal_handle) -> RobotCommand.Result:
        self.publish_feedback(goal_handle, "볼트 3D 검사 시퀀스 시작", 0.1)
        success, message = run_pointcloud_inspection(
            self.pointcloud_task_node,
            OBJECT_TYPE_BOLT,
        )
        if not success:
            return self.abort_result(goal_handle, f"BOLT_INSPECT 실행 실패: {message}")
        self.publish_feedback(goal_handle, "볼트 3D 검사 시퀀스 완료", 1.0)
        return self.success_result(goal_handle, f"BOLT_INSPECT 실행 완료: {message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotCommandServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bolt_task_node.destroy_node()
        node.pointcloud_task_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
