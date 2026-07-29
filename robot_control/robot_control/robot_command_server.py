"""음성/HMI 명령을 실제 작업 함수로 매핑하는 액션 서버.

프로젝트의 상위 제어 진입점이다. HMI와 옛 ``voice_command_dispatcher``는 모두
``/{ROBOT_ID}/robot_command`` 액션으로 목표를 보내고, 이 서버가 intent에 따라
볼트 체결, 플러그 삽입, 3D 검사, 공구 정리/전달 구현체를 호출한다.
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from outlet_assembly.plug_insert_task import run_plug_insert
from robot_control.bolt_assemble_task import run_bolt_assemble
from robot_control.pointcloud_inspector_task import (
    OBJECT_TYPE_BOLT,
    OBJECT_TYPE_MULTITAP,
    run_pointcloud_inspection,
)
from robot_control.task_config import ROBOT_ID
from robot_control.tool_cleanup_task import run_tool_cleanup
from robot_control.tool_handover_task import run_tool_handover
from voice_interfaces.action import RobotCommand

SUPPORTED_INTENTS = {
    "BOLT_ASSEMBLE",
    "OUTLET_ASSEMBLE",
    "INSPECT_FASTEN",
    "CONNECTOR_INSPECT",
    "TOOL_FETCH",
    "TOOL_CLEANUP",
}


class RobotCommandServerNode(Node):
    """intent 문자열을 작업 함수로 해석하고 결과를 액션 응답으로 반환한다."""

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
        # 공구 정리/전달용 runtime 노드. m0609_tool_sorter_* 노드들의 서비스와
        # 상태 토픽은 전역 이름이므로 namespace를 붙이지 않는다.
        self.tool_task_node = rclpy.create_node("tool_sorter_runtime")
        self._action_server = ActionServer(
            self,
            RobotCommand,
            f"/{ROBOT_ID}/robot_command",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            "RobotCommandServer ready. supported_intents="
            f"[{', '.join(sorted(SUPPORTED_INTENTS))}]"
        )

    def goal_callback(self, goal_request: RobotCommand.Goal) -> GoalResponse:
        if goal_request.intent not in SUPPORTED_INTENTS:
            self.get_logger().warn(
                f"지원하지 않는 intent를 거절합니다: {goal_request.intent}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        # 공구 정리/전달은 자체 stop 서비스(/integration/tool_sorter/stop,
        # /tool_sorter/handover/stop)로 중단한다. 액션 취소 경로는 아직 어느
        # task도 지원하지 않는다.
        self.get_logger().warn("취소 요청을 받았지만 현재 어떤 task도 중간 취소를 지원하지 않습니다.")
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
        if intent == "OUTLET_ASSEMBLE":
            return self.execute_plug_insert
        if intent == "INSPECT_FASTEN":
            return self.execute_bolt_inspect
        if intent == "CONNECTOR_INSPECT":
            if "multitap" in tools or "outlet" in tools:
                return self.execute_multitap_inspect
            return lambda goal_handle: self.abort_result(
                goal_handle,
                "현재 CONNECTOR_INSPECT는 multitap/outlet 검사만 지원합니다.",
            )
        if intent == "TOOL_FETCH":
            return self.execute_tool_fetch
        if intent == "TOOL_CLEANUP":
            return self.execute_tool_cleanup
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

    def execute_plug_insert(self, goal_handle) -> RobotCommand.Result:
        self.publish_feedback(goal_handle, "플러그 삽입 시퀀스 시작", 0.1)
        # 플러그 삽입도 볼트와 같은 runtime 노드를 써야 DR_init.__dsr__node가 갈리지 않는다.
        success, message = run_plug_insert(self.bolt_task_node)
        if not success:
            return self.abort_result(goal_handle, f"OUTLET_ASSEMBLE 실행 실패: {message}")
        self.publish_feedback(goal_handle, "플러그 삽입 시퀀스 완료", 1.0)
        return self.success_result(goal_handle, f"OUTLET_ASSEMBLE 실행 완료: {message}")

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

    def execute_tool_fetch(self, goal_handle) -> RobotCommand.Result:
        tools = list(goal_handle.request.tools)
        self.publish_feedback(goal_handle, "공구 전달 시퀀스 시작", 0.1)
        success, message = run_tool_handover(self.tool_task_node, tools)
        if not success:
            return self.abort_result(goal_handle, f"TOOL_FETCH 실행 실패: {message}")
        self.publish_feedback(goal_handle, "공구 전달 시퀀스 완료", 1.0)
        return self.success_result(goal_handle, f"TOOL_FETCH 실행 완료: {message}")

    def execute_tool_cleanup(self, goal_handle) -> RobotCommand.Result:
        self.publish_feedback(goal_handle, "공구 정리 시퀀스 시작", 0.1)
        success, message = run_tool_cleanup(self.tool_task_node)
        if not success:
            return self.abort_result(goal_handle, f"TOOL_CLEANUP 실행 실패: {message}")
        self.publish_feedback(goal_handle, "공구 정리 시퀀스 완료", 1.0)
        return self.success_result(goal_handle, f"TOOL_CLEANUP 실행 완료: {message}")

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
        node.tool_task_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
