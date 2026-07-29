"""레거시 음성 스택용 robot_command 액션 중계기.

현재 프로젝트의 주 음성 경로는 ``operator_ui.app_v5`` 안에 통합되어 있어 이
노드가
필수는 아니다. 다만 기존 ``/get_keyword`` 서비스 기반 음성 스택과 호환해야 할
때는 이 노드가 intent/tools/targets를 받아 ``robot_command_server`` 액션으로
전달한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from voice_interfaces.srv import GetKeyword
from voice_interfaces.action import RobotCommand
from robot_control.task_config import ROBOT_ID


class VoiceCommandDispatcherNode(Node):
    """
    두 가지 역할을 동시에 하는 "중계" 노드.
      1) /get_keyword 서비스의 클라이언트  -> 음성 인식 결과(intent/tools/targets)를 요청
      2) /robot_command 액션의 클라이언트  -> 받은 결과를 로봇 제어 노드에 실행 목표로 전달
    """

    def __init__(self):
        super().__init__("voice_command_dispatcher_node")

        self.get_keyword_client = self.create_client(GetKeyword, "get_keyword")
        self.robot_command_client = ActionClient(
            self,
            RobotCommand,
            f"/{ROBOT_ID}/robot_command",
        )

        self.get_logger().info(
            "VoiceCommandDispatcherNode started. supported voice flows="
            "[BOLT_ASSEMBLE, INSPECT_FASTEN, CONNECTOR_INSPECT, "
            "TOOL_FETCH, TOOL_CLEANUP]"
        )

    def request_voice_command(self):
        while not self.get_keyword_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("get_keyword 서비스를 기다리는 중...")

        request = GetKeyword.Request()
        future = self.get_keyword_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_robot_command(self, intent, tools, targets):
        if not self.robot_command_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("robot_command 액션 서버를 찾을 수 없습니다.")
            return False

        goal_msg = RobotCommand.Goal()
        goal_msg.intent = intent
        goal_msg.tools = tools
        goal_msg.targets = targets

        self.get_logger().info(f"로봇에게 명령 전송: {intent} {tools} -> {targets}")

        send_goal_future = self.robot_command_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("로봇이 목표를 거절했습니다.")
            return False

        self.get_logger().info("로봇이 목표를 수락했습니다. 작업 시작...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error("robot_command 액션 결과를 받지 못했습니다.")
            return False

        action_result = result.result
        self.get_logger().info(
            f"[결과] success={action_result.success}, message={action_result.message}"
        )
        return bool(action_result.success)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[피드백] status={feedback.status}, progress={feedback.progress:.0%}"
        )

    def run_once(self):
        voice_result = self.request_voice_command()

        if voice_result is None or not voice_result.success:
            self.get_logger().error("음성 인식 실패 (또는 알아들을 수 없는 명령)")
            return

        self.get_logger().info(
            f"음성 인식 결과 -> intent: {voice_result.intent}, "
            f"tools: {list(voice_result.tools)}, targets: {list(voice_result.targets)}, "
            f"raw_text: '{voice_result.raw_text}'"
        )

        self.send_robot_command(
            voice_result.intent, list(voice_result.tools), list(voice_result.targets)
        )


def main():
    rclpy.init()
    node = VoiceCommandDispatcherNode()

    try:
        while rclpy.ok():
            node.run_once()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
