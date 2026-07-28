#!/usr/bin/env python3
"""
DRCF 에뮬레이터 모드에서 RViz 그리퍼 시각화를 담당하는 가상 노드.

실제 OnRobot 드라이버의 /onrobot/sendCommand 서비스 인터페이스를 동일하게 구현해
grip_test.py가 모드에 무관하게 같은 서비스를 호출.
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from onrobot_rg_msgs.srv import SetCommand
from sensor_msgs.msg import JointState

GRIPPER_OPEN = -0.558505  # rg2_finger_joint open (URDF lower limit)
GRIPPER_CLOSED = 0.785398  # rg2_finger_joint closed (URDF upper limit)
PUBLISH_RATE = 1.0 / 50.0  # 50 Hz — matches real OnRobot driver
SPEED = 1.0  # rad/s animation speed
DONE_TOL = 0.01  # rad — "reached target" tolerance
RG2_MAX_WIDTH_TENTHS_MM = 1100


def width_command_to_joint(command):
    """Map the real RG2 driver's 0.1 mm command unit onto the URDF joint."""

    try:
        width_tenths_mm = int(command)
    except (TypeError, ValueError) as error:
        raise ValueError(f'Unknown command: {command!r}') from error
    if not 0 <= width_tenths_mm <= RG2_MAX_WIDTH_TENTHS_MM:
        raise ValueError(
            'RG2 width command must be between 0 and '
            f'{RG2_MAX_WIDTH_TENTHS_MM}'
        )
    open_ratio = width_tenths_mm / RG2_MAX_WIDTH_TENTHS_MM
    return GRIPPER_CLOSED + open_ratio * (
        GRIPPER_OPEN - GRIPPER_CLOSED
    )


class GripperVirtualNode(Node):

    def __init__(self):
        super().__init__('gripper_virtual_node')
        self._cb_group = ReentrantCallbackGroup()

        self._pub = self.create_publisher(
            JointState, '/gripper_joint_states', 10
        )
        self._position = 0.0
        self._target = 0.0
        self._lock = threading.Lock()

        self.create_timer(
            PUBLISH_RATE,
            self._publish_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            SetCommand, '/onrobot/sendCommand',
            self._send_command_cb, callback_group=self._cb_group,
        )
        self.get_logger().info(
            'GripperVirtualNode ready — /onrobot/sendCommand'
        )

    def _send_command_cb(self, req, res):
        if req.command == 'c':
            target = GRIPPER_CLOSED
        elif req.command == 'o':
            target = GRIPPER_OPEN
        else:
            try:
                target = width_command_to_joint(req.command)
            except ValueError as error:
                res.success = False
                res.message = str(error)
                return res

        with self._lock:
            self._target = target

        # 애니메이션이 목표 위치에 도달할 때까지 블로킹
        while True:
            with self._lock:
                done = abs(self._position - target) < DONE_TOL
            if done:
                break
            time.sleep(PUBLISH_RATE)

        res.success = True
        res.message = ''
        return res

    def _publish_cb(self):
        with self._lock:
            diff = self._target - self._position
            step = SPEED * PUBLISH_RATE
            if abs(diff) <= step:
                self._position = self._target
            else:
                self._position += step * (1.0 if diff > 0 else -1.0)
            pos = self._position

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['rg2_finger_joint']
        msg.position = [pos]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperVirtualNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # launch 종료(SIGINT) 시 rclpy 기본 시그널 핸들러가 이미 context를
        # shutdown한 뒤 ExternalShutdownException이 올라온다 → 중복 호출 시
        # RCLError('rcl_shutdown already called'). ok() 가드로 한 번만 호출.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
