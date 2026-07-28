#!/usr/bin/env python3
"""
OnRobot RG2 조인트 이름 변환 노드

OnRobot 드라이버가 퍼블리시하는 /onrobot_joint_states 의 조인트 이름에
rg2_ prefix를 붙여 /gripper_joint_states 로 재퍼블리시.(URDF 조인트명 일치 목적)

드라이버 조인트 이름:  finger_joint, left_inner_knuckle_joint, ...
URDF 조인트 이름:     rg2_finger_joint, rg2_left_inner_knuckle_joint, ...
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperJointStatePublisher(Node):

    PREFIX = 'rg2_'

    def __init__(self):
        super().__init__('gripper_joint_state_publisher')
        self.pub = self.create_publisher(JointState, 'gripper_joint_states', 10)
        self.sub = self.create_subscription(
            JointState, 'onrobot_joint_states', self.callback, 10)
        self.get_logger().info('Gripper joint state publisher started.')

    def callback(self, msg):
        out = JointState()
        out.header = msg.header
        out.name     = [self.PREFIX + name for name in msg.name]
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort   = list(msg.effort)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GripperJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
