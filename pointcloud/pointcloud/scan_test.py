#!/usr/bin/env python3

import time

import rclpy
import DR_init
from std_srvs.srv import Trigger


DR_init.__dsr__id = "dsr01"
DR_init.__dsr__model = "m0609"


DEFAULT_OBJECT_TYPE = "multitap"

MULTITAP_SCAN_JOINT_POSITIONS = [
    [4.21884871, 3.42193103, 95.0242310,
     -0.0846541375, 81.1648254, 4.84697104],

#    [4.15648031, 5.67621231, 102.712036,
#     -0.0859194621, 71.3817215, 4.84504747],

    [-29.61434174, 20.70296669, 104.96266174,
     33.40117645, 75.39304352, -43.33092117],

    [-16.77000046, 46.18884277, 66.66223907,
     25.83072662, 107.48310852, -59.56035995],

    [9.07588863, 63.96677017, 31.80211258,
     8.76166344, 129.89855957, -80.41046143],

    [29.55996704, 64.5065918, 25.59170151,
     -12.09520721, 132.20625305, -111.95813751],

    [54.16500092, 40.6347084, 66.8478241,
     -20.80620766, 105.03799438, -126.18702698],

    [76.31496429, 22.23127365, 103.56153107,
     -33.89890289, 87.76088715, -126.80084991],

    [88.10170746, -10.48255539, 133.44250488,
     -37.14406967, 79.34391785, -145.4735260],

    [-28.10516357, -15.42932701, 132.07780457,
     50.87726593, 65.34779358, -181.69018555],
]

BOLT_SCAN_JOINT_POSITIONS = [
    # 1번
    [-26.85898018, 10.70242214, 87.51332855,
     -0.41825622, 81.90556335, -26.74835587],

     # 9번
    [-23.29460526, -12.12123013, 132.66659546,
     23.07864189, 31.99207687, -130.24792480],

    # 2번
    [-76.19980621, 18.09572220, 117.73093414,
     38.53387070, 71.44489288, -49.60745621],

    # 3번
    [-51.99671173, 46.69021988, 72.75593567,
     26.72519875, 101.15863037, -56.38158035],

    # 4번
    [-25.98138237, 60.19123840, 45.02897644,
     12.05761337, 118.08054352, -75.48970795],

    # 5번
    [-4.34822369, 63.22199249, 39.43050003,
     -6.17316198, 124.74003601, -97.00015259],

    # 6번
    [16.96628380, 47.55668640, 64.62080383,
     -18.75974274, 112.50939178, -115.90266418],

    # 7번
    [43.54332352, 19.82193375, 104.80126953,
     -28.99685669, 92.05529785, -126.29026794],

    # 8번
    [51.46983719, -1.13892090, 131.63854980,
     -42.11459732, 72.93746185, -139.62373352],
]

VEL = 30
ACC = 30
SETTLE_SEC = 1.0

RESET_SERVICE = "/pointcloud_pipeline/reset"
CAPTURE_SERVICE = "/pointcloud_pipeline/capture"
FINALIZE_SERVICE = "/pointcloud_pipeline/finalize"

RESET_TIMEOUT_SEC = 10.0
CAPTURE_TIMEOUT_SEC = 10.0
FINALIZE_TIMEOUT_SEC = 120.0


def resolve_scan_joint_positions() -> tuple[str, list[list[float]]]:
    node = DR_init.__dsr__node
    object_type = str(
        node.get_parameter("object_type").value
    ).strip().lower()
    scan_positions_map = {
        "multitap": MULTITAP_SCAN_JOINT_POSITIONS,
        "bolt": BOLT_SCAN_JOINT_POSITIONS,
    }

    if object_type not in scan_positions_map:
        raise ValueError(
            "OBJECT_TYPE must be one of: multitap, bolt"
        )

    return object_type, scan_positions_map[object_type]

def wait_for_service(node, client, service_name: str) -> bool:
    """서비스가 나타날 때까지 대기한다."""
    while rclpy.ok():
        if client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(
                f"서비스 연결 완료: {service_name}"
            )
            return True

        node.get_logger().info(
            f"서비스 대기 중: {service_name}"
        )

    return False

#테스트
def call_trigger_service(node, client, service_name: str, timeout_sec: float) -> tuple[bool, str]:
    """Trigger 서비스를 호출하고 성공 여부와 메시지를 반환한다."""
    request = Trigger.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=timeout_sec,
    )

    if not future.done():
        future.cancel()
        return False, f"{service_name} 호출 시간 초과"

    try:
        response = future.result()
    except Exception as error:
        return False, f"{service_name} 호출 예외: {error}"

    if response is None:
        return False, f"{service_name} 응답 없음"

    return response.success, response.message


def main():
    rclpy.init()

    node = rclpy.create_node("movej_scan_test_node", namespace=DR_init.__dsr__id,)
    node.declare_parameter("object_type", DEFAULT_OBJECT_TYPE)

    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej,
        posj,
        get_robot_mode,
        get_robot_state,
        get_last_alarm,
        get_current_posj,
        ROBOT_MODE_AUTONOMOUS,
    )

    try:
        object_type, scan_joint_positions = resolve_scan_joint_positions()

        reset_client = node.create_client(
            Trigger,
            RESET_SERVICE,
        )
        capture_client = node.create_client(
            Trigger,
            CAPTURE_SERVICE,
        )
        finalize_client = node.create_client(
            Trigger,
            FINALIZE_SERVICE,
        )

        if not wait_for_service(node, reset_client, RESET_SERVICE):
            return

        if not wait_for_service(node, capture_client, CAPTURE_SERVICE):
            return

        if not wait_for_service(node, finalize_client, FINALIZE_SERVICE):
            return

        reset_success, reset_message = call_trigger_service(
            node,
            reset_client,
            RESET_SERVICE,
            RESET_TIMEOUT_SEC,
        )
        if not reset_success:
            node.get_logger().error(f"파이프라인 초기화 실패: {reset_message}")
            return

        node.get_logger().info(f"파이프라인 초기화 완료: {reset_message}")

        mode = get_robot_mode()
        state = get_robot_state()

        node.get_logger().info(
            f"[진단] robot_mode={mode}, "
            f"ROBOT_MODE_AUTONOMOUS={ROBOT_MODE_AUTONOMOUS}, "
            f"robot_state={state}"
        )

        if mode != ROBOT_MODE_AUTONOMOUS:
            node.get_logger().error(
                "로봇이 Autonomous 모드가 아닙니다. 스캔을 중단합니다."
            )
            return


        ### 다중 시점 스캔 시작
        total = len(scan_joint_positions)
        captured_count = 0
        scan_failed = False

        node.get_logger().info(
            f"스캔 대상={object_type}, 총 {total}개 지점 스캔 시작"
        )

        for i, joints in enumerate(scan_joint_positions, start=1):
            node.get_logger().info(f"[{i}/{total}] movej 이동: {joints}")

            ret = movej(posj(*joints),vel=VEL,acc=ACC)

            if ret != 0:
                alarm = get_last_alarm()

                node.get_logger().error(
                    f"[{i}] 이동 실패, ret={ret}\n"
                    f"robot_mode={get_robot_mode()}\n"
                    f"robot_state={get_robot_state()}\n"
                    f"last_alarm={alarm}"
                )

                scan_failed = True
                break

            # 로봇 이동 완료 후 진동 안정화
            node.get_logger().info(f"[{i}/{total}] 이동 완료, "f"{SETTLE_SEC:.1f}초 안정화 대기")
            time.sleep(SETTLE_SEC)

            # pointcloud 캡쳐 요청
            # 가장 최근 PointCloud2 프레임을 저장
            capture_success, capture_message = (
                call_trigger_service(node, capture_client, CAPTURE_SERVICE, CAPTURE_TIMEOUT_SEC))

            if not capture_success:
                node.get_logger().error(f"[{i}/{total}] 캡처 실패: {capture_message}")
                scan_failed = True
                break

            captured_count += 1

            node.get_logger().info(f"[{i}/{total}] 캡쳐 완료: {capture_message}")

        if scan_failed:
            node.get_logger().warning(f"스캔 실패로 종료합니다. 캡쳐 성공 = {captured_count}/{total}")
            return

        if captured_count != total:
            node.get_logger().warning(f"캡쳐 수가 부족해 종료합니다. 캡쳐 성공 = {captured_count}/{total}")
            return
        
        node.get_logger().info(f"모든 캡쳐 완료 {captured_count}/{total}")

        finalize_success, finalize_message = call_trigger_service(
            node,
            finalize_client,
            FINALIZE_SERVICE,
            FINALIZE_TIMEOUT_SEC,
        )
        if not finalize_success:
            node.get_logger().error(f"후처리 파이프라인 실패: {finalize_message}")
            return

        node.get_logger().info(f"후처리 파이프라인 완료: {finalize_message}")

    except KeyboardInterrupt:
        node.get_logger().warning("사용자에 의해 스캔이 중단되었습니다.")

    except Exception as error:
        node.get_logger().error(f"스캔 실행 중 예외: {error}")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
