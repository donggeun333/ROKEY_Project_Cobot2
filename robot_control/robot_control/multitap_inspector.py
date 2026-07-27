from __future__ import annotations

import re
import time

import DR_init
import rclpy
from od_msg.srv import SrvPointCloudCompare
from rclpy.node import Node
from std_srvs.srv import Trigger


MULTITAP_SCAN_JOINT_POSITIONS = [
    [4.21884871, 3.42193103, 95.0242310, -0.0846541375, 81.1648254, 4.84697104],
    [-29.61434174, 20.70296669, 104.96266174, 33.40117645, 75.39304352, -43.33092117],
    [-16.77000046, 46.18884277, 66.66223907, 25.83072662, 107.48310852, -59.56035995],
    [9.07588863, 63.96677017, 31.80211258, 8.76166344, 129.89855957, -80.41046143],
    [29.55996704, 64.5065918, 25.59170151, -12.09520721, 132.20625305, -111.95813751],
    [54.16500092, 40.6347084, 66.8478241, -20.80620766, 105.03799438, -126.18702698],
    [76.31496429, 22.23127365, 103.56153107, -33.89890289, 87.76088715, -126.80084991],
    [88.10170746, -10.48255539, 133.44250488, -37.14406967, 79.34391785, -145.4735260],
    [-28.10516357, -15.42932701, 132.07780457, 50.87726593, 65.34779358, -181.69018555],
]

VEL = 30
ACC = 30
SETTLE_SEC = 1.0

RESET_SERVICE = "/pointcloud_pipeline/reset"
CAPTURE_SERVICE = "/pointcloud_pipeline/capture"
FINALIZE_SERVICE = "/pointcloud_pipeline/finalize"
COMPARE_SERVICE = "/pointcloud_comparison/compare"

RESET_TIMEOUT_SEC = 10.0
CAPTURE_TIMEOUT_SEC = 10.0
FINALIZE_TIMEOUT_SEC = 180.0
COMPARE_TIMEOUT_SEC = 180.0


def wait_for_service(node: Node, client, service_name: str) -> bool:
    while rclpy.ok():
        if client.wait_for_service(timeout_sec=1.0):
            return True
        node.get_logger().info(f"서비스 대기 중: {service_name}")
    return False


def call_service(node: Node, client, request, timeout_sec: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        future.cancel()
        raise TimeoutError(f"service timeout: {client.srv_name}")
    response = future.result()
    if response is None:
        raise RuntimeError(f"service returned no response: {client.srv_name}")
    return response


def parse_filtered_path(finalize_message: str) -> str:
    match = re.search(r"filtered=([^,]+)", finalize_message)
    if match is None:
        raise ValueError(f"filtered path parse failed: {finalize_message}")
    return match.group(1).strip()

def prepare_multitap_runtime(node: Node):
    clients = getattr(node, "_multitap_clients", None)
    if clients is not None:
        return clients

    clients = {
        "reset": node.create_client(Trigger, RESET_SERVICE),
        "capture": node.create_client(Trigger, CAPTURE_SERVICE),
        "finalize": node.create_client(Trigger, FINALIZE_SERVICE),
        "compare": node.create_client(SrvPointCloudCompare, COMPARE_SERVICE),
    }
    setattr(node, "_multitap_clients", clients)
    return clients


def run_multitap_inspection(node: Node) -> tuple[bool, str]:
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import movej, posj

    clients = prepare_multitap_runtime(node)
    reset_client = clients["reset"]
    capture_client = clients["capture"]
    finalize_client = clients["finalize"]
    compare_client = clients["compare"]

    if not wait_for_service(node, reset_client, RESET_SERVICE):
        return False, f"서비스 연결 실패: {RESET_SERVICE}"
    if not wait_for_service(node, capture_client, CAPTURE_SERVICE):
        return False, f"서비스 연결 실패: {CAPTURE_SERVICE}"
    if not wait_for_service(node, finalize_client, FINALIZE_SERVICE):
        return False, f"서비스 연결 실패: {FINALIZE_SERVICE}"
    if not wait_for_service(node, compare_client, COMPARE_SERVICE):
        return False, f"서비스 연결 실패: {COMPARE_SERVICE}"

    try:
        reset_response = call_service(
            node,
            reset_client,
            Trigger.Request(),
            RESET_TIMEOUT_SEC,
        )
        if not reset_response.success:
            return False, reset_response.message

        total = len(MULTITAP_SCAN_JOINT_POSITIONS)
        for index, joints in enumerate(MULTITAP_SCAN_JOINT_POSITIONS, start=1):
            node.get_logger().info(f"[MULTITAP][{index}/{total}] movej 이동: {joints}")
            ret = movej(posj(*joints), vel=VEL, acc=ACC)
            if ret != 0:
                return False, f"scan movej failed: index={index}, ret={ret}"

            time.sleep(SETTLE_SEC)

            capture_response = call_service(
                node,
                capture_client,
                Trigger.Request(),
                CAPTURE_TIMEOUT_SEC,
            )
            if not capture_response.success:
                return False, capture_response.message

        finalize_response = call_service(
            node,
            finalize_client,
            Trigger.Request(),
            FINALIZE_TIMEOUT_SEC,
        )
        if not finalize_response.success:
            return False, finalize_response.message

        filtered_path = parse_filtered_path(finalize_response.message)
        compare_request = SrvPointCloudCompare.Request()
        compare_request.test_path = filtered_path
        compare_response = call_service(
            node,
            compare_client,
            compare_request,
            COMPARE_TIMEOUT_SEC,
        )
        if not compare_response.success:
            return False, compare_response.message

        return True, compare_response.message
    except Exception as error:
        return False, str(error)
