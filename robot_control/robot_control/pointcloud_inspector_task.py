from __future__ import annotations

import re
import time

import DR_init
import rclpy
from od_msg.srv import SrvPointCloudCompare
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger
from robot_control.task_config import (
    CAPTURE_SERVICE,
    CAPTURE_TIMEOUT_SEC,
    COMPARE_SERVICE,
    COMPARE_TIMEOUT_SEC,
    COMPARISON_NODE_NAME,
    FINALIZE_SERVICE,
    FINALIZE_TIMEOUT_SEC,
    OBJECT_TYPE_BOLT,
    OBJECT_TYPE_MULTITAP,
    PIPELINE_NODE_NAME,
    POINTCLOUD_SCAN_ACC,
    POINTCLOUD_SCAN_VEL,
    POINTCLOUD_SETTLE_SEC,
    RESET_SERVICE,
    RESET_TIMEOUT_SEC,
    SCAN_POSITIONS_BY_OBJECT,
)


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


def prepare_pointcloud_runtime(node: Node):
    clients = getattr(node, "_pointcloud_task_clients", None)
    if clients is not None:
        return clients

    clients = {
        "reset": node.create_client(Trigger, RESET_SERVICE),
        "capture": node.create_client(Trigger, CAPTURE_SERVICE),
        "finalize": node.create_client(Trigger, FINALIZE_SERVICE),
        "compare": node.create_client(SrvPointCloudCompare, COMPARE_SERVICE),
    }
    setattr(node, "_pointcloud_task_clients", clients)
    return clients


def set_remote_object_type(node: Node, object_type: str) -> None:
    parameter = Parameter("object_type", Parameter.Type.STRING, object_type)
    client_map = getattr(node, "_pointcloud_param_clients", None)
    if client_map is None:
        client_map = {
            "pipeline": AsyncParameterClient(node, PIPELINE_NODE_NAME),
            "comparison": AsyncParameterClient(node, COMPARISON_NODE_NAME),
        }
        setattr(node, "_pointcloud_param_clients", client_map)

    for client_name, client in client_map.items():
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{client_name} parameter service unavailable")
        future = client.set_parameters([parameter])
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            future.cancel()
            raise TimeoutError(f"{client_name} parameter update timeout")
        result = future.result()
        if not result or not result[0].successful:
            reason = result[0].reason if result else "no response"
            raise RuntimeError(f"{client_name} object_type update failed: {reason}")


def run_pointcloud_inspection(node: Node, object_type: str) -> tuple[bool, str]:
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import movej, posj

    if object_type not in SCAN_POSITIONS_BY_OBJECT:
        return False, f"unsupported object_type: {object_type}"

    clients = prepare_pointcloud_runtime(node)
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
        set_remote_object_type(node, object_type)

        reset_response = call_service(
            node,
            reset_client,
            Trigger.Request(),
            RESET_TIMEOUT_SEC,
        )
        if not reset_response.success:
            return False, reset_response.message

        scan_positions = SCAN_POSITIONS_BY_OBJECT[object_type]
        total = len(scan_positions)
        for index, joints in enumerate(scan_positions, start=1):
            node.get_logger().info(f"[{object_type.upper()}][{index}/{total}] movej 이동: {joints}")
            ret = movej(posj(*joints), vel=POINTCLOUD_SCAN_VEL, acc=POINTCLOUD_SCAN_ACC)
            if ret != 0:
                return False, f"scan movej failed: index={index}, ret={ret}"

            time.sleep(POINTCLOUD_SETTLE_SEC)

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
