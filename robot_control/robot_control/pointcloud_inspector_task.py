"""3D 검사 시퀀스 실행기.

``robot_command_server``가 ``INSPECT_FASTEN`` 또는 ``CONNECTOR_INSPECT``를
받았을 때 이 모듈을 호출한다. 로봇을 사전 정의된 촬영 포즈로 이동시키고,
``pointcloud_pipeline``의 reset/capture/finalize와
``pointcloud_comparison``의 compare 서비스를 순서대로 호출한다.

이 모듈은 "검사 모션 오케스트레이션"만 담당한다. 실제 점군 처리와 비교 알고리즘은
``inspection_3d`` 패키지 쪽 구현체가 맡는다.
"""

from __future__ import annotations

import re
import time

import DR_init
import rclpy
from od_msg.srv import SrvPointCloudCompare
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from robot_control.gripper_service import GripperServiceError, OnRobotServiceGripper
from robot_control.task_config import (
    CAPTURE_SERVICE,
    CAPTURE_TIMEOUT_SEC,
    COMPARE_SERVICE,
    COMPARE_TIMEOUT_SEC,
    COMPARISON_NODE_NAME,
    FINALIZE_SERVICE,
    FINALIZE_TIMEOUT_SEC,
    HOME_JOINT,
    OBJECT_TYPE_BOLT,
    OBJECT_TYPE_MULTITAP,
    POINTCLOUD_CAPTURE_DIR_BY_OBJECT,
    POINTCLOUD_REFERENCE_PATH_BY_OBJECT,
    PIPELINE_NODE_NAME,
    POINTCLOUD_SCAN_ACC,
    POINTCLOUD_SCAN_VEL,
    POINTCLOUD_SETTLE_SEC,
    RESET_SERVICE,
    RESET_TIMEOUT_SEC,
    SCAN_POSITIONS_BY_OBJECT,
)

_SCAN_GRIPPER_SETTLE_SEC = 1.0


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


def call_set_parameters(node: Node, client, client_name: str, parameters: list[Parameter]) -> None:
    request = SetParameters.Request()
    request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        future.cancel()
        raise TimeoutError(f"{client_name} parameter update timeout")
    response = future.result()
    if response is None or len(response.results) != len(parameters):
        raise RuntimeError(f"{client_name} parameter update failed: no response")
    for result in response.results:
        if not result.successful:
            raise RuntimeError(f"{client_name} parameter update failed: {result.reason}")


def return_to_home(node: Node) -> None:
    from DSR_ROBOT2 import movej, mwait, posj

    movej(posj(*HOME_JOINT), vel=POINTCLOUD_SCAN_VEL, acc=POINTCLOUD_SCAN_ACC)
    mwait()


def open_gripper_for_scan(node: Node) -> None:
    gripper = OnRobotServiceGripper(node)
    gripper.open_gripper()
    node.get_logger().info("검사 시작 전 그리퍼 최대 개방")
    time.sleep(_SCAN_GRIPPER_SETTLE_SEC)


def set_remote_pointcloud_config(node: Node, object_type: str) -> None:
    client_map = getattr(node, "_pointcloud_param_clients", None)
    if client_map is None:
        client_map = {
            "pipeline": node.create_client(SetParameters, f"{PIPELINE_NODE_NAME}/set_parameters"),
            "comparison": node.create_client(SetParameters, f"{COMPARISON_NODE_NAME}/set_parameters"),
        }
        setattr(node, "_pointcloud_param_clients", client_map)

    for client_name, client in client_map.items():
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{client_name} parameter service unavailable")
    call_set_parameters(
        node,
        client_map["pipeline"],
        "pipeline",
        [
            Parameter("object_type", Parameter.Type.STRING, object_type),
            Parameter(
                "filtered_dir",
                Parameter.Type.STRING,
                POINTCLOUD_CAPTURE_DIR_BY_OBJECT[object_type],
            ),
        ],
    )
    comparison_parameters = [
        Parameter("object_type", Parameter.Type.STRING, object_type),
        Parameter(
            f"{object_type}_reference_path",
            Parameter.Type.STRING,
            POINTCLOUD_REFERENCE_PATH_BY_OBJECT[object_type],
        ),
    ]
    call_set_parameters(
        node,
        client_map["comparison"],
        "comparison",
        comparison_parameters,
    )


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

    success = False
    message = ""
    try:
        open_gripper_for_scan(node)
        set_remote_pointcloud_config(node, object_type)

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

            node.get_logger().info(f"[{object_type.upper()}][{index}/{total}] capture 호출")
            capture_response = call_service(
                node,
                capture_client,
                Trigger.Request(),
                CAPTURE_TIMEOUT_SEC,
            )
            node.get_logger().info(
                f"[{object_type.upper()}][{index}/{total}] capture 응답: "
                f"success={capture_response.success}, message={capture_response.message}"
            )
            if not capture_response.success:
                return False, capture_response.message

        node.get_logger().info(f"[{object_type.upper()}] finalize 호출")
        finalize_response = call_service(
            node,
            finalize_client,
            Trigger.Request(),
            FINALIZE_TIMEOUT_SEC,
        )
        node.get_logger().info(
            f"[{object_type.upper()}] finalize 응답: "
            f"success={finalize_response.success}, message={finalize_response.message}"
        )
        if not finalize_response.success:
            return False, finalize_response.message

        # pipeline_node(handle_finalize)가 이미 filtered PCD를 object_type에 맞는
        # HMI captures 폴더(~/cobot_ws/src/cobot2_ws/operator_ui/pointclouds/<bolt|outlet>/captures)에
        # 직접 저장하므로, 여기서 별도로 복사할 필요가 없다.
        filtered_path = parse_filtered_path(finalize_response.message)

        compare_request = SrvPointCloudCompare.Request()
        compare_request.test_path = filtered_path
        node.get_logger().info(
            f"[{object_type.upper()}] compare 호출: test_path={filtered_path}"
        )
        compare_response = call_service(
            node,
            compare_client,
            compare_request,
            COMPARE_TIMEOUT_SEC,
        )
        node.get_logger().info(
            f"[{object_type.upper()}] compare 응답: "
            f"success={compare_response.success}, message={compare_response.message}"
        )
        if not compare_response.success:
            # 비교는 실패했지만 스캔 파일 자체는 남아있으므로 filtered 경로는 계속 실어 보낸다
            # (호출자가 디버깅/재시도를 위해 파일 위치를 알 수 있도록).
            return False, f"filtered={filtered_path}, {compare_response.message}"

        # compare_response에는 output_dir/result_text/similarity 등 구조화된 결과가 담겨 있지만
        # RobotCommand.Result에는 message(문자열) 하나만 담을 수 있으므로, 호출자(HMI 등)가
        # 파싱해서 쓸 수 있도록 key=value 형태로 모두 실어 보낸다. filtered 경로는 항상 맨 앞에 둔다.
        summary = (
            f"filtered={filtered_path}, "
            f"result={compare_response.result_text}, "
            f"similarity={compare_response.similarity * 100:.1f}%, "
            f"missing={compare_response.missing_ratio * 100:.1f}%, "
            f"added={compare_response.added_ratio * 100:.1f}%, "
            f"output_dir={compare_response.output_dir}"
        )
        return True, summary
    except GripperServiceError as error:
        return False, f"그리퍼 개방 실패: {error}"
    except Exception as error:
        message = str(error)
        return False, message
    finally:
        try:
            return_to_home(node)
        except Exception as error:
            node.get_logger().error(f"검사 종료 후 홈 복귀 실패: {error}")
