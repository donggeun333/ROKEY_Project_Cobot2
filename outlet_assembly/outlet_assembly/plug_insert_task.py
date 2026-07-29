"""플러그 삽입(OUTLET_ASSEMBLE) task.

멀티탭을 찾아 구멍 좌표를 뽑고, 플러그를 집어 삽입한다. 인식은 공구 정리와
같은 13클래스 seg 모델(`multi` / `plug`)을 쓰고, 구멍은 `multi` 마스크에서
cv2로 추출한다.

`robot_command_server`가 `run_plug_insert(node)`를 부르는 것이 통합 경로다.
단독 실행(`ros2 run outlet_assembly plug_insert_standalone`)도 되며 그때만
미리보기 창이 뜬다.
"""

import math
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from ultralytics import YOLO

import DR_init

from outlet_assembly.geometry import (
    calculate_angle_diff,
    posx_to_matrix,
    print_log,
)

# ---------------------------------------------------------------------------
# 로봇 / 모션
# ---------------------------------------------------------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY_F = 70
VELOCITY_S = 40
ACC_F = 50
ACC_S = 20

HOME_POS = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# ---------------------------------------------------------------------------
# 인식
# ---------------------------------------------------------------------------
_UNIFIED_SHARE = Path(get_package_share_directory("tool_sorter_core"))
INTEGRATED_MODEL_PATH = str(_UNIFIED_SHARE / "models" / "best.pt")
CALIB_MATRIX_PATH = str(_UNIFIED_SHARE / "models" / "T_gripper2camera.npy")

CONFIDENCE_THRESHOLD = 0.4
FIXED_SCAN_Z = 250

BUFFER_SIZE = 20
REQUIRED_DETECTIONS = 0.3
TARGET_FRAMES_FOR_AVG = 8

# ---------------------------------------------------------------------------
# 그리퍼
# ---------------------------------------------------------------------------
GRIPPER_SERVICE_NAME = "/onrobot/sendCommand"
GRIPPER_SETTLE_SEC = 0.6
GRIPPER_SERVICE_WAIT_SEC = 5.0

PLUG_INSERT_TIMEOUT_SEC = 300.0


class GripperServiceError(RuntimeError):
    """그리퍼 서비스가 없거나 명령이 거절/타임아웃 됐을 때 던진다."""


class OnRobotServiceGripper:
    """RG2를 `/onrobot/sendCommand` 서비스로 동기 제어한다."""

    def __init__(self, node: Node, service_name: str, wait_sec: float) -> None:
        from onrobot_rg_msgs.srv import SetCommand

        self._node = node
        self._service_type = SetCommand
        self._service_name = service_name
        self._client = node.create_client(SetCommand, service_name)
        if not self._client.wait_for_service(timeout_sec=float(wait_sec)):
            raise GripperServiceError(
                f"그리퍼 서비스를 찾을 수 없습니다: {service_name}"
            )

    def _send(self, command: str) -> None:
        request = self._service_type.Request()
        request.command = command
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=GRIPPER_SERVICE_WAIT_SEC
        )
        if not future.done():
            future.cancel()
            raise GripperServiceError(
                f"그리퍼 명령 '{command}' 응답 시간 초과 "
                f"({GRIPPER_SERVICE_WAIT_SEC}s)"
            )
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            raise GripperServiceError(
                f"그리퍼 명령 '{command}' 실패: {message}"
            )

    def open(self) -> None:
        self._send("o")

    def close(self) -> None:
        self._send("c")


def run_plug_insert(
    node: Node,
    show_window: bool = True,
    timeout_s: float = PLUG_INSERT_TIMEOUT_SEC,
) -> tuple[bool, str]:
    DR_init.__dsr__node = node
    _started_at = time.monotonic()
    gripper = OnRobotServiceGripper(
        node,
        service_name=GRIPPER_SERVICE_NAME,
        wait_sec=GRIPPER_SERVICE_WAIT_SEC,
    )

    bridge = CvBridge()
    sensor_data = {
        'depth_img': None, 'color_img': None,
        'fx': None, 'fy': None, 'cx': None, 'cy': None,
        'frame_w': 1280, 'frame_h': 720
    }

    def depth_callback(msg):
        try: sensor_data['depth_img'] = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except: pass

    def camera_info_callback(msg):
        sensor_data['fx'] = msg.k[0]
        sensor_data['cx'] = msg.k[2]
        sensor_data['fy'] = msg.k[4]
        sensor_data['cy'] = msg.k[5]

    def color_callback(msg):
        try:
            sensor_data['color_img'] = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = sensor_data['color_img'].shape[:2]
            sensor_data['frame_w'] = w
            sensor_data['frame_h'] = h
        except Exception: 
            pass

    node.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', depth_callback, 10)
    node.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', camera_info_callback, 10)
    node.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', color_callback, 10)
    print("✅ Depth(Raw), Color(Compressed) 영상 및 Camera Info 토픽 구독 시작!")

    try:
        from DSR_ROBOT2 import (
            movel, get_current_posx, get_current_posj, set_tool, set_tcp, movej, movejx,
            task_compliance_ctrl, set_desired_force, release_compliance_ctrl,
            DR_FC_MOD_REL, check_motion
        )
    except ImportError as e:
        return False, f"DSR_ROBOT2 임포트 실패: {e}"

    # =========================================================================
    # 🛠️ 안전한 좌표 조회 헬퍼 함수 정의
    # =========================================================================
    def get_safe_posx(retries=5, delay=0.1):
        """IndexError 발생 시 재시도하여 안전하게 현재 직교 좌표를 반환합니다."""
        for _ in range(retries):
            try:
                res = get_current_posx()
                if res and len(res) > 0 and len(res[0]) >= 3:
                    return res[0]
            except IndexError:
                time.sleep(delay)
        raise RuntimeError("로봇 직교 좌표(posx) 통신 실패")

    def get_safe_posj(retries=5, delay=0.1):
        """IndexError 발생 시 재시도하여 안전하게 현재 관절 좌표를 반환합니다."""
        for _ in range(retries):
            try:
                res = get_current_posj()
                if res and len(res) > 0:
                    return res
            except IndexError:
                time.sleep(delay)
        raise RuntimeError("로봇 관절 좌표(posj) 통신 실패")
    # =========================================================================

    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")

    try:
        CAM_TO_TCP_MATRIX = np.load(CALIB_MATRIX_PATH)
        print_log("Hand-Eye 캘리브레이션 행렬(.npy) 로드 성공!", "SUCCESS")
    except Exception as e:
        return False, f"캘리브레이션 행렬 로드 실패: {e}"

    print("🧠 통합 YOLO Segmentation 모델 로딩 중...")
    vision_model = YOLO(INTEGRATED_MODEL_PATH)
    print_log("✅ 모델 로딩 완료!")

    print_log("⏳ 로봇 상태 데이터를 기다리는 중...")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            _ = get_current_posx()
            break  
        except IndexError:
            time.sleep(0.5)

    print_log("✅ 로봇 통신 연결 완료! 비전 루프를 시작합니다.")
    print_log("==================================================")

    ROBOT_STATE = "INIT_SCAN"
    detection_buffer = deque(maxlen=BUFFER_SIZE)
    angle_buffer = []
    
    saved_hole_angle = None
    saved_plug_angle = None 

    multitap_target_pos = None 
    multitap_depth_mm = None
    centering_finish_pos = None
    surface_z = None # 스텝 하강 시 측정된 표면 Z 좌표 보관용

    plug_missed_count = 0
    plug_detect_count = 0 
    centering_attempt_count = 0
    
    scan_step_x, scan_step_y = 30, 35  
    last_multitap_box = None
    final_holes_data, final_plug_data = None, None

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() - _started_at > timeout_s:
                return False, f"플러그 삽입 시간 초과({timeout_s:.0f}s, state={ROBOT_STATE})"
            frame = sensor_data['color_img']
            if frame is None: continue

            display_frame = frame.copy()
            FRAME_W, FRAME_H = sensor_data['frame_w'], sensor_data['frame_h']
            CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2
            
            detected_this_frame = 0
            multitap_box = None
            plug_obb_data = None
            
            # ==========================================
            # 비전 인식부
            # ==========================================
            results = vision_model(frame, verbose=False)
            
            if results[0].masks is not None:
                for box, mask_pts in zip(results[0].boxes, results[0].masks.xy):
                    cls_name = vision_model.names[int(box.cls[0])]
                    conf_val = float(box.conf[0])
                    
                    if conf_val < CONFIDENCE_THRESHOLD: continue
                        
                    if cls_name == 'multi' and ROBOT_STATE in ["INIT_SCAN", "SCANNING", "CENTERING", "EXTRACT_HOLES"]:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        w, h = x2 - x1, y2 - y1
                        if w > 40 and h > 40 and x1 > 5 and x2 < (FRAME_W - 5):
                            multitap_box = (x1, y1, x2, y2)
                            last_multitap_box = multitap_box 
                            detected_this_frame = 1
                            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                            cv2.putText(display_frame, f"Multi {conf_val:.2f}", (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                            
                    elif cls_name == 'plug' and ROBOT_STATE in ["SCAN_PLUG", "FIND_PLUG", "PICK_PLUG"]:
                        if len(mask_pts) >= 3:
                            rect = cv2.minAreaRect(np.array(mask_pts, dtype=np.int32))
                            points = np.int32(cv2.boxPoints(rect))
                            
                            min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
                            min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
                            MARGIN = 20
                            is_fully_visible = (min_x > MARGIN and max_x < FRAME_W - MARGIN and 
                                                min_y > MARGIN and max_y < FRAME_H - MARGIN)

                            cv2.polylines(display_frame, [points.reshape((-1, 1, 2))], True, (0, 0, 255), 2)
                            cv2.putText(display_frame, f"Plug {conf_val:.2f}", (min_x, max(20, min_y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            
                            if is_fully_visible:
                                pt0, pt1, pt2, pt3 = points[0], points[1], points[2], points[3]
                                dist01, dist12 = math.dist(pt0, pt1), math.dist(pt1, pt2)
                                
                                if dist01 < dist12:
                                    m1 = (int((pt0[0] + pt1[0]) / 2), int((pt0[1] + pt1[1]) / 2))
                                    m2 = (int((pt2[0] + pt3[0]) / 2), int((pt2[1] + pt3[1]) / 2))
                                else:
                                    m1 = (int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2))
                                    m2 = (int((pt3[0] + pt0[0]) / 2), int((pt3[1] + pt0[1]) / 2))
                                
                                center_axis_angle = calculate_angle_diff(m1, m2)                            
                                plug_obb_data = {
                                    'cx': rect[0][0], 'cy': rect[0][1], 
                                    'angle': center_axis_angle, 'points': points,
                                    'm1': m1, 'm2': m2, 'is_fully_visible': True
                                }
                                cv2.line(display_frame, m1, m2, (0, 255, 255), 2)
                            else:
                                plug_obb_data = {
                                    'cx': rect[0][0], 'cy': rect[0][1], 
                                    'angle': None, 'points': points, 'is_fully_visible': False
                                }
                            detected_this_frame = 1
                                
            detection_buffer.append(detected_this_frame)
            current_detections = sum(detection_buffer)

            # ==================================================
            # 🤖 [상태 머신 로직]
            # ==================================================
            current_robot_pos = None
            robot_motion_state = 0 
            
            try:
                # 메인 루프에서의 기본 좌표 조회
                res_pos = get_current_posx()
                if res_pos and len(res_pos) > 0:
                    current_robot_pos = res_pos[0]
                robot_motion_state = check_motion()
            except Exception:
                pass

            if ROBOT_STATE == "INIT_SCAN":
                print_log("1️⃣ [초기화] 홈 포지션으로 이동합니다.")
                movej(HOME_POS, v=60, a=60)
                print_log("🖐️ 초기 상태를 위해 그리퍼를 엽니다.")
                gripper.open()
                time.sleep(1.0)
                print_log("✅ 초기화 완료.")
                print_log("==================================================")
                
                print_log(f"2️⃣ [멀티탭] 초기 스캔 자세로 로봇 이동 시작 (Z: {FIXED_SCAN_Z}mm)")
                movel([400.0, 0.0, FIXED_SCAN_Z, 0.0, 180.0, 0.0], v=80, a=80)
                time.sleep(1.0)
                print_log("✅ 이동 완료. 멀티탭 주변 스캔을 시작합니다.")
                print_log("==================================================")
                ROBOT_STATE = "SCANNING"

            elif ROBOT_STATE == "SCANNING":
                if current_detections >= REQUIRED_DETECTIONS:
                    print_log("🎯 멀티탭 감지 완료! 중앙 정렬(Centering) 모드로 전환합니다.")
                    ROBOT_STATE = "CENTERING"
                else:
                    if detected_this_frame == 0 and current_robot_pos is not None:
                        print_log(f"👀 멀티탭 미발견 -> 🤖 로봇 이동 시작 (X축 탐색)...")
                        movel([current_robot_pos[0] + scan_step_x, current_robot_pos[1], FIXED_SCAN_Z, current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(1.5) 
                        print_log("✅ 이동 완료. 새로운 위치에서 다시 확인합니다.")

            elif ROBOT_STATE == "CENTERING":
                if current_detections < 5:
                    print_log("⚠️ 객체 놓침. 다시 스캔 모드로 복귀.")
                    ROBOT_STATE = "SCANNING"
                    continue

                if multitap_box:
                    x1, y1, x2, y2 = multitap_box
                    multi_cx = (x1 + x2) / 2
                    multi_cy = (y1 + y2) / 2

                    err_x = multi_cx - CENTER_X
                    err_y = multi_cy - CENTER_Y

                    is_centered = (abs(err_x) < 20 and abs(err_y) < 20)

                    if is_centered or centering_attempt_count >= 5:
                        print_log("멀티탭 중앙 정렬 완료, 정밀 구멍 추출 연산을 시작합니다.", "SUCCESS")
                        
                        if current_robot_pos is not None:
                            centering_finish_pos = list(current_robot_pos)
                            print_log(f"센터링 완료 위치 백업 [X:{centering_finish_pos[0]:.1f}, Y:{centering_finish_pos[1]:.1f}]", "INFO")

                        centering_attempt_count = 0 
                        time.sleep(3.0)    
                        ROBOT_STATE = "EXTRACT_HOLES"
                    else:
                        centering_attempt_count += 1
                        if current_robot_pos is not None:
                            P_GAIN = 0.30 
                            step_x = err_x * P_GAIN
                            step_y = err_y * P_GAIN
                            
                            step_x = max(-30.0, min(30.0, step_x))
                            step_y = max(-30.0, min(30.0, step_y))
                            
                            move_x = current_robot_pos[0] + step_x
                            move_y = current_robot_pos[1] - step_y 
                            
                            print_log(f"멀티탭 렌즈 중앙 정렬 조준 중... ({centering_attempt_count}/5회)", "INFO")
                            movel([move_x, move_y, current_robot_pos[2], current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_S, a=ACC_S)
                            time.sleep(1.0) 

                            for _ in range(5):
                                rclpy.spin_once(node, timeout_sec=0.01)

            elif ROBOT_STATE == "EXTRACT_HOLES":
                if last_multitap_box is None:
                    ROBOT_STATE = "SCANNING"
                    continue
                    
                x1, y1, x2, y2 = last_multitap_box
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(FRAME_W, x2), min(FRAME_H, y2)
                roi_frame = frame[y1:y2, x1:x2]
                
                if roi_frame.size == 0:
                    continue

                hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                valid_centers = []
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if 3 < area < 2000: 
                        (local_cx, local_cy), radius = cv2.minEnclosingCircle(cnt)
                        circle_area = math.pi * (radius ** 2)
                        if circle_area > 0 and (area / circle_area) > 0.1 and 2<= radius <= 1000: 
                            valid_centers.append((int(local_cx) + x1, int(local_cy) + y1, int(radius)))

                paired_holes = None
                min_dist = float('inf')
                if len(valid_centers) >= 2:
                    for i in range(len(valid_centers)):
                        for j in range(i + 1, len(valid_centers)):
                            dist = math.dist((valid_centers[i][0], valid_centers[i][1]), (valid_centers[j][0], valid_centers[j][1]))
                            if 10.0 <= dist <= 200.0 and dist < min_dist:
                                min_dist = dist
                                paired_holes = (valid_centers[i], valid_centers[j])
                
                if paired_holes:
                    pt1, pt2 = (paired_holes[0][0], paired_holes[0][1]), (paired_holes[1][0], paired_holes[1][1])
                    if pt1[0] < pt2[0]: pt1, pt2 = pt2, pt1

                    if multitap_target_pos is None and current_robot_pos is not None:
                        try:
                            FX, FY, CX, CY = sensor_data['fx'], sensor_data['fy'], sensor_data['cx'], sensor_data['cy']
                            if FX is not None:
                                mid_u, mid_v = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
                                depth_img = sensor_data['depth_img']
                                
                                if depth_img is not None:
                                    dh, dw = depth_img.shape[:2]
                                    if 0 <= mid_u < dw and 0 <= mid_v < dh:
                                        d = float(depth_img[mid_v, mid_u])
                                        multitap_depth_mm = d if d > 0 else 450.0
                                        
                                        X_cam = (mid_u - CX) * multitap_depth_mm / FX
                                        Y_cam = (mid_v - CY) * multitap_depth_mm / FY
                                        P_cam = np.array([X_cam, Y_cam, multitap_depth_mm, 1.0])

                                        T_base_tcp = posx_to_matrix(current_robot_pos)
                                        P_tcp = np.dot(CAM_TO_TCP_MATRIX, P_cam)
                                        P_base = np.dot(T_base_tcp, P_tcp)
                                        
                                        multitap_target_pos = list(current_robot_pos)
                                        multitap_target_pos[0] = P_base[0] 
                                        multitap_target_pos[1] = P_base[1] 
                                        multitap_target_pos[2] = P_base[2] 
                                        print_log(f"멀티탭 구멍 중점 좌표 저장", "SUCCESS")
                                        
                        except Exception as e:
                            print_log(f"멀티탭 정밀 위치 백업 에러: {e}", "WARNING")
                    
                    current_angle = calculate_angle_diff(pt1, pt2)
                    angle_buffer.append(current_angle)
                    
                    if len(angle_buffer) >= TARGET_FRAMES_FOR_AVG:
                        saved_hole_angle = sum(angle_buffer) / len(angle_buffer)
                        final_holes_data = paired_holes 
                        print_log("==================================================")
                        print_log(f"멀티탭 평균 각도 도출 성공: {saved_hole_angle:.2f} 도")
                        print_log("==================================================")
                        ROBOT_STATE = "DONE_CALC"
                else:
                    if len(angle_buffer) > 0: angle_buffer.clear()

            elif ROBOT_STATE == "DONE_CALC":
                print_log("🔄 모델 교체(멀티탭->플러그) 및 탐색 모드로 진입합니다.")
                detection_buffer.clear()
                ROBOT_STATE = "SCAN_PLUG"

            elif ROBOT_STATE == "SCAN_PLUG":
                if current_robot_pos is not None:
                    print_log("플러그 탐색을 위해 스캔 상공으로 로봇 이동 중...")
                    movel([current_robot_pos[0], current_robot_pos[1], FIXED_SCAN_Z+50, current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_F, a=ACC_F)
                    time.sleep(1.0)
                    
                    print_log("상공 도착 / 플러그를 인식할 수 있도록 대기")
                    time.sleep(10) 
                    
                    print_log("대기 완료. Y축 방향 플러그 스캔을 시작합니다.")
                    print_log("==================================================")
                    ROBOT_STATE = "FIND_PLUG"

            elif ROBOT_STATE == "FIND_PLUG":
                if plug_obb_data is not None:
                    print("==================================================")
                    print("플러그 감지 성공! 전체 모습 확보를 위해 위치 조정을 시작합니다.")
                    print("==================================================")
                    ROBOT_STATE = "PICK_PLUG"
                    centering_attempt_count = 0
                else:
                    if current_robot_pos is not None:
                        if current_robot_pos[2] > FIXED_SCAN_Z:
                            print_log(f"👇 플러그 미발견! 정밀 탐색을 위해 Z축을 {FIXED_SCAN_Z}mm 로 하강합니다.")
                            movel([current_robot_pos[0], current_robot_pos[1], FIXED_SCAN_Z, current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_F, a=ACC_F)
                            time.sleep(1.0) 
                            continue
                            
                        next_y = current_robot_pos[1] + scan_step_y
                        next_x = current_robot_pos[0] 
                        
                        if abs(next_y) > 150.0:
                            scan_step_y *= -20  
                            next_y = current_robot_pos[1] 
                            scan_step_x_plug = 25.0  
                            next_x = current_robot_pos[0] + scan_step_x_plug
                            print_log(f"🔄 Y축 끝 도달! X축으로 {scan_step_x_plug}mm 살짝 전진 후 반대로 훑습니다.")
                            
                        print_log(f"👀 플러그 탐색 중... (목표 X: {next_x:.1f}, Y: {next_y:.1f})")
                        movel([next_x, next_y, FIXED_SCAN_Z, current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(1.0) 
            
            elif ROBOT_STATE == "PICK_PLUG":
                if plug_obb_data is not None:
                    plug_missed_count = 0 
                    
                    err_x = plug_obb_data['cx'] - CENTER_X
                    err_y = plug_obb_data['cy'] - CENTER_Y

                    is_centered = (abs(err_x) < 15 and abs(err_y) < 15) 

                    if plug_obb_data['is_fully_visible'] and (is_centered or centering_attempt_count >= 8):
                        plug_detect_count += 1
                
                        saved_plug_angle = plug_obb_data['angle']
                        final_plug_data = plug_obb_data
                        print_log(f"정밀 행렬 연산 파지 돌입!")
                        
                        try:
                            FX, FY, CX, CY = sensor_data['fx'], sensor_data['fy'], sensor_data['cx'], sensor_data['cy']
                            depth_image = sensor_data['depth_img']
                            if depth_image is None or FX is None or current_robot_pos is None: continue
                            
                            u, v = int(plug_obb_data['cx']), int(plug_obb_data['cy'])
                            dh, dw = depth_image.shape[:2]
                            if u < 0 or u >= dw or v < 0 or v >= dh: continue

                            Z_mm = float(depth_image[v, u])
                            if Z_mm <= 0: continue

                            X_cam = (u - CX) * Z_mm / FX
                            Y_cam = (v - CY) * Z_mm / FY
                            P_cam = np.array([X_cam, Y_cam, Z_mm, 1.0])

                            T_base_tcp = posx_to_matrix(current_robot_pos)
                            P_tcp = np.dot(CAM_TO_TCP_MATRIX, P_cam)
                            P_base = np.dot(T_base_tcp, P_tcp)

                            target_x = P_base[0]
                            target_y = P_base[1]
                            base_z = P_base[2]

                            GRIPPER_OFFSET = 50.0 
                            target_z = base_z - GRIPPER_OFFSET
                                
                            print(f"👇 행렬 기반 정밀 3D 좌표(X:{target_x:.1f}, Y:{target_y:.1f}, Z:{target_z:.1f})로 하강합니다.")
                            movel([target_x, target_y, target_z, current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_F, a=ACC_F)
                            time.sleep(1.0)
                                
                            gripper.close()
                            time.sleep(2.0) 
                                
                            lift_z = FIXED_SCAN_Z -30
                            print_log(f"플러그 파지 완료! 제자리 수직(Z: {lift_z:.1f}mm)으로 들어올립니다.", "ACTION")
                                
                            grip_finish_pos = get_safe_posx()
                            movel([grip_finish_pos[0], grip_finish_pos[1], lift_z, grip_finish_pos[3], grip_finish_pos[4], grip_finish_pos[5]], v=VELOCITY_F, a=ACC_F)
                            time.sleep(3.0) 

                            # 기존 단일 INSERT_PLUG 상태를 INSERT_APPROACH로 전환하여 파편화
                            ROBOT_STATE = "INSERT_APPROACH"
                            print_log("체결 준비 모드(INSERT_APPROACH)로 전환합니다.", "INFO")

                        except Exception as e:
                            print_log(f"파지 연산 중 에러: {e}", "ERROR")

                    else:
                        plug_detect_count = 0 
                        centering_attempt_count += 1 
                        if current_robot_pos is not None:
                            P_GAIN = 0.30 
                            
                            step_x = err_x * P_GAIN
                            step_y = err_y * P_GAIN
                            step_x = max(-30.0, min(30.0, step_x))
                            step_y = max(-30.0, min(30.0, step_y))
                            
                            move_x = current_robot_pos[0] + step_x
                            move_y = current_robot_pos[1] - step_y 
                            
                            print_log(f"렌즈 중앙 정렬 조준 중... ({centering_attempt_count}/3회)", "INFO")
                            movel([move_x, move_y, current_robot_pos[2], current_robot_pos[3], current_robot_pos[4], current_robot_pos[5]], v=VELOCITY_S, a=ACC_S)
                            time.sleep(3.0) 

                            for _ in range(5):
                                rclpy.spin_once(node, timeout_sec=0.01)

                else:
                    plug_missed_count += 1
                    plug_detect_count = 0 
                    if plug_missed_count > 5: 
                        print_log("⚠️ 시야에서 벗어남! 다시 스캔합니다.", "WARNING")
                        plug_missed_count = 0 
                        centering_attempt_count = 0 
                        ROBOT_STATE = "FIND_PLUG"
                    else: 
                        time.sleep(0.1)

            # ----------------------------------------------------
            # 상태 1. INSERT_APPROACH: 상공 이동 및 각도 회전
            # ----------------------------------------------------
            elif ROBOT_STATE == "INSERT_APPROACH":    
                try:
                    if centering_finish_pos is not None:
                        print_log("기억해둔 멀티탭 센터링 상공으로 이동", "ACTION")
                        ready_insert_pos = get_safe_posx()
                        movel([centering_finish_pos[0], centering_finish_pos[1], FIXED_SCAN_Z, ready_insert_pos[3], ready_insert_pos[4], ready_insert_pos[5]], v=VELOCITY_F, a=ACC_F)
                        time.sleep(1.0)

                        print_log("체결 전 J6 관절 0도 초기화", "ACTION")
                        current_j = get_safe_posj()
                        movej([current_j[0], current_j[1], current_j[2], current_j[3], current_j[4], 0.0], v=VELOCITY_F, a=ACC_F)
                        time.sleep(1.0)
                    
                    if saved_hole_angle is not None:
                        print_log("멀티탭 구멍 각도에 맞춰 제자리 회전", "ACTION")
                        
                        target_j6 = (saved_plug_angle - saved_hole_angle)
                        while target_j6 > 180.0: target_j6 -= 360.0
                        while target_j6 < -180.0: target_j6 += 360.0

                        start_rotate_pos = get_safe_posj()
                        movej([start_rotate_pos[0], start_rotate_pos[1], start_rotate_pos[2], start_rotate_pos[3], start_rotate_pos[4], start_rotate_pos[5]+target_j6], v=VELOCITY_F, a=ACC_F)
                        time.sleep(1.0)

                    if multitap_target_pos is not None:
                        print_log("멀티탭 구멍 중점 좌표의 상공으로 이동", "ACTION")
                        start_insert_pos = get_safe_posx()
                        movel([multitap_target_pos[0], multitap_target_pos[1], multitap_target_pos[2]+50, start_insert_pos[3], start_insert_pos[4], start_insert_pos[5]], v=VELOCITY_F, a=ACC_F)
                        time.sleep(1.0)
                    else:
                        print_log("경고: 멀티탭 백업본 누락!", "WARNING")
                        
                    ROBOT_STATE = "INSERT_MEASURE"
                
                except Exception as e:
                    print_log(f"❌ 체결 접근(APPROACH) 동작 중 에러 발생: {e}", "ERROR")
                    time.sleep(0.5)

            # ----------------------------------------------------
            # 상태 2. INSERT_MEASURE: 표면 높이 스텝 하강 측정
            # ----------------------------------------------------
            elif ROBOT_STATE == "INSERT_MEASURE":
                try:
                    print_log("표면 높이 측정을 위해 스텝 하강을 시작합니다...", "INFO")
                    task_compliance_ctrl([2000, 2000, 2000, 2000, 2000, 2000]) 
                    time.sleep(0.5)
                    set_desired_force([0, 0, -10, 0, 0, 0], [0, 0, 1, 0, 0, 0], DR_FC_MOD_REL) 

                    while_insert_pos = get_safe_posx()
                    prev_z = while_insert_pos[2]
                    surface_z = prev_z 
                    insert_success = False

                    for _ in range(25):
                        next_z = prev_z - 3.0
                        movel([while_insert_pos[0], while_insert_pos[1], next_z, while_insert_pos[3], while_insert_pos[4], while_insert_pos[5]], v=VELOCITY_S, a=ACC_S)
                        
                        time.sleep(1.0)
                        current_z = get_safe_posx()[2]
                        
                        # Z축 위치 변화가 0.5mm 이하면 표면에 닿았다고 판단
                        if abs(prev_z - current_z) < 0.5:
                            surface_z = current_z
                            expected_surface_z = multitap_target_pos[2]
                            
                            if surface_z < (expected_surface_z):
                                print_log(f"한 번에 구멍에 꽂혔습니다!", "SUCCESS")
                                insert_success = True
                            else:
                                print_log(f"멀티탭 표면 접촉 확인 (Z: {surface_z:.1f}mm).", "SUCCESS")
                            break
                            
                        prev_z = current_z
                   
                    # 한 번에 꽂혔을 경우의 처리
                    if insert_success:
                        try: release_compliance_ctrl() 
                        except: pass
                        
                        print_log("완벽하게 체결을 완료했습니다.", "SUCCESS")
                        gripper.open()
                        time.sleep(1.0)
                        
                        cur_pos = get_safe_posx()
                        movel([cur_pos[0], cur_pos[1], cur_pos[2] + 50.0, cur_pos[3], cur_pos[4], cur_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(1.0)
                        ROBOT_STATE = "DONE_ALL"
                    else:
                        # 한 번에 꽂히지 않았다면 다음 탐색 상태로 넘어갑니다.
                        ROBOT_STATE = "INSERT_SEARCH"

                except Exception as e:
                    print_log(f"❌ 체결 측정(MEASURE) 동작 중 에러 발생: {e}", "ERROR")
                    time.sleep(0.5)

            # ----------------------------------------------------
            # 상태 3. INSERT_SEARCH: 미세 이동(Offset) 탐색 
            # ----------------------------------------------------
            elif ROBOT_STATE == "INSERT_SEARCH":
                try:
                    release_compliance_ctrl()
                    
                    detect_pos = get_safe_posx()
                    # 이전 상태에서 구한 surface_z 활용
                    movel([detect_pos[0], detect_pos[1], surface_z + 10.0, detect_pos[3], detect_pos[4], detect_pos[5]], v=VELOCITY_F, a=ACC_F)
                    time.sleep(1.0)
                    
                    print_log("탐색을 시작합니다!", "ACTION")
                    search_offsets = [
                        (0.0, 0.0),   
                        (2.0, 0.0), (-2.0, 0.0), (0.0, 2.0), (0.0, -2.0), 
                        (3.0, 0.0), (-3.0, 0.0), (0.0, 3.0), (0.0, -3.0), 
                        (4.0, 0.0), (-4.0, 0.0), (0.0, 4.0), (0.0, -4.0)  
                    ]
                    
                    center_x = detect_pos[0]
                    center_y = detect_pos[1]
                    insert_success = False

                    for offset_x, offset_y in search_offsets:
                        next_x = center_x - offset_x
                        next_y = center_y + offset_y
                        
                        movel([next_x, next_y, surface_z + 10.0, detect_pos[3], detect_pos[4], detect_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(0.5)

                        task_compliance_ctrl([1000, 1000, 1000, 1000, 1000, 1000]) 
                        time.sleep(0.5)
                        set_desired_force([0, 0, -20, 0, 0, 0], [0, 0, 1, 0, 0, 0], DR_FC_MOD_REL)
                        
                        movel([next_x, next_y, surface_z - 10.0, detect_pos[3], detect_pos[4], detect_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(1.0) 
                        
                        current_z = get_safe_posx()[2]
                        if current_z < surface_z:
                            print_log(f"✨ 쏙 들어갔습니다! 구멍 일치 확인 (하강량: {surface_z - current_z:.1f}mm)", "SUCCESS")
                            insert_success = True
                            gripper.open()
                            break

                        release_compliance_ctrl()
                        
                        movel([next_x, next_y, surface_z + 10.0, detect_pos[3], detect_pos[4], detect_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(0.5)

                    try:
                        release_compliance_ctrl() 
                    except:
                        pass

                    if insert_success:
                        print_log("완벽하게 체결을 완료했습니다.", "SUCCESS")
                        gripper.open()
                        time.sleep(1.0)
                        
                        cur_pos = get_safe_posx()
                        movel([cur_pos[0], cur_pos[1], cur_pos[2] + 50.0, cur_pos[3], cur_pos[4], cur_pos[5]], v=VELOCITY_S, a=ACC_S)
                        time.sleep(1.0)
                    else:
                        print_log("⚠️ 탐색 반경 내에서 구멍을 찾지 못했습니다.", "WARNING")

                    ROBOT_STATE = "DONE_ALL"

                except Exception as e:
                    print_log(f"❌ 체결 탐색(SEARCH) 동작 중 에러 발생: {e}", "ERROR")
                    time.sleep(0.5)

            elif ROBOT_STATE == "DONE_ALL":
                return True, "플러그 삽입 완료"
                                   
            # ==========================================
            # 최종 시각화 렌더링 유지
            # ==========================================
            if final_holes_data is not None:
                h1, h2 = final_holes_data
                cv2.circle(display_frame, (h1[0], h1[1]), h1[2], (0, 255, 0), 2)
                cv2.circle(display_frame, (h2[0], h2[1]), h2[2], (0, 255, 0), 2)
                cv2.line(display_frame, (h1[0], h1[1]), (h2[0], h2[1]), (255, 0, 0), 2)

                mid_px_x = int((h1[0] + h2[0]) / 2)
                mid_px_y = int((h1[1] + h2[1]) / 2)
                cv2.circle(display_frame, (mid_px_x, mid_px_y), 5, (0, 255, 255), -1)

            if final_plug_data is not None:
                pts = final_plug_data['points']
                pm1, pm2 = final_plug_data['m1'], final_plug_data['m2']
                cv2.polylines(display_frame, [pts.reshape((-1, 1, 2))], True, (0, 0, 255), 2)
                cv2.line(display_frame, pm1, pm2, (0, 255, 255), 2)

            # 상태, 움직임, 좌표 실시간 텍스트 출력
            cv2.putText(display_frame, f"STATE: {ROBOT_STATE}", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            motion_text = "MOVING" if robot_motion_state != 0 else "STOPPED"
            motion_color = (0, 165, 255) if robot_motion_state != 0 else (200, 200, 200)
            cv2.putText(display_frame, f"MOTION: {motion_text}", (30, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, motion_color, 2)

            if current_robot_pos is not None:
                cx, cy, cz = current_robot_pos[0], current_robot_pos[1], current_robot_pos[2]
                cv2.putText(display_frame, f"CUR POS: [X:{cx:.1f} Y:{cy:.1f} Z:{cz:.1f}]", (30, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if multitap_target_pos is not None:
                t_x, t_y, t_z = multitap_target_pos[0], multitap_target_pos[1], multitap_target_pos[2]
                cv2.putText(display_frame, f"Target 3D [X:{t_x:.1f} Y:{t_y:.1f} Z:{t_z:.1f}]", (30, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if show_window:
                cv2.imshow("Plug Insert", display_frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    return False, "사용자가 ESC로 중단했습니다"

    except Exception as error:
        return False, f"플러그 삽입 중 예외: {error}"
    finally:
        if show_window:
            cv2.destroyAllWindows()

    return False, "플러그 삽입 루프가 예기치 않게 끝났습니다"


def main(args=None):
    """단독 실행 진입점. 시작 즉시 삽입 시퀀스를 돌리고 미리보기 창을 띄운다."""
    rclpy.init(args=args)
    node = rclpy.create_node("plug_insert_node", namespace=ROBOT_ID)
    try:
        ok, message = run_plug_insert(node, show_window=True)
        print_log(message, "SUCCESS" if ok else "ERROR")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
