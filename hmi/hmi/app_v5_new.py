# ==========================================
# 필수 시스템 및 라이브러리 임포트
# ==========================================
import os           # 시스템 환경변수(OPENAI_API_KEY 등) 및 경로 조작용 모듈
import sqlite3      # SQLite 데이터베이스 조작 및 쿼리 실행용 모듈
import json         # JSON 형식 데이터의 인코딩/디코딩 처리용 모듈
import threading    # 백그라운드 타이머, ROS2 스핀, 비동기 처리를 위한 멀티스레딩 모듈
import csv          # 이력 데이터의 CSV 파일 생성 및 스트리밍용 모듈
import io           # 메모리 내 바이너리 및 문자열 I/O 스트림 처리용 모듈
import time         # Tact Time 측정, 대기 및 타임스탬프 처리용 모듈
import math         # 라디안(Radian) <-> 도(Degree) 단위 변환용 수학 모듈
import queue        # 로봇 제어 명령의 직렬 처리(FIFO)를 위한 스레드 세이프 큐[cite: 2]
import re           # 정규표현식 기반 명령 문자열(MOVEJ, JOG_JOINT 등) 파싱용 모듈[cite: 2]
import shutil       # 후처리 완료된 PCD 검사 파일의 복사 및 이동 처리용 모듈[cite: 2]
from pathlib import Path  # 객체지향 디렉토리 및 파일 경로 다루기용 모듈[cite: 2]
from datetime import datetime  # 타임스탬프 표준 포맷팅 및 날짜/시간 처리용 모듈[cite: 2]
from flask import Flask, render_template, request, redirect, session, jsonify, Response, send_file  # Flask 웹 프레임워크 기능 모듈[cite: 2]

import rclpy  # ROS 2 Python 클라이언트 라이브러리[cite: 2]
from rclpy.node import Node  # ROS 2 노드 베이스 클래스[cite: 2]
from rclpy.executors import MultiThreadedExecutor  # 멀티스레드 기반 ROS 2 노드 스핀 실행기[cite: 2]
from rclpy.action import ActionClient  # robot_command_server 액션 서버 호출용 클라이언트[cite: 2]
from rclpy.qos import (  # 공구 작업 상태 토픽(transient-local) 구독용 QoS 설정
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String, Bool  # ROS 2 표준 메시지 타입 (문자열, 불리언)[cite: 2]
from sensor_msgs.msg import JointState  # 로봇 관절 상태 정보 메시지 타입[cite: 2]
from sensor_msgs.msg import Image  # 카메라 이미지 프레임 수신용 메시지 타입[cite: 2]
from voice_interfaces.action import RobotCommand  # robot_control 패키지의 검사/조립 모션 실행 액션[cite: 2]

import DR_init  # 두산로봇(Doosan Robotics) 초기화 및 설정용 전역 모듈[cite: 2]
import cv2  # OpenCV 비전 이미지 처리 및 MJPEG 스트리밍 모듈[cite: 2]
from cv_bridge import CvBridge  # ROS Image 메시지 <-> OpenCV CV2 이미지 변환 브리지[cite: 2]
import numpy as np  # 3D 포인트클라우드 좌표 배열 연산용 수치해석 라이브러리[cite: 2]
from scipy.spatial import cKDTree  # 최근접점 탐색 기반 포인트클라우드 거리 비교 알고리즘 모듈[cite: 2]
import open3d as o3d  # PCD 포맷 파싱 및 3D 공간 포인트 데이터 후처리 라이브러리[cite: 2]

from ament_index_python.packages import get_package_share_directory  # ROS 2 패키지 공유 자원 경로 탐색 모듈[cite: 2]

# ==========================================
# 음성 처리 라이브러리 모듈
# ==========================================
import pyaudio  # 마이크 음성 오디오 입력 스트림 개설용 모듈[cite: 2]
import sounddevice as sd  # Wake Word 감지 즉시 재생할 알림음(Beep) 출력용 모듈[cite: 2]
from dotenv import load_dotenv  # .env 파일로부터 환경변수 자동 로드 모듈[cite: 2]
from langchain_openai import ChatOpenAI  # LangChain 통합 OpenAI LLM 인터페이스[cite: 2]
from langchain.prompts import PromptTemplate  # LLM 프롬프트 템플릿 생성기[cite: 2]

from hmi.voice.MicController import MicController, MicConfig  # 음성 입력 마이크 제어 및 설정 클래스[cite: 2]
from hmi.voice.wakeup_word import WakeupWord  # 호출어("헬로우 로키") 실시간 감지 클래스[cite: 2]
from hmi.voice.stt import STT  # Speech-to-Text(음성-텍스트 변환) 모듈[cite: 2]
from hmi.voice.tts import TTS  # Text-to-Speech(텍스트-음성 변환) 모듈[cite: 2]
from hmi.voice.phrases import get_phrase  # 상황별 음성 안내 멘트 관리 모듈[cite: 2]

# 두산 로봇 비상정지(move_stop) 서비스 직접 호출용 예외 처리[cite: 2]
try:
    from dsr_msgs2.srv import MoveStop  # 두산 로봇 비상정지 서비스 메시지[cite: 2]
    if MoveStop is None:  # 임포트 성공했으나 None인 경우 예외 발생[cite: 2]
        raise ImportError("MoveStop 클래스가 None 입니다.")
    HAS_MOVE_STOP = True  # 서비스 지원 활성화 플래그[cite: 2]
except Exception:
    MoveStop = None  # 비상정지 서비스 미지원 시 예외 처리[cite: 2]
    HAS_MOVE_STOP = False  # 비활성화 플래그 설정[cite: 2]

# ==========================================
# 두산로봇 모듈 및 관절 제어 초기화 설정
# ==========================================
ROBOT_ID = "dsr01"  # 로봇 고유 네임스페이스 ID[cite: 2]
ROBOT_MODEL = "m0609"  # 로봇 기종 모델명[cite: 2]
DR_init.__dsr__id = ROBOT_ID  # 두산 로봇 초기화 ID 설정[cite: 2]
DR_init.__dsr__model = ROBOT_MODEL  # 두산 로봇 초기화 모델 설정[cite: 2]

# 로봇 홈(Home) 포지션 각 관절 각도 (도 단위: [J1, J2, J3, J4, J5, J6])[cite: 2]
HOME_JOINTS = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# M0609 로봇 관절별 소프트웨어 가동 제한 범위 (소프트 리밋 범위, deg)[cite: 2]
JOINT_LIMITS = [
    (-360.0, 360.0),  # J1[cite: 2]
    (-95.0, 95.0),    # J2[cite: 2]
    (-135.0, 135.0),  # J3[cite: 2]
    (-360.0, 360.0),  # J4[cite: 2]
    (-135.0, 135.0),  # J5[cite: 2]
    (-360.0, 360.0),  # J6[cite: 2]
]

MANUAL_VEL = 30.0  # 수동 JOG 제어 이동 속도 (deg/s)[cite: 2]
MANUAL_ACC = 30.0  # 수동 JOG 제어 이동 가속도 (deg/s^2)[cite: 2]

# ==========================================
# 음성 시나리오 및 검사 설정값
# ==========================================
TACT_TARGET_BOLT = 15     # 볼트 조립 공정 목표 Tact Time 제한시간 (초)[cite: 2]
TACT_TARGET_OUTLET = 20   # 콘센트 조립 공정 목표 Tact Time 제한시간 (초)[cite: 2]

# 음성 인식용 공구 키워드. m0609_tool_sorter_handover가 다루는 5종과 맞춰둔다.
# (별칭 표의 원본은 m0609_tool_sorter_handover/tool_request.py의 TOOL_ALIASES이고,
#  robot_control 쪽 task가 최종 정규화를 한 번 더 한다. 여기 목록은 "공구를
#  말했는지"를 판정하는 용도다.)
TOOL_KEYWORDS = [
    "hammer", "screwdriver", "wrench", "monkey wrench", "monkey_wrench", "vise",
    "망치", "해머", "드라이버", "스크류드라이버",
    "몽키렌치", "몽키스패너", "몽키", "렌치", "스패너", "바이스",
]

POINTCLOUD_MATCH_THRESHOLD_MM = 1.0  # 3D 포인트 최근접 매칭 최대 허용 거리 (mm)[cite: 2]
INSPECTION_PASS_THRESHOLD_PCT = 95.0  # 3D 스캔 검사 양품/불량 판정 일치율 기준 (95.0%)[cite: 2]
INSPECTION_TIMEOUT_SEC = 15.0  # 비전 노드 응답 대기 타임아웃 제한시간 (초)[cite: 2]
INSPECTION_ACTION_TIMEOUT_SEC = 300.0  # robot_command 액션(스캔+finalize+compare) 전체 응답 대기 제한시간 (초)
ROBOT_COMMAND_SERVER_WAIT_SEC = 5.0  # robot_command 액션 서버 탐색 대기시간 (초)
# 공구 정리/전달 액션 대기시간 (초). 정리는 5종을 순차로 옮기고, 전달은 사람이
# 공구를 잡아당길 때까지 기다리므로 검사(300초)보다 넉넉해야 한다.
# robot_control 쪽 task 타임아웃(600초)보다 길게 잡아, 사유가 담긴 실패 메시지가
# 도착하기 전에 HMI가 먼저 포기하지 않도록 한다.
TOOL_ACTION_TIMEOUT_SEC = 660.0

# ==========================================
# 패키지 자원 및 환경변수 경로 설정
# ==========================================
PACKAGE_NAME = "hmi"  # ROS 2 HMI 패키지명[cite: 2]
MODULE_DIR = Path(__file__).resolve().parent  # 현재 실행 중인 모듈 디렉토리 경로[cite: 2]

def resolve_resource_dir():
    """템플릿 및 정적 자원 디렉토리 경로 자동 탐색 함수"""
    candidates = []
    try:
        candidates.append(Path(get_package_share_directory(PACKAGE_NAME)))  # ROS2 install 설치 경로[cite: 2]
    except Exception:
        pass
    candidates.extend([MODULE_DIR, MODULE_DIR.parent])  # 소스 코드 상대 경로 추가[cite: 2]

    for candidate in candidates:
        if (candidate / "templates").is_dir():  # templates 폴더 존재 유무 확인[cite: 2]
            return candidate  # 유효 디렉토리 반환[cite: 2]
    return MODULE_DIR.parent  # 기본 상위 디렉토리 반환[cite: 2]

RESOURCE_DIR = resolve_resource_dir()  # 자원 디렉토리 바인딩[cite: 2]
DB_PATH = RESOURCE_DIR / "database" / "assembly_inspection.db"  # SQLite 데이터베이스 파일 경로 지정[cite: 2]

def resolve_env_path():
    """.env 환경변수 설정 파일 경로 탐색 함수"""
    candidates = [RESOURCE_DIR / "resource" / ".env"]
    try:
        share_dir = Path(get_package_share_directory(PACKAGE_NAME))
        candidates.append(share_dir / "resource" / ".env")
    except Exception:
        pass
    candidates += [MODULE_DIR.parent / "resource" / ".env", Path.cwd() / ".env"]
    for c in candidates:
        if c.exists():  # 파일 존재 확인[cite: 2]
            return c
    return candidates[0]

ENV_PATH = resolve_env_path()  # .env 경로 설정[cite: 2]
load_dotenv(dotenv_path=ENV_PATH)  # 환경변수 로드[cite: 2]
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")  # OpenAI API 키 가져오기[cite: 2]

# Flask 웹 애플리케이션 객체 생성[cite: 2]
app = Flask(
    __name__,
    template_folder=str(RESOURCE_DIR / "templates"),
    static_folder=str(RESOURCE_DIR / "static"),
)
app.secret_key = "assembly-robot-hmi-secret-key"  # Flask 세션 암호화 키[cite: 2]

# ==========================================
# 1. SQLite Database 관리 모듈
# ==========================================
def get_db():
    """SQLite 데이터베이스 커넥션 생성 및 반환 함수"""
    con = sqlite3.connect(DB_PATH)
    con.row_factory = sqlite3.Row  # 조회 결과를 Dict 인덱싱 형태(Row)로 변환 설정[cite: 2]
    return con

def init_db():
    """DB 테이블 및 3D PCD 파일 저장용 디렉토리 초기화 함수 (기존 데이터 누적 유지)"""
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)  # DB 디렉토리 생성[cite: 2]
    # 품목별(bolt, outlet) 양품(references) 및 촬영본(captures) PCD 저장 경로 사전 생성[cite: 2]
    for base_dir in [MODULE_DIR.parent / "pointclouds", RESOURCE_DIR / "pointclouds"]:
        (base_dir / "bolt" / "references").mkdir(parents=True, exist_ok=True)
        (base_dir / "bolt" / "captures").mkdir(parents=True, exist_ok=True)
        (base_dir / "outlet" / "references").mkdir(parents=True, exist_ok=True)
        (base_dir / "outlet" / "captures").mkdir(parents=True, exist_ok=True)
    
    with get_db() as con:
        # IF NOT EXISTS: 기존 테이블을 초기화(삭제)하지 않고 데이터 누적 보장[cite: 2]
        con.executescript("""
        CREATE TABLE IF NOT EXISTS inspection_results (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT NOT NULL,
            item_type TEXT NOT NULL,
            item_index INTEGER DEFAULT 0,
            measured_value REAL,
            result TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS process_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT NOT NULL,
            stage_name TEXT NOT NULL,
            message TEXT NOT NULL
        );
        """)
        con.commit()  # 스키마 변경 트랜잭션 커밋[cite: 2]

def log_event(stage_name, message):
    """공정 진행 이벤트를 DB의 process_logs 테이블에 기록하는 함수"""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        with get_db() as con:
            con.execute(
                "INSERT INTO process_logs (timestamp, stage_name, message) VALUES (?, ?, ?)",
                (now, stage_name, message)
            )
            con.commit()
    except Exception as e:
        print(f"[DB Log Error] {e}")

def record_inspection_result(category, result_status, item_index=0, match_pct=None):
    """품질 검사 결과 1건을 inspection_results 테이블에 DB 기록하는 함수"""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        with get_db() as con:
            con.execute(
                "INSERT INTO inspection_results (timestamp, item_type, item_index, measured_value, result) VALUES (?, ?, ?, ?, ?)",
                (now, category, item_index, match_pct if match_pct is not None else 0.0, result_status)
            )
            con.commit()
    except Exception as e:
        print(f"[DB Record Error] {e}")

def get_latest_inspection(category, item_index=None):
    """최신 품질 검사 결과 1건을 DB에서 조회하는 함수"""
    try:
        with get_db() as con:
            if item_index is not None:
                row = con.execute(
                    "SELECT * FROM inspection_results WHERE item_type=? AND item_index=? ORDER BY id DESC LIMIT 1",
                    (category, item_index)
                ).fetchone()
            else:
                row = con.execute(
                    "SELECT * FROM inspection_results WHERE item_type=? ORDER BY id DESC LIMIT 1",
                    (category,)
                ).fetchone()
            return dict(row) if row else None
    except Exception as e:
        print(f"[DB Query Error] {e}")
        return None

# ==========================================
# 2. 3D 포인트클라우드 PCD 파일 처리 모듈
# ==========================================
POINTCLOUD_FILENAME_FORMAT = "%Y%m%d_%H%M%S"  # 저장 파일명 포맷 (예: 20260727_120000.pcd)[cite: 2]

PRIMARY_POINTCLOUD_DIR = MODULE_DIR.parent / "pointclouds"  # 1순위 소스 디렉토리[cite: 2]
SECONDARY_POINTCLOUD_DIR = RESOURCE_DIR / "pointclouds"     # 2순위 공유 자원 디렉토리[cite: 2]

def _pointcloud_set_dir(category, kind):
    """PCD 파일이 위치한 실제 카테고리/유형별 디렉토리 경로 탐색 함수"""
    primary_path = PRIMARY_POINTCLOUD_DIR / category / kind
    if primary_path.exists() and any(primary_path.glob("*.pcd")):
        return primary_path

    secondary_path = SECONDARY_POINTCLOUD_DIR / category / kind
    if secondary_path.exists() and any(secondary_path.glob("*.pcd")):
        return secondary_path

    primary_path.mkdir(parents=True, exist_ok=True)
    return primary_path

def _list_pointcloud_set(category, kind):
    """저장된 .pcd 파일 목록을 최신순으로 조회 및 반환하는 함수"""
    d = _pointcloud_set_dir(category, kind)
    files = sorted(d.glob("*.pcd"), key=lambda p: p.name, reverse=True)
    result = []
    for f in files:
        try:
            ts = datetime.strptime(f.stem, POINTCLOUD_FILENAME_FORMAT).strftime("%Y-%m-%d %H:%M:%S")
        except ValueError:
            ts = f.stem
        result.append({"file": f.name, "timestamp": ts})
    return result

def _latest_pointcloud_path(category, kind):
    """가장 최근 저장된 PCD 파일의 전체 경로를 반환하는 함수"""
    items = _list_pointcloud_set(category, kind)
    if not items:
        return None
    return _pointcloud_set_dir(category, kind) / items[0]["file"]

def _save_pointcloud_pcd(category, kind, points, colors=None):
    """3D 좌표 배열을 Open3D를 이용해 .pcd 파일로 저장하는 함수"""
    filename = datetime.now().strftime(POINTCLOUD_FILENAME_FORMAT) + ".pcd"
    path = _pointcloud_set_dir(category, kind) / filename
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    if colors is not None and len(colors) == len(points):
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
    o3d.io.write_point_cloud(str(path), pcd)
    return filename

def list_references(category):
    """양품(기준) PCD 파일 목록 조회 함수"""
    return _list_pointcloud_set(category, "references")

def latest_reference_path(category):
    """최신 양품(기준) PCD 경로 조회 함수"""
    return _latest_pointcloud_path(category, "references")

def save_reference(category, points, colors=None):
    """양품(기준) PCD 데이터를 저장하는 함수"""
    return _save_pointcloud_pcd(category, "references", points, colors)

def list_captures(category):
    """촬영본 PCD 파일 목록 조회 함수"""
    return _list_pointcloud_set(category, "captures")

def latest_capture_path(category):
    """최신 촬영본 PCD 경로 조회 함수"""
    return _latest_pointcloud_path(category, "captures")

def save_capture(category, points, colors=None):
    """촬영본 PCD 데이터를 저장하는 함수"""
    return _save_pointcloud_pcd(category, "captures", points, colors)

def load_pointcloud_pcd(path):
    """Open3D를 이용하여 PCD 파일을 파싱하고 좌표 및 RGB 데이터를 반환하는 함수"""
    try:
        if path is None or not path.exists():
            print(f"[PCD Load Warning] 파일 존재하지 않음: {path}")
            return None, None
        
        pcd = o3d.io.read_point_cloud(str(path))
        points = np.asarray(pcd.points, dtype=np.float32)
        colors = np.asarray(pcd.colors, dtype=np.float32)

        if len(points) == 0:
            print(f"[PCD Load Warning] 포인트가 0개입니다: {path}")
            return None, None

        # 결측치(NaN/Inf) 정제
        valid_pts_mask = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
        points = points[valid_pts_mask]

        if len(colors) == len(valid_pts_mask):
            colors = colors[valid_pts_mask]
        else:
            colors = None

        if colors is not None and len(colors) > 0:
            if np.max(colors) > 1.0:
                colors = colors / 255.0
            if np.all(colors == 0):
                colors = None
        else:
            colors = None

        return points, colors
    except Exception as e:
        print(f"[PCD Load Error] {path}: {e}")
        return None, None

def compare_pointclouds(reference, captured, threshold_mm=POINTCLOUD_MATCH_THRESHOLD_MM):
    """cKDTree 기반으로 기준 및 촬영 3D 포인트 일치율(%)을 산출하는 알고리즘 함수"""
    if reference is None or captured is None or len(reference) == 0 or len(captured) == 0:
        return None
    tree = cKDTree(reference)
    dists, _ = tree.query(captured)
    dists_mm = dists * 1000.0  # PCD 좌표는 m 단위이므로 mm로 변환 후 threshold_mm과 비교[단위 불일치 버그 수정]
    matched = int(np.sum(dists_mm <= threshold_mm))
    return round(matched / len(captured) * 100, 1)

def run_pointcloud_inspection(category, capture_file=None, reference_file=None):
    """로컬 PCD 파이프라인 비교 검사 수행 함수"""
    ref_path = (_pointcloud_set_dir(category, "references") / reference_file) if reference_file else latest_reference_path(category)
    cap_path = (_pointcloud_set_dir(category, "captures") / capture_file) if capture_file else latest_capture_path(category)
    
    reference, _ = load_pointcloud_pcd(ref_path)
    captured, _ = load_pointcloud_pcd(cap_path)
    
    match_pct = compare_pointclouds(reference, captured)
    if match_pct is None:
        return None, "ERROR"
    result = "OK" if match_pct >= INSPECTION_PASS_THRESHOLD_PCT else "NG"
    return match_pct, result

# ==========================================
# 3. 두산 로봇 제어 컨트롤러 클래스
# ==========================================
class RobotController:
    """RobotController: DSR_ROBOT2 바인딩 기반 스레드 세이프 관절 제어기"""
    def __init__(self, log_fn=print):
        self._log = log_fn  # 로깅 함수 바인딩[cite: 2]
        self._cmd_queue = queue.Queue()  # 명령 직렬화 FIFO 큐[cite: 2]
        self._estop_flag = threading.Event()  # 비상정지 발생 플래그[cite: 2]
        self._dsr_ready = False  # DSR 모듈 지연 초기화 플래그[cite: 2]
        self._dsr_lock = threading.Lock()  # DSR 초기화 동기화 락[cite: 2]
        self._dsr = {}  # DSR API 바인딩 캐시[cite: 2]

        self.node = rclpy.create_node("hmi_robot_controller", namespace=ROBOT_ID)  # ROS 2 전용 노드 생성[cite: 2]
        threading.Thread(target=self._worker_loop, daemon=True).start()  # 워커 스레드 가동[cite: 2]

    def _init_dsr(self):
        """DSR_ROBOT2 바인딩 함수 지연 초기화"""
        with self._dsr_lock:
            if self._dsr_ready:
                return True
            try:
                if self.node is None:
                    raise RuntimeError("robot_ctrl.node 가 None 입니다")
                setattr(DR_init, "__dsr__node", self.node)

                from DSR_ROBOT2 import movej, mwait
                self._dsr["movej"] = movej
                self._dsr["mwait"] = mwait
                self._dsr_ready = True
                self._log("DSR_ROBOT2 초기화 완료 → 로봇 제어 준비됨")
                return True
            except Exception as e:
                self._log(f"DSR_ROBOT2 초기화 실패: {e}")
                return False

    def submit_movej(self, joints):
        """MOVEJ 관절 이동 명령을 안전 클램프 후 큐에 전송하는 함수"""
        clamped = []
        for i, v in enumerate(joints[:6]):
            lo, hi = JOINT_LIMITS[i]
            cv = max(lo, min(hi, float(v)))
            if cv != float(v):
                self._log(f"⚠️ J{i+1} 값 {v}° → 소프트리밋으로 {cv}° 클램프됨")
            clamped.append(cv)
        self._estop_flag.clear()
        self._cmd_queue.put(("MOVEJ", clamped))
        return clamped

    def submit_home(self):
        """홈 포지션 복귀 명령을 큐에 전달하는 함수"""
        self._estop_flag.clear()
        self._cmd_queue.put(("MOVEJ", list(HOME_JOINTS)))

    def request_estop(self):
        """비상정지 신호 수신 시 큐 내 대기 중인 모든 명령을 파기하는 함수"""
        self._estop_flag.set()
        drained = 0
        while True:
            try:
                self._cmd_queue.get_nowait()
                drained += 1
            except queue.Empty:
                break
        if drained:
            self._log(f"비상정지: 대기 명령 {drained}건 폐기")

    def _worker_loop(self):
        """명령 큐에서 명령을 1건씩 차례대로 꺼내 DSR API로 구동하는 워커 스레드 루프"""
        while True:
            cmd, payload = self._cmd_queue.get()
            if self._estop_flag.is_set():
                continue
            if cmd == "MOVEJ":
                self._exec_movej(payload)

    def _exec_movej(self, joints):
        """실제 두산 DSR_ROBOT2 API를 호출하여 관절을 이동하는 함수"""
        if not self._init_dsr():
            return
        try:
            self._log(f"🚀 movej 실행: {joints} (vel={MANUAL_VEL}, acc={MANUAL_ACC})")
            self._dsr["movej"](joints, vel=MANUAL_VEL, acc=MANUAL_ACC)
            self._dsr["mwait"]()  # 이동 동작 완료 대기
            self._log("✅ movej 이동 완료")
        except Exception as e:
            self._log(f"❌ movej 실행 오류: {e}")

# ==========================================
# 4. 음성 처리 엔진 (VoiceEngine)
# ==========================================
KEYWORD_PROMPT = """
    당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.
    <도구 리스트> hammer, screwdriver, wrench, pos1, pos2, pos3
    <출력 형식> [도구1 도구2 ... / pos1 pos2 ...]
    <사용자 입력> "{user_input}"
"""

VOICE_MACRO_COMMANDS = {
    "비상정지": "ESTOP",
    "정지": "ESTOP",
    "홈": "HOME",
    "홈으로": "HOME",
}

VOICE_START_KEYWORD = "시작"
VOICE_SHUTDOWN_KEYWORD = "종료"
STOP_KEYWORDS = ["그만해줘", "멈춰"]

def classify_scenario_command(text):
    """음성 STT 텍스트 분석 및 비즈니스 분류 함수"""
    cmd = text.strip()

    if cmd in VOICE_MACRO_COMMANDS:
        return VOICE_MACRO_COMMANDS[cmd]

    if VOICE_SHUTDOWN_KEYWORD in cmd:
        return "SHUTDOWN"

    if any(k in cmd for k in STOP_KEYWORDS):
        return "STOP"

    if "정리" in cmd and ("줘" in cmd or "해" in cmd):
        return "TOOL_RETURN"

    # 코드/로그/문서는 "볼트 체결"로 부르는데 발화만 "조립"으로 받으면 현장에서
    # "볼트 체결해줘"가 조용히 무시된다. 두 표현을 모두 받는다.
    if "볼트" in cmd and ("조립" in cmd or "체결" in cmd):
        return "BOLT_ASSEMBLE"

    if "콘센트" in cmd and ("조립" in cmd or "체결" in cmd):
        return "OUTLET_ASSEMBLE"

    if "볼트" in cmd and "검사" in cmd:
        return "BOLT_INSPECT"

    if ("멀티탭" in cmd or "콘센트" in cmd) and ("검사" in cmd or "스캔" in cmd):
        return "MULTITAP_INSPECT"

    if cmd in ["검사해줘", "검사 시작", "스캔해줘", "3d 검사"]:
        return "INSPECT_GENERAL"

    if "1번" in cmd and "검사" in cmd:
        return "OUTLET_INSPECT_1"
    if "2번" in cmd and "검사" in cmd:
        return "OUTLET_INSPECT_2"

    matched_tool = find_tool_keyword(cmd)
    if matched_tool and ("가져" in cmd or cmd.endswith("줘")):
        return "TOOL_FETCH"

    if VOICE_START_KEYWORD in cmd:
        return "START_KEYWORD_ONLY"

    return "EXTRACT"

def find_tool_keyword(text):
    """텍스트 내 공구 매칭 검출 함수.

    긴 이름부터 본다. "몽키렌치"를 앞에서부터 훑으면 그 안에 든 "렌치"가 먼저
    걸려서 다른 공구를 가져오게 된다.
    """
    for tool in sorted(TOOL_KEYWORDS, key=len, reverse=True):
        if tool in text:
            return tool
    return None

class VoiceEngine:
    """Wake Word 감지, STT, LLM 및 TTS 통합 제어 엔진"""

    TTS_ECHO_COOLDOWN_SEC = 0.6  # TTS 종료 직후 잔향/에코가 다시 감지되는 걸 막기 위한 유예시간(초)

    def __init__(self, openai_api_key, log_fn=print):
        self._log = log_fn
        self.enabled = bool(openai_api_key)
        self.on_wake = None

        self.on_result = None
        self._speaking = threading.Event()  # TTS 재생 중 플래그(스레드 세이프)
        self._last_speak_end_time = 0.0      # 마지막 TTS 종료 시각 (에코 유예시간 계산용)

        if not self.enabled:
            return

        self.stt = STT(openai_api_key=openai_api_key)
        self.tts = TTS(openai_api_key=openai_api_key)

        self.llm = ChatOpenAI(model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key)
        self.prompt_template = PromptTemplate(input_variables=["user_input"], template=KEYWORD_PROMPT)
        self.lang_chain = self.prompt_template | self.llm

        target_device_index = 10
        try:
            p = pyaudio.PyAudio()
            info = p.get_device_info_by_index(target_device_index)
            p.terminate()
        except Exception:
            target_device_index = None

        mic_config = MicConfig(
            chunk=12000, rate=48000, channels=1, record_seconds=5,
            fmt=pyaudio.paInt16, device_index=target_device_index, buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except Exception as e:
            self._log(f"❌ 마이크 스트림 실패: {e}")
            self.enabled = False

    def start(self, on_wake, on_result):
        """'헬로우 로키' 상시 감지 루프 스레드 구동"""
        self.on_wake = on_wake
        self.on_result = on_result
        if not self.enabled:
            return
        threading.Thread(target=self._listen_loop, daemon=True).start()
        self._log("🎙️ 음성 엔진 시작: wake word('헬로우 로키') 상시 대기 중")

    def speak(self, text):
        """TTS 음성 재생 함수 (재생 중 및 직후 유예시간 동안 wake word 자기 감지를 막기 위해 플래그 관리)"""
        if not self.enabled or not text:
            return
        self._speaking.set()
        try:
            self.tts.speak(text)
        except Exception as e:
            self._log(f"TTS 재생 실패: {e}")
        finally:
            self._last_speak_end_time = time.time()
            self._speaking.clear()

    def beep(self, freq=880, duration=0.15, volume=0.3):
        """Wakeword 감지 시 알림음(Beep) 출력 함수"""
        if not self.enabled:
            return
        try:
            samplerate = 16000
            t = np.linspace(0, duration, int(samplerate * duration), endpoint=False)
            tone = (volume * np.sin(2 * np.pi * freq * t)).astype(np.float32)
            sd.play(tone, samplerate)
        except Exception:
            pass

    def extract_keyword(self, output_message):
        """LLM을 통한 공구 및 목적지 키워드 파싱 함수"""
        try:
            response = self.lang_chain.invoke({"user_input": output_message})
            result = response.content.strip()
            if "/" in result:
                object_part, target_part = result.split("/", 1)
                return object_part.split(), target_part.split()
            return result.split(), []
        except Exception:
            return [], []

    def _listen_loop(self):
        """'헬로우 로키' 상시 모니터링 및 STT 음성 수신 무한 루프"""
        while True:
            try:
                woke = self.wakeup_word.is_wakeup()
            except Exception:
                time.sleep(0.3)
                continue

            if not woke:
                continue

            # TTS가 재생 중이거나 방금 끝난 직후(에코/잔향 구간)라면, 이건 스피커가 낸
            # 자기 목소리를 마이크가 다시 주워들은 것일 가능성이 매우 높다.
            # (예: 완료 멘트에 "검사 시작"이 포함되어 있으면 그대로 재트리거되어 무한루프가 됨)
            if self._speaking.is_set() or (time.time() - self._last_speak_end_time) < self.TTS_ECHO_COOLDOWN_SEC:
                continue

            self._log("🎙️ Wakeword('헬로우 로키') 감지됨")

            if self.on_wake:
                try:
                    self.on_wake()
                except Exception as e:
                    self._log(f"Wake 콜백 에러: {e}")

            try:
                raw_text = self.stt.speech2text()
            except Exception as e:
                self._log(f"STT 실패: {e}")
                continue

            if self.on_result and raw_text:
                try:
                    self.on_result(raw_text)
                except Exception as e:
                    self._log(f"음성 처리 에러: {e}")

# ==========================================
# 5. ROS 2 Bridge Node 클래스
# ==========================================
class AssemblyHmiBridge(Node):
    """ROS 2 토픽 중계, HMI 비즈니스 로직 및 음성 엔진 오케스트레이터 노드"""
    def __init__(self, robot_ctrl: RobotController):
        super().__init__("hmi_bridge_node")
        self.robot_ctrl = robot_ctrl
        self.lock = threading.Lock()  # 데이터 동기화용 락[cite: 2]
        self.bridge = CvBridge()
        self.latest_frame = None

        self.joints = [0.0] * 6
        self.last_stage = "STAGE 0: 시작 대기 중"
        self.stt_text = "음성 입력 대기 중..."
        self.command_logs = []
        self.has_ng_event = False

        self.is_running = False
        self.cycle_start_time = None
        self.target_tact_time = TACT_TARGET_BOLT
        self.cycle_has_ng = False
        self.cycle_category = None

        self.last_assembled_category = "bolt"  # 직전 조립 품목 기억 ('bolt' 또는 'outlet')[cite: 2]
        self.system_started = False
        self.gripper_holding = False
        self.scenario_paused = False
        self.shutdown_in_progress = False

        self.vision_result_event = threading.Event()
        self.latest_vision_result = None

        # ROS 2 서브스크라이버 설정[cite: 2]
        self.joint_sub = self.create_subscription(JointState, f"/{ROBOT_ID}/joint_states", self._joint_callback, 10)
        self.status_sub = self.create_subscription(String, "/robot/process_state", self._process_state_callback, 10)
        self.image_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self._image_callback, 10)
        self.stt_sub = self.create_subscription(String, "/ui/stt_result", self._stt_callback, 10)
        self.gripper_sub = self.create_subscription(Bool, "/gripper/status", self._gripper_status_callback, 10)
        self.scenario_status_sub = self.create_subscription(String, "/scenario/status", self._scenario_status_callback, 10)
        self.vision_result_sub = self.create_subscription(String, "/vision/inspection_result", self._vision_result_callback, 10)

        # ROS 2 퍼블리셔 및 클라이언트 설정[cite: 2]
        self.admin_control_pub = self.create_publisher(String, "/ui/admin_control", 10)
        self.estop_pub = self.create_publisher(Bool, "/ui/emergency_stop", 10)

        self.move_stop_cli = None
        if HAS_MOVE_STOP:
            self.move_stop_cli = self.create_client(MoveStop, f"/{ROBOT_ID}/motion/move_stop")

        self.pick_place_pub = self.create_publisher(String, "/voice/pick_place_command", 10)
        self.stt_result_pub = self.create_publisher(String, "/ui/stt_result", 10)
        self.scenario_pub = self.create_publisher(String, "/scenario/command", 10)
        self.scenario_pause_pub = self.create_publisher(Bool, "/scenario/pause", 10)
        self.inspection_request_pub = self.create_publisher(String, "/vision/inspection_request", 10)

        # robot_control 패키지(robot_command_server)와 통신할 액션 클라이언트[cite: 2]
        # -> HMI가 DSR_ROBOT2를 직접 구동하지 않고, 실제 로봇 제어를 전담하는 프로세스에 위임한다.
        self.robot_command_client = ActionClient(self, RobotCommand, f"/{ROBOT_ID}/robot_command")
        self._inspection_action_lock = threading.Lock()
        self._inspection_action_busy = False

        # 조립 공정 전용 Tact Time 타이머 스레드 구동[cite: 2]
        threading.Thread(target=self._tact_time_loop, daemon=True).start()

        self.voice_engine = VoiceEngine(OPENAI_API_KEY, log_fn=self._add_log)
        self.voice_engine.start(on_wake=self._on_voice_wake, on_result=self._on_voice_result)
        self.voice_engine.speak(get_phrase("system_boot_prompt"))

        self._add_log("ROS 2 HMI Bridge Node가 구동되었습니다.")

    def _add_log(self, msg):
        """이벤트 메시지를 로그에 등록하고 DB 저장 스레드를 호출하는 함수"""
        stamp = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.command_logs.insert(0, f"[{stamp}] {msg}")
            self.command_logs = self.command_logs[:50]
        threading.Thread(target=log_event, args=("ROS2", msg), daemon=True).start()

    def _tact_time_loop(self):
        """조립 공정 전용 Tact Time 경과 감지 루프 (만료 시 자동 종료 및 멘트)"""
        while True:
            time.sleep(0.2)
            should_finish = False
            with self.lock:
                if self.is_running and self.cycle_start_time is not None:
                    elapsed = time.time() - self.cycle_start_time
                    if elapsed >= self.target_tact_time:
                        self.is_running = False
                        self.cycle_start_time = None
                        self.cycle_category = None
                        self.last_stage = "STAGE 1: 다음 음성 명령 대기 중"
                        should_finish = True

            if should_finish:
                self._add_log(f"⏱️ Tact Time({self.target_tact_time}초) 경과 -> 조립 자동 완료 처리")
                self.voice_engine.speak(get_phrase("assembly_complete"))  # "조립이 완료되었습니다."
                time.sleep(1.0)
                self.voice_engine.speak(get_phrase("wakeup_prompt"))      # "어떤 작업을 하시겠습니까?"

    def _joint_callback(self, msg):
        """J1~J6 관절 토픽 수신 및 순서 정밀 매칭 콜백 (3번/4번 스왑 방지)"""
        names = list(msg.name)
        positions = list(msg.position)

        with self.lock:
            if len(positions) < 6:
                return

            ordered = [None] * 6
            for i in range(1, 7):
                targets = [f"joint_{i}", f"joint{i}", f"j{i}", f"J{i}"]
                for t in targets:
                    if t in names:
                        ordered[i - 1] = positions[names.index(t)]
                        break

            if None not in ordered:
                self.joints = [round(math.degrees(p), 2) for p in ordered]
            else:
                self.joints = [round(math.degrees(p), 2) for p in positions[:6]]

    def _process_state_callback(self, msg):
        """로봇 공정 상태 토픽 수신 콜백"""
        try:
            data = json.loads(msg.data)
            stage_text = data.get("stage", "공정 진행 중")
            is_ng = data.get("is_ng", False)
            with self.lock:
                self.last_stage = stage_text
                if is_ng or "NG" in stage_text:
                    self.has_ng_event = True
                    self.cycle_has_ng = True
        except Exception:
            pass

    def _stt_callback(self, msg):
        """외부 STT 입력 수신 콜백"""
        with self.lock:
            self.stt_text = msg.data

    def _gripper_status_callback(self, msg):
        """그리퍼 상태 수신 콜백"""
        with self.lock:
            self.gripper_holding = bool(msg.data)

    def _scenario_status_callback(self, msg):
        """시나리오 상태 수신 및 조립 완료 토픽 처리 콜백"""
        try:
            data = json.loads(msg.data)
            scenario_type = data.get("type")
            result = data.get("result")

            if scenario_type in ("BOLT_ASSEMBLE", "OUTLET_ASSEMBLE") and result == "DONE":
                with self.lock:
                    if self.is_running:
                        self._stop_cycle()
                        self.voice_engine.speak(get_phrase("assembly_complete"))
                        time.sleep(1.0)
                        self.voice_engine.speak(get_phrase("wakeup_prompt"))
        except Exception:
            pass

    def _vision_result_callback(self, msg):
        """비전 검사 결과 수신 콜백"""
        try:
            data = json.loads(msg.data)
            self.latest_vision_result = data
            self.vision_result_event.set()
        except Exception:
            pass

    def run_inspection(self, category, sub_item=None):
        """외부 비전 검사 노드 요청 및 결과 반환 함수"""
        self.vision_result_event.clear()
        self.latest_vision_result = None
        req_msg = String()
        req_msg.data = json.dumps({"category": category, "sub_item": sub_item}, ensure_ascii=False)
        self.inspection_request_pub.publish(req_msg)

        got_response = self.vision_result_event.wait(timeout=INSPECTION_TIMEOUT_SEC)
        if got_response and self.latest_vision_result:
            return self.latest_vision_result

        match_pct, result = run_pointcloud_inspection(category)
        return {"result": result, "match_pct": match_pct}

    def _start_cycle(self, category):
        """조립 공정에 한해 Tact Time 사이클 구동"""
        target = TACT_TARGET_BOLT if category == "bolt" else TACT_TARGET_OUTLET
        with self.lock:
            self.is_running = True
            self.cycle_start_time = time.time()
            self.cycle_has_ng = False
            self.cycle_category = category
            self.target_tact_time = target
            self.last_stage = f"STAGE 2: {'볼트' if category == 'bolt' else '콘센트'} 조립 진행 중"

    def _stop_cycle(self):
        """Tact Time 타이머 중단 함수"""
        with self.lock:
            self.is_running = False
            self.cycle_start_time = None
            self.cycle_category = None
            self.last_stage = "STAGE 1: 다음 음성 명령 대기 중"

    def _on_voice_wake(self):
        """'헬로우 로키' 감지 시 반응 콜백"""
        with self.lock:
            cycle_active = self.cycle_category is not None

        if cycle_active:
            self.voice_engine.beep()
            return

        if not self.system_started:
            self.voice_engine.speak(get_phrase("system_boot_prompt"))
        else:
            self.voice_engine.speak(get_phrase("wakeup_prompt"))

    def _on_voice_result(self, raw_text):
        """STT 음성 인식 문장 해석 및 액션 실행 라우팅 콜백"""
        cmd_upper = raw_text.strip()
        with self.lock:
            self.stt_text = raw_text

        stt_msg = String()
        stt_msg.data = raw_text
        self.stt_result_pub.publish(stt_msg)

        if not self.system_started:
            if VOICE_START_KEYWORD in cmd_upper:
                self.system_started = True
                with self.lock:
                    self.last_stage = "STAGE 1: 다음 음성 명령 대기 중"
                self.voice_engine.speak(get_phrase("system_greeting"))
            else:
                self.voice_engine.speak(get_phrase("system_boot_retry"))
            return

        category = classify_scenario_command(cmd_upper)

        with self.lock:
            cycle_active = self.cycle_category is not None

        if cycle_active and category not in ("STOP", "ESTOP"):
            return

        if category == "STOP":
            if cycle_active:
                self.voice_engine.speak(get_phrase("cycle_stop_ack"))
                self._publish_scenario_command("STOP")
                self._stop_cycle()
                time.sleep(1.0)
                self.voice_engine.speak(get_phrase("wakeup_prompt"))
            return

        if category in ("ESTOP", "HOME"):
            self.voice_engine.speak(get_phrase("macro_ack", command=cmd_upper))
            self.publish_admin_control(category)
            return

        if category == "SHUTDOWN":
            self._begin_shutdown(raw_text)
            return

        if category == "BOLT_ASSEMBLE":
            self.voice_engine.speak(get_phrase("bolt_assemble_start"))
            with self.lock:
                self.last_assembled_category = "bolt"  # 직전 조립품 기록
            self._start_cycle("bolt")
            self._publish_scenario_command("BOLT_ASSEMBLE")
            return

        if category == "OUTLET_ASSEMBLE":
            self.voice_engine.speak(get_phrase("outlet_assemble_start"))
            with self.lock:
                self.last_assembled_category = "outlet"  # 직전 조립품 기록
            self._start_cycle("outlet")
            self._publish_scenario_command("OUTLET_ASSEMBLE")
            return

        # 3D 로봇 스캔 검사 (Tact Time 구동 없음)
        if category in ("BOLT_INSPECT", "MULTITAP_INSPECT", "INSPECT_GENERAL"):
            with self._inspection_action_lock:
                if self._inspection_action_busy:
                    self._add_log("⚠️ 이전 검사 모션이 아직 진행 중입니다. 요청을 무시합니다.")
                    self.voice_engine.speak("아직 이전 검사가 진행 중입니다. 잠시 후 다시 말씀해주세요.")
                    return
                self._inspection_action_busy = True

            self.voice_engine.speak(get_phrase("inspect_start"))

            with self.lock:
                target_cat = self.last_assembled_category

            if category == "MULTITAP_INSPECT":
                target_cat = "outlet"
            elif category == "BOLT_INSPECT":
                target_cat = "bolt"

            threading.Thread(
                target=self._run_robot_scan_motion,
                args=(target_cat, 0),
                daemon=True
            ).start()
            return

        self.voice_engine.speak(get_phrase("command_not_understood"))

    def _publish_scenario_command(self, command_type, tool=None):
        """외부 노드로 시나리오 실행 명령 토픽 발행 함수"""
        payload = {"type": command_type}
        if tool:
            payload["tool"] = tool
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.scenario_pub.publish(msg)

    # =========================================================================
    # [수정] robot_command_server 액션 서버와 통신하여 실제 검사 모션을 위임 실행
    # =========================================================================
    def _send_robot_command_and_wait(self, intent, tools=None, targets=None,
                                      timeout_sec=INSPECTION_ACTION_TIMEOUT_SEC):
        """robot_command_server(/{ROBOT_ID}/robot_command) 액션에 목표를 보내고 완료까지 대기.

        주의: 이 노드는 MultiThreadedExecutor로 이미 spin되고 있으므로(main() 참고),
        여기서 rclpy.spin_until_future_complete()를 다시 호출하면 안 된다(이중 spin).
        대신 콜백은 executor 스레드가 처리하고, 호출 스레드는 threading.Event로만 대기한다.
        """
        if not self.robot_command_client.wait_for_server(timeout_sec=ROBOT_COMMAND_SERVER_WAIT_SEC):
            return False, f"robot_command 액션 서버를 찾을 수 없습니다 (/{ROBOT_ID}/robot_command)"

        goal_msg = RobotCommand.Goal()
        goal_msg.intent = intent
        goal_msg.tools = list(tools or [])
        goal_msg.targets = list(targets or [])

        done_event = threading.Event()
        outcome = {"success": False, "message": "결과를 받지 못했습니다."}

        def _on_result(result_future):
            try:
                action_result = result_future.result().result
                outcome["success"] = bool(action_result.success)
                outcome["message"] = action_result.message
            except Exception as e:
                outcome["message"] = f"결과 수신 오류: {e}"
            finally:
                done_event.set()

        def _on_goal_response(goal_future):
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                outcome["message"] = "로봇이 목표를 거절했습니다."
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(_on_result)

        def _on_feedback(feedback_msg):
            fb = feedback_msg.feedback
            self._add_log(f"[진행] {fb.status} ({fb.progress:.0%})")

        send_goal_future = self.robot_command_client.send_goal_async(
            goal_msg, feedback_callback=_on_feedback
        )
        send_goal_future.add_done_callback(_on_goal_response)

        if not done_event.wait(timeout=timeout_sec):
            return False, f"robot_command 액션 응답 시간 초과({timeout_sec}s)"

        return outcome["success"], outcome["message"]

    def _run_robot_scan_motion(self, category="outlet", item_index=0):
        """[음성 인식 전용] robot_command_server 액션에 검사 모션을 위임하여 실행.
        실제 movej/캡처/finalize/compare는 모두 robot_control 패키지(robot_command_server) 프로세스가 전담한다.
        판정 결과 문구는 로그로만 남기며, HMI DB(inspection_results) 기록은 여전히
        웹 UI '검사 시작' 버튼 -> _run_pointcloud_comparison_and_report 에서만 수행한다(이중 기록 방지)."""

        cat_kr = "볼트" if category == "bolt" else "콘센트(멀티탭)"
        self._add_log(f"🤖 {cat_kr} 3D 로봇 스캔 모션을 robot_command_server에 요청합니다.")

        if category == "bolt":
            intent, tools = "INSPECT_FASTEN", []
        else:
            intent, tools = "CONNECTOR_INSPECT", ["outlet"]

        try:
            success, message = self._send_robot_command_and_wait(intent, tools=tools, targets=[])
        finally:
            with self._inspection_action_lock:
                self._inspection_action_busy = False

        # robot_command_server(pointcloud_inspector_task.py)가 스캔이 끝나는 즉시
        # ~/cobot_ws/src/hmi/pointclouds/<bolt|outlet>/captures/ 에 직접 저장하므로,
        # 여기서 filtered= 경로를 다시 파싱해 복사할 필요가 없다.
        # (이전에는 여기서 MODULE_DIR 기준 상대경로로 복사를 시도했는데, install 빌드 방식에 따라
        #  경로가 어긋날 수 있어 robot_control 쪽의 고정 절대경로 저장으로 옮겼다.)

        if not success:
            self.voice_engine.speak(get_phrase("inspect_error"))
            self._add_log(f"❌ {cat_kr} 스캔 모션 실패: {message}")
            return

        # 파이프라인(comparison_node)이 이미 계산한 1차 결과는 참고용으로만 로그/음성 안내한다.
        # 공식 판정 및 DB 기록(inspection_results)은 여전히 웹 UI '검사 시작' 버튼
        # -> _run_pointcloud_comparison_and_report 에서만 수행한다 (이중 기록 방지 원칙 유지).
        result_match = re.search(r"result=([^,]+)", message)
        similarity_match = re.search(r"similarity=([\d.]+)%", message)
        prelim_result = result_match.group(1).strip() if result_match else None
        prelim_similarity = similarity_match.group(1).strip() if similarity_match else None

        self._add_log(f"✅ {cat_kr} 스캔/검사 모션 완료: {message}")
        if prelim_result and prelim_similarity:
            self._add_log(
                f"ℹ️ (참고용 1차 결과, 미기록) {cat_kr} 판정={prelim_result}, 일치율={prelim_similarity}%"
            )
        self.voice_engine.speak(get_phrase("inspect_complete"))

    def _run_pointcloud_comparison_and_report(self, category="outlet", item_index=0,
                                                capture_file=None, reference_file=None):
        """[웹 UI '검사 시작' 버튼 전용] 화면에서 선택된 특정 capture_file/reference_file을 비교하여
        양품/불량 판정 후 item_index 매칭 DB 기록. 파일명이 없으면(구버전 호출 호환) 최신 파일로 대체.
        로봇 모션은 실행하지 않음."""

        cat_kr = "볼트" if category == "bolt" else "콘센트(멀티탭)"
        self._add_log(
            f"🔍 {cat_kr} 3D 검사를 시작합니다. "
            f"(capture={capture_file or '최신'}, reference={reference_file or '최신'})"
        )

        match_pct, result_str = run_pointcloud_inspection(
            category, capture_file=capture_file, reference_file=reference_file
        )

        if match_pct is None:
            self.voice_engine.speak(get_phrase("inspect_error"))
            self._add_log(f"❌ {cat_kr} PCD 비교 실패: reference 또는 capture 파일이 없거나 로드에 실패했습니다.")
            record_inspection_result(category, "ERROR", item_index=item_index, match_pct=0.0)
            return

        # 요청받은 서브항목(item_index: 0, 1, 2 등) 조건으로 DB 저장 -> viewer3d.html 정상 표출[cite: 3]
        record_inspection_result(category, result_str, item_index=item_index, match_pct=match_pct)
        self._add_log(f"✅ {cat_kr} 검사 완료: {result_str} ({match_pct}%)")

        # 음성 피드백 재생
        if result_str == "OK":
            self.voice_engine.speak("양품입니다.")
        else:
            self.voice_engine.speak("불량입니다.")

        self._add_log(f"🔍 {cat_kr}(#{item_index}) 3D 검사 완료: {result_str} ({match_pct}%)")

    def _begin_shutdown(self, raw_text):
        """시스템 종료 절차 함수"""
        if self.shutdown_in_progress:
            return
        self.shutdown_in_progress = True

        self.voice_engine.speak(get_phrase("system_shutdown_returning_home"))
        self._stop_cycle()
        self.robot_ctrl.submit_home()

        def _wait_home_then_exit():
            deadline = time.time() + 15.0
            while time.time() < deadline:
                with self.lock:
                    current = list(self.joints)
                if all(abs(c - h) < 3.0 for c, h in zip(current, HOME_JOINTS)):
                    break
                time.sleep(0.2)
            self.voice_engine.speak(get_phrase("system_shutdown"))
            time.sleep(2.0)
            os._exit(0)

        threading.Thread(target=_wait_home_then_exit, daemon=True).start()

    def _image_callback(self, msg):
        """카메라 원시 프레임 수신 콜백"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_frame = cv_img
        except Exception:
            pass

    def get_latest_frame(self):
        """최신 프레임 복사본 반환 함수"""
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

    def publish_admin_control(self, cmd_str):
        """웹 UI 및 수동 명령 처리 함수 (단일 관절 JOG 및 MOVEJ 포함)"""
        try:
            cmd_upper = cmd_str.upper().strip()

            msg = String()
            msg.data = cmd_upper
            self.admin_control_pub.publish(msg)

            if cmd_upper == "ESTOP":
                estop_msg = Bool()
                estop_msg.data = True
                self.estop_pub.publish(estop_msg)
                self.robot_ctrl.request_estop()
                self._trigger_move_stop()
                self._add_log("🚨 비상정지(ESTOP) 실행")
                return True, "비상정지 명령을 전송했습니다."

            if cmd_upper == "HOME":
                self.robot_ctrl.submit_home()
                self._add_log(f"🏠 홈 위치 이동 명령: {HOME_JOINTS}")
                return True, "홈 위치로 이동을 시작합니다."

            # 단일 관절 축 개별 JOG 제어 (예: JOG_JOINT:3,5.0 -> 3번 관절만 +5도 상대 이동)
            if cmd_upper.startswith("JOG_JOINT:"):
                parts = cmd_upper.replace("JOG_JOINT:", "").split(",")
                axis_idx = int(parts[0].strip()) - 1
                delta_val = float(parts[1].strip())

                with self.lock:
                    target_joints = list(self.joints)

                target_joints[axis_idx] += delta_val
                applied = self.robot_ctrl.submit_movej(target_joints)
                self._add_log(f"🕹️ J{axis_idx+1} 관절 단독 이동 ({delta_val}°): {applied}")
                return True, f"J{axis_idx+1} 관절 단독 이동 시작"

            # 전체 관절 각도 직접 이동 (예: MOVEJ:[0,0,90,0,90,0])
            if cmd_upper.startswith("MOVEJ:"):
                match = re.search(r"MOVEJ:\[(.*?)\]", cmd_upper)
                if not match:
                    return False, "MOVEJ 명령 형식 오류"
                try:
                    joint_vals = [float(v.strip()) for v in match.group(1).split(",")]
                except ValueError:
                    return False, "관절 각도 변환 실패"
                if len(joint_vals) != 6:
                    return False, "관절 값은 6개여야 합니다."

                applied = self.robot_ctrl.submit_movej(joint_vals)
                self._add_log(f"관절 이동 명령 접수: {applied}")
                return True, f"관절 이동을 시작합니다: {applied}"

            return True, f"명령 '{cmd_upper}' 전송 성공"

        except Exception as e:
            return False, f"명령 전송 실패: {e}"

    def _trigger_move_stop(self):
        """두산 로봇 비상정지 서비스 비동기 호출 함수"""
        if self.move_stop_cli is None or not self.move_stop_cli.service_is_ready():
            return
        req = MoveStop.Request()
        req.stop_mode = 1
        self.move_stop_cli.call_async(req)

    def get_status(self):
        """웹 UI 모니터링용 상태 객체 생성 반환 함수"""
        today_prefix = datetime.now().strftime("%Y-%m-%d") + "%"
        ok_cnt, ng_cnt = 0, 0
        try:
            with get_db() as con:
                ok_cnt = con.execute("SELECT COUNT(*) FROM inspection_results WHERE timestamp LIKE ? AND result='OK'", (today_prefix,)).fetchone()[0]
                ng_cnt = con.execute("SELECT COUNT(*) FROM inspection_results WHERE timestamp LIKE ? AND result='NG'", (today_prefix,)).fetchone()[0]
        except Exception:
            pass

        total_cnt = ok_cnt + ng_cnt
        yield_rate = round((ok_cnt / total_cnt * 100), 1) if total_cnt > 0 else 100.0

        with self.lock:
            if self.is_running and self.cycle_start_time:
                elapsed_cycle = int(min(time.time() - self.cycle_start_time, self.target_tact_time))
            else:
                elapsed_cycle = 0

            status_data = {
                "joints": list(self.joints),
                "stage": self.last_stage,
                "stt": self.stt_text,
                "logs": list(self.command_logs),
                "has_ng": self.has_ng_event,
                "tact_time": int(elapsed_cycle),
                "target_tact": int(self.target_tact_time),
                "category": self.cycle_category,
                "total_count": total_cnt,
                "ok_count": ok_cnt,
                "ng_count": ng_cnt,
                "yield_rate": yield_rate,
                "is_running": self.is_running
            }
            self.has_ng_event = False
            return status_data

ros_node = None
robot_ctrl = None

def generate_camera_stream():
    """웹 스트리밍용 MJPEG 생성기 함수"""
    while True:
        if ros_node is not None:
            frame = ros_node.get_latest_frame()
            if frame is not None:
                ret, buffer = cv2.imencode(".jpg", frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.03)

# ==========================================
# 6. Flask 웹 라우트 및 REST API 정의
# ==========================================
@app.route("/")
def index():
    return redirect("/admin/dashboard")

@app.route("/admin/dashboard")
def admin_dashboard():
    return render_template("admin_v3.html")

@app.route("/admin/viewer3d")
def admin_viewer3d():
    return render_template("viewer3d_v4.html")

@app.route("/admin/control")
def admin_control():
    return render_template("control_v3.html")

@app.route("/admin/history")
def admin_history():
    return render_template("history_v2.html")

@app.get("/video_feed")
def video_feed():
    return Response(generate_camera_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/robot/status")
def api_robot_status():
    if ros_node is None:
        return jsonify({"stage": "Node 미구동", "joints": [0]*6, "stt": "-", "logs": [],
                        "has_ng": False, "tact_time": 0, "target_tact": 10, "category": None,
                        "total_count": 0, "ok_count": 0, "ng_count": 0,
                        "yield_rate": 100.0, "is_running": False})
    return jsonify(ros_node.get_status())

@app.post("/api/robot/command")
def api_robot_command():
    data = request.get_json(force=True)
    cmd = data.get("command", "")
    if ros_node is None:
        return jsonify({"ok": False, "message": "ROS2 Node 미구동"}), 503
    ok, msg = ros_node.publish_admin_control(cmd)
    return jsonify({"ok": ok, "message": msg})

@app.get("/api/inspection/pointcloud")
def api_inspection_pointcloud():
    category = request.args.get("category", "bolt")
    which = request.args.get("which", "reference")
    file_name = request.args.get("file")

    if which == "reference":
        path = (_pointcloud_set_dir(category, "references") / file_name) if file_name else latest_reference_path(category)
    else:
        path = (_pointcloud_set_dir(category, "captures") / file_name) if file_name else latest_capture_path(category)

    points, colors = load_pointcloud_pcd(path)
    if points is None:
        return jsonify({"points": [], "colors": []})
        
    if len(points) > 30000:
        idx = np.random.choice(len(points), 30000, replace=False)
        points = points[idx]
        if colors is not None:
            colors = colors[idx]

    res_colors = colors.tolist() if colors is not None else []
    return jsonify({"points": points.tolist(), "colors": res_colors})

@app.get("/api/inspection/references")
def api_inspection_references():
    category = request.args.get("category", "bolt")
    return jsonify({"references": list_references(category)})

@app.get("/api/inspection/captures")
def api_inspection_captures():
    category = request.args.get("category", "bolt")
    return jsonify({"captures": list_captures(category)})

@app.get("/api/inspection/latest")
def api_inspection_latest():
    category = request.args.get("category", "bolt")
    item_index = request.args.get("item_index", type=int)
    row = get_latest_inspection(category, item_index=item_index)
    if row is None:
        return jsonify({"result": None, "match_pct": None, "timestamp": None})
    return jsonify({
        "result": row["result"],
        "match_pct": row["measured_value"],
        "timestamp": row["timestamp"],
    })

# HTML3D 뷰어 "검사 시작" 버튼 클릭 시 3D 로봇 스캔 검사 연동 라우트 (item_index 바인딩)[cite: 3]
@app.post("/api/inspection/run")
def api_inspection_run():
    if ros_node is None:
        return jsonify({"ok": False, "message": "ROS2 Node 미구동"}), 503
    
    data = request.get_json(force=True) or {}
    category = data.get("category", "outlet")
    item_index = data.get("item_index") or 0  # viewer3d_v4.html의 서브항목 전달값 수신[cite: 3]
    capture_file = data.get("capture_file") or None   # 화면에서 선택 중인 촬영 이미지 파일명
    reference_file = data.get("reference_file") or None  # 화면에서 선택 중인 기준 이미지 파일명

    ros_node.voice_engine.speak(get_phrase("inspect_start"))
    
    # 웹 요청 시 저장된 PCD 파일(reference/capture) 기반 3D 비교 검사 독립 실행
    # (로봇 모션 없음, item_index + 화면에서 선택한 정확한 파일명까지 전달)[cite: 3]
    threading.Thread(
        target=ros_node._run_pointcloud_comparison_and_report,
        args=(category, item_index, capture_file, reference_file),
        daemon=True
    ).start()
        
    return jsonify({"ok": True, "message": f"{category} 저장된 PCD 파일 기반 3D 검사를 시작합니다."})

@app.get("/api/history/data")
def api_history_data():
    with get_db() as con:
        rows = con.execute("SELECT * FROM inspection_results ORDER BY id DESC LIMIT 100").fetchall()
        data = [dict(row) for row in rows]
    return jsonify(data)

@app.get("/api/history/download_csv")
def download_csv():
    with get_db() as con:
        rows = con.execute("SELECT * FROM inspection_results ORDER BY id DESC").fetchall()

    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["ID", "검사시간", "종류(볼트/콘센트)", "서브항목", "일치율(%)", "판정결과"])
    for r in rows:
        writer.writerow([r["id"], r["timestamp"], r["item_type"], r["item_index"], r["measured_value"], r["result"]])

    mem = io.BytesIO()
    mem.write(output.getvalue().encode('utf-8-sig'))
    mem.seek(0)
    return send_file(mem, mimetype="text/csv", as_attachment=True,
                     download_name=f"inspection_history_{datetime.now().strftime('%Y%m%d')}.csv")

# ==========================================
# 메인 엔트리포인트 실행 함수
# ==========================================
def main(args=None):
    global ros_node, robot_ctrl
    init_db()  # DB 및 디렉토리 초기화 (데이터 누적)
    rclpy.init(args=args)  # ROS 2 초기화

    def safe_logger(msg):
        if ros_node is not None:
            ros_node._add_log(msg)
        else:
            print(f"[PRE-INIT LOG] {msg}")

    robot_ctrl = RobotController(log_fn=safe_logger)
    ros_node = AssemblyHmiBridge(robot_ctrl)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_node)
    threading.Thread(target=executor.spin, daemon=True).start()  # ROS 2 스핀 백그라운드 구동

    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)  # Flask 서버 실행

    rclpy.shutdown()  # 종료 처리

if __name__ == "__main__":
    main()