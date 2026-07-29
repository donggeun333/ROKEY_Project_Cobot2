"""robot_control 런타임 공통 설정.

여기 값들은 단순 상수 모음이 아니라, 여러 패키지가 같은 전제를 공유하도록 묶은
계약이다. 예를 들어:

- RG2 서비스 이름
- inspection_3d 서비스 이름과 타임아웃
- HMI 캡처 저장 위치
- 볼트/멀티탭 스캔 포즈

경로는 설치형(colcon install)과 소스 실행 둘 다 동작하도록 계산식으로 유지한다.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def resolve_package_resource(package_name: str, relative_path: str) -> str:
    share_dir = Path(get_package_share_directory(package_name)).resolve()
    return str((share_dir / relative_path).resolve())


def resolve_workspace_root() -> Path:
    """현재 파일 위치에서 워크스페이스 루트를 역추적한다."""

    current = Path(__file__).resolve()
    parts = current.parts
    for marker in ("install", "src"):
        if marker in parts:
            return Path(*parts[: parts.index(marker)])
    return current.parents[4]


def resolve_operator_ui_pointcloud_dir(workspace_root: Path) -> Path:
    """operator_ui 패키지의 inspection_3d 저장 루트를 찾는다."""

    candidates = [
        workspace_root / "src" / "cobot2_ws" / "operator_ui" / "pointclouds",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return candidates[0].resolve()


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

GRIPPER_NAME = "rg2"
# 볼트 체결도 공구 정리/전달과 같은 이 서비스를 쓴다. Compute Box의 Modbus
# 소켓(192.168.1.1:502)을 여는 주체를 onrobot_rg_control 드라이버 하나로
# 묶기 위해서다. 예전처럼 이 프로세스가 소켓을 따로 열면 드라이버와 소유권이
# 갈려서 볼트와 공구 작업을 한 세션에서 섞어 쓸 수 없었다.
GRIPPER_SERVICE_NAME = "/onrobot/sendCommand"
GRIPPER_SERVICE_WAIT_SEC = 10.0
GRIPPER_CALL_TIMEOUT_SEC = 3.0

# 드라이버를 거치지 않는 pick_bolt.py 단독 스크립트 전용. 볼트 체결 경로는
# 더 이상 이 값들로 소켓을 열지 않는다.
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
TOOL_NAME = "Tool Weight"
TCP_NAME = "GripperDA_v1"

BOLT_MODEL_PATH = resolve_package_resource("robot_control", "resource/bolt.pt")
BOLT_COLOR_TOPIC = "/camera/camera/color/image_raw"
BOLT_DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
BOLT_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
BOLT_CAMERA_LINK_FRAME = "camera_link"
BOLT_BASE_FRAME = "base_link"
BOLT_TARGET_LABEL = "bolt"
BOLT_DETECT_CONFIDENCE = 0.5
BOLT_DETECT_TIMEOUT_SEC = 5.0
BOLT_TF_TIMEOUT_SEC = 1.0
BOLT_DEPTH_UNIT_SCALE = 0.001
BOLT_SHOW_DETECTION_WINDOW = True
BOLT_DETECTION_WINDOW_NAME = "bolt_detection"
BOLT_MONITOR_DETECTION_ONLY = False
BOLT_LOG_INTERVAL_SEC = 1.0

MOVE_VEL = 100.0
MOVE_ACC = 45.0
SLOW_MOVE_VEL = 10.0
SLOW_MOVE_ACC = 10.0
APPROACH_Z_OFFSET_MM = 50.0
LIFT_Z_OFFSET_MM = 80.0
PICK_X_OFFSET_MM = 6.0
PICK_Y_OFFSET_MM = 8.0
PICK_Z_OFFSET_MM = 4.5
FASTEN_APPROACH_Z_OFFSET_MM = 50.0
FASTEN_LIFT_Z_OFFSET_MM = 80.0

HOME_JOINT = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
DETECT_START_ENABLED = True
DETECT_START_POSE = [
    563.7344970703125,
    -10.817046165466309,
    257.74615478515625,
    143.15733337402344,
    -179.8685760498047,
    -126.5936279296875,
]
FASTEN_POSE_1 = [
    326.1980896,
    -79.53913879,
    16.89053345,
    172.89131165,
    179.95211792,
    173.40356445,
]
FASTEN_POSE_2 = [
    395.29953003,
    -78.62123871,
    16.09443855,
    169.16052246,
    179.99923706,
    169.69648743,
]
FASTEN_POSE_3 = [
    326.7164917,
    -148.31852722,
    16.95344925,
    152.22473145,
    179.99935913,
    152.79296875,
]
FASTEN_POSE_4 = [
    396.34875488,
    -147.29405212,
    16.71938133,
    158.6555481,
    179.9928894,
    159.06808472,
]
FASTEN_SEQUENCE = [
    ("1", FASTEN_POSE_1),
    ("4", FASTEN_POSE_4),
    ("2", FASTEN_POSE_2),
    ("3", FASTEN_POSE_3),
]

OBJECT_TYPE_MULTITAP = "multitap"
OBJECT_TYPE_BOLT = "bolt"
WORKSPACE_ROOT = resolve_workspace_root()
HMI_POINTCLOUD_DIR = resolve_operator_ui_pointcloud_dir(WORKSPACE_ROOT)
POINTCLOUD_REFERENCE_PATH_BY_OBJECT = {
    OBJECT_TYPE_BOLT: resolve_package_resource("inspection_3d", "resource/good_bolt.pcd"),
    OBJECT_TYPE_MULTITAP: resolve_package_resource("inspection_3d", "resource/good_multitap.pcd"),
}
POINTCLOUD_CAPTURE_DIR_BY_OBJECT = {
    OBJECT_TYPE_BOLT: str((HMI_POINTCLOUD_DIR / "bolt" / "captures").resolve()),
    OBJECT_TYPE_MULTITAP: str((HMI_POINTCLOUD_DIR / "outlet" / "captures").resolve()),
}


MULTITAP_SCAN_JOINT_POSITIONS = [
    [4.21884871, 3.42193103, 95.0242310, -0.0846541375, 81.1648254, 4.84697104],
    # [-12.69774652, 12.06244886, 99.99344637, 16.65826116, 78.27893446, -19.24197507],
    [-29.61434174, 20.70296669, 104.96266174, 33.40117645, 75.39304352, -43.33092117],
    # [-23.19217110, 33.44590473, 85.81245041, 29.61595153, 91.43807602, -51.44564056],
    [-16.77000046, 46.18884277, 66.66223907, 25.83072662, 107.48310852, -59.56035995],
    # 3 -> 4 중간 포즈
    # [-3.84705592, 55.07780647, 49.23217582, 17.29619503, 118.69083405, -69.98541069],
    [9.07588863, 63.96677017, 31.80211258, 8.76166344, 129.89855957, -80.41046143],
    # 4 -> 5 중간 포즈
    # [19.31792784, 64.23668098, 28.69690704, -1.66677189, 131.05240631, -96.18429947],
    [29.55996704, 64.5065918, 25.59170151, -12.09520721, 132.20625305, -111.95813751],
    # 5 -> 6 중간 포즈
    # [41.86248398, 52.57065010, 46.21976280, -16.45070744, 118.62212371, -119.07258224],
    [54.16500092, 40.6347084, 66.8478241, -20.80620766, 105.03799438, -126.18702698],
    # 6 -> 7 중간 포즈
    # [65.23998261, 31.43299102, 85.20467759, -27.35255528, 96.39944076, -126.49393845],
    [76.31496429, 22.23127365, 103.56153107, -33.89890289, 87.76088715, -126.80084991],
    # 7 -> 8 중간 포즈
    # [82.20833588, 5.87435913, 118.50201798, -35.52148628, 83.55240250, -136.13718796],
    [88.10170746, -10.48255539, 133.44250488, -37.14406967, 79.34391785, -145.4735260],
    # 8 -> 9 중간 포즈
    # [29.99827195, -12.95594120, 132.76015472, 6.86659813, 72.34585572, -163.58185577],
    [-28.10516357, -15.42932701, 132.07780457, 50.87726593, 65.34779358, -181.69018555],
]
BOLT_SCAN_JOINT_POSITIONS = [
    [-26.85898018, 10.70242214, 87.51332855, -0.41825622, 81.90556335, -26.74835587],
    # 1 -> 2 중간 포즈
    # [-25.07679272, -0.70940400, 110.08996200, 11.33019284, 56.94882011, -78.49814034],
    [-23.29460526, -12.12123013, 132.66659546, 23.07864189, 31.99207687, -130.24792480],
    # 2 -> 3 중간 포즈
    # [-49.74720574, 2.98724604, 125.19876480, 30.80625630, 51.71848488, -89.92769051],
    [-76.19980621, 18.09572220, 117.73093414, 38.53387070, 71.44489288, -49.60745621],
    # 3 -> 4 중간 포즈
    # [-64.09825897, 32.39297104, 95.24343491, 32.62953472, 86.30176162, -52.99451828],
    [-51.99671173, 46.69021988, 72.75593567, 26.72519875, 101.15863037, -56.38158035],
    # 4 -> 5 중간 포즈
    # [-38.98904705, 53.44072914, 58.89245605, 19.39140606, 109.61958695, -65.93564415],
    [-25.98138237, 60.19123840, 45.02897644, 12.05761337, 118.08054352, -75.48970795],
    # 5 -> 6 중간 포즈
    # [-15.16480303, 61.70661544, 42.22973823, 2.94222570, 121.41028977, -86.24493027],
    [-4.34822369, 63.22199249, 39.43050003, -6.17316198, 124.74003601, -97.00015259],
    # 6 -> 7 중간 포즈
    # [6.30903005, 55.38933944, 52.02565193, -12.46645236, 118.62471389, -106.45140839],
    [16.96628380, 47.55668640, 64.62080383, -18.75974274, 112.50939178, -115.90266418],
    # 7 -> 8 중간 포즈
    # [30.25480366, 33.68931007, 84.71103668, -23.87829972, 102.28234482, -121.09646606],
    [43.54332352, 19.82193375, 104.80126953, -28.99685669, 92.05529785, -126.29026794],
    # 8 -> 9 중간 포즈
    # [47.50658035, 9.34150643, 118.21990966, -35.55572701, 82.49637985, -132.95700073],
    [51.46983719, -1.13892090, 131.63854980, -42.11459732, 72.93746185, -139.62373352],
]
SCAN_POSITIONS_BY_OBJECT = {
    OBJECT_TYPE_MULTITAP: MULTITAP_SCAN_JOINT_POSITIONS,
    OBJECT_TYPE_BOLT: BOLT_SCAN_JOINT_POSITIONS,
}

POINTCLOUD_SCAN_VEL = 30
POINTCLOUD_SCAN_ACC = 30
POINTCLOUD_SETTLE_SEC = 0.5

# ---------------------------------------------------------------------------
# 공구 정리(TOOL_CLEANUP) / 공구 전달(TOOL_FETCH)
#
# 실제 인식과 로봇 동작은 tool_sorter_cleanup / _handover 노드가
# 수행한다. robot_control은 아래 인터페이스로 그 노드들을 구동하고 상태
# 토픽으로 완료를 판정하기만 한다.
# ---------------------------------------------------------------------------
TOOL_CLEANUP_PREFIX = "/integration/tool_sorter"
TOOL_HANDOVER_PREFIX = "/tool_sorter/handover"

TOOL_CLEANUP_START_SERVICE = f"{TOOL_CLEANUP_PREFIX}/organize"
TOOL_CLEANUP_STOP_SERVICE = f"{TOOL_CLEANUP_PREFIX}/stop"
TOOL_HANDOVER_START_SERVICE = f"{TOOL_HANDOVER_PREFIX}/start"
TOOL_HANDOVER_STOP_SERVICE = f"{TOOL_HANDOVER_PREFIX}/stop"
TOOL_HANDOVER_REQUEST_TOPIC = f"{TOOL_HANDOVER_PREFIX}/request"

# 서비스 연결/응답 대기. 응답은 "접수했다"만 돌려주므로 짧아도 된다.
TOOL_SERVICE_CONNECT_TIMEOUT_SEC = 5.0
TOOL_SERVICE_RESPONSE_TIMEOUT_SEC = 10.0

# 작업 완료 대기. 정리는 5종을 순차 처리하므로 가장 길다.
TOOL_CLEANUP_TIMEOUT_SEC = 600.0
TOOL_HANDOVER_TIMEOUT_SEC = 600.0
# start 이후 handover가 관측 자세로 이동해 WAITING_REQUEST가 되기까지.
TOOL_HANDOVER_READY_TIMEOUT_SEC = 30.0
# 공구 요청을 발행한 뒤 handover가 그것을 집어들기까지.
TOOL_HANDOVER_PICKUP_TIMEOUT_SEC = 30.0
# stop을 부른 뒤 STOPPED로 내려오기까지.
TOOL_HANDOVER_STOP_TIMEOUT_SEC = 30.0

RESET_SERVICE = "/pointcloud_pipeline/reset"
CAPTURE_SERVICE = "/pointcloud_pipeline/capture"
FINALIZE_SERVICE = "/pointcloud_pipeline/finalize"
COMPARE_SERVICE = "/pointcloud_comparison/compare"

RESET_TIMEOUT_SEC = 10.0
CAPTURE_TIMEOUT_SEC = 20.0
FINALIZE_TIMEOUT_SEC = 180.0
COMPARE_TIMEOUT_SEC = 180.0
PIPELINE_NODE_NAME = "/pointcloud_pipeline"
COMPARISON_NODE_NAME = "/pointcloud_comparison"
