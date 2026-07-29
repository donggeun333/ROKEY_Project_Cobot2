# M0609 Tool Sorter Unified

이 패키지는 기존 `m0609_tool_sorter`와
`m0609_timestamped_rgbd_transform`을 하나의 ROS 2 패키지로 합친
비교·전환용 통합본입니다. 기존 두 패키지는 그대로 유지하며, 통합본만
사용할 때는 두 원본 패키지의 launch를 동시에 실행하지 않습니다.

통합본은 패키지만 하나로 합쳤으며 실행 노드는 기존과 동일하게
TF broadcaster 2개, perception, task manager, dashboard로 분리합니다.

두산 M0609, OnRobot RG2, eye-in-hand RealSense를 이용해 무작위로 놓인 공구를
검출하고, 겹친 공구는 depth가 작은(카메라에 가까운) 공구부터 집는 ROS 2
Humble 패키지입니다.

기존 `grasp_vision/grasp_sequence.py`에서 한 프로세스에 섞여 있던 카메라
구독, YOLO, GUI, 사용자 입력, 블로킹 로봇 명령을 세 노드로 분리했습니다.

```
RealSense ──> perception_node ──> annotated image + scene JSON
          └─────────────────────> dashboard 실시간 원본 + 최신 검출 overlay
                              ├──> dashboard 검출표
                              ├──> task_manager ──> M0609 / RG2
                              └──> LAN cable-port HSV 검사 ──> Bool + TTS 문장
```

## 바뀐 핵심

- `perception_node`: RGB와 aligned depth를 timestamp로 맞추고, 추론 중 새
  프레임이 쌓이면 오래된 프레임을 버리고 항상 최신 1장만 처리합니다.
  카메라 QoS queue도 1장으로 제한해 오래된 입력을 처리하지 않습니다.
- `dashboard`: RealSense 원본을 직접 구독하고 가장 최근 검출 결과를 로컬에서
  오버레이하므로 영상 움직임이 YOLO 추론 FPS에 묶이지 않습니다. 로봇 동작과
  다른 프로세스이므로 `move_line` 수행 중에도 영상, 검출표, 작업 상태가 계속
  갱신됩니다. 원본과 scene timestamp 차이가 `overlay_max_age_s`를 넘으면
  움직이는 화면에 오래된 박스를 그리지 않습니다.
- `task_manager`: 하강 목표까지 비동기 `MoveLine`을 **한 번만** 보내 연속
  궤적으로 움직입니다. 그동안 20 Hz로 외력을 읽고 설정값을 연속 2회
  초과하면 `MoveStop(SOFT_STOP)`을 호출합니다. 기존처럼 8 mm마다
  `movel/mwait`를 반복하지 않아 정지-출발 현상이 없습니다.
- 자동 스캔을 활성화하면 `MoveJ(Home)`에서 전체 장면을 먼저 새로 검출하고,
  파지 가능한 공구가 있으면 Bird view를 생략하고 바로 작업합니다. Home에서
  대상이 없을 때만 Base +Z Bird view로 상승합니다. 한 번 배치한 뒤에는 마지막 Pick의 Base XY를
  기억해 Hand-Eye 카메라 오프셋이 보정된 local view로 이동합니다. 주변에
  파지 대상이 3프레임 연속 없을 때만 다시 Home을 거쳐 Bird view로 갑니다.
- Bird/Home 관측 자세와 공구 hover 사이의 안전 높이 이동은 Cartesian
  목표를 관절 보간하는 `MoveJointx`를 사용해 `MoveLine` 직선 경로의 wrist
  특이점 통과를 피합니다. 파지·배치 수직 하강은 `MoveLine`과 외력 감시를
  그대로 유지합니다.
- `MoveJointx`는 현재 IK solution space를 그대로 유지합니다. 목표 pose가 그
  분기에 해가 없으면 컨트롤러는 서비스만 수락하고 `NOT REACHABLE`을 출력한 뒤
  움직이지 않으므로, 도착 검증에서만 드러납니다. 그래서 안전 높이 수평
  이동은 **항상 Home에서 출발**합니다. Home은 관절 pose라 분기가 매번
  동일하고, Pick 위치처럼 분기를 예측할 수 없는 자세에서 출발하지 않습니다.
  파지 후 공구함으로 갈 때도 Home을 먼저 경유합니다.
- scene에는 영상의 ROS `sec/nanosec`를 그대로 보존합니다. 픽셀·depth는
  추론 완료 시점의 현재 로봇 pose가 아니라 **영상 촬영 시각의 TF**로 Base
  좌표에 투영됩니다.
- 겹침 순서는 마스크가 겹치는 쌍에 대해 depth로 위/아래 제약을 만든 뒤
  위상 정렬합니다. 겹치지 않는 물체는 depth를 tie-breaker로 사용합니다.
- `grasp_selector.py`는 공통 `handle`/`vise_head` 마스크를 실제 공구에
  containment 기준으로 연결합니다. sorter에서만 사용하는 로직이므로 별도
  ROS 패키지나 노드로 분리하지 않고 perception 내부 모듈로 유지합니다.
- Pick & Place 대상은 모델의 실제 클래스명 `hammer`, `screwdriver`,
  `wrench`, `monkey_wrench`, `vise` 다섯 종류로 제한합니다.
  `handle`, `vise_head`, LAN 관련 클래스 등은 인식·표시는 유지하지만 로봇의
  작업 대상으로 선택하지 않습니다. `target_names`는 이 다섯 종류 중 일부만
  작업하고 싶을 때 사용하는 추가 필터입니다.
- handle과 `vise_head` 작업에 최대 개방 폭이 필요하지 않아 접근 전과 배치
  해제 시 RG2를 고정 `50 mm`까지만 엽니다. 유효한 비전 폭이면 측정
  폭에서 기본 `5 mm`를 뺀 목표로 닫고, 비전 폭이 없거나 50 mm 범위 밖이면
  로봇이 접근하기 전에 해당 검출을 건너뜁니다.
- 공구함 좌표가 비어 있으면 집은 위치에서 `place_offset_xy_mm`만큼 옆에
  임시 배치합니다. 실제 공구함 좌표를 정하면 `place_pose`가 우선합니다.
- `lan_cable`과 `lan_port`는 기존 YOLO 추론 마스크 안에서 대표 HSV를 구하고,
  공간적으로 가장 가까운 쌍의 색을 비교합니다. 3프레임 연속 판정만 확정하며
  검출·색상·연결 관계가 불분명하면 오판하지 않고 `UNKNOWN`으로 보류합니다.

## 빌드

```bash
cd /home/jwanryu/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tool_sorter_core
source install/setup.bash
```

Ultralytics가 없다면 한 번 설치해야 합니다.

```bash
python3 -m pip install ultralytics
```

패키지의 `models/best.pt`와 `models/T_gripper2camera.npy`는 기존
`grasp_vision`에서 검증하던 파일의 사본입니다.

## 실행 순서

1. 실물 로봇, RG2, RealSense를 먼저 기동합니다.
2. 티칭 펜던트에 TCP `GripperDA_v1`와 Tool Weight `Tool Weight`가 등록되어
   있는지 확인합니다. task manager가 실행 시작 시 두 preset을 선택하고
   서비스 응답으로 다시 검증합니다.

```bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py \
  mode:=real host:=192.168.1.100
```

3. 처음에는 계산만 하는 `inspect` 모드로 실행합니다.

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py
```

GUI의 `작업 시작`을 누르면 현재 1순위 물체의 base 좌표를 계산하지만 로봇은
움직이지 않습니다.

GUI의 독립 `Bird Scan` 버튼은 실제 이동 경고 확인 후
`Home → Base +Z Bird view → 카메라 안정화 → 새 scene 확인`까지만 수행하고
`BIRD_READY` 상태에서 멈춥니다. 그 뒤 `작업 시작 (가능 N)` 버튼으로
좌표 계산 또는 Pick & Place를 별도로 시작합니다. Bird Scan은 새로 실행한
task manager의 `IDLE` 또는 이미 촬영이 끝난 `BIRD_READY` 상태에서만
가능합니다. `ERROR`/`SAFETY_STOP` 이후에는 현장을 확인하고 task manager를
재시작해야 두 동작 버튼이 다시 활성화됩니다. `홈 복귀` 버튼은 촬영 없이
`home_joint_pose` 관절 자세로만 이동하며, 정지 상태(`IDLE`, `BIRD_READY`,
`PAUSED`, `STOPPED`, `COMPLETE`, `BLOCKED`)에서만 활성화됩니다.

시작 버튼 이름과 활성화는 실제로 일어날 동작을 따릅니다. `execution_mode`에
따라 `좌표 계산 시작` / `포인팅 시작` / `공구 집기 시작`으로 바뀌고, 지금
화면이 곧 작업 시야일 때(= `auto_scan_motion: false`이거나 Bird Scan·Home
관측이 끝난 뒤) `가능 N`이 0이면 비활성화됩니다. 세션이 시작하면서 Home/Bird로
이동해 새로 촬영하는 경우에는 현재 화면이 판단 근거가 아니므로 그대로
활성화되고, 툴팁이 "새로 촬영한 장면에서 대상을 선택"한다고 알려줍니다.
`가능 N`은 task manager의 `target_names`와 파지부 매칭 규칙을 그대로 적용해
세션이 건너뛸 공구를 세지 않습니다(대시보드 `target_names` 파라미터를 task
manager와 같은 값으로 유지하세요).
GUI 영상은 `/camera/camera/color/image_raw` 원본을 직접 표시하고 최신 scene의
박스·파지축·LAN 연결선을 로컬에서 합성합니다. 추론 시점의 완전한 annotated
영상은 `/tool_sorter/perception/annotated`에 별도로 유지됩니다.
TCP `GripperDA_v1`는 Hand-Eye 좌표 정확도를 위해 강제 검증합니다. Tool
preset 이름은 좌표계와 무관하므로 `enforce_tool_preset: false`에서는 현재
controller 값을 사용하고 불일치만 경고합니다. 단, 힘 감시와 중력 보상이
정확하려면 활성 Tool의 실제 무게와 무게중심이 RG2 장착 상태와 맞아야 합니다.
ROS에서 `place_pose: []`, `target_names: []`가 `None`으로 전달돼도 빈
목록으로 정규화하며, 시작 설정 오류는 task manager를 종료하지 않고 GUI
상태로 반환합니다.

4. `config/tool_sorter.yaml`의 `table_z_mm`, `finger_offset_deg`를 실제 셀
값으로 맞춘 뒤 hover 위치만 왕복하는 포인팅 검증을 합니다.

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py execution_mode:=point
```

5. 우선은 `place_offset_xy_mm: [150.0, 0.0]` 기본값으로 집은 물체 옆에
   내려놓게 할 수 있습니다. 공구함 pose를 티칭한 뒤에는 YAML의 `place_pose`에
`[X, Y, Z, RX, RY, RZ]`를 입력하면 임시 offset보다 우선합니다.

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py \
  execution_mode:=pick_place
```

`place_pose`가 빈 배열이어도 위 명령은 임시 offset으로 실행됩니다.

### 파지 Z 실측 오프셋

XY와 각도는 맞지만 실제 파지 높이가 모든 물체에서 일정하게 어긋나면 launch
CLI의 `grasp_z_offset_mm`로 Base Z 바이어스를 보정할 수 있습니다. 음수는
아래로, 양수는 위로 이동합니다. 현재 현장 기본값은 `-15 mm`이며 Unified와
Autonomous에 공통 적용됩니다. 기본값을 명시해서 실행하려면 다음과 같습니다.

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py \
  execution_mode:=pick_place \
  grasp_z_offset_mm:=-15.0
```

실행 시 `top`, 보정 전 `raw`, `offset`, 최종 `final` Z를 로그로 남깁니다.
음수 오프셋을 주더라도 최종 TCP Z는
`table_z_mm + minimum_table_clearance_mm` 아래로 내려가지 않습니다.

### RG2 50 mm 사전 개방과 비전 파지 폭

공구의 handle과 `vise_head`만 잡는 현재 시나리오에서는 최대 110 mm까지
열지 않습니다. 접근 전과 배치 후 해제 폭을 `50 mm`로 고정하고, 실제 파지는
비전이 계산한 공구 파지부 폭을 사용합니다.

```text
사전 개방/배치 해제: 50 mm
0 < 비전 폭 <= 50 mm: target = max(grip_width_mm - 5 mm, 0 mm)
비전값 없음/NaN/0 이하/50 mm 초과: 로봇 접근 전 검출 제외
```

관련 파라미터는 `gripper_preopen_width_mm`과
`gripper_width_margin_mm`입니다. 실제 OnRobot 서비스는 폭을 `0.1 mm`
단위 정수 문자열로 받으므로 `50 mm`는 `"500"`으로 변환됩니다. 가상
그리퍼도 같은 단위를 사용합니다. 기본값을 명시해서 실행하려면 다음과
같습니다.

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py \
  execution_mode:=pick_place \
  gripper_preopen_width_mm:=50.0
```

## Home / Bird / Local 재검출 설정

Home TCP Base Z 약 `183 mm`, 동일 X/Y·자세에서 Jog 도달 Z 약 `430 mm`인
현장 실측을 기준으로 `bird_raise_mm: 200.0`을 첫 검증값으로 설정했습니다.
목표 TCP Z는 약 `383 mm`이며 상단까지 약 `47 mm` 여유가 있습니다.
시야가 부족하면 실제 도착을 확인한 뒤 `220.0`까지 단계적으로 올립니다.
`table_z_mm: -0.6`도 기본 설정에 반영되어 있습니다.
자동 이동은 여전히 `auto_scan_motion:=false`가 기본이며,
`local_scan_standoff_mm`는 현장에서 launch CLI로 지정합니다. 먼저 수동
운전으로 다음을 확인합니다.

1. `home_joint_pose: [0, 0, 90, 0, 90, 0]`로 가는 관절 경로가 셀 전체에서
   충돌하지 않는지 확인합니다.
2. Home TCP의 X/Y와 자세를 유지한 채 Base +Z로 올렸을 때 전체 테이블이
   보이는 상승량 `bird_raise_mm`를 측정합니다.
3. 마지막 Pick 주변의 공구가 충분히 크게 보이는 카메라 optical
   axis 방향 거리 `local_scan_standoff_mm`를 측정합니다.
4. Bird 자세에서 calibrated camera optical +Z가 Base -Z를 향하는지
   확인합니다. 기본 허용 기울기는 `bird_max_tilt_deg: 15.0`입니다.
5. 각 MoveLine 뒤 실제 TCP XYZ와 목표값 차이가 기본 `3 mm` 이내인지
   확인합니다. 도달하지 못하면 Pick/Place로 진행하지 않고 ERROR로
   중단합니다.

Bird 이동과 좌표 계산만 먼저 검증:

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py \
  execution_mode:=inspect \
  auto_scan_motion:=true
```

`inspect`라도 `auto_scan_motion:=true`이면 Home/Bird 관측 이동은 수행하지만
검출 대상으로 접근하지는 않습니다. 그다음 `point`를 검증하고 마지막에만
실제 Pick/Place와 local 재검출을 실행합니다.

```bash
ros2 launch tool_sorter_core tool_sorter.launch.py \
  execution_mode:=pick_place \
  auto_scan_motion:=true \
  local_scan_standoff_mm:=<실측값>
```

실행 상태 흐름은 다음과 같습니다.

```text
Home → 대상 있음 → Pick/Place → Local(last Pick XY)
  └─ 대상 없음 → Bird(global)       ├─ 대상 있음 → Pick/Place → Local
                                     └─ 3회 없음 → Home → Bird(global)
```

Local 영상에 다른 공구가 함께 보여도
`local_scan_radius_mm` 밖의 대상은 선택하지 않습니다. 이동 중 또는 카메라
안정화 전에 촬영된 scene도 sequence와 촬영 timestamp 장벽으로 폐기합니다.

## 주요 토픽과 서비스

| 이름 | 용도 |
|---|---|
| `/tool_sorter/perception/annotated` | 추론 시점의 검출 영상 |
| `/tool_sorter/perception/scene` | 순서, depth, 파지 픽셀/각도 JSON |
| `/tool_sorter/bird_scan` | Home/Bird 이동과 새 전체 장면 촬영만 수행 |
| `/tool_sorter/inspection/lan_color` | LAN 색상 검사 상세 JSON |
| `/tool_sorter/inspection/lan_color_ok` | 확정된 LAN 공정 OK/NG Bool |
| `/tool_sorter/speech` | 외부 TTS 노드가 읽을 한국어 판정 문장 |
| `/tool_sorter/task/status` | 작업 상태 JSON |
| `/tool_sorter/start` | 현재 모드 시작 |
| `/tool_sorter/pause` | soft stop 요청 |
| `/tool_sorter/stop` | quick stop 요청 |
| `/tool_sorter/go_home` | `home_joint_pose` 관절 자세로만 복귀 (촬영 없음) |

## 발표용 좌표 변환 자료

Segmentation, robust depth, Hand-Eye Calibration을 이용해 파지점을 Robot
Base 좌표로 변환하는 전체 구현과 포트폴리오 작성 예시는
[Segmentation + Depth 기반 Base 좌표 변환 발표·포트폴리오 문서](docs/segmentation_depth_base_portfolio.md)에
정리되어 있습니다.

YOLO Segmentation의 bounding box·mask·handle 연결·depth·겹침 순서를 처음부터
쉽게 설명한 문서는
[YOLO Segmentation Mask 쉬운 설명](docs/yolo_segmentation_mask_explainer.md)을
참고하세요.

검출된 공구 방향에 맞춰 RG2가 회전하는 원리, Base yaw 계산,
접근축·기울기 유지, `finger_offset_deg` 보정 방법은
[그리퍼 파지 방향 회전 비전공자용 설명](docs/gripper_orientation_explainer.md)에
정리되어 있습니다.

픽셀 좌표가 로봇 base 좌표로 변환되는 과정을 설명할 때는
[발표용 한글 인포그래픽](docs/pixel_to_base_infographic.png)과
[상세 발표 설명 문서](docs/pixel_to_base_explainer.md)를 사용하면 됩니다.
도형과 글자를 직접 편집하려면
[SVG 버전](docs/pixel_to_base_diagram.svg)을 사용할 수 있습니다.

TCP, RG2 파지 중심, Hand-Eye Calibration, 테이블 높이의 관계는
[TCP와 Calibration 쉬운 설명](docs/tcp_calibration_guide.md)에 정리되어 있습니다.

## 실물 검증 체크리스트

1. `inspect`에서 순서와 파지 축이 맞는지 확인
2. 고정된 공구 하나로 `table_z_mm`와 hand-eye 좌표 오차 확인
3. `point`에서 그리퍼가 손잡이와 수직인지 확인 후
   `finger_offset_deg` 보정
4. 낮은 속도와 높은 hover로 빈 그리퍼 pick/place 경로 확인
5. 외력을 손으로 가볍게 가해 soft stop 동작과 임계값 확인
6. 마지막에만 실제 공구로 `pick_place` 실행

GUI의 중지 버튼과 이 패키지의 외력 감시는 안전 인증 기능이 아니며 물리
E-stop을 대체하지 않습니다. 외력 정지 후에는 물체를 쥐고 있을 수 있어 자동
복귀하지 않고 정지 상태를 유지합니다.

LAN 케이블 테이프와 포트 색상 검사 원리, 튜닝값, 현장 검증 조건은
[LAN 케이블–포트 색상 공정 검사](docs/lan_color_inspection.md)를 참고하세요.

공통 파지부를 실제 공구에 연결하는 containment 정책과 라벨링 조건은
[YOLO Segmentation Mask 쉬운 설명 — 13장 설계 근거](docs/yolo_segmentation_mask_explainer.md#13-generic-handle--containment-설계-근거)를
참고하세요.
