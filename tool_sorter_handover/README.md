# M0609 Tool Sorter Handover

요청받은 공구를 찾아 집어서 건네주는 ROS 2 패키지입니다. 인식, timestamp 기반
좌표변환, 로봇·그리퍼 제어는 `tool_sorter_core`에서 재사용하고, 이
패키지는 전달 정책만 담당합니다.

요청은 **키워드 한 낱말**로 받습니다. 그 키워드를 어떻게 만드는지는 이
패키지 밖의 일입니다 — GUI 버튼, 외부 STT 제어부, `ros2 topic pub` 모두
같은 입구를 씁니다. 오디오 장치는 쓰지 않습니다.

## 작업 순서

1. `/tool_sorter/handover/start` 요청 수신
2. 설정과 카메라·로봇 서비스를 검증
3. Home(= 전달 자세)을 거쳐 **공구함 관측 자세**로 이동
4. `WAITING_REQUEST` 상태로 공구 키워드 대기
5. 요청받은 공구를 비전으로 탐색하고 파지 (겹침을 반영한 검출 순서 그대로)
6. Home으로 이동해 공구를 든 채 정지
7. 사용자가 공구를 잡아당겨 **외력이 기준값 이상 상승**하면 그리퍼 개방
8. 관측 자세로 복귀, 다음 요청 대기

## 노드 구성

로봇 노드 하나입니다. 오디오 장치를 열지 않고 아무것도 말하지 않습니다.

| 노드 | 역할 | 로봇 접근 |
| --- | --- | --- |
| `handover_task_manager` | 로봇 제어, 비전, 전달 판정 | O |

```text
/tool_sorter/handover/request   # std_msgs/String, 공구 키워드 (입력)
/tool_sorter/handover/status    # transient-local JSON 상태 (출력)
/tool_sorter/handover/start     # std_srvs/Trigger
/tool_sorter/handover/stop      # std_srvs/Trigger
```

검출 화면과 조작 버튼은 이 패키지의 `handover_dashboard`가 담당합니다
(`use_gui:=false`로 끕니다). organize dashboard를 상속해 영상·검출 테이블은
그대로 쓰고, 전달 작업에 맞게 다음을 바꿉니다.

- **지금 할 일** 카드: `WAITING_REQUEST`면 "필요한 공구를 선택하세요",
  `WAITING_PULL`이면 "공구를 잡아당기세요"를 크게 띄웁니다.
- 시작 버튼 이름이 `전달 시작`입니다.
- Bird Scan 버튼이 없습니다. 전달은 `tool_pickup_scan_pose`에서 관측하는데
  Bird Scan은 organize의 Bird view로 가버리기 때문입니다.
- `공구 요청` 버튼 줄은 요청 대기 상태에서만 눌립니다.

## 관측 자세

공구함 위 실측 절대 자세를 씁니다. 공구가 전부 화면에 들어오는 위치를
현장에서 측정해 넣습니다.

```yaml
tool_pickup_scan_pose: [563.734, -10.817, 257.746, 143.157, -179.869, -126.594]
```

단위는 Doosan Base 기준 위치 mm, 자세 deg입니다. 이 자세의 카메라 광축
기울기는 아래 방향 기준 **1.4도**입니다.

비워두면 organize와 같은 **Bird view**(Home TCP + `bird_raise_mm`)로 자동
대체합니다. 두 경로 모두 이동 전에 `_validate_observation_pose()`가 Hand-Eye
보정 기준으로 카메라가 실제 아래를 보는지 검사하며, `bird_max_tilt_deg`
(15도)를 넘으면 이동하지 않습니다.

전달 자세는 별도 좌표가 없습니다. 이 팔 구조에서는 Home이 사람이 공구를
받기 좋은 위치이므로 `home_joint_pose`를 그대로 씁니다.

## 왜 관측 자세로 갈 때도 Home을 거치는가

`MoveJointx`는 **현재 IK 해 공간(solution space)을 그대로 유지**합니다
(`motion.py:490`). 어깨·팔꿈치·손목 형상이 이동 중에 뒤집히는 것을 막는
안전장치인데, 대가로 목표 자세가 현재 분기에서 도달 불가능하면 컨트롤러가
`NOT REACHABLE`만 내고 멈춥니다.

그래서 이 패키지의 모든 안전 높이 수평 이동은 **분기가 항상 같은 Home에서
출발**합니다. 관측 자세 이동도 예외가 아닙니다.

```text
Home(MoveJ) → 수직 상승(MoveLine) → 수평·yaw 이동(MoveJointx) → 하강(MoveLine)
```

관측 자세에 **MoveJointx로 도착**하기 때문에, 거기서 다시 공구로 가는
MoveJointx도 여전히 Home의 분기를 들고 있습니다. 이 성질이 전달을 마친 뒤
Home으로 복귀하는 흐름과 맞물려 매 반복마다 분기를 초기화합니다.

관측 자세를 `MoveLine`으로 가면 두 가지가 동시에 깨집니다. 직선 경로 중간에
wrist 특이점이 들어갈 수 있고, 도착 후 분기가 검증되지 않은 상태가 됩니다.

`auto_scan_motion: true`가 필요합니다. 꺼져 있으면 상위가 `home_joint_pose`와
`safe_jointx_*` 속도를 검증하지 않으므로 요청을 `BLOCKED` 처리합니다.

> 티칭 펜던트로 조그해서 좌표를 딴 경우, **그때의 관절 형상과 실행 시
> 형상이 다를 수 있습니다.** Home 분기로 같은 Cartesian 자세에 도달하기
> 때문입니다. 첫 실행은 저속으로 경로를 확인하십시오.

## 전달 판정: 왜 절대값이 아니라 상승분인가

공구를 쥐고 있으면 자중으로 이미 힘이 걸려 있고, 그 크기는 공구마다
다릅니다. 그래서 전달 자세에 도착한 뒤 정지 상태의 힘을 먼저 측정해
기준값으로 삼고, **그 기준값 대비 상승분**으로 당김을 판정합니다.

```yaml
release_force_n: 12.0          # 기준값 대비 상승분 (절대값 아님)
release_baseline_samples: 5    # 기준값 측정 샘플 수
release_timeout_s: 30.0
```

`release_force_n`은 충돌 안전 중단 기준인 `force_limit_n`보다 반드시 작아야
하며, 시작 시 검증합니다.

이동 중 힘 감시(`monitor_force`)와는 반대 개념입니다. 이동 중 외력은 충돌
신호라 즉시 정지·중단하지만, 전달 대기 중 외력은 **기대하는 신호**이므로
정지 명령도 `ForceLimitExceeded`도 발생하지 않습니다.

한 번의 스파이크로는 열리지 않습니다. 손이 스치기만 해도 개방되지 않도록
`force_debounce_samples`만큼 연속으로 유지되어야 확정합니다.

**대기 시간이 지나도 그리퍼를 열지 않습니다.** 아무도 잡지 않은 상태에서
열면 공구가 떨어지기 때문입니다. 다시 안내하고 계속 대기하므로, 중단하려면
stop 서비스를 호출하십시오.

## 공구 요청: 키워드 하나

요청은 `/tool_sorter/handover/request`에 **키워드 한 낱말**을 발행하는 것이
전부입니다. 그 낱말을 누가 만드는지는 이 패키지가 관여하지 않습니다.

```bash
ros2 topic pub --once /tool_sorter/handover/request \
  std_msgs/msg/String '{data: "몽키렌치"}'
```

**GUI 버튼** — `handover_dashboard`의 `공구 요청` 줄에서 누르면 같은 토픽으로
발행됩니다. `handover.launch.py`가 dashboard에 `tool_request_topic`과
`tool_request_names`를 넘길 때만 이 줄이 생기므로 organize 화면은 그대로입니다.
버튼 글자가 곧 발행 문자열입니다.

**외부 제어부** — STT나 다른 시스템에서 키워드를 뽑아 이 토픽에 발행하면
됩니다. 이 패키지는 마이크도 스피커도 쓰지 않으므로 인식 스택은 전적으로
바깥에서 관리합니다.

### 받아들이는 키워드

`tool_request.py`의 화이트리스트를 지나야 로봇이 움직입니다. 모르는 낱말은
버리고 상태 토픽에 사유를 올립니다(fail-closed).

| 공구 | 인식되는 표현 |
| --- | --- |
| `hammer` | 망치, 해머, 햄머, hammer |
| `screwdriver` | 드라이버, 스크류드라이버, screwdriver |
| `wrench` | 렌치, 스패너, wrench |
| `monkey_wrench` | 몽키, 몽키렌치, 몽키스패너, monkey wrench |
| `vise` | 바이스, 바이스그립, vise |

정확 매칭이 언제나 먼저이고 가장 긴 별칭이 이깁니다. "몽키렌치"가 자기 안에
든 "렌치"로 해석되지 않는 이유입니다. 실패하면 자모 단위 퍼지 매칭이 인식
오류를 흡수합니다("만치"→`hammer`). 초성이 다르면 거부하고("장치", "반지"),
1·2위 점수가 붙으면 추측하지 않습니다.

```yaml
request_fuzzy_threshold: 0.78   # 1.0으로 두면 사실상 정확 매칭만 남는다
```

외부 제어부가 이미 정제된 키워드를 보낸다면 퍼지 매칭이 필요 없습니다.
그 경우 `1.0`으로 올려 화이트리스트를 더 좁게 쓰십시오.

### 상태 피드백

이 노드는 말하지 않으므로 사용자 안내가 전부 상태로 나갑니다. 세 곳에서
같은 문구를 볼 수 있습니다.

**1. launch 터미널** — 상태가 바뀔 때마다 한 줄씩 찍힙니다. `SEARCHING`처럼
새 장면마다 재발행되는 상태는 도배되지 않도록 `(state, message)` 쌍이 직전과
다를 때만 로그합니다. 상태를 유지한 채 문구만 바뀌는 재안내는 잡힙니다.

```text
[WAITING_REQUEST] 필요한 공구를 선택하세요
[SEARCHING] monkey_wrench 위치를 확인하는 중
[WAITING_PULL] 몽키렌치 전달 대기; 공구를 잡아당기세요
[WAITING_PULL] 몽키렌치를 아직 가져가지 않았습니다; 계속 대기 중이니 ...
```

`ERROR`/`SAFETY_STOP`은 error, `BLOCKED`/`PAUSED`/`STOPPED`는 warning
레벨이라 색으로 구분됩니다.

**2. dashboard**의 `현재 작업` 카드.

**3. 토픽 구독** — 외부 제어부용 JSON입니다.

```bash
ros2 topic echo /tool_sorter/handover/status
```

| state | 뜻 |
| --- | --- |
| `WAITING_REQUEST` | 공구 키워드 대기. **이때만 요청을 받는다** |
| `SEARCHING` | 요청받은 공구를 화면에서 찾는 중 |
| `PLANNED` | 파지 좌표 확정 |
| `WAITING_PULL` | 사용자가 공구를 잡아당기기를 기다리는 중 |
| `RELEASING` | 그리퍼 개방 |

`WAITING_REQUEST`가 아닐 때 들어온 요청은 버립니다. 세션이 시작되지 않았으면
상태 토픽에 사유를 올리고, 이동 중이면 로그만 남깁니다(진행 중인 동작 상태를
덮지 않기 위해서입니다).

## 빌드 및 실행

```bash
cd /home/jwanryu/ws_cobot_pjt/ws_dsr
colcon build --symlink-install \
  --packages-select tool_sorter_handover
source install/setup.bash
```

실물 로봇, RG2, RealSense를 먼저 실행합니다.

```bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py \
  mode:=real host:=192.168.1.100
```

전달 launch:

```bash
ros2 launch tool_sorter_handover handover.launch.py
```

시작·중지. dashboard의 `작업 시작` 버튼은 `/tool_sorter/start`를 부르는데,
같은 핸들러라 결과가 동일합니다.

```bash
ros2 service call /tool_sorter/handover/start std_srvs/srv/Trigger '{}'
ros2 service call /tool_sorter/handover/stop  std_srvs/srv/Trigger '{}'
```

`WAITING_REQUEST`가 되면 요청을 넣습니다.

```bash
ros2 topic pub --once /tool_sorter/handover/request \
  std_msgs/msg/String '{data: "몽키렌치"}'
```

## 안전

- 자동 정리(`tool_sorter_cleanup`)와 **동시에 실행하지 마십시오.**
  같은 로봇 서비스와 TF를 중복 사용합니다.
- 모든 수평 이동은 관측 높이에서 `MoveJointx`로만 합니다. 파지 후 Home으로
  돌아올 때도 관측 높이까지 먼저 수직 후퇴한 뒤 관절 공간으로 이동합니다.
- 이송 구간에도 충돌 감시가 걸립니다(`transfer_force_rise_n`, 기본 20 N).
  이동 직전 정지 상태 기준값 대비 상승분이라 쥔 공구의 자중은 세지 않습니다.
  `MoveJointx`는 끝점만 고정하고 팔꿈치·손목이 지나는 부피는 직선이 아니므로
  이 감시가 그 구간의 유일한 소프트웨어 방어선입니다.
- 이 패키지는 요청 직후 실제 로봇을 움직이므로 `tool_pickup_scan_pose`,
  테이블 높이, 파지 Z 보정을 현장에서 먼저 검증해야 합니다.
- 소프트웨어 force stop은 물리 E-stop을 대체하지 않습니다.
