# outlet_assembly

멀티탭을 찾아 구멍 좌표를 뽑고, 플러그를 집어 삽입한다.
음성으로는 **`콘센트 체결해줘`** (intent `OUTLET_ASSEMBLE`).

## 인식

공구 정리/전달과 **같은 13클래스 seg 모델**을 쓴다
(`tool_sorter_core/models/best.pt`). 이 task가 보는 클래스는 둘뿐이다.

| 클래스 | 용도 | valid mask mAP50 |
| --- | --- | --- |
| `multi` | 멀티탭 본체 | 0.943 |
| `plug` | 플러그 | 0.895 |

**구멍은 YOLO가 아니라 `multi` 마스크에서 cv2로 뽑는다.** 구멍 클래스를 따로
학습하지 않은 이유는, 구멍이 작고 조명에 따라 사라져서 검출보다 기하 추출이
안정적이기 때문이다.

모델을 별도로 두지 않은 이유는 추론을 두 번 하지 않기 위해서, 그리고 라벨
세대가 갈라지지 않게 하기 위해서다.

## 상태 머신

```
INIT_SCAN → SCANNING → CENTERING → EXTRACT_HOLES → DONE_CALC
          → SCAN_PLUG → FIND_PLUG → PICK_PLUG → INSERT_PLUG → DONE_ALL
```

앞 절반이 멀티탭 구멍 좌표를 확정하는 구간, 뒤 절반이 플러그를 집어 넣는
구간이다. `INSERT_PLUG`는 힘 제어(`task_compliance_ctrl`)로 밀어 넣는다.

## 그리퍼 — 직접 Modbus를 쓰지 않는다

`tool_sorter_core`의 `OnRobotGripper`(드라이버 서비스)를 쓴다.
원본 스크립트는 `onrobot.RG`로 Compute Box에 직접 붙었는데, 그러면 소켓
소유자가 갈려 **볼트 체결·공구 정리/전달과 한 세션에서 같이 못 쓴다.**

배경은 `robot_control/robot_control/gripper_service.py` 주석에 있다.
고칠 때 `onrobot.RG`를 다시 끌어오지 않는다.

## 실행

통합(권장) — 음성이든 액션이든 같은 문으로 들어간다.

```bash
ros2 action send_goal /dsr01/robot_command \
  voice_interfaces/action/RobotCommand \
  '{intent: "OUTLET_ASSEMBLE", tools: [], targets: []}' --feedback
```

단독 — 미리보기 창이 뜬다. 카메라·로봇 드라이버가 먼저 떠 있어야 한다.

```bash
ros2 launch outlet_assembly plug_insert.launch.py

또는 직접 실행:

```bash
ros2 run outlet_assembly plug_insert_standalone
```
```

## 테스트

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```

`geometry.py`의 순수 함수만 덮는다. 상태 머신은 로봇과 카메라가 있어야
돌아가므로 실기로 검증한다.

## 알려진 제약

- 설정값이 `plug_insert_task.py` 상단 상수로 있다. 파라미터화는 안 했다 —
  원본 스크립트의 동작을 그대로 보존하는 것을 우선했다.
- 삽입 실패 시 재시도 횟수 제한이 `INSERT_PLUG` 안에 하드코딩돼 있다.
- 타임아웃은 `PLUG_INSERT_TIMEOUT_SEC`(기본 300초)이며, 넘으면 현재 상태를
  담은 메시지와 함께 실패로 끝난다.
