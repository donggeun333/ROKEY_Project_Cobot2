# Robot Control Integration Guide

이 문서는 다른 팀원이 개발 중인 아래 2개 기능을 현재 `cobot2_ws` 구조에
어떻게 통합하면 되는지 설명한다.

- 멀티탭 체결
- 공구 전달

현재 기준 통합 대상 구조:

- 음성 해석: `voice_processing/voice_processing/get_keyword_node.py`
- 음성 전달: `robot_control/robot_control/voice_command_dispatcher.py`
- 실행 라우터: `robot_control/robot_control/robot_command_server.py`
- 볼트 체결 task: `robot_control/robot_control/bolt_assemble_task.py`
- 3D 검사 task: `robot_control/robot_control/pointcloud_inspector_task.py`
- 공통 설정: `robot_control/robot_control/task_config.py`

## 현재 전체 기능

현재 이 워크스페이스에서 이미 연결된 기능은 아래와 같다.

- 음성 명령 해석
- 볼트 체결
- inspection_3d 기반 3D 검사
  - 볼트 체결 검사
  - 멀티탭 체결 검사
- **공구 전달 (`TOOL_FETCH`)** — 연결 완료, 아래 2장 참고
- **공구 정리 (`TOOL_CLEANUP`)** — 연결 완료, 아래 3장 참고

추가 예정 기능:

- 멀티탭 체결

## 현재 실행 흐름 요약

### 볼트 체결

1. 사용자가 `볼트 체결해줘`를 말한다.
2. `get_keyword_node.py`가 `BOLT_ASSEMBLE`로 해석한다.
3. `voice_command_dispatcher.py`가 `/robot_command` 액션으로 전달한다.
4. `robot_command_server.py`가 `bolt_assemble_task.py`를 호출한다.
5. 볼트 검출, 파지, 체결을 수행한다.

### 3D 검사

1. 사용자가 `볼트 체결 검사해줘` 또는 `멀티탭 체결 검사해줘`를 말한다.
2. `get_keyword_node.py`가 각각 `INSPECT_FASTEN` 또는 `CONNECTOR_INSPECT`로 해석한다.
3. `voice_command_dispatcher.py`가 `/robot_command` 액션으로 전달한다.
4. `robot_command_server.py`가 `pointcloud_inspector_task.py`를 호출한다.
5. `inspection_3d` 패키지의 `pipeline_node.py`, `comparison_node.py` 서비스로 스캔과 비교를 수행한다.

## 현재 코드 트리

```text
cobot2_ws/
├── robot_control/
│   ├── README.md
│   ├── INTEGRATION_GUIDE.md
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   │   └── voice_command_stack.launch.py
│   └── robot_control/
│       ├── robot_command_server.py
│       ├── voice_command_dispatcher.py
│       ├── bolt_assemble_task.py
│       ├── pointcloud_inspector_task.py
│       ├── task_config.py
│       └── onrobot.py
├── voice_processing/
│   ├── README.md
│   ├── package.xml
│   ├── setup.py
│   └── voice_processing/
│       ├── get_keyword_node.py
│       ├── MicController.py
│       ├── stt.py
│       └── wakeup_word.py
├── voice_interfaces/
│   ├── action/
│   │   └── RobotCommand.action
│   └── srv/
│       └── GetKeyword.srv
├── inspection_3d/
│   ├── README.md
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   │   └── pipeline_with_comparison.launch.py
│   ├── resource/
│   │   ├── good_bolt.pcd
│   │   └── good_multitap.pcd
│   └── inspection_3d/
│       ├── pipeline_node.py
│       ├── comparison_node.py
│       └── occupancy_compare.py
└── od_msg/
    └── srv/
        └── SrvPointCloudCompare.srv
```

## 새 기능이 들어갈 위치

- 멀티탭 체결 (예정)
  - `robot_control/robot_control/connector_auto_connect_task.py`
- 공구 전달 (완료)
  - `robot_control/robot_control/tool_handover_task.py`
- 공구 정리 (완료)
  - `robot_control/robot_control/tool_cleanup_task.py`
  - 두 공구 task가 공유하는 Trigger/상태토픽 유틸은
    `robot_control/robot_control/tool_sorter_status.py`

`robot_command_server.py`에 직접 구현하지 말고,
새 task 파일을 만든 뒤 라우팅만 연결하는 방식으로 붙이는 것을 기준으로 한다.

## 그리퍼 제어 경로가 두 갈래다 — 한 세션에서 섞지 말 것

RG2를 잡는 방식이 기능마다 다르다.

- 공구 정리/전달: **ROS 서비스** (`/onrobot/sendCommand`,
  `onrobot_rg_control` 드라이버 노드가 Modbus 소켓을 소유)
- 볼트 체결: **직접 Modbus TCP** (`robot_control/onrobot.py`의 `RG`,
  `192.168.1.1:502` = `task_config.py`의 `TOOLCHARGER_IP/PORT`)

둘 다 같은 물리 소켓을 향한다. 다만 잡는 시점이 다르다.

- `RG()`는 `bolt_assemble_task.py:495`에서 **첫 BOLT_ASSEMBLE 실행 때** 생성되어
  노드에 캐시되고, 그 뒤로 프로세스가 끝날 때까지 연결을 놓지 않는다.
  `robot_command_server`가 떠 있는 것만으로는 잡지 않는다.
- `onrobot_rg_control` 드라이버는 기동 시 잡아 종료할 때까지 붙잡는다.
  프로젝트 launch에는 없고 `m0609_rg2_bringup`을 쓸 때만 뜬다.

### 실행 규칙

기능별로 bringup을 갈아끼우고 **`robot_command_server`까지 같이 재시작**하면
충돌하지 않는다.

| 구성 | 결과 |
| --- | --- |
| `m0609_rg2_bringup`(드라이버 O) + 공구 정리/전달만 | OK. `RG()`가 생성되지 않는다 |
| dsr bringup(드라이버 X) + 볼트 체결만 | OK. `RG()`가 유일한 소유자 |
| 한 세션에서 볼트 -> 공구 | 실패. `RG()`가 소켓을 계속 쥐고 있다 |
| 한 세션에서 공구 -> 볼트 | 실패. 드라이버가 쥐고 있다 |

서버만 살려두고 bringup만 바꾸는 것으로는 부족하다. 한 번이라도 BOLT_ASSEMBLE을
돌렸으면 그 프로세스가 소켓을 놓지 않기 때문이다.

### 전제는 아직 미검증이다

위 내용은 "RG2의 Modbus TCP 서버가 동시 접속을 거부한다"는 가정 위에 있고,
실기로 확인하지 않았다. 동시 접속을 허용하면 이 제약 자체가 없어진다.
드라이버를 띄운 채 볼트 체결의 그리퍼를 한 번 여닫아보면 바로 판별된다.

### 한 세션에서 섞고 싶어지면

지금은 데모를 기능별로 끊어 가므로 문제가 되지 않지만, 한 음성 세션에서
`볼트 체결해줘` 다음에 `망치 가져와줘`를 받으려면 소유자를 하나로 합쳐야 한다.

1. `bolt_assemble_task.py`를 ROS 서비스 경로로 옮긴다.
   `gripper.open_gripper()` / `close_gripper()` 3곳(380, 386, 414행)만
   바꾸면 되므로 변경 폭이 작고, 드라이버 하나로 통일되는 방향이다.
2. 공구 정리/전달을 직접 Modbus로 바꾼다.
   `tool_sorter_core/tool_sorter_core/motion.py`의 `OnRobotGripper`를 고쳐야
   한다.

## 현재 아키텍처

현재 음성 명령 실행 흐름은 아래와 같다.

1. 사용자가 음성 명령을 말한다.
2. `get_keyword_node.py`가 음성을 `intent`, `tools`, `targets`로 변환한다.
3. `voice_command_dispatcher.py`가 결과를 `/robot_command` 액션으로 전달한다.
4. `robot_command_server.py`가 `intent`에 따라 적절한 task를 호출한다.
5. 각 task 파일이 실제 로봇 동작을 수행한다.

즉, 새 기능도 반드시 같은 구조에 맞춰 붙이는 것을 권장한다.

현재 실행은 보통 아래처럼 한다.

1. robot bringup
2. realsense
3. `ros2 run voice_processing get_keyword`
4. 필요 시 `ros2 launch inspection_3d pipeline_with_comparison.launch.py`
5. `ros2 launch robot_control voice_command_stack.launch.py`

이 launch는 아래만 함께 실행한다.

- `robot_control/robot_command_server`
- `robot_control/voice_command_dispatcher`

## 절대 권장하지 않는 방식

아래 방식은 현재 구조를 망가뜨릴 가능성이 높다.

- `voice_command_dispatcher.py` 안에 로봇 동작 로직을 직접 넣는 것
- `get_keyword_node.py` 안에 작업 실행 코드를 넣는 것
- `robot_command_server.py` 안에 perception, motion, force 로직을 길게 직접 작성하는 것
- inspection_3d 서비스 호출을 여기저기 흩뿌리는 것

원칙:

- `get_keyword_node.py`는 해석만
- `voice_command_dispatcher.py`는 전달만
- `robot_command_server.py`는 라우팅만
- 실제 작업은 별도 `*_task.py` 파일에서 수행

## 1. 멀티탭 체결 기능 통합 방법

### 권장 새 파일

추천 파일명:

- `robot_control/robot_control/connector_auto_connect_task.py`

이 파일이 실제 멀티탭 체결 시퀀스를 담당하게 한다.

### get_keyword_node 쪽

현재 프롬프트에는 이미 아래 intent가 있다.

- `CONNECTOR_AUTO_CONNECT`

즉, `"멀티탭 체결해줘"` 같은 명령은 이미 이 intent로 해석될 수 있게 설계되어 있다.

다른 팀원이 확인할 것:

- `"멀티탭 체결해줘"`가 안정적으로
  `CONNECTOR_AUTO_CONNECT / multitap / multitap_position`
  형태로 나오도록 프롬프트 유지 또는 보강

### robot_command_server 쪽

현재 `robot_command_server.py`는 아래 구조로 동작한다.

- `intent -> handler` 라우팅
- handler는 task 함수 호출만 수행

여기에 아래 handler를 추가하면 된다.

- `execute_connector_auto_connect()`

그리고 `resolve_handler()` 안에 아래 분기를 넣으면 된다.

- `CONNECTOR_AUTO_CONNECT` -> `execute_connector_auto_connect`

### 새 task 파일에서 맡아야 할 역할

`connector_auto_connect_task.py`는 아래를 담당해야 한다.

1. 멀티탭 위치 또는 체결 대상 위치 인식
2. 접근 포즈 이동
3. 정렬
4. 삽입/체결
5. 필요 시 힘제어 기반 접촉 확인
6. 성공/실패 반환

### task_config에 추가해야 할 가능성이 큰 값

예상 추가 항목:

- 멀티탭 체결 시작 포즈
- 접근 포즈
- 정렬 포즈
- 삽입 깊이
- 힘제어 threshold
- 체결 완료 확인 기준

즉, 코드에 박지 말고 `task_config.py`로 모으는 것이 좋다.

## 2. 공구 전달 (`TOOL_FETCH`) — 연결 완료

### 어떻게 붙였는가

가이드가 처음 예상한 것과 한 가지가 다르다. 공구 인식·파지·전달은
`tool_handover_task.py`가 **직접 하지 않는다.** 그 일은 이미 별도 패키지
`tool_sorter_handover`가 검증된 상태로 수행하고 있어서, task 파일은 그
노드를 구동하고 완료를 판정하는 얇은 클라이언트가 되었다.

원칙은 그대로다 — `robot_command_server.py`는 라우팅만 하고, 실제 작업 흐름은
`*_task.py`에 있다.

```text
HMI(app_v5) --RobotCommand 액션--> robot_command_server
                                      └ execute_tool_fetch()
                                          └ tool_handover_task.run_tool_handover()
                                              ├ Trigger  /tool_sorter/handover/start
                                              ├ String   /tool_sorter/handover/request
                                              ├ (구독)   /tool_sorter/handover/status
                                              └ Trigger  /tool_sorter/handover/stop
```

### 실행 순서

`tool_sorter_handover`는 세션 개념을 쓴다. **순서를 지켜야 한다.**

1. `start`로 세션을 연다.
2. 관측 자세 이동이 끝나 `WAITING_REQUEST`가 되는 것을 확인한다.
3. 그때 공구 키워드를 `request` 토픽에 발행한다.
   세션이 열리기 전에 발행된 요청은 조용히 버려진다.
4. 요청을 실제로 집어들어 `WAITING_REQUEST`를 벗어나는지 확인한다.
5. `COMPLETE`면 전달 성공. `WAITING_REQUEST`로 되돌아왔으면 **미검출**이고,
   그때는 세션이 열린 채 남으므로 `stop`으로 닫아준다.

4번을 건너뛰면 "아직 시작도 안 한 `WAITING_REQUEST`"를 곧바로 종료로 오인한다.

### 공구 이름

HMI는 인식된 낱말을 그대로 실어 보내므로 `"망치"`일 수도 `"hammer"`일 수도
있다. 정규화는 `tool_sorter_handover/tool_sorter_handover/tool_request.py`의
`normalize_tool_request()`를 그대로 쓴다. 별칭 표를 robot_control에 다시
만들지 않는다 — 두 벌이 되면 반드시 어긋난다.

전달 가능한 공구는 5종이다: 망치, 드라이버, 렌치, 몽키렌치, 바이스.

### task_config에 추가된 값

포즈는 없다. 관측 자세, grasp offset, 전달 판정 힘은 전부
`tool_sorter_handover/config/handover.yaml`이 갖고 있다. robot_control에는
서비스/토픽 이름과 타임아웃만 있다 (`TOOL_HANDOVER_*`).

## 3. 공구 정리 (`TOOL_CLEANUP`) — 연결 완료

작업장의 공구를 클래스별 지정 위치로 되돌린다. 구조는 전달과 같고, 상대는
`tool_sorter_cleanup`다.

```text
robot_command_server
  └ execute_tool_cleanup()
      └ tool_cleanup_task.run_tool_cleanup()
          ├ Trigger  /integration/tool_sorter/organize
          └ (구독)   /integration/tool_sorter/status
```

요청은 한 번이고, 5종을 다 처리할 때까지 이어서 돈다. 작업장에 없는 공구는
건너뛰고 완료 메시지에 이름으로 남는다.

### 주의: 상태 토픽은 transient-local이다

시작 요청을 넣자마자 완료 상태를 기다리면 안 된다. 직전 작업의 `COMPLETE`가
아직 latch되어 있어서 그것을 이번 작업의 완료로 오인한다. 그래서 두 task 모두
**먼저 "실제로 바빠지는 것"을 확인한 뒤** 완료를 기다린다.

### 실측 좌표가 없으면 움직이지 않는다

`autonomous_tool_sorter.yaml`의 `place_pose_*` 5개가 비어 있으면 요청이
`BLOCKED`로 거절되고, 그 사유가 액션 result의 message로 그대로 올라온다.
현장에서 이 좌표를 먼저 실측해 넣어야 한다.

## robot_command_server에 기능을 붙일 때 규칙

반드시 아래 패턴을 유지하는 것을 권장한다.

1. `SUPPORTED_INTENTS`에 intent 추가
2. `resolve_handler()`에 intent 분기 추가
3. `execute_*()` 메서드 추가
4. 메서드 안에서는 task 함수만 호출
5. feedback/status/message만 여기서 처리

즉, `robot_command_server.py`는 라우터여야지 작업 구현 파일이 되면 안 된다.

## 현재 intent 라우팅

| intent | task 파일 | 실제 수행 |
| --- | --- | --- |
| `BOLT_ASSEMBLE` | `bolt_assemble_task.py` | robot_control 자체 |
| `INSPECT_FASTEN` | `pointcloud_inspector_task.py` (`bolt`) | `inspection_3d` 패키지 |
| `CONNECTOR_INSPECT` | `pointcloud_inspector_task.py` (`multitap`) | `inspection_3d` 패키지 |
| `TOOL_FETCH` | `tool_handover_task.py` | `tool_sorter_handover` |
| `TOOL_CLEANUP` | `tool_cleanup_task.py` | `tool_sorter_cleanup` |
| `CONNECTOR_AUTO_CONNECT` | `connector_auto_connect_task.py` (예정) | - |

## 실행 방법 (공구 기능 포함)

1. robot bringup
2. realsense
3. RG2 그리퍼 드라이버 — 위 "그리퍼 소유권 충돌" 항목을 먼저 읽을 것
4. 필요 시 `ros2 launch inspection_3d pipeline_with_comparison.launch.py`
5. `ros2 launch robot_control tool_sorter_stack.launch.py`
6. `ros2 launch robot_control voice_command_stack.launch.py`
7. `ros2 run operator_ui app_node`

5번이 인식/TF 노드와 두 task manager를 띄운다. 각 패키지의 개별 launch
(`autonomous_tool_sorter.launch.py`, `handover.launch.py`)를 대신 쓰면
`perception_node`와 TF broadcaster가 두 벌씩 떠서 충돌하므로 쓰지 않는다.

액션만 따로 확인하려면 음성 없이 직접 넣을 수 있다.

```bash
ros2 action send_goal /dsr01/robot_command \
  voice_interfaces/action/RobotCommand \
  '{intent: "TOOL_FETCH", tools: ["망치"], targets: []}' --feedback

ros2 action send_goal /dsr01/robot_command \
  voice_interfaces/action/RobotCommand \
  '{intent: "TOOL_CLEANUP", tools: [], targets: []}' --feedback
```

## inspection_3d 기능과의 관계

주의:

- 멀티탭 체결은 `pointcloud_inspector_task.py`가 아니라 별도 체결 task로 가야 한다.
- 멀티탭 체결 검사만 inspection_3d inspection을 사용한다.
- 공구 전달은 inspection_3d가 아니라 perception/motion 중심 흐름이 맞다.

즉:

- 체결 기능 = task 파일
- 검사 기능 = inspection_3d task

## 팀원에게 전달할 최소 규칙

다른 팀원이 기능을 붙일 때 아래 4개만 지키면 현재 구조와 충돌이 적다.

1. 새 기능은 `*_task.py` 파일로 추가한다.
2. `robot_command_server.py`에서는 라우팅만 한다.
3. 포즈/토픽/threshold는 `task_config.py`로 뺀다.
4. 음성 해석과 로봇 실행 로직을 섞지 않는다.

## 추천 통합 순서

### 멀티탭 체결

1. `connector_auto_connect_task.py` 작성
2. `task_config.py`에 체결용 포즈/파라미터 추가
3. `robot_command_server.py`에 `CONNECTOR_AUTO_CONNECT` handler 연결
4. `get_keyword_node.py` 프롬프트 확인
5. 단독 실행 테스트 후 음성 연결 테스트

### 공구 전달

1. `tool_handover_task.py` 작성
2. `task_config.py`에 handover 관련 포즈 추가
3. `robot_command_server.py`에 `TOOL_FETCH` handler 연결
4. `get_keyword_node.py`에서 공구 이름 매핑 확인
5. 단독 실행 테스트 후 음성 연결 테스트

## 마지막 정리

현재 구조에서 새 기능을 붙이는 핵심은 다음 한 줄로 정리된다.

새 기능은 `robot_command_server.py`에 직접 구현하지 말고,
새 `*_task.py` 파일을 만든 뒤 `robot_command_server.py`에서는 그 task를 호출만 하게 붙인다.
