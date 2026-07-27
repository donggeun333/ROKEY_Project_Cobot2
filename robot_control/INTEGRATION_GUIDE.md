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

현재 이 워크스페이스에서 이미 연결된 기능은 아래 3개다.

- 음성 명령 해석
- 볼트 체결
- pointcloud 기반 3D 검사
  - 볼트 체결 검사
  - 멀티탭 체결 검사

추가 예정 기능:

- 멀티탭 체결
- 공구 전달

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
5. `pointcloud` 패키지의 `pipeline_node.py`, `comparison_node.py` 서비스로 스캔과 비교를 수행한다.

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
├── pointcloud/
│   ├── README.md
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   │   └── pipeline_with_comparison.launch.py
│   ├── resource/
│   │   ├── good_bolt.pcd
│   │   └── good_multitap.pcd
│   └── pointcloud/
│       ├── pipeline_node.py
│       ├── comparison_node.py
│       └── occupancy_compare.py
└── od_msg/
    └── srv/
        └── SrvPointCloudCompare.srv
```

## 새 기능이 들어갈 위치

- 멀티탭 체결
  - `robot_control/robot_control/connector_auto_connect_task.py`
- 공구 전달
  - `robot_control/robot_control/tool_handover_task.py`

둘 다 `robot_command_server.py`에 직접 구현하지 말고,
새 task 파일을 만든 뒤 라우팅만 연결하는 방식으로 붙이는 것을 기준으로 한다.

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
4. 필요 시 `ros2 launch pointcloud pipeline_with_comparison.launch.py`
5. `ros2 launch robot_control voice_command_stack.launch.py`

이 launch는 아래만 함께 실행한다.

- `robot_control/robot_command_server`
- `robot_control/voice_command_dispatcher`

## 절대 권장하지 않는 방식

아래 방식은 현재 구조를 망가뜨릴 가능성이 높다.

- `voice_command_dispatcher.py` 안에 로봇 동작 로직을 직접 넣는 것
- `get_keyword_node.py` 안에 작업 실행 코드를 넣는 것
- `robot_command_server.py` 안에 perception, motion, force 로직을 길게 직접 작성하는 것
- pointcloud 서비스 호출을 여기저기 흩뿌리는 것

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

## 2. 공구 전달 기능 통합 방법

### 권장 새 파일

추천 파일명:

- `robot_control/robot_control/tool_handover_task.py`

이 파일이 공구 인식, 집기, 전달까지 담당하게 한다.

### get_keyword_node 쪽

현재 프롬프트에는 아래 intent가 이미 있다.

- `TOOL_FETCH`

즉, `"드라이버 가져다줘"` 같은 명령은 `TOOL_FETCH`로 해석되게 맞추면 된다.

다른 팀원이 확인할 것:

- tool 이름이 `tools` 배열로 잘 들어가는지
- 목적지가 없으면 기본적으로 `tool_box` 또는 내부 기본값으로 처리할지

### robot_command_server 쪽

아래 handler를 추가하면 된다.

- `execute_tool_fetch()`

그리고 `resolve_handler()`에 아래 분기를 넣는다.

- `TOOL_FETCH` -> `execute_tool_fetch`

### 새 task 파일에서 맡아야 할 역할

`tool_handover_task.py`는 아래를 담당해야 한다.

1. 공구 박스 또는 작업대 스캔
2. 목표 공구 인식
3. 집기 포즈 계산
4. 공구 파지
5. handover zone 이동
6. 작업자에게 전달
7. 성공/실패 반환

### task_config에 추가해야 할 가능성이 큰 값

예상 추가 항목:

- tool box scan pose
- handover zone pose
- 공구별 grasp offset
- 안전 리프트 높이
- 전달 높이

## robot_command_server에 기능을 붙일 때 규칙

반드시 아래 패턴을 유지하는 것을 권장한다.

1. `SUPPORTED_INTENTS`에 intent 추가
2. `resolve_handler()`에 intent 분기 추가
3. `execute_*()` 메서드 추가
4. 메서드 안에서는 task 함수만 호출
5. feedback/status/message만 여기서 처리

즉, `robot_command_server.py`는 라우터여야지 작업 구현 파일이 되면 안 된다.

## 현재 구조에 맞는 통합 예시

현재:

- `BOLT_ASSEMBLE` -> `bolt_assemble_task.py`
- `INSPECT_FASTEN` -> `pointcloud_inspector_task.py` with `bolt`
- `CONNECTOR_INSPECT` -> `pointcloud_inspector_task.py` with `multitap`

추가 후 권장 구조:

- `BOLT_ASSEMBLE` -> `bolt_assemble_task.py`
- `INSPECT_FASTEN` -> `pointcloud_inspector_task.py`
- `CONNECTOR_INSPECT` -> `pointcloud_inspector_task.py`
- `CONNECTOR_AUTO_CONNECT` -> `connector_auto_connect_task.py`
- `TOOL_FETCH` -> `tool_handover_task.py`

## pointcloud 기능과의 관계

주의:

- 멀티탭 체결은 `pointcloud_inspector_task.py`가 아니라 별도 체결 task로 가야 한다.
- 멀티탭 체결 검사만 pointcloud inspection을 사용한다.
- 공구 전달은 pointcloud가 아니라 perception/motion 중심 흐름이 맞다.

즉:

- 체결 기능 = task 파일
- 검사 기능 = pointcloud task

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
