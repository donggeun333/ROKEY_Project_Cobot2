# 조립/검사 HMI (Assembly Inspection HMI)

두산 M0609 협동로봇 + ROS2 + Flask 기반 웹 HMI입니다.
현재 이 패키지는 다음 기능을 한 프로세스에서 함께 담당합니다.

- 웹 대시보드/이력/3D 뷰어
- wake word + STT + TTS 기반 음성 인터페이스
- `robot_command_server` 액션 클라이언트
- 실시간 카메라 스트림 표시
- inspection_3d 검사 결과 조회

> **[통합] `voice_processing` 패키지 병합 안내**
> 기존에 별도 ROS2 패키지였던 `voice_processing`(wake word 감지 + STT + LLM 키워드 추출 + TTS)이
> 이 `operator_ui` 패키지 안으로 병합되었습니다. 현재는 `app_node`(`operator_ui/app_v5.py`) 하나가
> **Flask HMI + ROS2 브리지 + 음성 인식/안내(`VoiceEngine`)** 를 한 프로세스에서 함께 실행합니다.
> - 음성 처리 모듈: `operator_ui/operator_ui/voice/` (`MicController.py`, `wakeup_word.py`, `stt.py`, `tts.py`, `phrases.py`)
> - 통합 로직: `operator_ui/operator_ui/app_v5.py` 안의 `VoiceEngine` 클래스 + `AssemblyHmiBridge._on_voice_result()`
> - 기존 개별 노드(Azure STT, `get_keyword_v2` 등)는 `operator_ui/operator_ui/legacy/` 에 참고/수동 디버깅용으로만 남겨두었으며,
>   기본 실행 흐름(`app_node`)에는 포함되지 않습니다.
> - 자세한 동작 방식은 아래 "6. [통합] 음성 인식 파이프라인" 절 참고.

---

## 1. 시스템 구성

```
[웹 브라우저 UI]
   │  HTTP (fetch / REST API)
   ▼
[Flask 서버] ── app_v5.py
   │
   ├── AssemblyHmiBridge (ROS2 Node, executor로 spin)
   │     ├─ 토픽 구독/발행 (통신 중계)
   │     └─ [통합] VoiceEngine (백그라운드 스레드)
   │            └─ wake word → STT → LLM 키워드추출 → TTS
   │
   └── RobotController (ROS2 Node, executor에 넣지 않음)
         └─ DSR_ROBOT2 라이브러리로 실제 로봇 구동 전담
```

두 개의 ROS2 노드로 역할을 분리한 이유:
`DSR_ROBOT2`의 `movej()`는 내부적으로 `spin_until_future_complete()`를 직접 호출합니다.
이미 `MultiThreadedExecutor`에서 spin 중인 노드로 이 호출을 하면 충돌이 나기 때문에,
**로봇 구동 전용 노드(RobotController.node)는 절대 executor에 추가하지 않고 자체적으로 spin** 합니다.
반면 카메라/음성/공정상태 등 일반 토픽 통신은 `AssemblyHmiBridge`가 담당하며 이 노드만 executor로 돌립니다.

---

## 2. 주고받는 ROS2 토픽 상세

### 2-1. 구독(Subscribe) — 로봇/센서 → HMI

| 토픽명 | 타입 | 발행 주체(가정) | 역할 |
|---|---|---|---|
| `/dsr01/joint_states` | `sensor_msgs/JointState` | 두산 로봇 드라이버 | 로봇의 현재 6축 관절 각도를 실시간으로 받아온다. **주의**: 드라이버는 라디안(rad) 단위로 발행하므로, HMI 내부에서 `math.degrees()`로 도(°) 단위로 변환해 저장한다. 또한 `position` 배열의 순서가 항상 J1~J6 순서라는 보장이 없어(드라이버에 따라 사전순 정렬 등으로 뒤바뀔 수 있음), `msg.name`(`joint_1`~`joint_6` 등)을 기준으로 재정렬한 뒤 사용한다. 이 값은 수동 제어 페이지의 슬라이더/입력창에 1초 주기로 실시간 반영된다. |
| `/robot/process_state` | `std_msgs/String` (JSON) | 검사/공정 제어 노드 | 현재 공정 단계(`stage`)와 불량 여부(`is_ng`)를 JSON 문자열로 받는다. 예: `{"stage": "STAGE 3: 볼트 체결 검사 중", "is_ng": true}`. `is_ng`가 참이거나 `stage` 문자열에 "NG"가 포함되면 대시보드에 3D 뷰어 팝업이 자동으로 뜨도록 플래그(`has_ng_event`)를 세운다. |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense 등 컬러 카메라 드라이버 | 실시간 비전 영상을 받아 OpenCV 배열로 변환 후 메모리에 저장한다. 이 프레임은 `/video_feed` 엔드포인트를 통해 MJPEG 스트림으로 웹 UI에 전달되어, 대시보드의 "실시간 비전 공정 가이드" 화면에 표시된다. |
| `/ui/stt_result` | `std_msgs/String` | **[통합]** `AssemblyHmiBridge` 내장 `VoiceEngine` (자기 자신이 재발행) 또는 외부 STT 노드 | 음성 인식 결과 텍스트를 받는다. 이 토픽 수신을 트리거로 Tact Time(10초 사이클) 측정이 시작된다 — 즉 "음성 명령이 들어와야 공정 사이클이 시작"되는 구조다. 통합 이후에는 내장 `VoiceEngine`이 인식한 문장을 직접 처리(`_on_voice_result`)하면서 동시에 이 토픽으로도 재발행해 기존 외부 구독자와의 호환성을 유지한다. |

### 2-2. 발행(Publish) — HMI → 로봇/타 노드

| 토픽명 | 타입 | 구독 주체(가정) | 역할 |
|---|---|---|---|
| `/ui/admin_control` | `std_msgs/String` | 검사/공정 제어 노드 등 | 웹 UI에서 보낸 모든 제어 명령(`ESTOP`, `HOME`, `MOVEJ:[...]`, `RETRY` 등)을 원문 그대로 재발행한다. 검사 노드가 이 토픽을 구독해 `RETRY`(재검사) 같은 매크로 명령에 반응할 수 있다. 실제 로봇 구동(MOVEJ/HOME)은 이 토픽이 아니라 `RobotController`가 직접 `DSR_ROBOT2`를 호출해 수행하며, 이 토픽 발행은 "다른 노드에게 상황을 알리는" 부가적 역할이다. |
| `/ui/emergency_stop` | `std_msgs/Bool` | 안전 감시 노드 등 | 비상정지(ESTOP) 버튼을 눌렀을 때 `True`로 발행된다. 실제 로봇 정지는 아래 `move_stop` 서비스가 담당하고, 이 토픽은 다른 안전 관련 노드(예: 조명/부저 제어, 컨베이어 정지 등)가 함께 반응하도록 알리는 용도다. |
| `/voice/pick_place_command` | `std_msgs/String` (JSON) | **[통합]** pick & place 로봇 제어 노드 등 | 내장 `VoiceEngine`이 wake word 이후 인식한 문장에서 LLM으로 추출한 도구(`tools`)/목적지(`targets`)를 `{"raw_text", "tools", "targets"}` JSON으로 발행한다. "비상정지"/"홈"/"시작" 같은 매크로 명령이 아닐 때만 발행된다. |

### 2-3. 서비스(Service) 호출

| 서비스명 | 타입 | 역할 |
|---|---|---|
| `/dsr01/motion/move_stop` | `dsr_msgs2/srv/MoveStop` | ESTOP 시 두산 드라이버에 직접 호출하는 서비스. `stop_mode=1`(Quick Stop)로 호출하여 진행 중인 모션을 즉시 감속 정지시킨다. `dsr_msgs2` 패키지가 없는 환경에서는 자동으로 비활성화되고 로그에 안내 문구만 남는다. |
| `DSR_ROBOT2.movej()` (내부적으로 `/dsr01/motion/move_joint` 서비스 호출) | - | 관절 이동(MOVEJ) 및 홈 이동(HOME) 명령을 실제로 로봇에 전달한다. `RobotController`가 큐를 통해 명령을 하나씩 순서대로 실행하며, 서버측 소프트 리밋(J1~J6 별 허용 각도 범위)을 통과한 값만 실행된다. |

---

## 3. Flask REST API

| 엔드포인트 | 메서드 | 역할 |
|---|---|---|
| `/admin/dashboard`, `/admin/viewer3d`, `/admin/control`, `/admin/history` | GET | 각 HMI 페이지(공정 모니터링, 3D 품질분석, 수동 제어, 이력/DB) 렌더링 |
| `/video_feed` | GET | 카메라 실시간 영상을 MJPEG 스트림으로 제공 |
| `/api/robot/status` | GET | 관절 각도, 공정 단계, STT 결과, 로그, Tact Time, 양품/불량 수량 등 실시간 상태를 JSON으로 반환 (프론트가 1초마다 폴링) |
| `/api/robot/command` | POST | UI에서 보낸 제어 명령(`{"command": "MOVEJ:[...]"}` 등)을 받아 `RobotController`/토픽 발행으로 위임 |
| `/api/history/data` | GET | 최근 검사 이력 100건 조회 |
| `/api/history/download_csv` | GET | 전체 검사 이력을 CSV로 다운로드 |

---

## 4. 안전 관련 설계 포인트

- **관절 소프트 리밋**: `JOINT_LIMITS`에 M0609 실제 가동 범위(J2 ±95°, J3/J5 ±135° 등)를 정의해 두고, UI 입력값과 서버 양쪽에서 이중으로 클램프한다.
- **비상정지 이원화**: ESTOP 시 ①대기 중인 이동 명령을 큐에서 전부 폐기하고 ②`move_stop` 서비스로 실제 모션을 감속 정지시킨다. 둘 중 하나만 있으면 "정지 명령은 갔는데 이미 큐에 있던 다음 이동이 이어서 실행되는" 문제가 생길 수 있어 함께 처리한다.
- **명령 직렬 실행**: 모든 로봇 이동 명령은 큐 + 단일 워커 스레드로 순서대로만 실행되어, 여러 요청이 동시에 로봇 제어를 시도해 충돌하는 상황을 방지한다.

---

## 5. 실행 방법

```bash
# 실사용 표준 bringup: 로봇 + RG2 + RealSense
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=<ROBOT_IP>

# 3D 검사 사용 시
ros2 launch inspection_3d pipeline_with_comparison.launch.py

# 공구 전달/정리 사용 시
ros2 launch robot_control tool_sorter_stack.launch.py

# 액션 서버
ros2 run robot_control robot_command_server

# HMI + 음성 인식(통합)
ros2 run operator_ui app_node
```

---

## 6. [통합] 음성 인식 파이프라인 (voice_processing 병합)

`AssemblyHmiBridge` 노드가 생성될 때 `VoiceEngine` 인스턴스를 함께 생성하고
백그라운드 스레드로 구동한다(`operator_ui/operator_ui/app_v5.py`). 별도로 실행해야 하는 음성
노드는 없으며, `app_node` 하나만 실행하면 된다.

**동작 흐름**

1. **wake word 상시 감지**: `operator_ui/operator_ui/voice/wakeup_word.py`의 `WakeupWord`가 마이크 입력을 계속 읽으며
   `hello_rokey_8332_32.tflite` 모델로 "헬로우 로키"를 감지한다.
2. **감지 시 TTS 안내**: `operator_ui/operator_ui/voice/tts.py`(OpenAI TTS)로 "어떤 작업을 하시겠습니까?"를 재생한다.
3. **STT**: `operator_ui/operator_ui/voice/stt.py`(OpenAI Whisper)로 5초간 녹음 후 텍스트로 변환한다.
4. **명령 라우팅** (`AssemblyHmiBridge._on_voice_result`):
   - `"비상정지"` / `"정지"` → `ESTOP` 매크로 실행 (대기 명령 폐기 + `move_stop` 서비스 호출)
   - `"홈"` / `"홈으로"` → `HOME` 매크로 실행 (홈 자세로 이동)
   - 텍스트에 `"시작"` 포함 → 기존 `/ui/stt_result` 콜백과 동일하게 Tact Time(10초 사이클) 측정 시작
   - 그 외 → LLM(`gpt-4o`, langchain)으로 도구/목적지 키워드를 추출해 `/voice/pick_place_command` 로 발행
5. **결과 TTS 안내**: 도구가 인식되면 "OO를 OO로 옮기겠습니다", 실패하면 "다시 한번 말씀해주세요" 재생.

**필요 환경변수** (`resource/.env`)

```
OPENAI_API_KEY=sk-...       # STT(Whisper) · TTS · 키워드 추출(gpt-4o)에 사용
```

**마이크 장치 설정**: `VoiceEngine.__init__`의 `MicConfig(device_index=10, ...)` 값을 실제 마이크 장치 번호에 맞게 수정해야 한다.
장치 번호 확인:
```python
import pyaudio
p = pyaudio.PyAudio()
[(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count())]
```

**레거시 참고 코드**: 통합 전 개별 노드였던 Azure STT(`stt_node.py`), 이전 버전 `get_keyword`/`get_keyword_v2`는
`operator_ui/operator_ui/legacy/`에 그대로 남아있다. 필요 시 `ros2 run operator_ui legacy_stt`, `ros2 run operator_ui legacy_get_keyword`로
수동 실행/디버깅할 수 있지만, 기본 실행 흐름(`app_node`)에는 관여하지 않는다.
