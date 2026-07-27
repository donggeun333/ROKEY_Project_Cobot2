# Voice Processing Package README

이 패키지는 음성 명령을 `intent`, `tools`, `targets`로 변환하는 입력 계층이다.

## 현재 역할

- wake word 감지
- STT
- LLM intent 해석
- `/get_keyword` 서비스 응답 반환

즉, 로봇을 직접 움직이지는 않고 음성을 구조화된 명령으로 바꾼다.

## 현재 실행 흐름

1. `get_keyword_node.py`가 `/get_keyword` 서비스를 제공한다.
2. `robot_control/voice_command_dispatcher.py`가 서비스를 호출한다.
3. wake word 감지, STT, intent 해석을 수행한다.
4. `voice_interfaces/srv/GetKeyword` 응답으로 `intent`, `tools`, `targets`, `raw_text`를 반환한다.

## 실행 명령

```bash
ros2 run voice_processing get_keyword
```

`setup.py` 기준으로 `get_keyword` 엔트리포인트는
`voice_processing/get_keyword_node.py`를 실행한다.

## 핵심 파일

- `voice_processing/get_keyword_node.py`
  - 현재 메인 음성 처리 노드
- `voice_processing/stt.py`
  - STT 래퍼
- `voice_processing/wakeup_word.py`
  - wake word 감지
- `voice_processing/MicController.py`
  - 마이크/녹음 제어

## 주요 intent

- `BOLT_ASSEMBLE`
- `INSPECT_FASTEN`
- `CONNECTOR_INSPECT`
- `CONNECTOR_AUTO_CONNECT`
- `TOOL_FETCH`
- `TOOL_CLEANUP`
- `CABLE_FETCH_HANDOVER`
- `CLARIFY_NEEDED`

## 관련 패키지

- `voice_interfaces`
  - `GetKeyword.srv`
- `robot_control`
  - `voice_command_dispatcher.py`
  - `robot_command_server.py`

## git에 같이 올릴 파일

- `voice_processing/voice_processing/get_keyword_node.py`
- `voice_processing/voice_processing/stt.py`
- `voice_processing/voice_processing/wakeup_word.py`
- `voice_processing/voice_processing/MicController.py`
- `voice_processing/setup.py`
- `voice_processing/package.xml`
- `voice_processing/README.md`

보통 올리지 않는 파일:

- `voice_processing/resource/.env`
- `__pycache__/`
- `build/`
- `install/`
- `log/`
