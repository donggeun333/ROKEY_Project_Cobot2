# Robot Control Package README

이 패키지는 HMI/음성 명령을 실제 로봇 작업으로 연결하는 상위 실행 패키지다.

현재 기준으로 다음 두 작업을 담당한다.

- 볼트 체결
- 플러그 삽입(콘센트/멀티탭 체결)
- inspection_3d 기반 3D 검사
  - 볼트 체결 검사
  - 멀티탭 체결 검사
- 공구 가져오기
- 공구 정리

## 현재 구조

실행 흐름은 아래와 같다.

1. `operator_ui/app_v5.py`
   - wake word/STT/TTS/HMI 웹 UI를 함께 실행
   - 음성 문장을 intent로 분류하고 `/dsr01/robot_command` 액션 목표를 보냄
2. `robot_control/robot_command_server.py`
   - intent별 task 실행
3. task 파일
   - `bolt_assemble_task.py`
   - `outlet_assembly.plug_insert_task`
   - `pointcloud_inspector_task.py`
   - `tool_handover_task.py`
   - `tool_cleanup_task.py`

레거시 음성 경로를 유지해야 할 때는 `voice_processing/get_keyword_node.py` +
`robot_control/voice_command_dispatcher.py` 조합도 쓸 수 있다.

현재는 `voice_command_stack.launch.py`로 아래를 함께 실행할 수 있지만,
실사용 최소 구성은 `robot_command_server` 단독 실행이다.

- `robot_control/robot_command_server`
- `robot_control/voice_command_dispatcher`

## 지원 명령

- `볼트 체결해줘`
  - `BOLT_ASSEMBLE`
  - `bolt_assemble_task.py` 실행
- `콘센트 체결해줘` / `멀티탭 체결해줘`
  - `OUTLET_ASSEMBLE`
  - `outlet_assembly.plug_insert_task` 실행
- `볼트 체결 검사해줘`
  - `INSPECT_FASTEN`
  - `pointcloud_inspector_task.py`를 `object_type=bolt`로 실행
- `멀티탭 체결 검사해줘`
  - `CONNECTOR_INSPECT`
  - `pointcloud_inspector_task.py`를 `object_type=multitap`으로 실행
- `망치 가져와줘` (망치/드라이버/렌치/몽키렌치/바이스)
  - `TOOL_FETCH`
  - `tool_handover_task.py`가 `tool_sorter_handover`를 구동
- `공구 정리해줘`
  - `TOOL_CLEANUP`
  - `tool_cleanup_task.py`가 `tool_sorter_cleanup`를 구동

## 핵심 파일 역할

- `robot_control/robot_command_server.py`
  - `/dsr01/robot_command` 액션 서버
  - intent를 받아 실제 task 함수 호출
- `robot_control/voice_command_dispatcher.py`
  - 레거시 음성 경로 전용 중계기
  - `/get_keyword` 서비스 클라이언트
  - `/robot_command` 액션 클라이언트
- `robot_control/bolt_assemble_task.py`
  - 볼트 검출, 파지, 체결 task
  - YOLO + depth + TF + movej/movel 사용
- `outlet_assembly/outlet_assembly/plug_insert_task.py`
  - 콘센트/멀티탭 체결 task
  - 멀티탭 구멍 인식, 플러그 파지, 삽입 시퀀스 수행
- `robot_control/pointcloud_inspector_task.py`
  - inspection_3d 스캔/비교 task
  - inspection_3d 패키지의 reset/capture/finalize/compare 서비스 호출
- `robot_control/tool_handover_task.py`
  - 공구 전달 task. `tool_sorter_handover`의 start/request/stop을 순서대로
    구동하고 상태 토픽으로 완료를 판정한다
- `robot_control/tool_cleanup_task.py`
  - 공구 정리 task. `tool_sorter_cleanup`의 organize를 구동한다
- `robot_control/tool_sorter_status.py`
  - 위 두 task가 공유하는 Trigger 호출 / 상태 토픽 대기 유틸
- `robot_control/task_config.py`
  - 볼트 체결 포즈, 스캔 포즈, 토픽, 타임아웃, 서비스명 같은 공통 설정 모음

## 참고/보조 파일 역할

- `robot_control/onrobot.py`
  - 예전 직접 Modbus RG2 제어기
  - 현재 표준 실행 경로는 `gripper_service.py`를 통해 `/onrobot/sendCommand`를 사용

## task_config.py에서 관리하는 값

이 파일에서 주로 관리한다.

- robot id / model
- 그리퍼 및 TCP 설정
- 볼트 체결용 카메라 토픽
- 볼트 체결용 포즈
- inspection_3d 검사용 스캔 포즈
- inspection_3d 서비스 이름
- 타임아웃, 속도, settle time

즉 실기 조정이 필요한 값은 이 파일에서 먼저 확인하면 된다.

## 기본 실행 구성

현재 권장 실행:

1. `ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=<ROBOT_IP>`
2. 필요 시 `ros2 launch inspection_3d pipeline_with_comparison.launch.py`
3. 공구 기능 사용 시 `ros2 launch robot_control tool_sorter_stack.launch.py`
4. `ros2 run robot_control robot_command_server`
5. `ros2 run operator_ui app_node`

레거시 음성 서비스(`/get_keyword`)까지 써야 하면 아래 launch를 대신 사용할 수 있다.

```bash
ros2 launch robot_control voice_command_stack.launch.py
```

## 관련 외부 패키지

- `voice_interfaces`
  - `RobotCommand.action`, `GetKeyword.srv`
- `operator_ui`
  - Flask HMI + 통합 음성 인식 + robot_command 액션 클라이언트
- `inspection_3d`
  - 스캔/후처리/비교 엔진
- `od_msg`
  - `SrvPointCloudCompare.srv`
- `outlet_assembly`
  - `OUTLET_ASSEMBLE` 구현체
- `tool_sorter_cleanup`, `tool_sorter_handover`
  - 공구 정리/전달 구현체

## 파일별 현재 사용 여부

현재 직접 실행 경로에서 핵심으로 쓰는 파일:

- `robot_control/robot_command_server.py`
- `robot_control/bolt_assemble_task.py`
- `robot_control/pointcloud_inspector_task.py`
- `robot_control/task_config.py`
- `robot_control/tool_handover_task.py`
- `robot_control/tool_cleanup_task.py`
- `robot_control/gripper_service.py`

지금 구조에서 참고 또는 보조 성격이 강한 파일:

- `robot_control/voice_command_dispatcher.py`
- `robot_control/onrobot.py`

## git에 같이 올릴 파일

- `robot_control/robot_control/robot_command_server.py`
- `robot_control/robot_control/voice_command_dispatcher.py`
- `robot_control/robot_control/bolt_assemble_task.py`
- `robot_control/robot_control/pointcloud_inspector_task.py`
- `robot_control/robot_control/tool_handover_task.py`
- `robot_control/robot_control/tool_cleanup_task.py`
- `robot_control/robot_control/gripper_service.py`
- `robot_control/robot_control/task_config.py`
- `robot_control/launch/voice_command_stack.launch.py`
- `robot_control/launch/tool_sorter_stack.launch.py`
- `robot_control/setup.py`
- `robot_control/package.xml`
- `robot_control/README.md`

필요 시 함께 포함:

- `robot_control/resource/bolt.pt`
- `robot_control/resource/T_gripper2camera.npy`
