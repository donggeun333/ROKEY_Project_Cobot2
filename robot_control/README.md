# Robot Control Package README

이 패키지는 음성 명령을 실제 로봇 작업으로 연결하는 상위 실행 패키지다.

현재 기준으로 다음 두 작업을 담당한다.

- 볼트 체결
- pointcloud 기반 3D 검사
  - 볼트 체결 검사
  - 멀티탭 체결 검사

## 현재 구조

실행 흐름은 아래와 같다.

1. `voice_processing/get_keyword_node.py`
   - 음성을 `intent/tools/targets`로 해석
2. `robot_control/voice_command_dispatcher.py`
   - 음성 해석 결과를 `/robot_command` 액션으로 전달
3. `robot_control/robot_command_server.py`
   - intent별 task 실행
4. task 파일
   - `bolt_assemble_task.py`
   - `pointcloud_inspector_task.py`

현재는 `voice_command_stack.launch.py`로 아래를 함께 실행할 수 있다.

- `robot_control/robot_command_server`
- `robot_control/voice_command_dispatcher`

## 지원 명령

- `볼트 체결해줘`
  - `BOLT_ASSEMBLE`
  - `bolt_assemble_task.py` 실행
- `볼트 체결 검사해줘`
  - `INSPECT_FASTEN`
  - `pointcloud_inspector_task.py`를 `object_type=bolt`로 실행
- `멀티탭 체결 검사해줘`
  - `CONNECTOR_INSPECT`
  - `pointcloud_inspector_task.py`를 `object_type=multitap`으로 실행

## 핵심 파일 역할

- `robot_control/robot_command_server.py`
  - `/dsr01/robot_command` 액션 서버
  - intent를 받아 실제 task 함수 호출
- `robot_control/voice_command_dispatcher.py`
  - `/get_keyword` 서비스 클라이언트
  - `/robot_command` 액션 클라이언트
- `robot_control/bolt_assemble_task.py`
  - 볼트 검출, 파지, 체결 task
  - YOLO + depth + TF + movej/movel 사용
- `robot_control/pointcloud_inspector_task.py`
  - pointcloud 스캔/비교 task
  - pointcloud 패키지의 reset/capture/finalize/compare 서비스 호출
- `robot_control/task_config.py`
  - 볼트 체결 포즈, 스캔 포즈, 토픽, 타임아웃, 서비스명 같은 공통 설정 모음

## 참고/보조 파일 역할

- `robot_control/onrobot.py`
  - OnRobot RG2 그리퍼 제어
- `robot_control/grasp_visualizer_node.py`
  - grasp 시각화용 유틸 노드
- `robot_control/robot_control.py`
  - 예전/단독 제어 흐름 참고용
- `robot_control/make3d.py`
  - 예전 3D 관련 테스트/참고용 코드

## task_config.py에서 관리하는 값

이 파일에서 주로 관리한다.

- robot id / model
- 그리퍼 및 TCP 설정
- 볼트 체결용 카메라 토픽
- 볼트 체결용 포즈
- pointcloud 검사용 스캔 포즈
- pointcloud 서비스 이름
- 타임아웃, 속도, settle time

즉 실기 조정이 필요한 값은 이 파일에서 먼저 확인하면 된다.

## 기본 실행 구성

현재 권장 실행:

1. robot bringup
2. realsense
3. `ros2 run voice_processing get_keyword`
4. 필요 시 `ros2 launch pointcloud pipeline_with_comparison.launch.py`
5. `ros2 launch robot_control voice_command_stack.launch.py`

이 launch가 같이 띄우는 것:

- `robot_control/robot_command_server`
- `robot_control/voice_command_dispatcher`

## 관련 외부 패키지

- `voice_processing`
  - 음성 입력 및 intent 추출
- `voice_interfaces`
  - `RobotCommand.action`, `GetKeyword.srv`
- `pointcloud`
  - 스캔/후처리/비교 엔진
- `od_msg`
  - `SrvPointCloudCompare.srv`

## 파일별 현재 사용 여부

현재 직접 실행 경로에서 핵심으로 쓰는 파일:

- `robot_control/robot_command_server.py`
- `robot_control/voice_command_dispatcher.py`
- `robot_control/bolt_assemble_task.py`
- `robot_control/pointcloud_inspector_task.py`
- `robot_control/task_config.py`
- `robot_control/onrobot.py`

지금 구조에서 참고 또는 보조 성격이 강한 파일:

- `robot_control/robot_control.py`
- `robot_control/make3d.py`
- `robot_control/grasp_visualizer_node.py`

## git에 같이 올릴 파일

- `robot_control/robot_control/robot_command_server.py`
- `robot_control/robot_control/voice_command_dispatcher.py`
- `robot_control/robot_control/bolt_assemble_task.py`
- `robot_control/robot_control/pointcloud_inspector_task.py`
- `robot_control/robot_control/task_config.py`
- `robot_control/robot_control/onrobot.py`
- `robot_control/launch/voice_command_stack.launch.py`
- `robot_control/setup.py`
- `robot_control/package.xml`
- `robot_control/README.md`

필요 시 함께 포함:

- `robot_control/resource/bolt.pt`
- `robot_control/resource/T_gripper2camera.npy`
