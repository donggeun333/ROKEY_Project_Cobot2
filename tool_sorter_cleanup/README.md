# M0609 Tool Sorter Autonomous

상위 노드 매니저의 요청이나 전용 GUI 버튼 하나로 다섯 종류의 공구를 자동으로
정리하는 ROS 2 패키지입니다. GUI 없이(`use_gui:=false`) 돌려도 동일합니다. 인식, timestamp 기반 좌표변환, 로봇·그리퍼
제어는 `tool_sorter_core`에서 재사용하고 이 패키지는 자동운전
정책과 통합용 서비스만 담당합니다.

## 작업 순서

1. `/integration/tool_sorter/organize` 요청 수신
2. 설정과 카메라·로봇 서비스를 검증
3. Home을 거쳐 Bird view로 이동
4. object mask의 대표 depth가 가장 작은 공구부터 Pick & Place
5. 매 Pick 후 Bird view로 복귀해 움직인 장면을 새로 촬영하고 남은 공구의
   depth 순서를 다시 계산
6. 각 공구를 클래스별 절대 Base pose에 배치
7. 남은 공구가 없으면 Bird view에서 `COMPLETE` 발행

## 일부 공구가 없을 때

5종이 모두 있어야 시작하지는 않습니다. 작업장에 있는 공구만 정리하고,
끝까지 보지 못한 공구는 `COMPLETE` 상태의 message에 이름으로 남깁니다.

```text
2종 정리 완료; 작업장에서 확인되지 않은 공구: wrench, monkey_wrench, vise
```

파지 가능한 공구가 하나도 안 잡히면 `empty_scene_timeout_s`(기본 3초)
동안 Bird view에서 계속 재확인한 뒤에 종료합니다. 금속 공구는 반사 때문에
depth가 몇 프레임씩 비는데, 그 순간을 "공구 없음"으로 오판하지 않기 위해
프레임 수가 아니라 시간으로 판정합니다.

## 실측 좌표 입력

`config/autonomous_tool_sorter.yaml`의 아래 다섯 값을 직접 실측해 입력해야
합니다.

```yaml
place_pose_hammer: [X, Y, Z, RX, RY, RZ]
place_pose_screwdriver: [X, Y, Z, RX, RY, RZ]
place_pose_wrench: [X, Y, Z, RX, RY, RZ]
place_pose_monkey_wrench: [X, Y, Z, RX, RY, RZ]
place_pose_vise: [X, Y, Z, RX, RY, RZ]
```

좌표 단위는 Doosan Base 기준 위치 mm, 자세 deg입니다. 한 좌표라도 비어
있거나 값이 유효하지 않으면 요청을 `BLOCKED` 처리하며 로봇은 움직이지
않습니다. 실측 전에는 빈 배열을 유지하십시오. 공구별 pose 파라미터는
ROS 2가 빈 배열을 `BYTE_ARRAY`로 잘못 고정하지 않도록 `DOUBLE_ARRAY`로
명시 선언됩니다.

## 빌드 및 실행

```bash
cd /home/jwanryu/ws_cobot_pjt/ws_dsr
colcon build --symlink-install \
  --packages-select tool_sorter_cleanup
source install/setup.bash
```

실물 로봇, RG2, RealSense를 먼저 실행합니다.

```bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py \
  mode:=real host:=192.168.1.100
```

자동 정리 launch (자동 정리 전용 GUI 포함):

```bash
ros2 launch tool_sorter_cleanup \
  autonomous_tool_sorter.launch.py
```

## 자동 정리 전용 dashboard

`autonomous_dashboard`는 organize dashboard를 상속해 영상·검출 테이블은
그대로 쓰고, 이 작업에만 필요한 것을 더합니다.

- **공구 5종 정리 진행** 카드: `망치·드라이버·렌치·몽키렌치·바이스`를 칩으로
  두고 배치가 끝난 것부터 ✅로 바뀝니다. 진행 상황은 상태 JSON의
  `completed_names`를 그대로 읽으므로 메시지 문자열 파싱이 없습니다.
- 시작 버튼 이름이 `자동 정리 시작`입니다. 한 번 누르면 5종을 모두 처리할
  때까지 이어서 동작합니다.
- `홈 복귀`, `일시정지`, `동작 중지`는 organize와 같습니다.

상위 제어부만으로 운전하려면 창 없이 띄웁니다.

```bash
ros2 launch tool_sorter_cleanup \
  autonomous_tool_sorter.launch.py use_gui:=false
```

## GUI 없이 영상·검출 결과 확인

`use_gui:=false`로 띄우면 Qt 창이 없지만, perception 노드는 기존과
동일하게 다음 결과를 계속 발행합니다.

```text
/tool_sorter/perception/annotated   # 박스·마스크·파지축이 그려진 검출 영상
/tool_sorter/perception/scene       # 검출·depth·파지 정보 JSON
/tool_sorter/perception/camera_info # 카메라 내부 파라미터
```

따라서 화면에 자동으로 뜨지는 않고, 필요할 때 별도 viewer로 확인합니다.

```bash
# 검출 결과 영상
ros2 run rqt_image_view rqt_image_view
# viewer에서 /tool_sorter/perception/annotated 선택

# 또는 명령형 viewer
ros2 run image_view image_view --ros-args \
  -r image:=/tool_sorter/perception/annotated
```

원본 카메라 영상은 RealSense 드라이버가 발행하는
`/camera/camera/color/image_raw` 토픽에서 확인합니다. 창만 따로 띄우려면
아래처럼 실행할 수 있지만, 자동운전 launch나 기존 task manager를 중복
실행하면 안 됩니다.

```bash
ros2 run tool_sorter_cleanup autonomous_dashboard
```

상위 노드 매니저가 호출할 시작 서비스:

```bash
ros2 service call /integration/tool_sorter/organize \
  std_srvs/srv/Trigger '{}'
```

서비스 응답은 요청의 수락 여부만 즉시 반환합니다. 장시간 작업의 실제 결과는
다음 transient-local JSON 상태 토픽으로 확인합니다.

```text
/integration/tool_sorter/status
```

중지 인터페이스:

```text
/integration/tool_sorter/pause
/integration/tool_sorter/stop
```

모두 `std_srvs/srv/Trigger` 형식입니다.

## 안전

- 기존 GUI와 기존 `tool_sorter_core` launch를 동시에 실행하지
  마십시오. 같은 로봇 서비스와 TF를 중복 사용하게 됩니다.
- 이 패키지는 요청 직후 실제 로봇을 움직이므로 절대 배치 pose, Bird 높이,
  테이블 높이, 파지 Z 보정을 현장에서 먼저 검증해야 합니다.
- 소프트웨어 force stop은 물리 E-stop을 대체하지 않습니다.
