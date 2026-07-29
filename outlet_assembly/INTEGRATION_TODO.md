# 인계 — 이 패키지를 붙이는 방법

**이 zip에는 `outlet_assembly` 패키지만 들어 있다.** 패키지 자체는 완성돼
있고 단위 테스트도 통과하지만, 받는 쪽 워크스페이스에서 **배선 2건**을 해야
음성/액션으로 부를 수 있다.

| 할 일 | 어디를 고치나 | 난이도 |
| --- | --- | --- |
| ① `robot_command_server`에 intent 등록 | `robot_control` 4곳 | 작음 |
| ② HMI 음성 경로 연결 | `operator_ui/operator_ui/app_v5.py` | 중간 |

①만 해도 **액션으로는 바로 쓸 수 있다.** ②는 음성으로 부르고 싶을 때만 한다.

전제: 이 패키지는 `tool_sorter_core`에 의존한다 — 13클래스 seg
모델(`models/best.pt`), 캘리브레이션(`models/T_gripper2camera.npy`),
서비스 기반 `OnRobotGripper`를 거기서 가져다 쓴다. 그 패키지가 없으면 빌드가
안 된다.

---

## 1. 배선 ① — robot_command_server에 intent 등록

`robot_control` 패키지에서 네 곳을 고친다. 볼트 체결(`BOLT_ASSEMBLE`)이 같은
구조라 바로 옆에 참고할 코드가 있다.

**(1) `robot_control/robot_command_server.py` — import 추가**

```python
from outlet_assembly.plug_insert_task import run_plug_insert
```

**(2) 같은 파일 — `SUPPORTED_INTENTS`에 추가**

```python
SUPPORTED_INTENTS = {
    "BOLT_ASSEMBLE",
    "OUTLET_ASSEMBLE",   # ← 추가
    ...
}
```

**(3) 같은 파일 — `resolve_handler()`에 분기 추가**

```python
if intent == "OUTLET_ASSEMBLE":
    return self.execute_plug_insert
```

**(4) 같은 파일 — 핸들러 추가** (`execute_bolt_assemble` 바로 아래가 자연스럽다)

```python
def execute_plug_insert(self, goal_handle) -> RobotCommand.Result:
    self.publish_feedback(goal_handle, "플러그 삽입 시퀀스 시작", 0.1)
    # 볼트와 같은 runtime 노드를 쓴다. 둘 다 DR_init.__dsr__node 를 잡으므로
    # 노드를 나누면 어느 쪽이 마지막에 잡았는지에 따라 동작이 갈린다.
    success, message = run_plug_insert(self.bolt_task_node)
    if not success:
        return self.abort_result(
            goal_handle, f"OUTLET_ASSEMBLE 실행 실패: {message}"
        )
    self.publish_feedback(goal_handle, "플러그 삽입 시퀀스 완료", 1.0)
    return self.success_result(
        goal_handle, f"OUTLET_ASSEMBLE 실행 완료: {message}"
    )
```

> `self.bolt_task_node`는 볼트 체결이 쓰는 runtime 노드다. 이름이 다르면
> 그쪽에서 `DR_init.__dsr__node`로 쓰는 노드를 그대로 넘긴다.

**(5) `robot_control/package.xml` — 의존 추가**

```xml
<depend>outlet_assembly</depend>
```

### 확인

```bash
colcon build --symlink-install --packages-select outlet_assembly robot_control
source install/setup.bash
ros2 launch robot_control voice_command_stack.launch.py
```

로그에 `OUTLET_ASSEMBLE`이 보여야 한다:

```text
RobotCommandServer ready. supported_intents=[BOLT_ASSEMBLE, CONNECTOR_INSPECT,
INSPECT_FASTEN, OUTLET_ASSEMBLE, TOOL_CLEANUP, TOOL_FETCH]
```

---

## 2. 여기까지 하면 되는 것

로봇·카메라·스택이 떠 있으면 액션으로 바로 돌아간다.

```bash
ros2 action send_goal /dsr01/robot_command \
  voice_interfaces/action/RobotCommand \
  '{intent: "OUTLET_ASSEMBLE", tools: [], targets: []}' --feedback
```

성공하면 `OUTLET_ASSEMBLE 실행 완료: 플러그 삽입 완료`,
실패하면 사유가 `result.message`에 그대로 담긴다
(`플러그 삽입 시간 초과(300s, state=FIND_PLUG)` 같은 형태).

단독 실행(미리보기 창 포함, robot_control 없이도 됨):

```bash
ros2 launch outlet_assembly plug_insert.launch.py
```

---

## 3. 배선 ② — HMI 음성

`app_v5.py:1266-1272`가 액션이 아니라 `/scenario/command` 토픽으로 발행한다.

```python
if category == "OUTLET_ASSEMBLE":
    self.voice_engine.speak(get_phrase("outlet_assemble_start"))
    with self.lock:
        self.last_assembled_category = "outlet"
    self._start_cycle("outlet")
    self._publish_scenario_command("OUTLET_ASSEMBLE")   # ← 구독자 없음
    return
```

**이 토픽을 구독하는 노드가 통합 src에 없다.** 음성으로 말해도 아무 일도
일어나지 않는다. `operator_ui/operator_ui/revision/app_v4.py`(구버전, 배포 안 됨)에만 발행부가
남아 있던 레거시 경로다.

---

## 4. LLM에 그대로 붙여넣을 지시문

아래를 그대로 주면 된다. 볼트 체결이 이미 같은 구조로 되어 있어 참고할 코드가
바로 옆에 있다.

````text
ROS 2 Humble 워크스페이스에서 작업한다. HMI 패키지는 `operator_ui`, 액션 서버는
`robot_control`, 이 기능의 구현체는 `outlet_assembly` 패키지에 있다.

목표: HMI(`operator_ui/operator_ui/app_v5.py`)의 음성 명령 `콘센트 체결해줘`(intent
`OUTLET_ASSEMBLE`)를 죽은 토픽 대신 `robot_command_server` 액션으로 보내도록
고친다. 전제: 배선 ①(robot_command_server의 `SUPPORTED_INTENTS`에
`OUTLET_ASSEMBLE` 등록 + `execute_plug_insert` 핸들러)이 이미 끝나 있어야 한다.
액션으로 직접 호출해서 동작하는 것을 먼저 확인하고 시작한다.

**참고할 기존 구현**: 같은 파일의 `BOLT_ASSEMBLE` 경로가 정확히 같은 모양이다.
`app_v5.py`의 `if category == "BOLT_ASSEMBLE":` 분기와 `_run_bolt_action()`
메서드를 읽고 그 패턴을 그대로 따른다.

해야 할 일:

1. `if category == "OUTLET_ASSEMBLE":` 분기(1266줄 근처)를 BOLT_ASSEMBLE 분기와
   같은 모양으로 바꾼다.
   - `self._claim_tool_action()` 으로 슬롯을 잡는다 (팔을 통째로 쓰므로 공구
     작업과 같은 슬롯을 나눠 쓴다). 실패하면 return.
   - `self._start_cycle("outlet", auto_finish=False)` — 타이머가 아니라 액션
     결과가 사이클을 닫아야 한다.
   - `self._publish_scenario_command("OUTLET_ASSEMBLE")` 를 지우고, 대신
     `threading.Thread(target=self._run_outlet_action, daemon=True).start()`.

2. `_run_bolt_action()` 바로 아래에 `_run_outlet_action()` 을 추가한다.
   `_run_bolt_action`을 그대로 베끼고 intent만 `"OUTLET_ASSEMBLE"` 로 바꾼다.
   `self._send_robot_command_and_wait("OUTLET_ASSEMBLE", tools=[], targets=[],
   timeout_sec=TOOL_ACTION_TIMEOUT_SEC)` 를 쓴다.
   finally에서 `_tool_action_busy = False` 와 `self._stop_cycle()` 을 반드시
   한다 (안 하면 다음 명령이 영영 거절된다).

3. **UI 라벨 슬롯을 새로 만든다.** 이게 빠지면 플러그 삽입 중에 화면에
   "볼트 체결"로 표시된다.
   - `TOOL_JOB_OUTLET = "콘센트 체결"` 상수 추가 (753~756줄의 `TOOL_JOB_*` 옆).
   - `resolve_tool_job(handover, cleanup, bolt=None)` 시그니처에 `outlet=None`
     을 추가하고, 내부 `entries` 리스트에 `(TOOL_JOB_OUTLET, outlet)` 을 넣는다.
   - `self.outlet_status = None` 을 `self.bolt_status = None`(931줄) 옆에 추가.
   - `_set_bolt_status` 를 본떠 `_set_outlet_status` 를 추가.
   - `get_status()`(1652줄 근처)의 `resolve_tool_job(...)` 호출에
     `self.outlet_status` 를 넘긴다.

제약:
- `app_v5.py` 외의 파일은 고치지 않는다. 배선 ①이 끝났다면 백엔드는 맞다.
- 기존 BOLT_ASSEMBLE / TOOL_FETCH / TOOL_CLEANUP 동작은 바뀌면 안 된다.
- 주석은 한국어로, 주변 코드의 밀도와 어투에 맞춘다.

검증:
```bash
cd <워크스페이스>/src/cobot2_ws/operator_ui
python3 -c "
import sys, importlib.util
spec = importlib.util.spec_from_file_location('app_v5', 'operator_ui/app_v5.py')
m = importlib.util.module_from_spec(spec); sys.modules['app_v5']=m
spec.loader.exec_module(m)
f = m.classify_scenario_command
for p in ['콘센트 체결해줘','볼트 체결해줘','공구 정리해줘','망치 가져와줘']:
    print(f'{p!r:16} -> {f(p)}')
"
```
`콘센트 체결해줘 -> OUTLET_ASSEMBLE` 이 나와야 하고 나머지 셋도 그대로여야 한다.

파이썬 파일만 고쳤으면 재빌드 없이 `ros2 run operator_ui app_node` 재시작으로 반영된다.
````

---

## 5. 실기로 확인할 것

HMI 배선이 끝나면 순서대로 본다.

1. `ros2 action send_goal ...` 로 액션 직접 호출 — 변수를 줄이고 시작한다.
2. 음성 `헬로우 로키` → `시작` → `콘센트 체결해줘`.
3. 볼트 체결 → 콘센트 체결을 **한 세션에서** 연달아 해본다. 그리퍼를
   드라이버 서비스로 통일했으므로 되어야 한다. 안 되면 `onrobot.RG`가 어딘가에
   남아 있다는 뜻이다.

미검증 구간: 이 패키지는 **아직 실기로 돌려본 적이 없다.** 원본
`plug_insert.py`는 단독 스크립트로 동작했지만, 그리퍼를 드라이버 서비스로
바꾼 뒤로는 검증되지 않았다. 첫 실행은 저속으로, 경로를 눈으로 확인하며 한다.
