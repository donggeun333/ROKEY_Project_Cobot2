"""상황별 TTS 문구 사전.

실행 로직과 안내 문구를 분리해 두면, HMI 동작을 건드리지 않고도 현장 멘트만
교체할 수 있다. 음성 UX를 자주 조정하는 프로젝트에서 유지보수 비용을 줄이는
의도가 있다.
"""

PHRASES = {
    # ---------- 시스템 부팅 / 시작 ----------
    "system_boot_prompt": "안녕하십니까.",
    "system_boot_retry": "'시작'이라고 말씀해주세요.",
    "wakeup_prompt": "어떤 작업을 하시겠습니까?",
    "system_greeting": "시스템이 시작되었습니다.",

    # ---------- [매크로] ESTOP/HOME 확인 안내 ----------
    "macro_ack": "{command} 명령을 실행합니다.",
    "cycle_stop_ack": "공정을 종료하겠습니다.",

    # ---------- 시나리오 1: 공구 가져오기/정리 ----------
    "tool_fetch_start": "{tool}를 가지러 가겠습니다.",
    "tool_fetch_blocked": "공구를 가지러 갈 수 없습니다.",
    "tool_fetch_no_match": "어떤 공구인지 인식하지 못했습니다. 다시 말씀해주세요.",
    "tool_fetch_complete": "{tool} 전달을 마쳤습니다.",
    "tool_return_start": "공구를 원래 위치로 정리하겠습니다.",
    "tool_return_complete": "공구 정리를 마쳤습니다.",
    "tool_action_failed": "공구 작업을 끝내지 못했습니다. 화면의 로그를 확인해주세요.",
    "scenario_paused_for_tool": "진행 중인 작업을 잠시 멈추고 공구를 먼저 처리하겠습니다.",
    "scenario_resumed": "멈췄던 작업을 다시 진행하겠습니다.",

    # ---------- 시나리오 2: 볼트 조립/검사 ----------
    "bolt_assemble_start": "볼트를 조립하겠습니다.",

    # ---------- 시나리오 3: 콘센트 조립/검사 ----------
    "outlet_assemble_start": "콘센트를 조립하겠습니다.",

    # [신규] 조립 1 사이클 완료 안내 멘트
    "assembly_complete": "조립이 완료되었습니다.",

    # ---------- 공통: 3D 비전 검사 ----------
    "inspect_start": "검사를 시작합니다.",
    "inspect_complete": "검사가 완료되었습니다. 화면에서 최종 판정 버튼을 눌러주세요.",
    "inspect_result_ok": "양품입니다.",
    "inspect_result_ng": "불량 발생",
    "inspect_error": "검사 결과를 받아오지 못했습니다. 다시 시도해주세요.",

    # ---------- 도구/목적지 인식 성공/실패 ----------
    "task_recognized": "{tool}를 {target}로 옮기겠습니다.",
    "task_failed": "죄송합니다. 다시 한번 말씀해주세요.",

    # ---------- 시스템 종료 ----------
    "system_shutdown": "종료하겠습니다.",
    "system_shutdown_returning_home": "홈 위치로 복귀한 뒤 종료하겠습니다.",

    # ---------- 인식은 됐지만 처리할 시나리오가 없을 때 ----------
    "command_not_understood": "명령을 이해하지 못했습니다. 다시 말씀해주세요.",
}


def get_phrase(key: str, **kwargs) -> str:
    template = PHRASES.get(key, "")
    try:
        return template.format(**kwargs)
    except (KeyError, IndexError):
        return template
