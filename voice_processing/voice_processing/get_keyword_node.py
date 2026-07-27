# ros2 service call /get_keyword voice_interfaces/srv/GetKeyword "{}"
# ↑ 커스텀 서비스로 바뀌었으므로 호출 명령의 서비스 타입도 std_srvs/Trigger가 아닌
#   voice_interfaces/srv/GetKeyword 로 바뀝니다.

import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate

# ---- 변경점 1: std_srvs.Trigger 대신 커스텀 서비스 GetKeyword 사용 ----
# 기존에는 Trigger(success, message)만 써서 intent/tools/targets를 문자열 하나에
# 억지로 합쳐 보냈지만, 이제는 각각이 정식 필드로 분리된 커스텀 서비스를 사용합니다.
from voice_interfaces.srv import GetKeyword

from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")


############ GetKeywordNode ############
class GetKeywordNode(Node):
    """
    음성 명령을 받아 (intent, tools, targets)를 추출하고,
    커스텀 서비스 voice_interfaces/GetKeyword의 응답 필드에 각각 담아 반환하는 노드.

    - 서비스 이름: /get_keyword
    - 서비스 타입: voice_interfaces/srv/GetKeyword
        Request  : (없음)
        Response : bool success / string intent / string[] tools / string[] targets / string raw_text
    """

    def __init__(self):
        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        # ---- LLM(GPT-4o) 설정 ----
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        # ---- 프롬프트 정의 (최종 시나리오 7종 intent + 되묻기용 CLARIFY_NEEDED) ----
        prompt_content = """
            당신은 조립 공정 보조 로봇의 음성 명령을 해석하는 역할을 합니다.
            사용자의 문장을 아래 8가지 의도(intent) 중 하나로 분류하고,
            문장에 등장하는 도구/부품과 목적지(위치)를 함께 추출해야 합니다.

            <의도(intent) 목록>
            - BOLT_ASSEMBLE          : (플레이트/커버는 사람이 이미 세팅한 상태) 볼트를 조립/체결해 달라는 명령
            - TOOL_FETCH              : 지정한 공구를 가져다/집어 달라는 명령
            - TOOL_CLEANUP            : 사용한 공구를 정리/반납해 달라는 명령 (마스킹테이프 색상별 정리)
            - INSPECT_FASTEN          : 볼트 체결 상태를 3D로 검사해 달라는 명령
            - CONNECTOR_AUTO_CONNECT  : 큰 커넥터(콘센트/멀티탭)를 로봇이 자동으로 인식해 체결해 달라는 명령
            - CABLE_FETCH_HANDOVER    : 작은 커넥터(랜선 등)를 가져와서 작업자에게 건네 달라는 명령
            - CONNECTOR_INSPECT       : 작업자가 체결한 커넥터의 체결 상태를 검사해 달라는 명령
            - CLARIFY_NEEDED          : 위 7개 중 어디에 해당하는지 문장만으로는 판단할 수 없을 때
                                        (예: 커넥터 종류가 특정되지 않은 "커넥터 체결해줘" 같은 모호한 명령)

            <도구/부품 리스트>
            - hammer(망치), monkey_wrench(몽키스페너), wrench(스페너), screwdriver(스크류 드라이버),
              vise(벤찌), bolt(볼트), plate(플레이트), cover(커버),
              outlet(콘센트), multitap(멀티탭), router(공유기), lan_cable(랜선), lan_port(랜포트)

            <위치 리스트>
            - tool_box(도구함), bolt_holder(볼트 거치대), multitap_position(멀티탭 위치),
              lan_cable_position(랜선 위치), lan_port_position(랜포트 위치), handover_zone(전달 위치)

            <intent 판별 힌트>
            - "검사"라는 단어만 있고 커넥터 언급이 없으면 INSPECT_FASTEN, 커넥터(멀티탭/콘센트/랜선/랜포트 등) 관련
              문맥이면 CONNECTOR_INSPECT로 분류하세요. 애매하면 INSPECT_FASTEN을 기본값으로 하세요.
            - "가져와/가져다줘" + 랜선/랜포트/공유기 -> CABLE_FETCH_HANDOVER
            - "가져와/가져다줘" + 그 외 공구(hammer, wrench 등) -> TOOL_FETCH
            - 콘센트/멀티탭처럼 큰 커넥터 종류가 명시된 체결 명령은 CONNECTOR_AUTO_CONNECT
            - "커넥터 연결/체결해줘"처럼 큰 커넥터(콘센트/멀티탭)인지 작은 커넥터(랜선/랜포트)인지
              문장만으로 구분할 수 없으면, 절대 임의로 추측하지 말고 CLARIFY_NEEDED로 분류하세요.
              이때 tools 칸에는 사용자가 실제로 언급한 애매한 단어(예: "connector")를 그대로 담아
              디스패처가 되물을 때 참고할 수 있게 하세요.

            <출력 형식>
            - [intent / 도구1 도구2 ... / 위치1 위치2 ...] 형식을 반드시 따르세요.
            - 한국어 명칭은 위 영어 토큰으로 변환해서 출력하세요.
            - 해당 정보가 없으면 그 칸은 비우되 구분자 '/'는 항상 2개 유지하세요.

            <예시>
            - 입력: "볼트 조립해줘"
            출력: BOLT_ASSEMBLE / bolt / bolt_holder

            - 입력: "공구 가져와줘, 몽키스패너"
            출력: TOOL_FETCH / monkey_wrench / tool_box

            - 입력: "공구 정리해줘"
            출력: TOOL_CLEANUP / / tool_box

            - 입력: "검사해줘"
            출력: INSPECT_FASTEN / /

            - 입력: "멀티탭 체결해줘"
            출력: CONNECTOR_AUTO_CONNECT / multitap / multitap_position

            - 입력: "멀티탭 체결 검사해줘"
            출력: CONNECTOR_INSPECT / multitap / multitap_position

            - 입력: "랜선 가져와"
            출력: CABLE_FETCH_HANDOVER / lan_cable / lan_cable_position

            - 입력: "커넥터 체결해줘"
            출력: CLARIFY_NEEDED / connector /

            - 입력: "랜선 체결됐는지 검사해줘"
            출력: CONNECTOR_INSPECT / lan_cable / lan_port_position

            <사용자 입력>
            "{user_input}"
        """
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)

        super().__init__("get_keyword_node")

        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)

        self.get_logger().info("GetKeywordNode initialized.")
        self.get_logger().info("wait for client's request...")

        # ---- 변경점 2: 서비스 타입을 GetKeyword로 등록 ----
        self.get_keyword_srv = self.create_service(
            GetKeyword, "get_keyword", self.get_keyword_callback
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):
        """
        STT 텍스트를 LLM에 전달하여 (intent, tools, targets)를 추출.
        반환 형식은 이전과 동일하지만, 호출부에서 커스텀 서비스 필드에 바로 대입할 수 있도록
        각 값이 이미 리스트/문자열로 정리된 상태로 반환됩니다.
        """
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        try:
            intent_str, tools_str, targets_str = result.strip().split("/")
        except ValueError:
            self.get_logger().error(f"Unexpected LLM output format: {result}")
            return "UNKNOWN", [], []

        intent = intent_str.strip()
        tools = tools_str.split()
        targets = targets_str.split()

        print(f"llm's raw response: {result}")
        print(f"intent: {intent}, tools: {tools}, targets: {targets}")
        return intent, tools, targets

    def get_keyword_callback(self, request, response):
        """
        /get_keyword 서비스 콜백.
        변경점: response의 필드가 intent / tools / targets로 분리되어 있어
        클라이언트 쪽에서 문자열 파싱(split) 없이 바로 response.intent, response.tools로
        접근할 수 있습니다.
        """
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            # 커스텀 서비스이므로 실패 시에도 모든 필드를 명시적으로 채워 반환
            response.success = False
            response.intent = "UNKNOWN"
            response.tools = []
            response.targets = []
            response.raw_text = ""
            return response

        while not self.wakeup_word.is_wakeup():
            pass

        output_message = self.stt.speech2text()
        intent, tools, targets = self.extract_keyword(output_message)

        self.get_logger().warn(
            f"Detected intent: {intent}, tools: {tools}, targets: {targets}"
        )

        # ---- 변경점 3: 문자열로 합치지 않고 각 필드에 직접 대입 ----
        response.success = (intent != "UNKNOWN")
        response.intent = intent
        response.tools = tools
        response.targets = targets
        response.raw_text = output_message  # 원본 발화 텍스트도 함께 반환 (로깅/디버깅용)
        return response


def main():
    rclpy.init()
    node = GetKeywordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
