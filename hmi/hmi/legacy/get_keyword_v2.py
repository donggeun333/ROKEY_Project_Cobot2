# [수정] 상시 대기(always-on) 구조로 변경 -> 더 이상 서비스 호출로 트리거하지 않음
# 실행 확인용 CLI 명령은 필요 없어짐 (노드 시작과 동시에 자동으로 wake word를 상시 감지)

import os
import json  # [추가] tools/targets를 JSON 문자열로 발행하기 위함
import threading  # [추가] wake word 상시 감지 루프를 백그라운드 스레드로 구동
import rclpy
import pyaudio
from rclpy.node import Node
from std_msgs.msg import String  # [추가] 추출된 작업 지시(도구/목적지)를 발행할 토픽 타입

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate  # d2 이거를 langchain_core로 바꿈
# from langchain.chains import LLMChain

from hmi.voice.MicController import MicController, MicConfig

from hmi.voice.wakeup_word import WakeupWord
from hmi.voice.stt import STT
from hmi.voice.tts import TTS  # [추가] "어떤 작업을 하시겠습니까?" 등 음성 안내 재생용
from hmi.voice.phrases import get_phrase  # [추가] 상황별 문구를 한 곳에서 관리

############ Package Path & Environment Setting ############

#----------------------------------------------------------------
# current_dir = os.getcwd()
# package_path = get_package_share_directory("pick_and_place_voice")

# env_path = "/home/rokey/cobot_ws/src/cobot2_ws/pick_and_place_voice/resource/.env"
# load_dotenv(dotenv_path=env_path)
# is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
# openai_api_key = os.getenv("OPENAI_API_KEY")
#-----------------------------------------------------------------

PACKAGE_NAME = "hmi"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - hammer, screwdriver, wrench, pos1, pos2, pos3

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "hammer를 pos1에 가져다 놔"  
            출력: hammer / pos1

            - 입력: "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"  
            출력: hammer wrench / pos1

            - 입력: "왼쪽에 있는 hammer를줘"  
            출력: hammer /

            - 입력: "왼쪽에 있는 못 박을 수 있는것을 줘"  
            출력: hammer /

            - 입력: "hammer는 pos2에 두고 screwdriver는 pos1에 둬"  
            출력: hammer screwdriver / pos2 pos1

            <사용자 입력>
            "{user_input}"                
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        # self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)
        self.tts = TTS(openai_api_key=openai_api_key)  # [추가] 음성 안내 재생용


        super().__init__("get_keyword_node")
        # 오디오 설정
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
        # self.ai_processor = AIProcessor()

        # [수정] 결과를 돌려줄 대상이 더 이상 서비스 호출자가 아니므로,
        # 추출된 작업 지시(도구/목적지)를 토픽으로 발행해 HMI 등 다른 노드가 구독하게 한다.
        self.result_pub = self.create_publisher(String, "/voice/pick_place_command", 10)

        self.get_logger().info("MicRecorderNode initialized.")
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # [수정] 마이크 스트림을 노드 시작 시점에 한 번만 열고, 이후 계속 재사용한다.
        # (기존에는 서비스 호출마다 open_stream()을 새로 호출했음)
        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            raise

        # [수정] "UI를 실행하면 마이크는 상시 대기" 요구사항 반영:
        # 노드가 생성되는 즉시 wake word 감지 루프를 백그라운드 스레드로 상시 구동한다.
        # (이전에는 /get_keyword 서비스가 호출되어야만 대기가 시작되는 구조였음)
        self.get_logger().info("wake word('헬로우 로키') 상시 대기 시작...")
        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def _listen_loop(self):
        """wake word를 상시 감지하고, 감지될 때마다 음성 안내 → 녹음 → 키워드 추출 → 토픽 발행을 반복하는 루프."""
        while rclpy.ok():
            if not self.wakeup_word.is_wakeup():
                continue  # wake word가 아직 감지되지 않았으면 계속 대기

            self.get_logger().info("Wakeword('헬로우 로키') 감지됨")

            # [추가] wake word 감지 시 음성 안내 재생 (실제 TTS 오디오 출력)
            try:
                self.tts.speak(get_phrase("wakeup_prompt"))
            except Exception as e:
                self.get_logger().error(f"TTS 재생 실패: {e}")

            # STT --> Keyword Extract --> Embedding
            try:
                output_message = self.stt.speech2text()
                tools, targets = self.extract_keyword(output_message)
            except Exception as e:
                self.get_logger().error(f"STT/키워드 추출 실패: {e}")
                continue

            self.get_logger().warn(f"Detected tools: {tools}, targets: {targets}")

            # [추가] 인식 결과에 따라 성공/실패 음성 안내 재생
            try:
                if tools:
                    tool_str = ", ".join(tools)
                    target_str = ", ".join(targets) if targets else "지정된 위치"
                    self.tts.speak(get_phrase("task_recognized", tool=tool_str, target=target_str))
                else:
                    self.tts.speak(get_phrase("task_failed"))
            except Exception as e:
                self.get_logger().error(f"TTS 재생 실패: {e}")

            # [수정] 도구(tools)뿐 아니라 목적지(targets)도 함께 JSON으로 발행
            # (app_v3.py의 AssemblyHmiBridge가 이 토픽을 구독해 처리)
            msg = String()
            msg.data = json.dumps({
                "raw_text": output_message,
                "tools": tools,
                "targets": targets,
            }, ensure_ascii=False)
            self.result_pub.publish(msg)
            self.get_logger().info(f"토픽 발행 완료 (/voice/pick_place_command) -> '{msg.data}'")

    def extract_keyword(self, output_message):  # d2 이 함수 일부 수정함
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object}")
        print(f"object: {object}")
        print(f"target: {target}")
        # [수정] 기존엔 object(도구)만 반환하고 target(목적지)은 버려졌었다.
        # pick-and-place 실행에는 목적지 정보가 반드시 필요하므로 둘 다 반환한다.
        return object, target


def main():  # d2 메인문 일부 수정
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
