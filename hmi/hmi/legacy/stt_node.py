#!/usr/bin/env python3
# ==========================================
# Azure STT -> ROS 2 Topic Publisher Node
# ==========================================
import os
import sys
import time
import termios
import tty
import threading
from dotenv import load_dotenv
from pathlib import Path  # 추가

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32  # [추가] 시작 명령 상태(0/1)를 알리는 Int32 토픽용

import azure.cognitiveservices.speech as speechsdk

# ----------------------------------------------------
# 1. 환경 변수 로드
# ----------------------------------------------------
# ----------------------------------------------------
# 1. 환경 변수 (.env) 경로 자동 탐색
# ----------------------------------------------------
# 현재 파일 위치: ~/cobot_ws/src/hmi/hmi/stt_node.py
# .env 파일 위치: ~/cobot_ws/src/hmi/resource/.env  (패키지 루트의 resource 폴더)
CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = CURRENT_DIR.parent  # hmi 패키지 루트 디렉토리 (resource/, setup.py 등이 위치)

# [수정] 여러 후보 경로를 우선순위대로 탐색한다.
# ① resource/.env  : 요청하신 위치 (source 트리 기준, colcon build 전 상태에서도 동작)
# ② share/<pkg>/resource/.env : colcon build 후 install 공간에서 실행할 때의 위치
#    (ament_index_python으로 설치된 share 디렉토리를 찾아 그 아래 resource/를 확인)
# ③ 기존 호환을 위한 위치들 (패키지 루트 직접, 현재 파일과 같은 디렉토리, 실행 디렉토리)
candidate_paths = [
    PACKAGE_ROOT / "resource" / ".env",   # ① 소스 트리의 resource 폴더 (요청하신 위치)
]

try:
    from ament_index_python.packages import get_package_share_directory
    share_dir = Path(get_package_share_directory("hmi"))
    candidate_paths.append(share_dir / "resource" / ".env")  # ② install 공간의 resource 폴더
except Exception:
    pass  # colcon build 전이거나 ROS2 환경이 아니면 무시하고 다음 후보로 진행

candidate_paths += [
    PACKAGE_ROOT / ".env",       # ③ 패키지 루트 직접 (이전 방식과의 호환)
    CURRENT_DIR / ".env",        # ③ stt_node.py와 같은 디렉토리
    Path.cwd() / ".env",        # ③ 실행 디렉토리(CWD)
]

env_path = None
for candidate in candidate_paths:
    if candidate.exists():
        env_path = candidate
        break

if env_path is None:
    # 어디에서도 못 찾았으면 ①번 경로로 시도해서 dotenv가 없다는 사실을 그대로 드러낸다
    env_path = candidate_paths[0]
    print(f"⚠️ .env 파일을 찾지 못했습니다. 확인한 경로: {[str(p) for p in candidate_paths]}")
else:
    print(f"✅ .env 파일 발견: {env_path}")

load_dotenv(dotenv_path=env_path)

azure_speech_key = os.getenv("AZURE_SPEECH_KEY")
azure_region = os.getenv("AZURE_REGION")


class AzureSttPublisherNode(Node):
    def __init__(self):
        super().__init__("azure_stt_publisher_node")

        if not azure_speech_key or not azure_region:
            self.get_logger().error(".env 파일에 AZURE_SPEECH_KEY 또는 AZURE_REGION이 설정되지 않았습니다.")

        # ROS 2 토픽 Publisher 생성
        self.stt_pub = self.create_publisher(String, "/ui/stt_result", 10)
        # [추가] "시작" 인식 여부를 0/1 정수로 알리는 토픽
        # 1 = 인식된 텍스트에 "시작"이 포함됨 (정상 시작)
        # 0 = 그 외 (다른 명령어이거나 인식 실패/무음 등) → app_v3 쪽에서 알람 처리용
        self.cmd_code_pub = self.create_publisher(Int32, "/ui/stt_command_code", 10)

        # Azure Speech Config 설정
        self.speech_config = speechsdk.SpeechConfig(
            subscription=azure_speech_key, region=azure_region
        )
        self.speech_config.speech_recognition_language = "ko-KR"

        # 침묵 대기 시간 설정 (Initial: 5초, Segmentation: 2초)
        self.speech_config.set_property(
            speechsdk.PropertyId.SpeechServiceConnection_InitialSilenceTimeoutMs, "5000"
        )
        self.speech_config.set_property(
            speechsdk.PropertyId.Speech_SegmentationSilenceTimeoutMs, "2000"
        )

        self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self.speech_recognizer = speechsdk.SpeechRecognizer(
            speech_config=self.speech_config, audio_config=self.audio_config
        )

        # 문맥 단어(Phrase List) 등록
        phrase_list_grammar = speechsdk.PhraseListGrammar.from_recognizer(self.speech_recognizer)
        phrase_list_grammar.addPhrase("퇴실")
        phrase_list_grammar.addPhrase("입실")
        phrase_list_grammar.addPhrase("재검사")
        phrase_list_grammar.addPhrase("비상정지")

        # 상태 관리 변수
        self.final_text = ""
        self.is_recognizing = False

        # Azure 콜백 연결
        self.speech_recognizer.recognizing.connect(self._on_recognizing)
        self.speech_recognizer.recognized.connect(self._on_recognized)

        self.get_logger().info("Azure STT ROS 2 Publisher Node가 성공적으로 시작되었습니다.")

    def _on_recognizing(self, evt):
        """실시간 인식 중 (자막 효과)"""
        if self.is_recognizing:
            sys.stdout.write(f"\r 🎙️ [실시간 수음 중]: {self.final_text} {evt.result.text}")
            sys.stdout.flush()

    def _on_recognized(self, evt):
        """음성 인식이 확정되었을 때"""
        if self.is_recognizing and evt.result.reason == speechsdk.ResultReason.RecognizedSpeech:
            text = evt.result.text.strip()
            if text:
                self.final_text = text
                print(f"\n ✅ [최종 확정 문장]: {self.final_text}")
                print("-" * 50)

                # 🚀 ROS 2 토픽 발행 (/ui/stt_result)
                msg = String()
                msg.data = self.final_text
                self.stt_pub.publish(msg)
                self.get_logger().info(f"토픽 발행 완료 (/ui/stt_result) -> '{self.final_text}'")

                # [추가] "시작" 인식 여부를 0/1 로 함께 발행
                # 1 = "시작"이 포함된 명령을 인식함 (정상 시작)
                # 0 = "시작"이 아닌 다른 명령을 인식함 (app_v3에서 알람 처리)
                code_msg = Int32()
                code_msg.data = 1 if "시작" in self.final_text else 0
                self.cmd_code_pub.publish(code_msg)
                self.get_logger().info(f"토픽 발행 완료 (/ui/stt_command_code) -> {code_msg.data}")
            else:
                # 인식은 됐지만 텍스트가 비어있는 경우 → 실패로 간주해 0 발행
                self._publish_alarm_code()

            # 인식 세션 종료 후 대기 상태 전환
            self.speech_recognizer.stop_continuous_recognition_async()
            self.is_recognizing = False
            print("\n [대기 중] 다시 녹음하려면 '스페이스바'를 누르세요. (종료: Ctrl + C)")
        elif self.is_recognizing:
            # [추가] 무음/타임아웃 등으로 인식에 실패한 경우 → 0(알람) 발행 후 대기 상태 전환
            print(f"\n ⚠️ [인식 실패/무음]: reason={evt.result.reason}")
            self._publish_alarm_code()
            self.speech_recognizer.stop_continuous_recognition_async()
            self.is_recognizing = False
            print("\n [대기 중] 다시 녹음하려면 '스페이스바'를 누르세요. (종료: Ctrl + C)")

    def _publish_alarm_code(self):
        """[추가] "시작" 인식 실패 상황을 0으로 알리는 헬퍼 함수"""
        code_msg = Int32()
        code_msg.data = 0
        self.cmd_code_pub.publish(code_msg)
        self.get_logger().info("토픽 발행 완료 (/ui/stt_command_code) -> 0 (인식 실패/시작 아님)")

    def start_listening(self):
        """스페이스바 입력 시 수음 시작"""
        if not self.is_recognizing:
            self.final_text = ""
            self.is_recognizing = True
            print("\n 🎙️ [녹음 시작] 마이크에 대고 말씀하세요... (말을 마치고 2초 후 자동 확정)")
            self.speech_recognizer.start_continuous_recognition_async()


def get_key():
    """터미널 즉시 키 입력 감지"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if ch == "\x03":  # Ctrl + C
        raise KeyboardInterrupt

    return ch


def main(args=None):
    rclpy.init(args=args)
    node = AzureSttPublisherNode()

    # ROS 2 Spin을 별도 백그라운드 스레드로 구동
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    print("\n" + "=" * 50)
    print(" PTT(스페이스바 제어) ROS 2 STT 노드가 실행되었습니다.")
    print(" 키보드의 '스페이스바'를 누르면 녹음이 시작됩니다.")
    print("=" * 50 + "\n")
    print(" [대기 중] 녹음하려면 '스페이스바'를 누르세요.")

    try:
        while rclpy.ok():
            if not node.is_recognizing:
                key = get_key()
                if key == " ":
                    node.start_listening()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n 프로그램을 안전하게 종료합니다...")
        node.speech_recognizer.stop_continuous_recognition_async()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()