#!/usr/bin/env python3
# ==========================================
# test_tts.py - TTS 단독 검증 (ROS2/wake word 배제)
# 실행: python3 test_tts.py            (기본 출력 장치로 재생)
#      python3 test_tts.py 4          (4번 장치로 강제 지정해서 재생)
# ==========================================
import os
import sys
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv


def main():
    PACKAGE_NAME = "hmi"
    PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
    ENV_PATH = os.path.join(PACKAGE_PATH, "resource", ".env")
    load_dotenv(dotenv_path=ENV_PATH)
    openai_api_key = os.getenv("OPENAI_API_KEY")

    print(f"[1/3] .env 로드 경로: {ENV_PATH}")
    print(f"      OPENAI_API_KEY 존재 여부: {bool(openai_api_key)}")

    print("[2/3] 오디오 출력 장치 목록:")
    import sounddevice as sd
    print(sd.query_devices())
    print(f"      현재 기본 출력 장치: {sd.default.device}")

    # [추가] 커맨드라인 인자로 장치 번호를 지정하면 해당 장치로 강제 재생
    forced_device = None
    if len(sys.argv) > 1:
        try:
            forced_device = int(sys.argv[1])
            print(f"      → {forced_device}번 장치로 강제 지정해서 재생합니다.")
        except ValueError:
            print(f"      ⚠️ 잘못된 장치 번호: {sys.argv[1]} (무시하고 기본 장치 사용)")

    print("[3/3] TTS 재생 테스트...")
    try:
        from hmi.voice.tts import TTS
        tts = TTS(openai_api_key=openai_api_key)
        if forced_device is not None:
            import sounddevice as sd
            sd.default.device = (sd.default.device[0], forced_device)
        tts.speak("테스트입니다. 소리가 들리시나요? ohohohohohohohohhohohohohohohohohohoho")
        print("✅ 재생 완료 (에러 없이 끝났습니다)")
    except Exception as e:
        import traceback
        print(f"❌ TTS 재생 실패: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()