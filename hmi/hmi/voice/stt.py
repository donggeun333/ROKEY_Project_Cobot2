"""OpenAI Whisper 기반 음성-텍스트 변환기."""

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile

class STT:
    """마이크에서 짧게 녹음한 뒤 Whisper API로 텍스트를 얻는다."""

    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        # self.openai_api_key = openai_api_key
        self.duration = 4  # seconds
        self.samplerate = 16000  # Whisper는 16kHz를 선호


    def speech2text(self):
        """지정 시간만큼 녹음하고 STT 결과 문자열을 반환한다."""

        # 녹음 설정
        print("음성 녹음을 시작합니다. \n 4초 동안 말해주세요...")
        audio = sd.rec(int(self.duration * self.samplerate), samplerate=self.samplerate, channels=1, dtype='int16')
        sd.wait()
        print("녹음 완료. Whisper에 전송 중...")

        # 임시 WAV 파일 저장
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)

            # Whisper API 호출
            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f)

        #print("STT 결과: ", transcript['text'])
        print("STT 결과: ", transcript.text)
        #return transcript['text']
        return transcript.text
