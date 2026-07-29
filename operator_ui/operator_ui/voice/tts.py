"""OpenAI TTS 기반 음성 안내기."""

import io
import wave
import numpy as np
import sounddevice as sd
from openai import OpenAI


class TTS:
    """텍스트를 즉시 음성으로 합성해 재생한다.

    HMI는 사용자에게 상태를 빠르게 안내해야 하므로, 파일 저장보다 즉시 재생
    중심으로 설계되어 있다.
    """

    def __init__(self, openai_api_key, voice="alloy", model="tts-1"):
        self.client = OpenAI(api_key=openai_api_key)
        self.voice = voice    # OpenAI TTS 목소리: alloy, echo, fable, onyx, nova, shimmer 중 선택
        self.model = model    # tts-1(저지연) 또는 tts-1-hd(고음질)

    def speak(self, text: str):
        """텍스트를 음성으로 합성해 즉시 재생(재생이 끝날 때까지 블로킹)."""
        print(f"🔊 [TTS 안내]: {text}")
        response = self.client.audio.speech.create(
            model=self.model,
            voice=self.voice,
            input=text,
            response_format="wav",  # wave 모듈로 바로 디코딩하기 위해 wav로 요청
        )
        audio_bytes = response.read()

        with wave.open(io.BytesIO(audio_bytes), "rb") as wf:
            samplerate = wf.getframerate()
            n_channels = wf.getnchannels()
            sampwidth = wf.getsampwidth()
            frames = wf.readframes(wf.getnframes())

        dtype_map = {1: np.int8, 2: np.int16, 4: np.int32}
        dtype = dtype_map.get(sampwidth, np.int16)
        audio_array = np.frombuffer(frames, dtype=dtype)
        if n_channels > 1:
            audio_array = audio_array.reshape(-1, n_channels)

        sd.play(audio_array, samplerate)
        sd.wait()  # 재생이 끝날 때까지 대기 (재생 중 다음 로직으로 넘어가지 않도록)
