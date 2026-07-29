import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'operator_ui'

def package_files(directory):
    paths = []
    if os.path.exists(directory):
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                paths.append(os.path.join(path, filename))
    return paths

_env_source = os.path.join('resource', '.env')

# [통합] voice_processing 패키지의 wake word 모델(.tflite)이 operator_ui/resource
# 아래로 병합됨
_resource_files = glob(os.path.join('resource', '*.tflite'))
if os.path.exists(_env_source):
    _resource_files.append(_env_source)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', package_files('templates')),
        ('share/' + package_name + '/static', package_files('static')),
        ('share/' + package_name + '/resource', _resource_files),
        ("share/" + package_name + "/pointclouds/outlet/references",
            glob("pointclouds/outlet/references/*.pcd"),
        ),
        ("share/" + package_name + "/pointclouds/bolt/references",
            glob("pointclouds/bolt/references/*.pcd"),
        ),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'opencv-python',
        'python-dotenv',
        # [통합] voice_processing 의존성 (wake word / STT / TTS / 키워드 추출)
        'pyaudio',
        'openwakeword',
        'scipy',
        'sounddevice',
        'numpy',
        'openai',
        'langchain',
        'langchain-openai',
    ],
    extras_require={
        # 레거시(operator_ui/legacy) 참고용 Azure STT 노드를 계속 쓰고 싶은 경우에만 필요
        'legacy-azure': ['azure-cognitiveservices-speech'],
    },
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='CoboTender Assembly/Bartender Robot: Flask HMI + ROS2 bridge + integrated voice pipeline (wake word/STT/LLM/TTS), merged from operator_ui + voice_processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # [통합] 메인 노드: Flask HMI + ROS2 브리지 + 음성 인식(wake word/STT/LLM/TTS)을
            # 하나의 노드/프로세스에서 함께 실행한다.
            'app_node = operator_ui.app_v5:main',
            'test = operator_ui.test.test_movej:main',
            # [레거시] 참고/수동 디버깅용 - 기본 실행 흐름에는 포함되지 않음
            'legacy_stt = operator_ui.legacy.stt_node:main',
            'legacy_get_keyword = operator_ui.legacy.get_keyword_v2:main',
        ],
    },
)
