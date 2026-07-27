from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'inspection_motion = robot_control.inspection_motion:main',
            'inspection_orchestrator = robot_control.inspection_orchestrator:main',
            'pick_bolt = robot_control.pick_bolt:main',
            'robot_command_server = robot_control.robot_command_server:main',
            'robot_control = robot_control.robot_control:main',
            'voice_command_dispatcher = robot_control.voice_command_dispatcher:main',
            'grasp_visualizer_node = robot_control.grasp_visualizer_node:main',
        ],
    },
)
