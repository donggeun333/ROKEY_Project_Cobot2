from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'outlet_assembly'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='M0609 플러그 삽입(OUTLET_ASSEMBLE)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 단독 실행용. 통합 경로는 robot_command_server가 run_plug_insert()를 직접 부른다.
            'plug_insert_standalone = outlet_assembly.plug_insert_task:main',
        ],
    },
)
