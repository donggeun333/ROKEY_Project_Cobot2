"""실사용 표준 bringup: 로봇 + RG2 + RealSense + RViz.

이 launch 하나에 다음 구성이 모두 포함된다.

- Doosan ros2_control bringup
- RG2 실제 드라이버(real) 또는 가상 gripper 노드(virtual)
- RealSense align-depth launch
- 상태 퍼블리셔와 RViz

따라서 실기에서는 보통 ``dsr_bringup2``나 ``onrobot_rg_control``를 따로 또
띄우지 않는다. 이 파일이 현재 프로젝트의 "하드웨어 기본 기동점"이다.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────
    # (virtual) ros2 launch m0609_rg2_bringup bringup_camera.launch.py
    # (real)    ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100
    args = [
        DeclareLaunchArgument('mode',       default_value='virtual',     description='Operation mode: real | virtual'),
        DeclareLaunchArgument('host',       default_value='127.0.0.1',   description='Robot IP (real mode)'),
        DeclareLaunchArgument('port',       default_value='12345',        description='Robot port'),
        DeclareLaunchArgument('camera',     default_value='false',       description='Force-spawn RealSense even in virtual mode (demo opt-in)'),
    ]

    is_real    = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])

    # ── [virtual] DRCF 에뮬레이터 (Docker) ───────────────────────────
    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace='dsr01',
        parameters=[
            {'name':    'dsr01'                      },
            {'host':    LaunchConfiguration('host')  },
            {'port':    LaunchConfiguration('port')  },
            {'mode':    LaunchConfiguration('mode')  },
            {'model':   'm0609'                      },
            {'gripper': 'none'                       },
            {'mobile':  'none'                       },
        ],
        condition=IfCondition(is_virtual),
        output='screen',
    )

    # ── [virtual] 이전 run 잔여 에뮬레이터 컨테이너 정리 ──────────────
    # run_drcf.sh의 중복 컨테이너 체크는 'docker ps -q'(running 상태만) 기반이라
    # Exited 상태로 남은 --rm 미정리 컨테이너를 놓친다. 그 경우 다음 bringup의
    # 'docker run --name dsr01_emulator'가 이름 충돌로 실패 → 에뮬레이터 미기동 →
    # ros2_control 하드웨어 init 실패로 연쇄. run_emulator 시작 전 동명 컨테이너를
    # 강제 제거해 launch를 idempotent하게 만든다.
    emulator_cleanup = ExecuteProcess(
        cmd=['bash', '-c', 'docker rm -f dsr01_emulator 2>/dev/null || true'],
        condition=IfCondition(is_virtual),
        output='log',
    )
    start_emulator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=emulator_cleanup,
            on_exit=[run_emulator_node],
        ),
    )

    # ── 커스텀 URDF (M0609 + RG2 + RealSense) ────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'urdf', 'm0609_with_rg2_camera.urdf.xacro'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'rviz', 'default.rviz'
    )

    # ── [real] Doosan URDF (ros2_control 하드웨어 인터페이스용) ───────
    doosan_xacro = PathJoinSubstitution([
        FindPackageShare('dsr_description2'), 'xacro', 'm0609.urdf.xacro'
    ])
    doosan_robot_description = Command([
        FindExecutable(name='xacro'), ' ', doosan_xacro,
        ' name:=dsr01',
        ' host:=', LaunchConfiguration('host'),
        ' port:=', LaunchConfiguration('port'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=m0609',
        ' update_rate:=100',
    ])

    # ── ros2_control_node (virtual/real 공통) ─────────────────────────
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='dsr01',
        parameters=[
            {'robot_description': ParameterValue(doosan_robot_description, value_type=str)},
            {'update_rate': 100},
            PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml']),
        ],
        output='both',
    )

    # ── joint_state_broadcaster (/dsr01/joint_states 퍼블리시) ────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='dsr01',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
    )

    # ── dsr_controller2 (motion service 등록) ─────────────────────────
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='dsr01',
        arguments=['dsr_controller2', '-c', 'controller_manager'],
    )
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
    )

    # ── [virtual] GripperVirtualNode (/onrobot/sendCommand 서비스) ───
    is_virtual_gripper = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])
    gripper_virtual_node = Node(
        package='m0609_rg2_bringup',
        executable='gripper_virtual_node.py',
        name='gripper_virtual_node',
        condition=IfCondition(is_virtual_gripper),
        output='screen',
    )

    # ── [real] OnRobot RG2 드라이버 ──────────────────────────────────
    # /joint_states → /onrobot_joint_states 로 remap (joint_state_publisher와 충돌 방지)
    onrobot_driver = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGControllerServer',
        name='OnRobotRGControllerServer',
        output='screen',
        parameters=[{
            '/onrobot/control':      'modbus',
            '/onrobot/ip':           '192.168.1.1',
            '/onrobot/port':         502,
            '/onrobot/changer_addr': 65,
            '/onrobot/gripper':      'rg2',
            '/onrobot/offset':       5,
        }],
        remappings=[('/joint_states', '/onrobot_joint_states')],
        condition=IfCondition(is_real),
    )

    # ── [real] 그리퍼 너비 → rg2_finger_joint 변환 노드 ──────────────
    # OnRobotRGInput.ggwd → /gripper_joint_states (rg2_finger_joint)
    gripper_joint_state_publisher = Node(
        package='m0609_rg2_bringup',
        executable='gripper_joint_state_publisher.py',
        name='gripper_joint_state_publisher',
        condition=IfCondition(is_real),
        output='screen',
    )

    # ── joint_state_publisher (virtual/real 공통) ─────────────────────
    # dsr01/joint_states와 /gripper/joint_states 통합 토픽
    # virtual 환경에서는 /gripper_joint_states 없음(DRCF 문제) → gripper joint 0으로 채워짐
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['/dsr01/joint_states', '/gripper_joint_states']}],
    )

    # ── robot_state_publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )
        }],
    )

    # ── Static TF (world → base_link) ────────────────────────────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
    )

    # ── RealSense 카메라 드라이버 (real 모드 또는 camera:=true 일 때) ──
    # 기본은 real 모드에서만 spawn (가상 시뮬에 USB 카메라 묶지 않기 위함).
    # 데모/홍보 영상처럼 가상 모드에서도 카메라 영상이 필요하면 camera:=true 로 opt-in.
    camera_enabled = PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'real' or '",
        LaunchConfiguration('camera'), "' == 'true'"
    ])
    # 프로젝트 기본 제공 명령과 동일한 공식 align-depth launch를 사용한다.
    # realsense2_camera_node를 일부 파라미터만 지정해 직접 실행하면 D435i의
    # Infra/Motion Module까지 활성화될 수 있으며, IIO 권한이 없는 환경에서는
    # Motion Module 초기화 실패와 장치 재연결 루프가 발생한다.
    realsense_launch = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'),
                'examples', 'align_depth', 'rs_align_depth_launch.py'
            )),
            launch_arguments={
                'depth_module.depth_profile': '848x480x30',
                'rgb_camera.color_profile': '1280x720x30',
                'initial_reset': 'true',
                'align_depth.enable': 'true',
                'enable_rgbd': 'true',
                'pointcloud.enable': 'true',
                # 카메라 프레임은 m0609_with_rg2_camera URDF가 발행한다.
                'publish_tf': 'false',
            }.items(),
        )],
        # 로봇 launch 인자(mode/host 등)가 RealSense 쪽으로 전달되지 않게 한다.
        scoped=True,
        forwarding=False,
        condition=IfCondition(camera_enabled),
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription(args + [
        emulator_cleanup,
        start_emulator,
        gripper_virtual_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_controller,
        onrobot_driver,
        gripper_joint_state_publisher,
        joint_state_publisher_node,
        robot_state_publisher,
        static_tf,
        realsense_launch,
        rviz_node,
    ])
