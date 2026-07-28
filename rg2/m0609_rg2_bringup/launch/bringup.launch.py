import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────
    # (virtual) ros2 launch m0609_rg2_bringup bringup.launch.py
    # (real)    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
    args = [
        DeclareLaunchArgument('mode',       default_value='virtual',     description='Operation mode: real | virtual'),
        DeclareLaunchArgument('host',       default_value='127.0.0.1',   description='Robot IP (real mode)'),
        DeclareLaunchArgument('port',       default_value='12345',        description='Robot port'),
    ]

    is_real    = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])

    # ── [virtual] DRCF 에뮬레이터 (Docker) ───────────────────────────
    # virtual mode 시 localhost에서 DRCF 에뮬레이터를 실행
    # 종료 시 terminate_drcf()로 컨테이너 자동 정리
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

    # ── 커스텀 URDF (M0609 + RG2) ────────────────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'urdf', 'm0609_with_rg2.urdf.xacro'
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
    # virtual: run_emulator_node가 먼저 Docker DRCF를 띄우고,
    #          dsr_hw_interface2 재시도 루프(0.5s × 20회)로 연결
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
    # dsr01/joint_states와 /gripper_joint_states 통합 토픽
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
        rviz_node,
    ])
