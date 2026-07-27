cmake_minimum_required(VERSION 3.8)
project(voice_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 이 패키지에 정의된 .srv, .action 파일들을 실제 ROS2 메시지/서비스/액션 코드로
# 자동 생성(codegen)하도록 등록하는 부분입니다.
# colcon build 시 이 목록에 있는 파일들이 Python/C++ 클래스로 변환됩니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetKeyword.srv"
  "action/RobotCommand.action"
)

ament_package()
