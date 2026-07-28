cmake_minimum_required(VERSION 3.8)
project(m0609_rg2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gripper_joint_state_publisher.py
    scripts/gripper_virtual_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
