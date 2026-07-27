cmake_minimum_required(VERSION 3.8)
project(rescue_robot_apriltag)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/camera_info_qos_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
