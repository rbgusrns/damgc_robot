cmake_minimum_required(VERSION 3.8)
project(rescue_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/arrow_key_teleop.py
  scripts/vslam_covariance_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
