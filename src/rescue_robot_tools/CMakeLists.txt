cmake_minimum_required(VERSION 3.8)
project(rescue_robot_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/depth_to_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
