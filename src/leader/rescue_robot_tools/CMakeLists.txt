cmake_minimum_required(VERSION 3.8)
project(rescue_robot_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/depth_to_csv.py
  scripts/dynamixel_orin.py
  scripts/dynamixel_orin_node.py
  scripts/gripper_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
