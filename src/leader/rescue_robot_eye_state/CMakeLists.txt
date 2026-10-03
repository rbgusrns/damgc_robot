cmake_minimum_required(VERSION 3.8)
project(rescue_robot_eye_state)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/survivor_eye_state_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/eye_state/eye_classifier.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/eye_state/face_eye_roi.py
  DESTINATION share/${PROJECT_NAME}/eye_state_tools
)

ament_package()
