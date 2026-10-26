# 리더 단독 자동 집기

팔로워 dd93e7c의 공용 집기/모터 코드와 카메라 영상·보정정보 동기화를 이식했다.
리더 ID 2, 열림 1000, 집기 477, RX-28 속도 40, 열기/닫기 대기 각각 9초.
태그 36h11 ID 0, 크기 0.050m. 카메라 목표 거리 0.20m(집기 허용 ±0.02m)로 실기 시험했다.
URDF 카메라 전방 오프셋 0.042m를 반영해 base_link 목표를 0.242m로 설정했다.
최고 접근 0.04m/s, 최종 접근 0.03m/s, 거리 제어 게인0.40.
base_link 최종 거리 오차 허용을 0.008m로 줄여 20cm 목표에 더 가까이 접근한다.
리프트와 태그 유실 후 전진은 비활성화했다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_tools rescue_robot_apriltag rescue_robot_bringup
source install/setup.bash
ros2 launch rescue_robot_bringup leader_grasp_demo.launch.py
```

별도 터미널에서 같은 환경을 source한 뒤:

```bash
ros2 service call /leader/grasp/start std_srvs/srv/Trigger '{}'
ros2 topic echo /leader/sequence/status
ros2 service call /leader/grasp/cancel std_srvs/srv/Trigger '{}'
```

시작 전에는 selector STOP, controller/guard 비활성화 상태다.
시작 요청 후 열기 → 태그 중심 접근 → 정렬과 거리 재확인 → guard OFF/selector STOP → 477 집기 → DONE.
DONE 이후 다시 시작 요청을 하기 전에는 재열리지 않는다. 취소 시 베이스를 정지하고 그리퍼 목표는 유지한다.
DONE은 모터 명령 ACK와 대기 완료이며 상자 접촉 센서 판정은 아니다.

기존 매핑이 카메라/TF/selector/bridge를 소유하고 있다면 해당 리소스만 재사용한다:

```bash
ros2 launch rescue_robot_bringup leader_grasp_demo.launch.py \
  start_camera:=false start_robot_state_publisher:=false \
  start_command_selector:=false use_stm32_bridge:=false
```

기존 AprilTag/운반/그리퍼 노드는 중복 실행하지 않는다.

## 첫 리더 실기 결과

- Dynamixel launch가 STM32 launch의 baudrate=230400을 상속하던 문제를 발견해 115200을 명시했다.
- RX-28 속도40 설정과 열림1000 명령 ACK 성공.
- 카메라 기준 약0.367m에서 자동 접근, 마지막 수신 거리는 약0.227m.
- 첫 시험은 근거리 태그 검출이 불안정해 집기 전 취소했다.
- 취소 응답과 /leader/cmd_vel 선속도/각속도 모두0을 확인했다.
- 정지 상태에서도 영상은 수신되지만 detector가 빈 검출을 내는 구간을 관측했다.

## 대기·접근 조정 후 재시험

- 열림/닫힘 대기를 각각25초에서9초로 줄이고 거리 게인을0.40, 최종 최고속도를0.03m/s로 조정했다.
- 시작 서비스 한 번으로 약0.380m에서 접근해 약0.206m에서 정지했다.
- 거리·정렬 재확인 후 집기477 명령의 모터 ACK를 수신하고 DONE까지 도달했다.
- 마지막 카메라 거리 약0.2055m. 리프트는 사용하지 않았다.
- 전체 약29.5초: 열기 약9초, 접근·정렬 약12초, 닫기 약9초.
- 완료 후 /leader/cmd_vel 선속도·각속도0을 확인했고 사용자가 결과를 확인했다.
- 접촉·미끄러짐 센서 피드백과 리프트·운반은 이번 검증 범위에 포함하지 않는다.
- 관련 자동 테스트309개 및3개 패키지 빌드 통과.
