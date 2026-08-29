# AprilTag 파이프라인 launch 검증 기록

## 구성

`follower_apriltag.launch.py`는 다음 네 노드를 재현 가능한 하나의 구성으로 실행한다.

- `/follower/camera/usb_cam`: `/dev/video0`, 640x480, 30 fps, mmap, native `yuyv`
- `/follower/camera/rectify_node`: image_raw와 camera_info를 image_rect로 보정
- `/follower/apriltag/apriltag`: 36h11, tag size `0.050 m`
- `/follower/apriltag_approach`: TF 기반 상태 판단

RViz, `cmd_vel`, STM32와 그리퍼 제어는 포함하지 않는다.

## 보존한 설정

기존 원본 `/home/kde/apriltag_config/apriltag.yaml`은 수정하거나 삭제하지 않았다.
실행에는 패키지 내부 복사본 `config/apriltag.yaml`을 사용한다. 당시 설치 복사본까지 SHA-256
`cde624e98c3e6beb60ea1c0ca1f405e4728b3cb087a025f20ceb47af0f39cb45`로
동일함을 확인했다. 카메라 보정 파일
카메라 보정값은 이후 패키지 내부 `config/follower_usb_camera.yaml`로 편입했으며,
기본 launch는 작업공간 밖의 보정 파일을 참조하지 않는다.

## 자동 검증 결과

- 두 Python launch 파일 compile 성공
- `colcon build --packages-select follower_supply_perception` 성공
- 전체 launch `--show-args` 성공
  - `video_device`
  - `camera_info_url`
  - `apriltag_config`
  - `approach_config`
- approach-only `--show-args` 성공
  - `approach_config`
- install 공간에서 두 launch와 두 YAML 발견
- approach-only launch를 timeout으로 실제 실행
  - 상태 노드 시작 확인
  - `TAG_LOST` 초기 상태 확인
  - SIGINT 후 child process clean 종료 확인

샌드박스 검증에서는 ROS 기본 로그 경로 대신 허용된
`ROS_LOG_DIR=/home/kde/ros2_ws/log`를 사용했다. 실제 운영 터미널에서는 기본 ROS
로그 경로를 그대로 사용할 수 있다.

## 전체 launch 실제 실행 결과

첫 확인에서는 다음 수동 노드가 실행 중이어서 자동 종료하거나 전체 launch를 시작하지
않았다.

- `/follower/camera/usb_cam`
- `/RectifyNode`
- `/follower/apriltag/apriltag`

사용자가 수동 노드를 종료한 뒤 중복이 없음을 재확인하고 전체 launch를 timeout 제한
세션에서 실제 실행했다. 다음 네 노드가 함께 시작됐다.

- `/follower/camera/usb_cam`
- `/follower/camera/rectify_node`
- `/follower/apriltag/apriltag`
- `/follower/apriltag_approach`

실행 중 `/follower/camera/image_raw`, `camera_info`, `image_rect`,
`/follower/apriltag/detections`, 8개 상태 판단 출력과 `/tf`를 확인했다.
`image_rect`는 관찰 구간에서 약 `11.8 Hz`, AprilTag `size`는 `0.05`였다. 카메라는
`/dev/video0`, 640x480, 30 fps, mmap, native `yuyv`로 시작했다. 이 장치는 같은
해상도의 MJPEG를 120.101 fps로만 제공하므로 30 fps pipeline에서는 사용하지 않는다.

카메라가 지원하지 않는 일부 선택 제어에 `unknown control` 경고가 있었지만 영상과
필수 토픽은 정상 생성됐다. 종료 시 timeout과 명시적 interrupt가 거의 동시에 들어가
rclpy Future 소멸 경고가 한 번 출력됐으나 네 child 모두 `process has finished cleanly`로
종료됐다. 이후 프로세스 검색과 ROS node list에서 launch child가 남지 않았음을
확인했다.

## 설치 경로

- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/launch/follower_apriltag.launch.py`
- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/launch/approach_only.launch.py`
- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/apriltag.yaml`
- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml`

## 운영 명령

전체 파이프라인:

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

장치 또는 설정 경로 override:

```bash
ros2 launch follower_supply_perception follower_apriltag.launch.py \
  video_device:=/dev/video0 \
  camera_info_url:=file:///home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/follower_usb_camera.yaml \
  apriltag_config:=/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/apriltag.yaml \
  approach_config:=/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml
```

기존 카메라·AprilTag 파이프라인에 상태 노드만 추가:

```bash
ros2 launch follower_supply_perception approach_only.launch.py
```

영상 확인은 launch와 별도 터미널에서 실행한다.

```bash
ros2 run rqt_image_view rqt_image_view /follower/camera/image_rect
```
