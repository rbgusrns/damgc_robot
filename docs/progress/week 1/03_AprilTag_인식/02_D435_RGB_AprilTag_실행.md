# AprilTag 인식 2 — D435 RGB 실행·검증

## 1. 개발 환경

- Jetson Orin Nano, ARM64
- Ubuntu 22.04
- ROS 2 Humble
- Intel RealSense D435
- D435 USB 2.x 연결
- RGB만 사용: 640x480, 목표 30 Hz
- Depth 및 infrared 스트림 비활성화
- CameraInfo 공장 보정값 사용

## 2. 실제 노드·토픽·프레임

### 노드

- 카메라: `/leader/camera`
- image_proc: `/RectifyNode`
- CameraInfo QoS bridge: `/camera_info_qos_bridge`
- AprilTag: `/leader/apriltag/apriltag`

### 입력·출력 토픽

- RGB 입력: `/leader/camera/color/image_raw`
- CameraInfo 입력: `/leader/camera/color/camera_info`
- image_proc용 bridge CameraInfo: `/leader/camera/color/camera_info_transient`
- rectified RGB: `/leader/camera/color/image_rect`
- AprilTag detections: `/leader/apriltag/detections`
- TF: `/tf`, `/tf_static`

### 실제 frame

- D435 color optical frame: `camera_color_optical_frame`
- AprilTag child frame: `tag36h11:0`
- `image_raw`와 CameraInfo의 실제 `header.frame_id`는 모두 `camera_color_optical_frame`

## 3. 태그 설정

- Family: `tag36h11`
- ID: `0`
- 실제 tag size: `0.050 m`
- Pose estimation: `pnp`
- Image transport: `raw`
- Detector threads: `4`
- Detector decimate: `1.0`
- Detector blur: `0.0`
- Detector refine: `true`
- Detector sharpening: `0.25`
- Detector debug: `false`

설정 파일:

```text
~/leader_apriltag_setup/apriltag_leader.yaml
```

## 4. 터미널별 실행 순서

모든 ROS 명령 전에 다음을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
```

### 터미널 1: D435

기존 카메라 노드를 종료하지 않은 상태에서, 재실행이 필요할 때 다음 명령을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=leader \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=false \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  rgb_camera.color_profile:=640x480x30
```

### 터미널 2: image_proc

```bash
source /opt/ros/humble/setup.bash
cd ~/leader_apriltag_setup
./run_image_proc.sh
```

이 스크립트는 CameraInfo QoS bridge를 먼저 실행한 뒤 `image_proc rectify_node`를 foreground로 실행합니다.

### 터미널 3: apriltag_ros

```bash
source /opt/ros/humble/setup.bash
cd ~/leader_apriltag_setup
./run_apriltag.sh
```

### 터미널 4: 검증

```bash
source /opt/ros/humble/setup.bash

ros2 node list | sort
ros2 topic list | sort
ros2 param get /leader/apriltag/apriltag family
ros2 param get /leader/apriltag/apriltag size
ros2 topic echo --once /leader/apriltag/detections
```

확인된 실제 TF frame을 사용합니다.

```bash
timeout 5 ros2 run tf2_ros tf2_echo \
  camera_color_optical_frame tag36h11:0
```

## 5. 정상 출력 기준

- `/leader/camera/color/image_rect`가 지속 발행됨
- `/leader/apriltag/apriltag`가 실행됨
- `family`가 `36h11`
- `size`가 `0.05`
- `/leader/apriltag/detections` 타입이 `apriltag_msgs/msg/AprilTagDetectionArray`
- detection에 `family: tag36h11`, `id: 0`이 포함됨
- `tf2_echo`에서 다음 방향의 transform이 반복 출력됨:

```text
camera_color_optical_frame -> tag36h11:0
```

Translation과 Rotation은 서로 다른 값입니다.

- Translation: `x, y, z` 위치(m)
- Quaternion: `x, y, z, w` 회전
- RPY: roll, pitch, yaw 회전각

## 6. USB 2.x 및 현재 영상 구성

D435는 USB 2.x로 연결되어 있으므로 RGB 설정 목표가 640x480x30이어도 실제 주기는 시스템 부하와 USB 대역폭에 따라 낮아질 수 있습니다. 현재 구성은 RGB 영상과 CameraInfo만 사용하며 Depth와 infrared는 비활성화되어 있습니다.

## 7. 자주 발생하는 오류와 점검 순서

### CameraInfo QoS 불일치

다음과 같은 경고가 발생할 수 있습니다.

```text
offering incompatible QoS
CameraInfo messages received: 0
```

현재 `run_image_proc.sh`는 `camera_info_qos_bridge.py`를 사용합니다. image_proc용 bridge 토픽을 확인합니다.

```bash
ros2 topic info -v /leader/camera/color/camera_info_transient
```

### image_rect가 없음

```bash
ros2 node list | sort
ps -ef | grep '[r]ectify_node'
ros2 topic info /leader/camera/color/image_rect
```

### AprilTag 노드가 없음

```bash
ros2 node list | sort
ros2 param get /leader/apriltag/apriltag family
ros2 param get /leader/apriltag/apriltag size
```

`size`가 `0.05`가 아니면 검출 결과를 사용하지 말고 YAML과 실행 인자를 먼저 점검합니다.

### detections가 비어 있음

다음 순서로 점검합니다.

1. `/leader/camera` 실행 여부
2. `/RectifyNode` 및 `/leader/camera/color/image_rect`
3. `/leader/apriltag/apriltag` 실행 여부
4. `/leader/apriltag/detections` 존재 여부
5. CameraInfo 수신 및 QoS
6. `family`, `size` 파라미터
7. 태그가 실제 영상에 보이는지

## 8. 전체 종료 및 재실행

각 foreground 터미널에서 `Ctrl+C`를 사용합니다.

1. 터미널 3의 AprilTag 노드 종료
2. 터미널 2의 image_proc 및 CameraInfo bridge 종료
3. 필요할 때만 터미널 1의 카메라 종료
4. 재실행은 터미널 1 → 터미널 2 → 터미널 3 순서

카메라 노드는 임의로 종료하거나 재시작하지 않습니다.

## 9. 거리 검증 결과

사용자가 요청하여 0.30m, 0.50m 거리 검증과 좌우·상하 이동 부호 검증은 수행하지 않았습니다. 아래 기록은 0.15m 목표 위치에서 사용자가 자로 측정한 실제 거리 0.16m에 대한 결과입니다.

`tf_distance_test.csv`에는 다음 3개 TF 샘플이 기록되어 있습니다.

```text
실제 거리: 0.160 m
평균 z:   0.170 m
오차:     +0.010 m (+1.0 cm)
상대 오차: 6.25%
```

0.15m 샘플의 대표 Translation은 다음과 같습니다.

```text
x = 0.057 m
y = 0.025 m
z = 0.170 m
```

좌우·상하 이동에 따른 x/y 부호 변화는 사용자가 직접 확인하는 항목으로 남겨 두었습니다. 따라서 이 문서는 해당 동작을 검증했다고 주장하지 않습니다.

## 10. 생성·변경 파일

- `camera_info_qos_bridge.py` — CameraInfo QoS bridge
- `run_image_proc.sh` — image_proc 및 bridge 실행 스크립트
- `apriltag_leader.yaml` — AprilTag 파라미터
- `run_apriltag.sh` — AprilTag 실행 스크립트
- `tf_distance_test.csv` — 0.15m 거리 측정 결과
- `README_RUN.md` — 재현 절차와 검증 기록
