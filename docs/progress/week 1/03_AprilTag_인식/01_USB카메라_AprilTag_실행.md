# AprilTag 인식 1 — USB 카메라 실행·검증

## 1. 문서 목적

Jetson Orin Nano에서 일반 USB 카메라 영상을 ROS 2로 발행하고, 카메라 보정값을 적용한 뒤 `apriltag_ros`를 사용하여 **tag36h11 ID 0의 상대 위치 TF**를 출력한 최종 성공 절차를 간단히 정리한 문서이다.

개발 중 반복 실행하거나 실패했던 과정은 제외하고, 정상 동작에 필요한 설정과 명령만 포함한다.

---

## 2. 최종 완료 상태

```text
USB 카메라
→ /follower/camera/image_raw 발행
→ /follower/camera/camera_info 발행
→ image_proc로 /follower/camera/image_rect 생성
→ apriltag_ros로 tag36h11 ID 0 검출
→ follower_camera_optical_frame → tag36h11:0 TF 출력
```

현재 설정은 다음과 같다.

| 항목 | 설정값 |
|---|---|
| 운영체제 | Ubuntu 22.04 |
| ROS 2 | Humble |
| USB 카메라 장치 | `/dev/video0` |
| 해상도 | 640×480 |
| 프레임 속도 | 30 fps |
| 카메라 namespace | `/follower/camera` |
| AprilTag namespace | `/follower/apriltag` |
| 카메라 optical frame | `follower_camera_optical_frame` |
| 태그 family | `36h11` |
| 태그 ID | `0` |
| 태그 실제 크기 | `0.050 m` |
| 태그 TF frame | `tag36h11:0` |

---

## 3. 주요 파일

### 카메라 보정 파일

```text
~/.ros/camera_info/follower_usb_camera.yaml
```

이 파일은 현재 USB 카메라의 **640×480 해상도 전용 보정값**이다. 다른 카메라나 다른 해상도에 그대로 사용하면 안 된다.

### AprilTag 설정 파일

```text
~/apriltag_config/apriltag.yaml
```

최종 내용:

```yaml
/follower/apriltag/apriltag:
  ros__parameters:
    image_transport: raw
    family: 36h11
    size: 0.050
    max_hamming: 0

    detector:
      threads: 4
      decimate: 1.0
      blur: 0.0
      refine: true
      sharpening: 0.25
      debug: false

    pose_estimation_method: "pnp"
```

중요 사항:

- YAML 최상단 노드 이름은 실제 노드 이름인 `/follower/apriltag/apriltag`와 일치해야 한다.
- `refine`은 `1`이 아니라 `true`로 작성한다.
- `debug`는 `0`이 아니라 `false`로 작성한다.
- `size: 0.050`은 태그의 흰색 외부 여백을 제외한 검출 정사각형 한 변의 실제 길이이다.

---

## 4. 최초 구축 시 수행한 작업

### 4.1 필요한 패키지 설치

```bash
sudo apt -o Acquire::ForceIPv4=true \
  -o Acquire::Retries=5 \
  install -y \
  ros-humble-apriltag-ros \
  ros-humble-image-proc \
  ros-humble-camera-calibration \
  ros-humble-rqt-image-view
```

### 4.2 USB 카메라 보정

사용한 체스보드:

- 사각형 배열: 9×7
- 내부 교차점: 8×6
- 한 칸 길이: 25 mm
- 보정 해상도: 640×480

보정 프로그램 실행 명령:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/follower/camera/image_raw \
  camera:=/follower/camera
```

보정 결과의 `ost.yaml`을 다음 위치에 저장하였다.

```text
~/.ros/camera_info/follower_usb_camera.yaml
```

그리고 파일 안의 카메라 이름을 다음과 같이 맞췄다.

```yaml
camera_name: follower_usb_camera
```

### 4.3 CameraInfo 적용 확인

```bash
ros2 topic echo /follower/camera/camera_info --once
```

정상 상태에서는 다음 값이 비어 있지 않아야 한다.

- `distortion_model`
- `d`
- `k`
- `p`
- `frame_id: follower_camera_optical_frame`

---

## 5. Jetson 재부팅 후 실행 순서

각 명령은 **서로 다른 터미널**에서 실행한다. 모든 새 터미널에서는 먼저 ROS 2 환경을 불러온다.

```bash
source /opt/ros/humble/setup.bash
```

## 터미널 1: USB 카메라 실행

역할:

- USB 카메라 영상 발행
- 카메라 보정 정보 발행

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/follower/camera \
  -r __node:=usb_cam \
  -p video_device:=/dev/video0 \
  -p framerate:=30.0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p pixel_format:=mjpeg2rgb \
  -p frame_id:=follower_camera_optical_frame \
  -p camera_name:=follower_usb_camera \
  -p camera_info_url:="file:///home/kde/.ros/camera_info/follower_usb_camera.yaml"
```

생성되는 주요 토픽:

```text
/follower/camera/image_raw
/follower/camera/camera_info
```

---

## 터미널 2: 왜곡 보정 영상 생성

역할:

- `image_raw`와 `camera_info`를 사용하여 보정 영상 생성

```bash
ros2 run image_proc rectify_node --ros-args \
  -r image:=/follower/camera/image_raw \
  -r camera_info:=/follower/camera/camera_info \
  -r image_rect:=/follower/camera/image_rect
```

생성되는 토픽:

```text
/follower/camera/image_rect
```

이 명령만 실행하면 영상 창은 자동으로 열리지 않는다.

---

## 터미널 3: 영상 화면 표시

역할:

- ROS 2 영상 토픽을 화면에 표시

```bash
ros2 run rqt_image_view rqt_image_view
```

창이 열리면 다음 토픽을 선택한다.

```text
/follower/camera/image_rect
```

원본 영상을 보고 싶으면 다음 토픽을 선택한다.

```text
/follower/camera/image_raw
```

---

## 터미널 4: AprilTag 검출 실행

역할:

- 보정된 색상 영상에서 AprilTag 검출
- 태그 ID와 TF 발행

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r __ns:=/follower/apriltag \
  -r image_rect:=/follower/camera/image_rect \
  -r camera_info:=/follower/camera/camera_info \
  --params-file ~/apriltag_config/apriltag.yaml
```

검출 토픽:

```text
/follower/apriltag/detections
```

---

## 터미널 5: 설정 및 TF 확인

### 태그 크기 확인

```bash
ros2 param get /follower/apriltag/apriltag size
```

정상 결과:

```text
Double value is: 0.05
```

### AprilTag 검출 확인

```bash
ros2 topic echo /follower/apriltag/detections
```

태그를 카메라에 보여주었을 때 ID와 검출 데이터가 출력되어야 한다.

### AprilTag 상대 위치 TF 출력

```bash
ros2 run tf2_ros tf2_echo \
  follower_camera_optical_frame \
  'tag36h11:0'
```

정상 상태에서는 다음 값이 반복 출력된다.

```text
Translation: [x, y, z]
Rotation: quaternion 또는 RPY
```

명령 시작 직후 태그가 아직 인식되지 않았다면 `frame does not exist`가 잠시 나올 수 있다. 이후 Translation과 Rotation이 출력되면 정상이다.

---

## 6. TF 좌표 해석

카메라 optical frame 기준:

| 값 | 의미 |
|---|---|
| `x` | 영상 오른쪽 방향. 왼쪽은 음수, 오른쪽은 양수 |
| `y` | 영상 아래쪽 방향. 위는 음수, 아래는 양수 |
| `z` | 카메라 정면 방향 거리 |

예시:

```text
Translation: [-0.016, 0.033, 0.143]
```

해석:

- 태그가 화면 중심보다 왼쪽 약 1.6 cm
- 태그가 화면 중심보다 아래쪽 약 3.3 cm
- 태그가 카메라 전방 약 14.3 cm

이후 제어 노드에서는 다음 값을 사용할 수 있다.

```text
전방 거리 = z
좌우 오차 = x
직선거리 = sqrt(x² + y² + z²)
수평 방향각 = atan2(x, z)
```

---

## 7. 정상 동작 점검

다음 항목을 확인한다.

```bash
ros2 topic hz /follower/camera/image_raw
ros2 topic hz /follower/camera/image_rect
ros2 topic echo /follower/camera/camera_info --once
ros2 param get /follower/apriltag/apriltag size
ros2 topic echo /follower/apriltag/detections
ros2 run tf2_ros tf2_echo follower_camera_optical_frame 'tag36h11:0'
```

정상 기준:

- `image_raw`와 `image_rect`가 지속적으로 발행됨
- `camera_info`의 `D`, `K`, `P`가 비어 있지 않음
- AprilTag `size`가 `0.05`
- ID 0 검출 데이터가 출력됨
- 태그를 가까이·멀리 움직이면 `z`가 자연스럽게 변함
- 태그를 좌우로 움직이면 `x`의 부호가 변함

---

## 8. 종료 방법

각 프로그램을 실행한 터미널에서 다음 키를 누른다.

```text
Ctrl + C
```

종료 순서는 크게 중요하지 않지만 다음 순서를 권장한다.

```text
TF 확인 종료
→ AprilTag 노드 종료
→ image_proc 종료
→ rqt_image_view 종료
→ usb_cam 종료
```

---

## 9. RealSense D435로 이식할 때

D435에서도 기본 구조는 같다.

```text
D435 color image
+ color CameraInfo
→ apriltag_ros
→ tag36h11 ID 0 검출
→ color optical frame 기준 TF 출력
```

차이점:

- `usb_cam` 대신 `realsense2_camera` 드라이버를 사용한다.
- AprilTag 검출에는 Depth 영상이 아니라 **RGB 색상 영상**을 사용한다.
- D435가 제공하는 color `CameraInfo`와 color optical frame 이름을 실제 토픽에서 확인해야 한다.
- 공장 보정 CameraInfo가 정상이라면 USB 카메라처럼 체스보드 보정을 다시 하지 않아도 될 수 있다.
- AprilTag 설정의 `family: 36h11`, `size: 0.050`은 동일하게 사용할 수 있다.
- 실제 토픽과 노드 이름에 맞게 namespace, remapping, YAML 최상단 키를 변경해야 한다.

---

## 10. 다음 개발 단계

현재 USB 카메라 쪽은 **AprilTag TF 출력까지 완료**되었다.

다음 단계는 TF의 `x`, `y`, `z`를 읽어 다음 기능을 구현하는 것이다.

```text
상대 위치 계산
→ 방향 및 거리 오차 판단
→ TURN_LEFT / TURN_RIGHT / APPROACH / ALIGNED 상태 생성
→ 저속 cmd_vel 생성
→ 안정적으로 정렬된 뒤 gripper close 요청
```
