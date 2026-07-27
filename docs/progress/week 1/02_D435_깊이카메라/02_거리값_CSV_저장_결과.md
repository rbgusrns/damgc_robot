# D435 깊이카메라 2 — 거리값 CSV 저장 결과

## 1. 작업 목적

Intel RealSense D435에서 발행되는 뎁스 영상 토픽을 ROS 2 Python 노드로 구독하고, 영상 중앙 영역의 대표 거리값을 계산하여 CSV 파일로 저장한 과정을 정리한 문서이다.

이 문서는 다음 내용을 중심으로 정리하였다.

- 실제로 사용한 명령어의 의미
- `depth_to_csv.py` 코드의 주요 동작
- 실행 중 출력된 결과의 의미
- 최종 저장 결과

---

## 2. 전체 동작 구조

```text
RealSense D435
→ 뎁스 영상 토픽 발행
→ depth_to_csv.py가 토픽 구독
→ 영상 중앙 20×20 픽셀 영역 추출
→ 유효하지 않은 거리값 제거
→ 유효값의 중앙값 계산
→ 거리 단위를 m로 변환
→ 1초마다 CSV 파일에 저장
```

사용한 뎁스 영상 토픽:

```text
/camera/camera/depth/image_rect_raw
```

CSV 저장 위치:

```text
/home/maze/depth_distance.csv
```

---

## 3. RealSense 카메라 실행

### 입력한 명령어

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### 명령어 의미

#### `source /opt/ros/humble/setup.bash`

현재 터미널에서 ROS 2 Humble의 명령어, 패키지, 환경변수를 사용할 수 있도록 설정을 불러오는 명령어이다.

#### `ros2 launch realsense2_camera rs_launch.py`

RealSense D435 카메라 노드를 실행하여 컬러 영상과 뎁스 영상을 ROS 2 토픽으로 발행하는 명령어이다.

거리값 저장 프로그램은 카메라가 발행하는 뎁스 토픽을 받아서 동작하기 때문에, 이 카메라 노드가 먼저 실행되어 있어야 한다.

---

## 4. 거리값 저장 코드 파일 확인

### 입력한 명령어

```bash
ls -l ~/depth_to_csv.py
```

### 명령어 의미

사용자의 홈 폴더인 `/home/maze`에 `depth_to_csv.py` 파일이 존재하는지 확인하는 명령어이다.

처음 실행 결과:

```text
ls: cannot access '/home/maze/depth_to_csv.py': No such file or directory
```

### 결과 해석

`/home/maze/depth_to_csv.py` 경로에 코드 파일이 아직 존재하지 않는다는 뜻이다.

카메라나 ROS 2의 오류가 아니라, 실행할 Python 코드 파일이 Jetson에 복사되지 않은 상태였다는 의미이다.

이후 `depth_to_csv.py` 파일을 Jetson의 다음 위치로 복사하였다.

```text
/home/maze/depth_to_csv.py
```

---

## 5. 거리값 저장 코드 실행

### 입력한 명령어

```bash
python3 ~/depth_to_csv.py
```

### 명령어 의미

Python 3로 `/home/maze/depth_to_csv.py` 파일을 실행하는 명령어이다.

코드가 실행되면 ROS 2 노드가 생성되고, 다음 뎁스 영상 토픽을 구독한다.

```text
/camera/camera/depth/image_rect_raw
```

구독한 뎁스 영상에서 중앙 영역의 거리값을 계산하여 CSV 파일에 저장한다.

---

# 6. `depth_to_csv.py` 코드 해석

## 6.1 사용한 라이브러리

```python
import csv
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
```

각 라이브러리의 역할은 다음과 같다.

| 라이브러리 | 역할 |
|---|---|
| `csv` | 측정한 거리값을 CSV 파일로 저장 |
| `math` | 거리값이 정상적인 숫자인지 확인 |
| `datetime` | 측정 시각 생성 |
| `Path` | 저장 파일 경로 관리 |
| `numpy` | 뎁스 영상 배열 처리 및 중앙값 계산 |
| `rclpy` | ROS 2 Python 노드 실행 |
| `CvBridge` | ROS 2 Image 메시지를 NumPy 영상으로 변환 |
| `Node` | ROS 2 노드 클래스 생성 |
| `qos_profile_sensor_data` | 카메라와 같은 센서 데이터에 적합한 QoS 설정 |
| `sensor_msgs.msg.Image` | 뎁스 영상 토픽의 메시지 형식 |

---

## 6.2 ROS 2 노드 정의

```python
class DepthCsvSaver(Node):
```

`DepthCsvSaver`라는 ROS 2 노드를 정의한다.

이 노드는 다음 작업을 수행한다.

```text
뎁스 토픽 구독
→ 거리값 계산
→ 터미널 출력
→ CSV 저장
```

노드 이름은 다음과 같이 설정된다.

```python
super().__init__("depth_csv_saver")
```

실행 로그에 표시된 다음 이름이 이 노드 이름이다.

```text
[depth_csv_saver]
```

---

## 6.3 주요 설정값

```python
TOPIC = "/camera/camera/depth/image_rect_raw"
OUTPUT_PATH = Path.home() / "depth_distance.csv"

ROI_SIZE = 20
SAVE_INTERVAL_SEC = 1.0
MIN_DISTANCE_M = 0.10
MAX_DISTANCE_M = 10.0
```

### `TOPIC`

```text
/camera/camera/depth/image_rect_raw
```

RealSense D435가 발행하는 원본 뎁스 영상 토픽이다.

각 픽셀에 해당 위치까지의 깊이값이 들어 있다.

### `OUTPUT_PATH`

```text
/home/maze/depth_distance.csv
```

측정한 거리값이 저장되는 CSV 파일 경로이다.

### `ROI_SIZE = 20`

영상 중앙을 기준으로 가로 20픽셀, 세로 20픽셀 영역을 사용한다.

전체 픽셀 수:

```text
20 × 20 = 400개
```

단일 중앙 픽셀 한 개만 사용하지 않고 주변 영역을 함께 사용하는 이유는 순간적인 노이즈나 잘못된 픽셀값의 영향을 줄이기 위해서이다.

### `SAVE_INTERVAL_SEC = 1.0`

거리값을 1초마다 한 번씩 CSV 파일에 저장한다.

카메라 뎁스 영상은 초당 약 13~14회 들어오지만, 모든 프레임을 저장하지 않고 1초 간격으로 대표값을 저장하도록 설정하였다.

### 거리값 허용 범위

```python
MIN_DISTANCE_M = 0.10
MAX_DISTANCE_M = 10.0
```

다음 값은 대표 거리 계산에서 제외한다.

```text
0.10 m 미만
10.0 m 초과
0
NaN
무한대
```

잘못 측정되거나 신뢰하기 어려운 값을 제외하기 위한 조건이다.

---

## 6.4 CSV 파일 생성

```python
self.csv_file = self.OUTPUT_PATH.open(
    "a", newline="", encoding="utf-8-sig"
)
```

CSV 파일을 추가 모드인 `a`로 연다.

따라서 프로그램을 다시 실행해도 기존 파일을 삭제하지 않고 새로운 측정값을 아래에 이어서 저장한다.

CSV 파일이 처음 생성된 경우 다음 헤더를 작성한다.

```python
[
    "timestamp",
    "frame_number",
    "distance_m",
    "valid_pixel_count",
    "center_x",
    "center_y",
    "encoding",
]
```

CSV 컬럼의 의미는 다음과 같다.

| 컬럼 | 의미 |
|---|---|
| `timestamp` | 거리값을 저장한 실제 시각 |
| `frame_number` | 저장된 데이터의 순서 |
| `distance_m` | 계산된 대표 거리값, 단위 m |
| `valid_pixel_count` | 대표 거리 계산에 사용된 유효 픽셀 수 |
| `center_x` | 영상 중심의 x좌표 |
| `center_y` | 영상 중심의 y좌표 |
| `encoding` | 수신한 뎁스 영상의 데이터 형식 |

---

## 6.5 뎁스 토픽 구독

```python
self.subscription = self.create_subscription(
    Image,
    self.TOPIC,
    self.depth_callback,
    qos_profile_sensor_data,
)
```

ROS 2의 `Image` 형식으로 다음 토픽을 구독한다.

```text
/camera/camera/depth/image_rect_raw
```

새로운 뎁스 영상이 들어올 때마다 다음 함수가 호출된다.

```python
self.depth_callback
```

카메라 영상처럼 데이터가 빠르게 들어오는 센서 토픽이므로 `qos_profile_sensor_data`를 사용하였다.

---

## 6.6 저장 주기 제한

```python
elapsed = (now - self.last_saved_time).nanoseconds / 1e9

if elapsed < self.SAVE_INTERVAL_SEC:
    return
```

새로운 뎁스 영상은 초당 약 13~14회 들어오지만, 마지막 저장 이후 1초가 지나지 않았다면 해당 프레임은 저장하지 않는다.

따라서 CSV에는 1초마다 한 줄씩 기록된다.

---

## 6.7 ROS Image를 NumPy 배열로 변환

```python
depth_image = self.bridge.imgmsg_to_cv2(
    msg, desired_encoding="passthrough"
)
```

ROS 2의 `sensor_msgs/Image` 메시지를 Python에서 계산할 수 있는 NumPy 배열로 변환한다.

`passthrough`를 사용하므로 카메라가 보내는 뎁스 영상 형식을 그대로 유지한다.

---

## 6.8 영상 중앙 영역 계산

```python
height, width = depth_image.shape
center_x = width // 2
center_y = height // 2
```

현재 영상 해상도는 640×480이므로 중심 좌표는 다음과 같다.

```text
center_x = 320
center_y = 240
```

중앙을 기준으로 20×20 픽셀 영역을 추출한다.

```python
depth_roi = depth_image[y1:y2, x1:x2]
```

`ROI`는 관심 영역을 뜻하며, 여기서는 카메라 화면 중앙 부분을 의미한다.

---

## 6.9 거리 단위 변환

RealSense의 뎁스 토픽이 `16UC1` 또는 `MONO16` 형식이면 다음 계산을 수행한다.

```python
depth_roi.astype(np.float32) * 0.001
```

RealSense 뎁스값은 일반적으로 밀리미터 단위이므로 `0.001`을 곱해 미터 단위로 변환한다.

예시:

```text
458 mm × 0.001 = 0.458 m
```

`32FC1` 형식인 경우에는 이미 미터 단위이므로 별도의 단위 변환 없이 사용한다.

---

## 6.10 유효하지 않은 값 제거

```python
valid_mask = (
    np.isfinite(depth_m)
    & (depth_m >= self.MIN_DISTANCE_M)
    & (depth_m <= self.MAX_DISTANCE_M)
)
```

다음 조건을 모두 만족하는 픽셀만 사용한다.

```text
숫자가 정상적으로 존재함
거리 0.10 m 이상
거리 10.0 m 이하
```

조건을 통과한 값은 다음 배열에 저장된다.

```python
valid_values = depth_m[valid_mask]
```

---

## 6.11 대표 거리값 계산

```python
distance_m = float(np.median(valid_values))
```

유효한 거리값들의 중앙값을 대표 거리로 사용한다.

중앙값은 400개 거리값을 크기순으로 나열했을 때 가운데 위치한 값이다.

평균값보다 순간적인 이상값이나 노이즈에 영향을 덜 받기 때문에 카메라 중앙 물체까지의 대표 거리를 계산하기에 적합하다.

---

## 6.12 CSV 저장

```python
self.csv_writer.writerow(
    [
        timestamp,
        self.frame_number,
        f"{distance_m:.4f}",
        int(valid_values.size),
        center_x,
        center_y,
        msg.encoding,
    ]
)
```

계산한 결과를 CSV 파일에 한 줄씩 저장한다.

저장되는 데이터 예시는 다음과 같다.

```csv
timestamp,frame_number,distance_m,valid_pixel_count,center_x,center_y,encoding
2026-07-20 12:15:41.290,1,0.4580,400,320,240,16UC1
```

`flush()`를 사용하여 측정값을 바로 파일에 기록한다.

```python
self.csv_file.flush()
```

프로그램이 실행되는 중에 예기치 않게 종료되더라도 이미 측정한 데이터가 파일에 남을 가능성을 높이기 위한 처리이다.

---

## 6.13 터미널 출력

```python
self.get_logger().info(
    f"{self.frame_number:05d} | 거리 {distance_m:.3f} m | "
    f"유효 픽셀 {valid_values.size}"
)
```

CSV 저장과 동시에 현재 측정 결과를 터미널에도 출력한다.

출력 예시:

```text
00001 | 거리 0.458 m | 유효 픽셀 400
```

---

## 7. 실제 실행 결과

프로그램을 실행했을 때 다음 내용이 출력되었다.

```text
[INFO] [1784518138.727654726] [depth_csv_saver]:
구독 토픽: /camera/camera/depth/image_rect_raw

[INFO] [1784518138.728668894] [depth_csv_saver]:
CSV 저장 위치: /home/maze/depth_distance.csv

[INFO] [1784518138.729431312] [depth_csv_saver]:
중앙 20×20 영역의 중앙값을 1.0초마다 저장합니다.

[INFO] [1784518141.290701360] [depth_csv_saver]:
00001 | 거리 0.458 m | 유효 픽셀 400

[INFO] [1784518142.356037112] [depth_csv_saver]:
00002 | 거리 0.458 m | 유효 픽셀 400

[INFO] [1784518143.355993958] [depth_csv_saver]:
00003 | 거리 0.459 m | 유효 픽셀 400
```

---

## 8. 실행 결과 해석

### 구독 토픽 확인

```text
구독 토픽: /camera/camera/depth/image_rect_raw
```

거리 저장 노드가 RealSense D435의 뎁스 영상 토픽을 정상적으로 구독하도록 설정되었다는 뜻이다.

---

### CSV 저장 위치 확인

```text
CSV 저장 위치: /home/maze/depth_distance.csv
```

거리 측정 결과가 다음 파일에 저장된다는 뜻이다.

```text
/home/maze/depth_distance.csv
```

---

### 측정 영역 및 저장 주기

```text
중앙 20×20 영역의 중앙값을 1.0초마다 저장합니다.
```

영상 정중앙의 20×20 픽셀 영역에서 유효한 깊이값을 골라 중앙값을 계산하고, 1초마다 CSV에 기록한다는 뜻이다.

---

### 첫 번째 저장값

```text
00001 | 거리 0.458 m | 유효 픽셀 400
```

의미:

```text
저장 순서: 1번째
대표 거리: 0.458 m
센티미터 환산: 약 45.8 cm
유효 픽셀: 400개
```

20×20 영역의 전체 400개 픽셀이 모두 정상적인 거리값으로 사용되었다.

---

### 두 번째 저장값

```text
00002 | 거리 0.458 m | 유효 픽셀 400
```

1초 후에도 대표 거리가 약 45.8 cm로 측정되었다.

첫 번째 측정값과 동일하므로 대상 물체와 카메라 사이의 거리가 안정적으로 유지되고 있음을 확인할 수 있다.

---

### 세 번째 저장값

```text
00003 | 거리 0.459 m | 유효 픽셀 400
```

세 번째 측정값은 약 45.9 cm이다.

앞선 값과의 차이는 약 1 mm이며, 뎁스카메라의 일반적인 측정 변화 또는 미세한 노이즈로 볼 수 있다.

세 번의 측정 결과:

```text
0.458 m
0.458 m
0.459 m
```

측정값이 거의 동일하게 유지되었으므로 거리값이 안정적으로 계산되고 있음을 확인하였다.

---

## 9. 최종 결과 정리

이번 작업의 전체 흐름은 다음과 같다.

```text
RealSense D435 카메라 노드 실행
→ /camera/camera/depth/image_rect_raw 토픽 발행
→ depth_to_csv.py 실행
→ 뎁스 영상 토픽 구독
→ 중앙 20×20 픽셀 영역 추출
→ 정상 거리값 400개 확인
→ 중앙값으로 대표 거리 계산
→ 약 0.458~0.459 m 측정
→ /home/maze/depth_distance.csv에 1초마다 저장
```

최종 확인 결과:

| 확인 항목 | 결과 |
|---|---|
| 뎁스 토픽 구독 | 정상 |
| 중앙 영역 추출 | 정상 |
| 유효 픽셀 수 | 400개 |
| 대표 거리 계산 | 정상 |
| 측정 거리 | 약 0.458~0.459 m |
| CSV 저장 주기 | 1초 |
| CSV 저장 위치 | `/home/maze/depth_distance.csv` |
| 거리값 안정성 | 정상 |

따라서 RealSense D435의 뎁스 영상에서 대표 거리값을 계산하고 CSV 파일로 저장하는 작업이 정상적으로 완료되었다.
