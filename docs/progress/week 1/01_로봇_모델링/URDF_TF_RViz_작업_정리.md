# 로봇 모델링 상세 기록 — ROS 2 URDF·TF·RViz

## 1. 작업 목적

Jetson Orin의 ROS 2 Humble 환경에서 실제 제작 중인 이동 로봇의 구조를 URDF로 단순화하여 모델링하고, `robot_state_publisher`와 RViz2를 이용해 로봇 형상 및 TF 좌표계가 정상적으로 연결되는지 확인한 과정을 정리한 문서입니다. (1주차~2주차 내용 및 과정)

이 문서는 다음 내용을 중심으로 정리하였습니다.

- Tailscale을 이용한 Jetson 원격 접속
- ROS 2 Humble 및 RViz2 실행 확인
- 개인 ROS 2 워크스페이스와 패키지 생성
- 임시 URDF 작성 및 실제 로봇 치수 반영
- launch 파일과 CMake 설치 설정 작성
- `colcon` 빌드 및 패키지 실행
- RViz2에서 RobotModel과 TF 확인
- `tf2_echo`를 이용한 높이 좌표 검증
- 진행 중 발생한 오류와 해결 방법
- 현재까지의 최종 개발 상태

---

## 2. 전체 작업 흐름

```text
Windows 노트북
→ Windows용 Tailscale 실행
→ Jetson에 SSH 및 원격 데스크톱 접속
→ Jetson Ubuntu에서 ROS 2 Humble 환경 불러오기
→ 개인 워크스페이스 jisu_ws 생성
→ rescue_robot_description 패키지 생성
→ URDF·launch 파일 작성
→ CMakeLists.txt에 설치 규칙 추가
→ colcon 빌드
→ robot_state_publisher 및 RViz2 실행
→ 실제 로봇 치수와 형상 반영
→ base_footprint 기준 TF 높이 검증
```

작업에 사용한 주요 경로는 다음과 같습니다.

```text
워크스페이스:
/home/maze/jisu_ws

패키지:
/home/maze/jisu_ws/src/rescue_robot_description

URDF:
/home/maze/jisu_ws/src/rescue_robot_description/urdf/rescue_robot.urdf

launch 파일:
/home/maze/jisu_ws/src/rescue_robot_description/launch/display.launch.py
```


## 3. ROS 2 Humble 및 RViz2 실행 확인

### ROS 2 환경 불러오기

```bash
source /opt/ros/humble/setup.bash
```

현재 터미널에서 ROS 2 Humble 명령어와 패키지를 사용할 수 있도록 환경 설정을 불러오는 명령어

### ROS 2 버전 확인

```bash
echo $ROS_DISTRO
```

정상 결과:

```text
humble
```

### RViz2 실행

```bash
rviz2
```

RViz2 창이 정상적으로 열리는 것을 확인하였다.

처음에는 URDF와 TF를 발행하는 노드가 없었기 때문에 다음 상태가 표시

```text
Fixed Frame: map
No tf data
```

이는 RViz2 자체의 오류가 아니라, 아직 로봇 좌표계를 발행하지 않은 상태라는 의미

---

## 5. 개인 ROS 2 워크스페이스 생성

공용 Jetson에서 다른 팀원의 작업과 파일이 섞이지 않도록 개인 워크스페이스를 생성

### 입력한 명령어

```bash
mkdir -p ~/jisu_ws/src
cd ~/jisu_ws/src
pwd
```

정상 경로:

```text
/home/maze/jisu_ws/src
```

워크스페이스 구조:

```text
jisu_ws/
└── src/
```

---

## 6. 로봇 설명 패키지 생성

### ROS 2 패키지 생성 명령어

```bash
source /opt/ros/humble/setup.bash
cd ~/jisu_ws/src
ros2 pkg create --build-type ament_cmake rescue_robot_description
```

생성된 패키지 구조:

```text
rescue_robot_description/
├── CMakeLists.txt
├── package.xml
├── include/
└── src/
```

### URDF 및 launch 폴더 생성

```bash
cd ~/jisu_ws/src/rescue_robot_description
mkdir -p urdf launch
```

최종 패키지 기본 구조:

```text
rescue_robot_description/
├── CMakeLists.txt
├── package.xml
├── include/
├── launch/
├── src/
└── urdf/
```

---

## 7. 초기 임시 URDF 작성

처음에는 ROS 2의 URDF와 TF 구조를 확인하기 위해 가장 단순한 임시 모델을 작성하였다.

초기 TF 구조:

```text
base_link
├── camera_link
└── imu_link
```

초기 모델의 역할:

| 구성 | 역할 |
|---|---|
| `base_link` | 로봇 몸통의 기준 좌표계 |
| `camera_link` | RealSense 카메라 기준 좌표계 |
| `imu_link` | BNO055 IMU 기준 좌표계 |
| `fixed joint` | 몸통과 센서가 고정된 위치 관계임을 표현 |

초기 모델은 차체를 직육면체 하나로 표현하고 카메라 및 IMU 위치를 임시값으로 설정하였다.

---

## 8. launch 파일 작성

URDF 파일만 저장하면 ROS 2에서 자동으로 TF가 발행되지 않기 때문에, URDF를 읽고 필요한 노드를 실행하는 launch 파일을 작성하였다.

launch 파일 경로:

```text
/home/maze/jisu_ws/src/rescue_robot_description/launch/display.launch.py
```

launch 파일에서 실행한 주요 노드:

```text
robot_state_publisher
→ URDF를 읽고 고정 관절의 TF를 발행

rviz2
→ RobotModel과 TF를 화면에 표시
```

실행 명령:

```bash
ros2 launch rescue_robot_description display.launch.py
```

---

## 9. CMakeLists.txt 설치 규칙 추가

URDF와 launch 폴더를 빌드 후 ROS 2가 찾을 수 있는 설치 경로에 복사하기 위해 `CMakeLists.txt`에 다음 내용을 추가하였다.

```cmake
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

이 설정의 의미:

```text
launch 폴더
urdf 폴더
→ install/rescue_robot_description/share/rescue_robot_description/
   경로에 설치
```

이 설정이 있어야 다음 명령에서 launch 파일을 정상적으로 찾을 수 있음.

```bash
ros2 launch rescue_robot_description display.launch.py
```

---

## 10. colcon 설치 및 패키지 빌드

처음 빌드할 때 다음 오류가 발생하였다.

```text
bash: colcon: command not found
```

### 오류 원인

Jetson에 ROS 2는 설치되어 있었지만 ROS 2 워크스페이스 빌드 도구인 `colcon`이 설치되지 않은 상태였다.

### colcon 설치

```bash
sudo apt install -y python3-colcon-common-extensions
```

### 패키지 빌드

```bash
cd ~/jisu_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_description
```

정상 빌드 결과:

```text
Summary: 1 package finished
```

### 빌드 결과 적용

```bash
source ~/jisu_ws/install/setup.bash
```

이 명령은 현재 터미널에서 빌드된 `rescue_robot_description` 패키지를 사용할 수 있도록 환경을 추가로 불러온다.

---

## 11. RViz2에서 초기 RobotModel 및 TF 확인

### 실행 명령

```bash
source /opt/ros/humble/setup.bash
source ~/jisu_ws/install/setup.bash
ros2 launch rescue_robot_description display.launch.py
```

RViz2에서 다음 설정을 사용하였다.

```text
Global Options
→ Fixed Frame: base_link

Add
→ RobotModel
→ TF
```

초기 확인 결과:

```text
RobotModel → Status: Ok
TF → Status: Ok
```

처음에는 TF 좌표축의 크기가 로봇 몸통보다 크게 표시되어 모델이 잘 보이지 않았다.

TF 설정에서 다음 값을 수정하였다.

```text
Marker Scale: 1.0 → 0.1
```

수정 후 몸통 모델과 카메라·IMU 좌표축을 정상적으로 확인하였다.

---

## 12. 실제 로봇 치수 확인

실제 제작된 로봇과 기구 치수를 기준으로 URDF를 수정하였다.

초기에 전달받은 주요 치수:

| 항목 | 값 |
|---|---:|
| 전체 앞뒤 길이 | 약 310 mm |
| 전체 좌우 폭 | 약 280 mm |
| 전체 높이 | 약 180 mm |
| 바퀴 반지름 | 60 mm |
| 바퀴 직경 | 120 mm |
| 바퀴 폭 | 약 60 mm |
| 좌우 바퀴 중심 간 거리 | 230 mm |
| 바퀴축의 차체 중심 기준 위치 | 뒤쪽 80 mm |
| 카메라 중심 높이 | 지면에서 130 mm |
| IMU 중심 높이 | 지면에서 100 mm |

최종 높이 기준은 다음과 같이 정리하였다.

```text
지면                         0 mm
차체 하판 밑면              30 mm
바퀴축 중심                 60 mm
그리퍼 밑면                 90 mm
IMU 중심                   100 mm
그리퍼 중심                120 mm
카메라 중심                130 mm
그리퍼 윗면                150 mm
차체 프레임 최고점         180 mm
```

---

## 13. ROS 좌표계 기준

로봇 좌표 방향은 다음과 같이 설정하였다.

```text
+X: 그리퍼가 향하는 로봇 전방
+Y: 로봇 왼쪽
+Z: 위쪽
```

기준 좌표계:

| 프레임 | 의미 |
|---|---|
| `base_footprint` | 실제 지면 기준 좌표계, z=0 |
| `base_link` | 좌우 바퀴축 중앙, 지면에서 60 mm |
| `camera_link` | 카메라 중심 좌표계 |
| `imu_link` | BNO055 중심 좌표계 |
| `left_wheel_link` | 왼쪽 바퀴 중심 |
| `right_wheel_link` | 오른쪽 바퀴 중심 |

`base_footprint`와 `base_link`의 관계:

```xml
<origin xyz="0 0 0.060" rpy="0 0 0"/>
```

이는 바퀴 반지름이 60 mm이므로 바퀴축 중심이 지면에서 60 mm 위에 있다는 의미이다.

---

## 14. 실제 사진에 가까운 차체 구조 모델링

실제 로봇은 단순 직육면체 하나가 아니므로 URDF를 여러 개의 박스와 원통으로 나누어 표현하였다.

최종 단순화 구조:

```text
base_footprint
└── base_link
    ├── 차체 하판
    ├── 차체 세로 프레임 4개
    ├── 센서 장착판
    ├── 상단 연결 프레임
    ├── 왼쪽 바퀴
    ├── 오른쪽 바퀴
    ├── 그리퍼 중앙 연결부
    ├── 그리퍼 가로 연결부
    ├── 왼쪽 그리퍼 암
    ├── 왼쪽 그리퍼 끝부분
    ├── 오른쪽 그리퍼 암
    ├── 오른쪽 그리퍼 끝부분
    ├── camera_link
    └── imu_link
```

### 차체 하판

```text
밑면: 지면에서 30 mm
두께: 10 mm
중심 높이: 35 mm
base_link 기준 z: -25 mm
```

URDF 좌표 계산:

```text
35 mm - 60 mm = -25 mm
```

### 차체 세로 프레임

```text
시작 높이: 40 mm
최고 높이: 180 mm
프레임 높이: 140 mm
중심 높이: 110 mm
base_link 기준 z: 50 mm
```

### 그리퍼

 그리퍼에는 부가적인 센서가 적용되지 않으므로 임의로 설정하였습니다.

```text
그리퍼 밑면: 90 mm
그리퍼 높이: 60 mm
그리퍼 윗면: 150 mm
그리퍼 중심: 120 mm
base_link 기준 z: 60 mm
```

좌우 그리퍼는 직선 암과 안쪽으로 들어오는 끝부분으로 나누어 실제 형상과 비슷하게 단순화

### 바퀴

```text
반지름: 60 mm
폭: 60 mm
좌우 중심 간 거리: 230 mm
왼쪽 바퀴 y: +115 mm
오른쪽 바퀴 y: -115 mm
```

URDF의 원통은 기본 축이 Z축이므로 바퀴 회전축이 Y축 방향이 되도록 다음 회전을 적용

```xml
<origin xyz="0 0 0" rpy="1.5708 0 0"/>
```

---

## 15. 카메라 및 IMU 높이 설정

### 카메라

카메라 중심은 지면에서 130 mm로 설정

```text
카메라 절대 높이: 130 mm
base_link 절대 높이: 60 mm
상대 높이: 130 - 60 = 70 mm
```

URDF 설정:

```xml
<origin xyz="0 0 0.070" rpy="0 0 0"/>
```

### IMU

IMU 중심은 지면에서 100 mm로 설정

```text
IMU 절대 높이: 100 mm
base_link 절대 높이: 60 mm
상대 높이: 100 - 60 = 40 mm
```

URDF 설정:

```xml
<origin xyz="0 0 0.040" rpy="0 0 0"/>
```

---

## 16. RViz2 최종 확인

실제 지면을 기준으로 높이를 확인하기 위해 RViz2의 Fixed Frame을 다음과 같이 변경하였다.

```text
Global Options
→ Fixed Frame: base_footprint
```

`base_link`를 Fixed Frame으로 사용하면 격자가 바퀴축 높이인 지면 위 60 mm에 표시되기 때문에, 차체가 지면에서 얼마나 떠 있는지 직접 확인하기 어렵다.

최종 RViz2 확인 항목:

| 확인 항목 | 결과 |
|---|---|
| Fixed Frame | `base_footprint` |
| RobotModel 상태 | Ok |
| TF 상태 | Ok |
| 좌우 바퀴 표시 | 정상 |
| 차체 하판 및 프레임 표시 | 정상 |
| 그리퍼 형상 표시 | 정상 |
| 카메라 및 IMU 표시 | 정상 |

---

## 17. tf2_echo를 이용한 TF 검증

TF 좌표가 설정값과 일치하는지 `tf2_echo` 명령으로 확인하였다.

launch 파일을 실행한 터미널은 계속 켜두고, 새로운 터미널에서 다음 환경을 불러왔다.

```bash
source /opt/ros/humble/setup.bash
source ~/jisu_ws/install/setup.bash
```

### 바퀴축 중심 확인

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

정상 출력:

```text
Translation: [0.000, 0.000, 0.060]
```

의미:

```text
base_footprint → base_link
z = 0.060 m = 60 mm
```

즉, 지면 기준으로 바퀴축 중심이 60 mm 위에 정상적으로 설정되었다.

### 카메라 높이 확인

```bash
ros2 run tf2_ros tf2_echo base_footprint camera_link
```

예상 정상값:

```text
Translation: [0.000, 0.000, 0.130]
```

### IMU 높이 확인

```bash
ros2 run tf2_ros tf2_echo base_footprint imu_link
```

예상 정상값:

```text
Translation: [0.000, 0.000, 0.100]
```

---

## 18. 실행 중 발생한 오류와 해결

### 18.1 `colcon: command not found`

출력:

```text
bash: colcon: command not found
```

원인:

```text
ROS 2는 설치되어 있었지만 colcon 빌드 도구가 설치되지 않음
```

해결:

```bash
sudo apt install -y python3-colcon-common-extensions
```

---

### 18.2 `base_footprint` frame does not exist

출력:

```text
Invalid frame ID "base_footprint"
frame does not exist
```

가능한 원인:

```text
수정 전 URDF가 실행 중임
빌드 후 launch를 다시 실행하지 않음
robot_state_publisher가 실행 중이 아님
```

소스 URDF 확인:

```bash
grep -n "base_footprint" \
~/jisu_ws/src/rescue_robot_description/urdf/rescue_robot.urdf
```

빌드 파일을 깨끗하게 삭제한 뒤 다시 빌드:

```bash
cd ~/jisu_ws
rm -rf build/rescue_robot_description
rm -rf install/rescue_robot_description

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select rescue_robot_description

source ~/jisu_ws/install/setup.bash
ros2 launch rescue_robot_description display.launch.py
```

새 터미널에서 다시 `tf2_echo`를 실행하여 정상값을 확인하였다.

---

### 18.3 launch 터미널을 종료한 상태에서 TF 확인

`robot_state_publisher`는 launch 명령이 실행 중일 때만 TF를 발행한다.

따라서 다음 구조로 실행해야 한다.

```text
터미널 1:
ros2 launch rescue_robot_description display.launch.py
→ 계속 실행 유지

터미널 2:
ros2 run tf2_ros tf2_echo ...
→ TF 확인
```

---

## 19. 최종 결과 정리

이번 작업의 전체 결과는 다음과 같다.

```text
Tailscale로 Jetson 원격 접속
→ ROS 2 Humble 및 RViz2 실행 확인
→ jisu_ws 개인 워크스페이스 생성
→ rescue_robot_description 패키지 생성
→ URDF 및 launch 폴더 작성
→ CMakeLists.txt 설치 규칙 추가
→ colcon 설치 및 빌드
→ 임시 RobotModel과 TF 확인
→ 실제 로봇 치수와 사진을 기준으로 URDF 수정
→ base_footprint 지면 좌표계 추가
→ 차체·바퀴·그리퍼·카메라·IMU 구조 모델링
→ RViz2에서 RobotModel 및 TF 상태 확인
→ tf2_echo로 바퀴축·카메라·IMU 높이 검증
```

최종 확인 결과:

| 확인 항목 | 결과 |
|---|---|
| Jetson 원격 접속 | 정상 |
| ROS 2 Humble 환경 | 정상 |
| 개인 워크스페이스 | `/home/maze/jisu_ws` |
| 패키지 생성 | 정상 |
| URDF 로딩 | 정상 |
| launch 실행 | 정상 |
| colcon 빌드 | 정상 |
| RobotModel 표시 | 정상 |
| TF 표시 | 정상 |
| 지면 기준 프레임 | `base_footprint` |
| 바퀴축 높이 | 60 mm |
| IMU 중심 높이 | 100 mm |
| 카메라 중심 높이 | 130 mm |
| 차체 하판 지상고 | 30 mm |
| 그리퍼 밑면 높이 | 90 mm |
| 그리퍼 높이 | 60 mm |
| 차체 프레임 최고점 | 180 mm |

따라서 실제 로봇의 주요 구조와 높이를 반영한 URDF 모델을 제작하고, RViz2 및 TF 명령을 통해 좌표계가 정상적으로 적용된 것을 확인하였다.

---

## 20. 현재 개발 진행 상태

현재까지 완료한 범위는 다음과 같다.

```text
1주차:
ROS 2 개발환경 및 원격 접속 확인
임시 URDF와 TF 구조 작성
RViz2에서 RobotModel 표시

2주차 초반:
실제 로봇 치수 반영
base_footprint 지면 좌표계 구성
차체·바퀴·그리퍼 형상 단순 모델링
카메라·IMU 높이 적용
TF 높이 검증
```

현재 작업 위치:

```text
URDF 실제 치수 반영 및 TF 검증 완료
→ 다음 단계는 ROS 주행 명령과 STM32 통신 확인
```

다음에 팀원과 확인할 항목:

```text
/cmd_vel 또는 /leader/cmd_vel 토픽 이름
geometry_msgs/msg/Twist 사용 여부
Jetson과 STM32의 통신 방식
좌우 주행모터 속도 명령 구조
wheel odometry 토픽 이름과 메시지 형식
IMU 실제 발행 토픽
```

후속 [개발 계획서](../../../Plan.md)에서는 로봇별 주행 토픽을
`/leader/cmd_vel`, `/follower/cmd_vel`로 정했다. 위 목록은 이 결정을 내리기 전의
확인 항목이며, 메시지 타입·QoS·STM32 패킷은 아직 통합 계약으로 확정해야 한다.

다음 단계를 같이 진행해야하는 담당자가 부재이기에
담당자 대면할 시에 빠른 시일 내로 진행할 예정

다음 개발 흐름:

```text
ROS 2에서 cmd_vel 발행
→ Jetson 통신 노드가 명령 수신
→ 좌우 바퀴 속도로 변환
→ STM32에 전달
→ 주행모터 직진·회전 시험
→ wheel odometry 수신
→ RViz2에서 odometry 확인
```
