# Survivor–VSLAM Map Integration Stage 6: Persistent Registry Validation

## 1. Stage 6 목적

Stage 6의 최종 목적은 현재 frame의 raw survivor candidate 표시를 보존하면서,
같은 mapping mission/runtime 안에서 confirmed survivor의 ID와 마지막 map 위치를
유지하는 persistent registry를 추가하는 것이다.

Typed interface, Registry core/node, reset service와 persistent marker 경로를 구현했고,
실제 Jetson/D435에서 단일 인물 confirmation, 지속 관측, 이동, FOV 이탈→LOST,
LAST SEEN marker, same-ID 재연결, reset과 로봇 수동 이동을 검증했다. 현재 상태는
`PHASE A/B/C/D + PHASE E(robot movement) PASS`다. PHASE E(two-person physical)는
사용자 요청으로 이번 검증 범위에서 제외한다.

여기서 persistent ID는 얼굴 인식이나 appearance Re-ID가 아니다. 동일인 판단은
map XY spatial association을 사용하며 한 process/mapping session 안에서만 유효하다.

## 2. Stage 5에서 인계된 상태

Stage 5는 실제 Jetson과 RealSense D435 환경에서 PASS했다. 현재 repository에는
다음 경로가 존재한다.

```text
/leader/survivor/camera_positions  geometry_msgs/msg/PoseArray
/leader/survivor/map_positions     geometry_msgs/msg/PoseArray
/leader/survivor/map_markers       visualization_msgs/msg/MarkerArray
```

`survivor_map_transform_node`는 RGB image의 exact timestamp로 camera optical
position을 `map`으로 변환한다. `survivor_map_visualizer_node`는 현재 frame 후보를
finite-lifetime sphere/text로 표시한다. Stage 5 raw marker와 기존 VSLAM, dual EKF,
nvblox, RealSense 구성을 Phase A/B/C에서 수정하지 않았다.

Phase A 시작 전에 `rescue_robot_survivor` 자동 테스트 53개가 모두 통과했다.

## 3. 시작 branch / commit / git status

2026-09-19 Phase A 시작 기준:

```text
branch: main
commit: 883bdba624b8db2e70615372894b4511b8a00fd5
subject: fear(survivor): visualize survivor map positions in RViz
tracked modifications: none
```

사전 조사에서 실행한 명령:

```bash
cd ~/damgc_robot
git status
git branch --show-current
git log -1 --oneline
git rev-parse HEAD
git diff
```

조사 과정에서 생성된 package-local `.pytest_cache`는 구현 파일이 아니며 휴지통으로
이동했다. `build/`, `install/`, `log/`는 repository의 기존 `.gitignore` 대상이다.

## 4. Stage 6가 해결하는 문제

Stage 5의 `PoseArray` index와 `Survivor candidate N`은 frame-local 번호다. 사람이
움직이거나 검출 순서가 달라지면 같은 사람을 나타내지 않는다. Stage 6 Registry는
반복 관측 confirmation, public ID, visible/LOST, last position 유지와 근거리
재association을 제공한다.

Phase A/B/C는 registry 상태를 typed ROS message로 전달·관리하고 persistent RViz
marker로 표시하는 기반을 해결한다.

## 5. Raw candidate와 Persistent Survivor의 차이

| 구분 | Raw candidate | Persistent survivor |
| --- | --- | --- |
| Topic | `/leader/survivor/map_positions` | `/leader/survivor/tracks` |
| 의미 | 현재 입력 frame의 유효 map detection | confirmed/LOST registry snapshot |
| 번호 | 배열 index, identity 아님 | confirmation 뒤 할당되는 mission-runtime ID |
| FOV 이탈 | upstream 출력 중단 | timer가 LOST로 보존 |
| Marker | `/leader/survivor/map_markers`, finite lifetime | `/leader/survivor/registry_markers`, infinite lifetime |
| 현재 상태 | Stage 5 구현·검증 완료 | 구현 및 단일 인물/로봇 이동 실물 검증 완료 |

## 6. 전체 Architecture

최종 구현 architecture는 다음과 같다.

```text
                               D435
                                |
                +---------------+---------------+
                |                               |
             VSLAM                        Person Detector
                |                               |
          dual EKF                         Camera XYZ
                |                               |
          map TF                       exact timestamp TF2
                                                |
                                           Map XYZ
                                                |
                         +----------------------+----------------+
                         |                                       |
                   Raw Visualizer                         Registry Node
                         |                                       |
                   map_markers                        spatial association
                                                               |
                                                         confirmation
                                                               |
                                                         persistent ID
                                                               |
                                                         stabilization
                                                               |
                                                  /leader/survivor/tracks
                                                               |
                                                    Registry Visualizer
                                                               |
                                            /leader/survivor/registry_markers
                                                               |
                                                               v
                                                              RViz
                                                               |
                                                   nvblox + Survivor IDs
```

## 7. 신규 interface package

신규 package는 다음 위치에 생성했다.

```text
src/leader/rescue_robot_interfaces/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── SurvivorTrack.msg
    └── SurvivorTrackArray.msg
```

Repository에는 `leader_alignment_msgs`와 `follower_alignment_msgs`가 있었지만,
각각 leader/follower AprilTag alignment command만 소유한다. Survivor registry
message를 어느 한쪽 역할 package에 넣으면 package 책임과 dependency 방향이
부정확해진다. 공용 rescue-domain type을 위한 독립 package를 선택했다.

Package type은 ROS 2 Humble의 `ament_cmake`/`rosidl` interface package다.
사용 dependency는 message field 생성에 필요한 것만 포함한다.

- `rosidl_default_generators`: language type support 생성
- `rosidl_default_runtime`: 설치 후 interface runtime
- `geometry_msgs`: `Point`
- `std_msgs`: `Header`
- `builtin_interfaces`: `Time`

`rescue_robot_survivor/package.xml`에는 generated message와 reset service를 위해
`rescue_robot_interfaces`, `std_srvs` runtime dependency를 추가했다. `setup.py`에는
Registry와 Registry visualizer executable entry point를 등록했다.

## 8. SurvivorTrack.msg

실제 message source:

```text
uint8 STATUS_CONFIRMED=1
uint8 STATUS_LOST=2

uint32 id
geometry_msgs/Point raw_position
geometry_msgs/Point filtered_position
uint8 status
bool visible
uint32 observation_count
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
```

Tentative candidate는 public ID가 없는 내부 상태로 유지할 설계이므로 public status
constant에는 CONFIRMED와 LOST만 선언했다.

## 9. SurvivorTrackArray.msg

실제 message source:

```text
std_msgs/Header header
SurvivorTrack[] tracks
```

같은 package의 message type은 ROS 2 IDL adapter가 지원하는 relative type 표기를
사용했다. Build와 generated Python type import로 정상 생성을 확인했다.

`header.frame_id=map`은 `.msg`가 강제할 수 있는 default가 아니라 후속 publisher가
지켜야 할 runtime contract다. `header.stamp`는 개별 관측 시각이 아니라 registry
snapshot 발행 시각으로 사용할 예정이다.

## 10. Message field별 의미

| Field | 의미 |
| --- | --- |
| `id` | confirmation 시 배정될 mission-runtime public ID |
| `raw_position` | 마지막 accepted map detection XYZ |
| `filtered_position` | RViz/mission용 안정화 map XYZ |
| `status` | `STATUS_CONFIRMED` 또는 `STATUS_LOST` |
| `visible` | visible timeout 안에 관측됐는지 여부 |
| `observation_count` | track에 accepted된 관측 누계 |
| `first_seen` | 최초 tentative 관측의 sensor-derived 시각 |
| `last_seen` | 가장 최근 accepted 실제 관측 시각 |

Generated Python class에서 `STATUS_CONFIRMED == 1`, `STATUS_LOST == 2`와 모든
field의 default instance 생성을 확인했다.

## 11. ROS topic 구조

Phase B에서 다음 endpoint를 추가했다.

| Direction | Name | Type | QoS |
| --- | --- | --- | --- |
| Input | `/leader/survivor/map_positions` | `geometry_msgs/msg/PoseArray` | reliable, volatile, depth 1 |
| Output | `/leader/survivor/tracks` | `rescue_robot_interfaces/msg/SurvivorTrackArray` | reliable, transient local, depth 1 |
| Visualizer input | `/leader/survivor/tracks` | `rescue_robot_interfaces/msg/SurvivorTrackArray` | reliable, transient local, depth 1 |
| Visualizer output | `/leader/survivor/registry_markers` | `visualization_msgs/msg/MarkerArray` | reliable, transient local, depth 1 |
| Service | `/leader/survivor/registry/reset` | `std_srvs/srv/Trigger` | ROS service |

실제 ROS smoke에서 node, typed topic, publisher 1개와 reset service를 확인했다.
기존 raw topic과 marker topic은 변경하지 않았다.

## 12. Registry node 구조

`survivor_registry_node.py`는 ROS I/O와 시간만 담당한다. Map frame/stamp를 검사하고
pose를 core `Position3D`로 변환하며, sensor-derived stamp를 observation time으로
전달한다. ROS timer로 expiry/LOST를 처리하고 callback 직후 및 2 Hz로 snapshot을
발행한다. Reset service는 core reset 뒤 empty snapshot을 즉시 발행한다. Core 접근은
lock으로 보호한다.

## 13. Registry core 구조

`survivor_registry_core.py`는 ROS import가 없는 pure Python 구현이다.
`RegistryConfig`, immutable `Position3D`/`TrackSnapshot`, internal tentative/confirmed
state와 `SurvivorRegistry`로 구성된다. Input update, 시간 전진, snapshot, reset을
독립 unit test할 수 있다. Duplicate/out-of-order timestamp는 state를 되돌리지 않고
거부한다.

## 14. Tentative lifecycle

전체 state machine은 다음과 같다.

```text
RAW DETECTION
      |
      v
  TENTATIVE -- tentative timeout --> 삭제
      |
      | sufficient hits (confirm_hits)
      v
  CONFIRMED
      |
      | no observation > visible_timeout_sec
      v
     LOST
      |
      | near-position re-detection
      +---------------------------> CONFIRMED
```

Tentative는 timeout 시 제거한다. Confirmed와 LOST는 자동 삭제하지 않고 Registry reset
전까지 유지한다.

Unmatched finite detection은 internal ID와 hit count를 가진 tentative가 된다. 최초
관측은 hit 1이며 `confirm_hits=3` 전에는 public snapshot에 나타나지 않는다. 최초
관측부터 `tentative_timeout_sec=2.0`을 초과하면 timer가 삭제한다.

## 15. Confirmed lifecycle

Tentative hit가 threshold에 도달하면 public ID를 배정하고 confirmed registry로
옮긴다. Association 성공마다 latest raw, EMA filtered, last seen과 observation count를
갱신하며 `visible=true`, `STATUS_CONFIRMED`로 복구한다. Confirmed track은 reset 전까지
자동 삭제하지 않는다.

## 16. LOST lifecycle

ROS clock의 현재 시각과 `last_seen` 차이가 `visible_timeout_sec=2.0`을 초과하면
`visible=false`, `STATUS_LOST`가 된다. ID, count, position과 timestamp는 보존되며
periodic snapshot에도 계속 포함된다.

## 17. ID allocation

Public ID는 confirmation 순서대로 1부터 배정한다. 한 mapping session에서 ID를
재사용하지 않으며 reset만 next ID를 1로 되돌린다. 동시에 생성된 candidate는 좌표
정렬과 stable internal sequence로 입력 배열 순서 영향을 줄였다.

## 18. One-to-one association

Threshold 안의 모든 detection-track XY pair를 만든 뒤 distance와 stable key로
정렬하고, 아직 사용되지 않은 detection과 track만 match한다. Public ID 보존을 위해
confirmed/LOST를 먼저 match하고 남은 detection만 tentative와 match한다.

## 19. Association distance

Map XY Euclidean distance `hypot(dx, dy)`를 사용한다. Visible confirmed와 tentative는
latest raw position, LOST는 마지막 filtered position을 기준으로 한다. Z는 저장하고
EMA를 적용하지만 identity association distance에는 넣지 않는다.

## 20. Association threshold parameter

Visible/tentative 기본값은 `0.50 m`, LOST reassociation은 `0.75 m`로 구현했다.
Stage 4의 약 `0.115 m` residual에 여유를 주면서 가까운 다중 인물 오association을
제한하기 위한 보수적 초기값이다. 단일 인물 실물 lifecycle에는 사용했지만 threshold
정확도 sweep과 다중 인물 교차 tuning은 다음 Stage 과제다.

| Parameter | Default | 단위/의미 | 너무 작으면 | 너무 크면 |
| --- | ---: | --- | --- | --- |
| `association_radius_m` | `0.50` | m, visible/tentative XY match gate | 이동·측정 오차로 ID 분리 | 가까운 다른 사람에 잘못 연결 |
| `reassociation_radius_m` | `0.75` | m, LOST XY match gate | 재진입 때 새 ID 생성 | 다른 사람을 기존 LOST ID로 복구 |
| `confirm_hits` | `3` | hit, public ID 전 최소 관측 | 순간 오검출이 confirmed | confirmation 지연/누락 |
| `tentative_timeout_sec` | `2.0` | s, tentative 생존 시간 | 저주기 검출 후보가 조기 삭제 | false candidate가 오래 남음 |
| `visible_timeout_sec` | `2.0` | s, 마지막 관측 후 LOST 전환 | 짧은 dropout에도 LOST | FOV 이탈 표시가 늦음 |
| `position_ema_alpha` | `0.50` | ratio, 새 raw 반영 비율 | 움직임 추종이 느림 | jitter 억제가 약함 (`1.0`은 무필터) |
| `registry_publish_hz` | `2.0` | Hz, snapshot/timer 주기 | LOST 표시·late 확인 지연 | 불필요한 CPU/DDS traffic |

검증 범위는 radius/time/rate가 finite `> 0`, `confirm_hits >= 1`, EMA alpha가
`0 < alpha <= 1`이다.

빈 topic/service/frame, 같은 input/output topic과 위 범위 밖 값은 startup에서 명확한
`ValueError` 또는 ROS parameter type error로 거부하며 safe fallback으로 숨기지 않는다.

## 21. Reassociation

LOST의 filtered last position에서 `0.75 m` 이내인 detection은 기존 track을 다시
CONFIRMED/visible로 만들고 같은 ID를 유지한다. ROS smoke에서 Survivor #1이 ID 1,
observation count 4로 복구됨을 확인했다. Threshold 밖 detection은 새 tentative가 된다.

## 22. 사람 이동 처리

Visible track은 latest raw position으로 association하므로 EMA lag가 이동 association을
방해하지 않는다. Update 사이 이동이 `0.50 m` 이내이면 ID를 유지하고 raw/filtered를
갱신한다. Phase D 실제 이동 구간에서 Survivor #1과 증가하는 observation count를
확인했다.

## 23. FOV 이탈 처리

현재 upstream map transform은 empty camera array를 map topic에 전달하지 않는다.
따라서 후속 Registry는 callback만으로 FOV 이탈을 판단할 수 없고 ROS timer가
필수다. Phase B timer와 synthetic ROS smoke에서 timeout 후 LOST, position/ID 유지와
계속된 snapshot 발행을 확인했다. Phase D 실제 FOV 이탈에서도 20초 동안 LOST와
동일 filtered position을 확인했다.

## 24. FOV 재진입 처리

근처 synthetic 재입력에 대한 same-ID 복구는 Phase B ROS smoke에서 PASS했다. 실제
runtime에서도 `#1 LOST obs=83` 뒤 `#1 CONFIRMED obs=84`로 같은 ID가 복구됐다.

## 25. 같은 ID 유지 조건

Association 기준 위치에서 상태별 radius 안에 있어야 한다. 이는 spatial heuristic이며
얼굴이나 appearance 신원을 뜻하지 않는다. 단일 인물 이동/재진입은 검증했으며,
다중 인물 교차 조건은 사용자 요청으로 이번 검증 범위에서 제외했다.

## 26. 동일인 보장 한계

Stage 6는 얼굴 인식, appearance Re-ID, DeepSORT, ByteTrack, BoT-SORT를 사용하지
않는다. 마지막 map 위치에서 멀리 이동하거나 두 사람이 가까이 교차하면 동일인
보장 또는 ID 보존이 불가능할 수 있다.

## 27. Position filtering

XYZ 모두에 `filtered = alpha*new_raw + (1-alpha)*previous_filtered` EMA를 구현했다.
기본 alpha는 `0.50`, 허용 범위는 `0 < alpha <= 1`이다. 최초 관측은 raw로 초기화하고
alpha 1은 filtering을 비활성화한다.

## 28. raw_position vs filtered_position

`raw_position`은 마지막 accepted map detection이고 visible association 기준이다.
`filtered_position`은 EMA 결과이며 LOST reassociation과 후속 RViz 위치에 쓴다.
LOST 동안 두 position이 freeze되는 것을 unit test했다.

## 29. visible

마지막 실제 관측부터 timeout 이내에는 true, 초과하면 false다. Reassociation 시 다시
true가 된다. Synthetic ROS smoke와 실제 FOV 이탈/재진입에서 모두 검증했다.

## 30. observation_count

Tentative 첫 hit부터 accepted association마다 정확히 1 증가한다. 양방향 one-to-one으로
한 callback에서 같은 track이 두 번 증가하지 않는 것을 unit test했다.

## 31. first_seen / last_seen

두 field는 input `PoseArray.header.stamp`의 observation 시각이다. Confirmation 전 최초
tentative 시각을 first seen으로 보존하고 accepted update만 last seen을 바꾼다.
Array header stamp는 별도의 registry snapshot 발행 ROS time이다.

## 32. Timer

`1 / registry_publish_hz` period의 ROS timer가 tentative expiry, LOST 전환과 periodic
snapshot publish를 수행한다. Upstream 무발행 상태에서도 동작한다.

## 33. Registry publish QoS

Output은 RELIABLE, KEEP_LAST(1), TRANSIENT_LOCAL이다. 실제 endpoint와 late subscriber가
최신 LOST snapshot을 받은 것을 ROS smoke에서 확인했다.

## 34. Reset service

`/leader/survivor/registry/reset` Trigger를 구현했다. 모든 internal/public state와
timestamp guard를 초기화하고 empty array를 즉시 발행한다. 실제 response와
`tracks: []` transient snapshot을 확인했다.

## 35. Mapping session reset 정책

새 map session에서는 이전 좌표를 재사용하지 않으며 reset service 호출이 필수다.
Reset 후 next ID는 1이다. Process restart 뒤 자동 복원과 map session을 넘는 영구 ID는
Stage 6 범위 밖이다.

## 36. Registry Visualizer

`survivor_registry_visualizer_node.py`를 별도 executable로 구현했다. Typed
`/leader/survivor/tracks`를 TRANSIENT_LOCAL로 구독하고
`/leader/survivor/registry_markers` MarkerArray를 RELIABLE/TRANSIENT_LOCAL로 발행한다.
Marker 좌표는 raw가 아니라 `filtered_position`만 사용한다. `map`이 아닌 snapshot,
non-finite position, status/ID가 잘못되거나 중복된 track은 skip하고 node는 계속 동작한다.

## 37. Raw Marker vs Registry Marker

기존 `survivor_map_visualizer_node.py`와 `/leader/survivor/map_markers` 계약은 수정하지
않았다. Raw는 현재 frame candidate와 finite lifetime을 나타낸다. Registry는 confirmed와
LOST public ID, filtered position과 infinite lifetime을 별도 topic에 표시한다. RViz의
`Survivor Raw`와 `Survivor Registry` display를 각각 toggle할 수 있다.

| Interface | 의미 | FOV 이탈 후 |
| --- | --- | --- |
| Raw `map_positions` — `/leader/survivor/map_positions` | 현재 frame의 raw map 측정 | 새 측정 없음 |
| Raw `map_markers` — `/leader/survivor/map_markers` | 현재 보이는 raw candidate | finite lifetime 뒤 삭제 |
| Registry `tracks` — `/leader/survivor/tracks` | mission-runtime confirmed/LOST registry | LOST snapshot 유지 |
| Registry `registry_markers` — `/leader/survivor/registry_markers` | Registry의 filtered 위치와 public ID | LAST SEEN marker 유지 |

RViz Before/After 의미는 다음과 같다.

```text
Stage 5 raw                 Stage 6 registry

Survivor candidate 1       Survivor #1
X / Y / Z                  X / Y / Z
      ●                    VISIBLE
                                 ●

FOV 이탈: 삭제             FOV 이탈:
                           Survivor #1
                           X / Y / Z
                           LAST SEEN
                                 ●

                           근처 재진입:
                           Survivor #1
                           갱신된 X / Y / Z
                           VISIBLE
                                 ●
```

## 38. VISIBLE Marker

Visible confirmed track은 Stage 5와 일관된 orange-red sphere와 white text로 표시한다.
Text는 `Survivor #ID`, X/Y/Z 각 줄과 `VISIBLE`을 포함한다. Synthetic smoke에서
ID 1의 marker ID `2/3`, filtered X `2.54`, map frame, zero lifetime을 확인했다.

## 39. LAST SEEN Marker

LOST track은 마지막 filtered 위치에 gray sphere(alpha `0.45`)와 dimmed text(alpha
`0.70`)로 계속 표시하고 text 끝에 `LAST SEEN`을 넣는다. Late-subscriber smoke에서
동일 ID `2/3`, 동일 XYZ의 LOST marker를 즉시 수신했다.

## 40. Marker lifetime 차이

Raw marker는 기존 `2.0 s` finite lifetime을 유지해 현재 detection이 사라지면 만료된다.
Registry marker는 FOV 이탈만으로 삭제하면 안 되므로 lifetime `(0,0)`을 사용한다.
Track 일부 제거는 ID별 DELETE, empty Registry/reset은 항상 DELETEALL을 발행한다.
Reset smoke에서 action `3` DELETEALL의 transient-local snapshot을 확인했다.

## 41. RViz config

`rviz/vslam_nvblox.rviz`의 기존 raw display를 `Survivor Raw`로 명확히 이름 붙이고 topic,
volatile QoS와 namespace는 유지했다. `Survivor Registry` MarkerArray display를 추가해
`/leader/survivor/registry_markers`, reliable/transient-local, depth 1로 설정했다.
Fixed Frame `map`, nvblox Mesh, TF, RobotModel, VSLAM/odometry와 image display는 제거하거나
변경하지 않았다. YAML parse 및 두 display/topic 보존을 자동 테스트했다.

## 42. 신규 파일

Stage 6 신규 파일:

```text
src/leader/rescue_robot_interfaces/CMakeLists.txt
src/leader/rescue_robot_interfaces/package.xml
src/leader/rescue_robot_interfaces/msg/SurvivorTrack.msg
src/leader/rescue_robot_interfaces/msg/SurvivorTrackArray.msg
docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE6_PERSISTENT_REGISTRY_VALIDATION.md
src/leader/rescue_robot_survivor/config/survivor_registry.yaml
src/leader/rescue_robot_survivor/launch/survivor_registry.launch.py
src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_registry_core.py
src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_registry_node.py
src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_registry_visualizer_node.py
src/leader/rescue_robot_survivor/test/test_survivor_registry_core.py
src/leader/rescue_robot_survivor/test/test_survivor_registry_launch.py
src/leader/rescue_robot_survivor/test/test_survivor_registry_node.py
src/leader/rescue_robot_survivor/test/test_survivor_registry_visualizer_node.py
```

## 43. 수정 파일

Stage 6 수정 파일:

```text
README.md
docs/README.md
src/leader/rescue_robot_survivor/README.md
src/leader/rescue_robot_survivor/package.xml
src/leader/rescue_robot_survivor/setup.py
rviz/vslam_nvblox.rviz
```

`package.xml`에는 interface와 `std_srvs` runtime dependency를 추가했고 `setup.py`에는
Registry와 Registry visualizer console entry point를 추가했다. RViz에는 Registry display만
추가하고 기존 raw display를 보존했다. 기존 entry point는 유지했다.

## 44. 보호한 기존 파일

Detector, map transform, raw visualizer 및 해당 launch, camera preprocessing,
VSLAM/EKF/nvblox/RealSense 설정과 `scripts/run_vslam_mapping.sh`는 수정하지 않았다.
RViz config는 두 survivor display를 분리하기 위한 최소 변경만 수행했다.

## 45. Build 방법

실제로 성공한 명령:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select rescue_robot_interfaces rescue_robot_survivor
source install/local_setup.bash
```

결과:

```text
Finished <<< rescue_robot_interfaces [1.35s]
Finished <<< rescue_robot_survivor [3.36s]
Summary: 2 packages finished [5.57s]
```

## 46. Custom message 확인

실제로 성공한 명령:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 interface show rescue_robot_interfaces/msg/SurvivorTrack
ros2 interface show rescue_robot_interfaces/msg/SurvivorTrackArray
```

두 명령 모두 source의 field, constants와 dependency-expanded nested fields를
정상 출력했다. 추가 Python import에서 두 generated class의 instance 생성과
status constant `1`, `2`를 확인했다.

## 47. Unit test

실제로 실행한 명령:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select \
  rescue_robot_interfaces rescue_robot_survivor \
  --event-handlers console_direct+
colcon test-result --test-result-base build/rescue_robot_survivor --verbose
```

기존 53개, Phase B 33개와 Phase C marker/RViz 10개를 합친
**96 tests, 0 errors, 0 failures, 0 skipped**를 확인했다. Phase C test는 visible/LOST,
ID/XYZ/status text, filtered position, map frame, zero lifetime, deterministic multi-track,
order reversal, DELETE/DELETEALL, transient QoS, parameter validation과 RViz raw/registry
display 보존을 포함한다. 신규/수정 Python 파일의 `ament_flake8`도 PASS했다.

## 48. 전체 실행 순서

Stage 5 pipeline을 그대로 시작한 뒤 별도 terminal에서 다음을 실행한다.

```bash
ros2 launch rescue_robot_survivor survivor_registry.launch.py
```

이 launch는 Registry와 Registry visualizer 두 node를 함께 시작한다. 각 executable은
`ros2 run`으로도 독립 실행할 수 있다. RealSense, detector, VSLAM, nvblox, map transform,
raw visualizer는 include하지 않는다.

## 49. Terminal별 실행 명령

실물 통합 검증에서 사용한 terminal 순서는 다음과 같다. Terminal 1은 D435 single-owner,
VSLAM, dual EKF, nvblox와 RViz를 시작하므로 별도 RealSense launch를 중복 실행하지 않는다.

```bash
# Terminal 1 — VSLAM + nvblox + RViz
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
./scripts/run_vslam_mapping.sh

# Terminal 2 — Camera preprocessing
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py

# Terminal 3 — YOLO
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
./scripts/run_survivor_detector.sh

# Terminal 4 — Camera XYZ -> Map XYZ
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py

# Terminal 5 — Stage 5 Raw Visualizer
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py

# Terminal 6 — Registry + Registry Visualizer
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_registry.launch.py
```

## 50. 한 사람 테스트

Jetson/D435 full stack을 `STM32_I2C_WRITE_ENABLED=0`으로 실행했다. 실제 반복 검출에서
`Survivor #1` ID 유지와 observation_count 증가를 확인했다. 정지 12초 관측 105개에서
raw delta는 `(0.045, 0.072, -0.010) m`였다.

## 51. 이동 테스트

사람 이동(로봇 정지) 결과는 다음과 같다. 30초 관측에서 `Survivor #1`이 유지되고
observation_count가 `429→644`로 증가했다. raw 범위는 x `1.628–2.599`, y
`-3.256–-0.575`, z `0.240–0.752 m`였다.

Phase E의 로봇 수동 이동(사람 고정)을 실제로 수행했다. 사용자가
`STM32_I2C_WRITE_ENABLED=1` mapping terminal에서 방향키를 조작했고, 90초 monitor에서
Camera XYZ 706 samples, Map XYZ 21 samples, VSLAM odometry 401 samples를 수집했다.
Camera XYZ 범위는 x `-0.687–1.576`, y `-1.019–-0.489`, z `2.175–5.204`였고,
Map XYZ 범위는 x `1.687–2.873`, y `-4.927–-0.121`, z `0.617–1.142`였다.
Registry `#1/#2` ID는 유지됐다. VSLAM 관측 rate는 약 `4.46 Hz`였다.
이 구간의 `#1/#2`는 두 사람이 동시에 있었다는 영상 근거가 없으므로 two-person
physical validation 증거로 사용하지 않는다.

## 52. FOV 이탈 테스트

실제 FOV 이탈에서 20초 동안 `id=1, visible=false, status=LOST`가 유지됐다.
observation_count `1784`, filtered 위치 `(1.430,-1.348,0.385 m)`가 고정됐고 marker는
`LAST SEEN`, lifetime 0이었다.

## 53. 재진입 테스트

실제 monitor에서 `#1 LOST obs=83` 후 `#1 CONFIRMED obs=84`로 복구됐고, 이후에도
`117→118`, `123→124`에서 새 ID 없이 재연결됐다. 검출 dropout과 의도적 FOV 이탈을
완전히 분리할 수 없으므로 동일 ID spatial reassociation runtime evidence로 기록한다.

## 54. 새로운 사람 테스트

실제 reset Trigger success 후 next ID가 1로 돌아갔다. detector를 중지한 상태에서
`/tracks` empty snapshot(`frame_id=map`)과 registry MarkerArray
`DELETEALL(action=3,id=0)`을 연속 수신했다.

## 55. 다중 사람 테스트

두 사람 physical validation은 사용자 요청으로 이번 목표에서 제외한다. 이전 150초
실물 실행에서 IDs `1–6`이 관측됐지만 두 명이 동시에 카메라에 있었다는 독립적인 영상
증거가 없어 two-person 결과로 사용하지 않는다.

## 56. candidate order 변경 테스트

`[-1, +1]`, `[+1, -1]`, `[-1, +1]` 순으로 detection 배열을 바꿔도 map 위치별
ID 1/2가 유지되는 자동 테스트가 PASS했다. 실제 다중 인물 검증은 Phase E 대상이다.

## 57. reset 테스트

Core unit test에서 tentative/confirmed clear, next ID 1과 timestamp guard 초기화를
검증했다. ROS smoke에서 Trigger success response와 즉시 발행된 empty typed snapshot을
확인했다. Phase C smoke에서는 visualizer가 empty snapshot을 받아 action `3` DELETEALL을
발행했고, 늦게 시작한 subscriber도 이 cleanup snapshot을 받았다.
실제 수동 이동 runtime 종료 직전에도 reset success, `/tracks` empty
(`frame_id=map`), MarkerArray `DELETEALL(action=3,id=0)`을 재확인했다.

## 58. Raw/Registry 비교

Raw는 current-frame `map_positions`, frame-local candidate 번호와 finite marker를 사용한다.
Registry는 confirmed/LOST typed tracks, persistent ID, filtered 위치와 infinite marker를
사용한다. Topic, node, namespace와 RViz display를 분리했으며 raw visualizer의 기존 7개
test가 계속 PASS한다. 실제 RViz 화면 비교는 Phase D 대상이다.

## 59. VSLAM 회귀 검증

실제 full stack graph에 `/visual_slam_node`와 tracking odometry가 존재한 상태로
registry를 동시 기동했다. 측정 snapshot에서 tracking odometry는 약 2.9 Hz였다.

## 60. EKF 회귀 검증

실제 graph에 local/global EKF와 `/leader/odom/raw`가 존재한 상태로 registry를 동시
기동했다. `/leader/odom/raw`는 측정 snapshot에서 약 38.5 Hz였다.

## 61. nvblox 회귀 검증

실제 graph에 `/nvblox_node`가 존재한 상태로 registry를 동시 기동했다. nvblox node와
mapping data path는 정상 기동됐으며 mesh/ESDF 정량 검증은 별도 후속 범위다.

## 62. 실제 테스트 결과

Phase A/B/C/D 결과와 Phase E 실행 시도:

| 항목 | 결과 |
| --- | --- |
| Interface package 발견/build | PASS |
| `SurvivorTrack.msg` 생성/type support | PASS |
| `SurvivorTrackArray.msg` 생성/type support | PASS |
| `ros2 interface show` 두 type | PASS |
| Generated Python import/constants | PASS |
| 전체 survivor 자동 테스트 | PASS, 96/96 |
| Registry core/lifecycle/association | PASS, unit tests |
| Registry launch/topic/QoS | PASS, synthetic ROS smoke |
| LOST 및 same-ID reassociation | PASS, ID 1 유지·count 4 |
| Reset empty snapshot | PASS, Trigger success |
| Visible marker | PASS, ID 2/3, filtered XYZ, zero lifetime |
| LOST marker | PASS, same ID/XYZ, LAST SEEN |
| Reset marker cleanup | PASS, transient DELETEALL |
| RViz raw/registry display config | PASS, YAML/static contract |
| Jetson/D435 repeated/visible movement | PASS (실물 smoke) |
| Jetson/D435 FOV exit, LOST freeze, LAST SEEN marker | PASS |
| Jetson/D435 re-entry same ID | PASS — #1 LOST→CONFIRMED, 새 ID 없음 |
| Registry hardware/실물 RViz GUI 렌더링 | PARTIAL — MarkerArray 확인 |
| Phase E two-person physical identity | EXCLUDED — 사용자 요청 범위 |
| Phase E manual robot movement | PASS — Camera/Map/Registry/VSLAM samples |
| Phase E tegrastats sample | CAPTURED — RAM 약 5.44/7.61 GB, CPU 80–99%, GPU 14–92% |

## 63. 발생 오류

첫 신규 test run은 `confirm_hits=1` 설정에서 unmatched detection이 즉시 ID 2로
confirmation되는 정상 동작을 tentative로 기대해 1건 실패했다. 구현 failure는 아니었다.
실물 실행 중 detector의 RGB/depth timestamp 차이 경고와 종료 시 camera-info bridge의
중복 shutdown 예외가 관찰됐지만, Stage 5 보호 파일은 변경하지 않았고 registry 동작과
mapping stack 종료에는 영향을 주지 않았다.

## 64. 오류 원인

실패한 test의 expectation이 `confirm_hits=1` lifecycle과 모순됐다. 해당 설정에서는
새 tentative의 첫 hit가 곧 confirmation threshold다.

## 65. 해결 방법

Test를 “기존 track observation count는 한 번만 증가하고 두 번째 detection은 별도 ID와
count 1을 가진다”로 수정했다. 최종 Phase E 범위까지 전체 96/96이 PASS했다. Test 삭제나 skip은
없었다.

## 66. 알려진 제한사항

- 실제 Jetson/D435 one-person repeated/LOST/reassociation 검증을 수행했다. multi-person은 이번 Phase D 범위가 아니다.
- MarkerArray 내용과 lifetime은 실물 runtime에서 확인했으며 RViz GUI 육안 확인은 보강이 필요하다.
- Detector가 사람을 계속 검출하는 동안에는 Registry가 LOST로 전환할 수 없으므로 FOV 검증은 debug image에서 실제 완전 이탈을 함께 확인해야 한다.
- Persistent ID는 현재 mission/runtime 밖으로 저장되지 않는다.
- Spatial-only association은 실제 사람 신원 인식이 아니다.
- Appearance Re-ID, 얼굴 인식, ByteTrack/BoT-SORT/DeepSORT는 사용하지 않는다.
- 사람이 radius보다 멀리 이동하거나 가까운 두 사람이 교차하면 새 ID 또는 ID swap이
  발생할 수 있다.
- 사람이 FOV 밖에서 마지막 위치로부터 멀리 이동하면 동일 ID를 보장할 수 없다.
- Process restart persistence와 mission-level disk/CSV/JSON storage는 없다.
- 새로운 VSLAM mapping session은 map 좌표 원점이 바뀔 수 있으므로 Registry reset이
  필수다.
- `header.frame_id=map`은 publisher runtime validation으로 강제하며 `.msg` 자체 default는 아니다.

## 67. Stage 6 PASS/FAIL

**Stage 6 Phase A/B/C/D + Phase E manual robot movement: PASS.**
**Phase E two-person physical identity: EXCLUDED BY USER SCOPE.**

PASS 조건:

- [x] Interface package 정상
- [x] `SurvivorTrack.msg` 정상
- [x] `SurvivorTrackArray.msg` 정상
- [x] Build 성공
- [x] `ros2 interface show` 성공
- [x] 기존 survivor runtime code 자동 테스트 회귀 없음
- [x] Pure Python Registry core
- [x] One-to-one XY association 및 input-order regression
- [x] Tentative confirmation과 confirmation-time public ID
- [x] EMA raw/filtered position
- [x] Timer 기반 visible/LOST와 position freeze
- [x] LOST same-ID reassociation
- [x] Typed transient-local periodic snapshot
- [x] Trigger reset와 empty snapshot
- [x] Registry-only launch 및 ROS synthetic smoke
- [x] Registry visualizer와 filtered-position sphere/text
- [x] VISIBLE/LAST SEEN persistent marker
- [x] Deterministic public-ID marker ID와 order regression
- [x] Reset DELETEALL 및 late-subscriber marker QoS
- [x] RViz Raw/Registry 독립 display
- [x] 실제 한 사람 반복 검출 → public ID 1개 유지
- [x] visible 이동 중 raw/filtered position 갱신
- [x] FOV 이탈 → LOST, Registry/marker/last position 유지
- [x] 근처 재검출 → 같은 ID CONFIRMED 복구
- [x] 실제 reset → empty tracks, DELETEALL, next ID 1
- [x] 사용자 수동 로봇 이동 중 Camera/Map/Registry/VSLAM 관측
- [x] VSLAM·dual EKF·nvblox·Stage 5 raw pipeline 보호/회귀 근거
- [x] 자동 테스트 96/96
- [x] 최종 architecture/message/state/parameter/실행/제한사항 문서화
- [ ] 두 사람 physical validation — 사용자 요청으로 이번 PASS 범위에서 제외

Phase D 단일 인물 lifecycle과 Phase E manual robot movement는 PASS다. Phase E
two-person physical identity는 사용자 요청으로 이번 범위에서 제외하며, map-session
외부 영구 저장은 후속 Stage 범위다.

## 68. 현재 최종 Architecture

현재 repository는 Stage 5 raw pipeline을 그대로 유지하면서
`map_positions -> survivor_registry -> tracks -> registry_visualizer -> registry_markers`
branch를 추가했다. Tracks에는 confirmed와 LOST만 포함되고 tentative는 core 내부에만
있다. Raw와 Registry marker는 서로 독립적인 RViz display다.

## 69. Manual Validation Quick Reference

Phase A/B/C 재현:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select rescue_robot_interfaces rescue_robot_survivor
source install/local_setup.bash
ros2 interface show rescue_robot_interfaces/msg/SurvivorTrack
ros2 interface show rescue_robot_interfaces/msg/SurvivorTrackArray
ros2 launch rescue_robot_survivor survivor_registry.launch.py --show-args
colcon test --packages-select \
  rescue_robot_interfaces rescue_robot_survivor \
  --event-handlers console_direct+
colcon test-result --test-result-base build/rescue_robot_survivor --verbose
```

Runtime terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_registry.launch.py
```

관찰 및 reset:

```bash
ros2 topic echo /leader/survivor/tracks \
  --qos-durability transient_local
ros2 topic info -v /leader/survivor/tracks
ros2 topic echo /leader/survivor/registry_markers \
  --qos-durability transient_local
ros2 topic info -v /leader/survivor/registry_markers
ros2 service call /leader/survivor/registry/reset std_srvs/srv/Trigger "{}"
```

전체 수동 확인 명령:

```bash
# Custom messages
ros2 interface show rescue_robot_interfaces/msg/SurvivorTrack
ros2 interface show rescue_robot_interfaces/msg/SurvivorTrackArray

# Raw Stage 3/4/5
ros2 topic echo /leader/survivor/camera_positions
ros2 topic echo /leader/survivor/map_positions
ros2 topic echo /leader/survivor/map_markers

# Persistent Registry
ros2 topic echo /leader/survivor/tracks --qos-durability transient_local
ros2 topic info -v /leader/survivor/tracks
ros2 topic echo /leader/survivor/registry_markers --qos-durability transient_local
ros2 topic info -v /leader/survivor/registry_markers
ros2 service call /leader/survivor/registry/reset std_srvs/srv/Trigger "{}"
ros2 service list | grep survivor

# TF / VSLAM / dual EKF
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /leader/odometry/local
ros2 topic hz /leader/odometry/global

# nvblox mesh / ESDF
ros2 topic info -v /nvblox_node/mesh
ros2 service list | grep /nvblox_node/get_esdf_and_gradient

# ROS graph
ros2 node list
ros2 topic list
```

3D ESDF mode에서는 `static_esdf_pointcloud` 무출력을 실패로 판단하지 않는다. Stage 5
통합 검증에서 `/nvblox_node/mesh` vertex stream과
`/nvblox_node/get_esdf_and_gradient` success response를 확인했다.

종료는 Terminal 6→5→4→3→2 순서로 `Ctrl+C`, 마지막 Terminal 1에서 `SPACE` 후
`Ctrl+C`로 수행한다. Mapping script의 rosbag finalization과 analysis 완료 메시지까지
기다린 뒤 카메라 owner와 ROS node가 남지 않았는지 확인한다.

## 70. Rollback

자동 commit 또는 push는 수행하지 않았다. Rollback이 필요하면 먼저 `git diff`와
`git status`로 Stage 6 파일만 식별한다. broad `git reset --hard`, `git restore .`,
`git checkout .`는 사용하지 않는다. Commit 뒤에는 해당 Stage 6 commit을
`git revert`하는 방식을 우선한다.

최종 검토 및 선택적 commit 명령은 다음과 같다. 이 문서 작성 과정에서는 실행하지
않았다.

```bash
cd ~/damgc_robot
git status
git diff
git add README.md docs/README.md \
  docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE6_PERSISTENT_REGISTRY_VALIDATION.md \
  rviz/vslam_nvblox.rviz \
  src/leader/rescue_robot_interfaces \
  src/leader/rescue_robot_survivor/README.md \
  src/leader/rescue_robot_survivor/package.xml \
  src/leader/rescue_robot_survivor/setup.py \
  src/leader/rescue_robot_survivor/config/survivor_registry.yaml \
  src/leader/rescue_robot_survivor/launch/survivor_registry.launch.py \
  src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_registry_core.py \
  src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_registry_node.py \
  src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_registry_visualizer_node.py \
  src/leader/rescue_robot_survivor/test/test_survivor_registry_core.py \
  src/leader/rescue_robot_survivor/test/test_survivor_registry_launch.py \
  src/leader/rescue_robot_survivor/test/test_survivor_registry_node.py \
  src/leader/rescue_robot_survivor/test/test_survivor_registry_visualizer_node.py
git commit -m "Add persistent survivor registry and spatial association"
```

`git push`는 사용자 허가 없이는 수행하지 않는다.

## 71. 다음 Stage

Goal 4/Phase D 진행 가능 여부: **YES**. Phase E 로봇 수동 이동은 실제 사용자 조작
결과를 기록했고, two-person physical validation은 사용자 요청으로 제외했다. Codex가
모터를 자동 구동하지 않으며, 사용자가 안전한 공간에서 직접 천천히 이동한 경우에만
runtime evidence로 기록한다.
