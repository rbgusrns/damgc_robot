# Stage 6.1 Survivor Registry RViz 가시성 개선

> 상태: **구현 및 자동 검증 완료, 실제 RViz 수동 검증 미실행**  
> 시작 commit: `6b3ef7a351022cfaed9ef7d8989a4280b8aa4d96`

## 1. 작업 개요

이 작업은 Stage 6.1 Registry Robustness 이후의 visualization-only 후속 개선이다.
Survivor Registry의 tracking, persistent ID, association, Tentative confirmation,
visible/LOST lifecycle 및 position filtering은 변경하지 않았다.

목표는 다음 세 가지다.

1. Registry text를 실제 생존자 위치보다 1.0 m 위에 표시한다.
2. LOST Survivor의 마지막 위치 sphere를 밝은 노란색 불투명 marker로 표시한다.
3. VISIBLE과 LOST의 text를 모두 흰색 불투명으로 통일한다.

## 2. 실제 테스트에서 발견된 문제

### 문제 A — Registry text와 nvblox mesh의 겹침

기존 Registry visualizer의 `text_z_offset` 기본값은 `0.30 m`였다. 실제 RViz에서는
Survivor text가 nvblox 3D mesh와 비슷한 높이에 놓이면서 일부 글자가 mesh 뒤에 가려지거나
끊겨 보였고, Survivor ID, XYZ, VISIBLE/LAST SEEN 상태를 빠르게 읽기 어려웠다.

### 문제 B — LOST 위치의 낮은 가시성

기존 LOST sphere는 회색 반투명 RGBA `(0.55, 0.55, 0.55, 0.45)`였고 LOST text도
회색 반투명 RGBA `(0.75, 0.75, 0.75, 0.70)`였다. 이 표현은 nvblox map 위에서
마지막 확인 위치를 즉시 찾기 어려웠다.

LOST는 삭제된 Survivor가 아니라 이미 Confirmed되었지만 현재 detection에 보이지 않는
Survivor다. 따라서 마지막 stable map position을 더 명확히 표시할 필요가 있었다.

## 3. 변경 전 실제 구현

| 항목 | 변경 전 값 |
| --- | --- |
| Registry text Z offset | `0.30 m` |
| VISIBLE sphere | RGBA `(1.0, 0.35, 0.10, 0.95)` |
| LOST sphere | RGBA `(0.55, 0.55, 0.55, 0.45)` |
| VISIBLE text | RGBA `(1.0, 1.0, 1.0, 1.0)` |
| LOST text | RGBA `(0.75, 0.75, 0.75, 0.70)` |
| sphere/text position source | `track.filtered_position` |
| sphere/text lifetime | `0`, persistent |

## 4. 해결 설계

Text marker에만 `+1.0 m` Z offset을 적용한다. Sphere는 VISIBLE과 LOST 모두
`track.filtered_position`에 그대로 두어 실제 map XYZ를 왜곡하지 않는다.

VISIBLE sphere는 기존 red/orange 계열을 유지하고 alpha만 완전 불투명 `1.0`으로 맞췄다.
LOST sphere는 밝은 노란색 RGBA `(1.0, 0.85, 0.0, 1.0)`으로 변경했다. 모든 상태의
text는 흰색 RGBA `(1.0, 1.0, 1.0, 1.0)`을 사용한다.

Marker 종류는 기존 `SPHERE + TEXT_VIEW_FACING`을 유지했다. Line marker나 새 color
parameter는 추가하지 않았다.

## 5. 좌표 불변 원칙

Sphere 위치는 항상 다음과 같다.

```text
x = track.filtered_position.x
y = track.filtered_position.y
z = track.filtered_position.z
```

Text 위치만 다음과 같다.

```text
x = track.filtered_position.x
y = track.filtered_position.y
z = track.filtered_position.z + text_z_offset
```

따라서 `text_z_offset=1.0`은 RViz 표시용 값일 뿐 raw/filtered map position,
`SurvivorTrack` message의 XYZ 또는 Registry association에는 영향을 주지 않는다.

## 6. VISIBLE 표시

- Sphere: 기존 red/orange, RGBA `(1.0, 0.35, 0.10, 1.0)`
- Text: white, RGBA `(1.0, 1.0, 1.0, 1.0)`
- Sphere 위치: current `filtered_position`
- Text 위치: `filtered_position.z + 1.0 m`
- Status 문구: `VISIBLE`

## 7. LOST / LAST SEEN 표시

- Sphere: yellow, RGBA `(1.0, 0.85, 0.0, 1.0)`
- Text: white, RGBA `(1.0, 1.0, 1.0, 1.0)`
- Sphere 위치: 마지막 `filtered_position`
- Text 위치: 마지막 `filtered_position.z + 1.0 m`
- Status 문구: `LAST SEEN`

LOST marker는 삭제하거나 원점으로 이동하지 않으며 새 위치를 추정하지 않는다.

## 8. Marker lifetime과 reset

Registry sphere와 text의 lifetime은 `(0, 0)`으로 유지된다. LOST marker도 시간이 지났다는
이유로 자동 삭제되지 않는다. Empty Registry 또는 reset snapshot에서는 기존 계약대로
`DELETEALL`을 발행하고, 일부 ID가 제거되면 해당 sphere/text에 `DELETE`를 발행한다.

Stage 5 Raw Marker의 finite lifetime과 혼동하지 않는다.

## 9. Before / After

| 항목 | Before | After | 이유 |
| --- | --- | --- | --- |
| Registry text Z offset | `0.30 m` | `1.0 m` | nvblox mesh occlusion 감소 |
| VISIBLE sphere | red/orange, alpha `0.95` | 기존 red/orange, alpha `1.0` | 현재 위치 의미 유지 및 가시성 확보 |
| LOST sphere | gray, alpha `0.45` | yellow `(1.0, 0.85, 0.0)`, alpha `1.0` | last-known position 강조 |
| VISIBLE text | white, alpha `1.0` | 유지 | 문자 가독성 유지 |
| LOST text | gray, alpha `0.70` | white, alpha `1.0` | 상태와 관계없이 문자 가독성 통일 |
| 실제 Survivor XYZ | `filtered_position` | 변경 없음 | localization 정보 보존 |
| Registry marker lifetime | `0` | `0` | mission memory 유지 |

## 10. 변경 파일과 이유

- `survivor_registry_visualizer_node.py`: offset default와 marker RGBA 변경
- `survivor_registry.yaml`: Registry visualizer의 `text_z_offset=1.0` 적용
- `test_survivor_registry_visualizer_node.py`: 위치, 색상, lifetime, multi-state 계약 검증
- `test_survivor_registry_launch.py`: YAML default 1.0 검증
- 이 문서와 문서 index: 문제, 설계, 자동/수동 검증 절차 기록

`survivor_registry_core.py`, `survivor_registry_node.py`, interfaces, Stage 5 raw visualizer,
map transform, person detector, VSLAM, EKF, nvblox는 변경하지 않았다.

## 11. 자동 테스트

| Test | 검증 계약 | 결과 |
| --- | --- | --- |
| VISIBLE position | sphere는 filtered XYZ, text는 Z+1.0 m | PASS |
| LOST position | sphere는 마지막 filtered XYZ, text는 Z+1.0 m | PASS |
| VISIBLE text | white RGBA `(1,1,1,1)` | PASS |
| LOST sphere | yellow RGBA `(1,0.85,0,1)` | PASS |
| LOST text | white RGBA `(1,1,1,1)` | PASS |
| Offset isolation | 1.0 m offset이 sphere XYZ를 바꾸지 않음 | PASS |
| Marker lifetime | VISIBLE/LOST sphere와 text 모두 0 | PASS |
| Reset/empty | `DELETEALL` 유지 | PASS |
| Multiple survivors | VISIBLE red/orange + white, LOST yellow + white | PASS |

기존 test를 삭제하거나 skip하지 않았다.

## 12. Build 및 전체 회귀 결과

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_interfaces rescue_robot_survivor
source install/local_setup.bash
colcon test --packages-select \
  rescue_robot_interfaces rescue_robot_survivor \
  --event-handlers console_direct+
colcon test-result --test-result-base build/rescue_robot_survivor --verbose
```

결과는 **114 tests, 0 errors, 0 failures, 0 skipped**다. Stage 1~6.1 detection,
depth, geometry, map transform, Stage 5 raw marker, Registry temporal logic, association,
reset, Registry marker test가 모두 계속 통과했다.

설치된 launch의 `--show-args`에서도 `text_z_offset` default `1.0`을 확인했다.
Registry launch를 하드웨어 없이 짧게 실행한 뒤 실제
`/leader/survivor_registry_visualizer`에서 `ros2 param get`과 `ros2 param dump`를 수행해
runtime 값도 `1.0`임을 확인했다. 두 node는 Ctrl+C 후 clean 종료했다.

## 13. 전체 실행 명령

```bash
# Terminal 1 — VSLAM + nvblox + RViz
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh

# Terminal 2 — Camera preprocessing
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py

# Terminal 3 — YOLO
cd ~/damgc_robot
./scripts/run_survivor_detector.sh

# Terminal 4 — Camera -> Map transform
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py

# Terminal 5 — Raw Stage 5 Visualizer
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

## 14. 수동 검증 전 reset

```bash
ros2 service call /leader/survivor/registry/reset \
  std_srvs/srv/Trigger "{}"
ros2 topic echo --once /leader/survivor/tracks \
  --qos-durability transient_local
```

Reset 직후 `tracks: []`인지 확인한다.

## 15. 관찰 및 parameter 확인

```bash
ros2 topic echo /leader/survivor/tracks \
  --qos-durability transient_local
ros2 topic echo /leader/survivor/registry_markers \
  --qos-durability transient_local
ros2 topic info -v /leader/survivor/registry_markers
ros2 node list | grep survivor
ros2 param get /leader/survivor_registry_visualizer text_z_offset
```

기대값은 `1.0`이다. 실제 node 이름이 다르면 `ros2 node list` 결과를 사용한다.

## 16. Manual Test A — VISIBLE

- sphere가 실제 filtered map position에 있는지 확인
- 기존 red/orange이며 alpha 1.0인지 확인
- text가 흰색이며 sphere보다 1.0 m 위인지 확인
- ID/XYZ/VISIBLE과 mesh occlusion 개선 여부 확인

상태: **NOT RUN**.

## 17. Manual Test B — LOST

- ID와 마지막 filtered position 유지 확인
- yellow sphere와 alpha 1.0 확인
- white LAST SEEN text와 alpha 1.0 확인
- text가 마지막 위치보다 1.0 m 위인지 확인

상태: **NOT RUN**.

## 18. Manual Test C — Re-entry

- 같은 ID 복구 확인
- sphere가 yellow에서 red/orange로 돌아오는지 확인
- text는 계속 white인지 확인
- LAST SEEN이 VISIBLE로 바뀌고 position update가 재개되는지 확인

상태: **NOT RUN**.

## 19. Manual Test D — VISIBLE + LOST 동시 표시

VISIBLE은 red/orange sphere, LOST는 yellow sphere로 즉시 구분되고 두 text는 모두 white인지
확인한다.

상태: **NOT RUN**.

## 20. 수동 검증 체크리스트

- [ ] VISIBLE sphere 실제 위치 유지
- [ ] VISIBLE sphere red/orange, alpha 1.0
- [ ] VISIBLE text white, alpha 1.0, Z+1.0 m
- [ ] text mesh occlusion 개선
- [ ] LOST sphere 마지막 위치 유지
- [ ] LOST sphere yellow, alpha 1.0
- [ ] LOST text white, alpha 1.0, Z+1.0 m
- [ ] LAST SEEN 문구 정상
- [ ] 재진입 시 same ID와 visible 색상 복귀
- [ ] filtered_position 자체 변경 없음
- [ ] Registry tracking/temporal logic regression 없음

확인하지 않은 항목은 PASS로 기록하지 않는다.

## 21. 남은 제한

1.0 m offset이 모든 camera angle과 mesh 높이에서 최적인지는 실제 RViz 검증이 필요하다.
White text는 밝은 배경과 겹칠 수 있다. 이번 단계에서는 background panel, line marker,
outline 또는 상태별 text 색상을 추가하지 않았다.

## 22. 판정

코드, build, launch default 및 자동 테스트 기준으로 Registry Visualization Improvement는
**PASS**다. 실제 VISIBLE/LOST/re-entry 렌더링과 mesh occlusion 개선은 아직 수행하지
않았으므로 hardware/RViz 최종 판정은 **NOT RUN**이다.

## 23. Rollback 및 Git

자동 commit과 push는 수행하지 않는다. 변경을 commit한 뒤 되돌릴 때는 해당 commit을
`git revert`한다. broad reset은 사용하지 않는다.

권장 commit message:

```bash
git commit -m "Improve survivor registry RViz visibility"
```
