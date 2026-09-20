# 생존자 VSLAM 지도 통합 — Stage 6.1 Registry 안정성 검증

> 상태: **PHASE C — 시간 정책 자동 검증 및 전체 회귀 검증 완료**  
> 수정 후 실물 검증은 아직 시작하지 않았다.  
> 기준 commit: `9a71d99191a9f23e887a176ac76730953a2814f6`

## Stage 6.1이 필요했던 이유

Stage 6에서는 Stage 5의 raw map detection 위에 임무 실행 중 유지되는 Persistent
Survivor Registry를 추가했다. Tentative 추적, 공간 기반 one-to-one association,
사용자에게 공개되는 Survivor ID, LOST 및 재연결, 위치 EMA, reset, typed track message,
영구 RViz marker가 이 단계에서 구현되었다. 이 기능들은 정상 동작하고 있으며,
Stage 6.1은 새로운 전체 tracking 시스템으로 대체하는 작업이 아니다.

그러나 Jetson과 RealSense D435를 사용한 실제 물리 테스트에서 confirmation 정책의
안정성 문제가 확인되었다. Stage 6은 Tentative Track에 detection 3개가 누적되는 즉시
Confirmed로 승격한다. 따라서 의자나 다른 사물이 YOLO에서 약 1초 동안 사람으로
오검출되면 3-hit 조건을 만족해 영구 public ID를 받을 수 있다. 일단 승격된 ID와 marker는
관측이 LOST가 된 뒤에도 유지되는 것이 Registry의 의도된 동작이다. 실제 사람 3명이 있던
물리 테스트 환경에서는 과거에 잘못 생성된 ID가 누적되어 실제 사람 수보다 많은 Survivor
ID가 RViz에 표시된 사례가 있었다.

두 번째 문제는 detection dropout이다. 현재 Confirmed Survivor는 마지막 detection 이후
2초를 초과하면 LOST가 된다. 짧은 YOLO miss, 부분 가림, 로봇 회전, invalid depth 또는
bbox drop만으로도 물리 운용에서 원하는 시점보다 일찍 visible 상태가 바뀔 수 있다.
움직이던 사람이 detection에서 사라진 뒤 충분히 이동한 위치에서 다시 나타나면 고정된
reassociation gate를 벗어나 새 ID가 만들어질 가능성도 있다. Stage 6.1에서는 먼저 시간
기반 confirmation과 dropout tolerance를 개선한다. motion prediction, adaptive gate,
track merge, appearance Re-ID, ByteTrack, BoT-SORT는 이번 단계에서 즉시 추가하지 않는다.

기존 spatial association 값이 근본 원인이라고 가정하지 않고 별도 재테스트도 수행했다.
정지 상태의 사람 #1과 #2가 있고 로봇 자체가 이동하여 VSLAM/map 좌표가 변하는 조건에서도
두 사람은 새 ID로 분리되지 않고 각자의 persistent ID를 유지했다. 이 결과를 근거로
Stage 6.1에서는 `association_radius_m=0.50`, `reassociation_radius_m=0.75`,
`position_ema_alpha=0.50`을 유지하고 시간 정책 변경 효과만 분리해 확인한다.

이 문서에서는 아직 어떤 Stage 6.1 구현 또는 검증 결과도 PASS로 기록하지 않는다.
위의 짧은 false positive 현상과 정지한 두 사람 및 이동 로봇 재테스트는 문제와 해결 범위를
정의하기 위한 수정 전 관측 결과다.

## Phase A 저장소 기준 상태

Phase A에서는 `main` branch의 다음 commit을 조사했다.

```text
9a71d99191a9f23e887a176ac76730953a2814f6
9a71d99 fear(survivor): Add persistent survivor registry and spatial association
```

이 문서를 생성하기 전 worktree는 clean 상태였다. Stage 6.1 source를 변경하기 전에 다음
명령으로 구현 기준선을 확인했다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select \
  rescue_robot_interfaces rescue_robot_survivor \
  --event-handlers console_direct+
colcon test-result --test-result-base build/rescue_robot_survivor --verbose
```

결과는 **96 tests, 0 errors, 0 failures, 0 skipped**다. 이 결과는 수정 전 회귀
기준선이며 Stage 6.1 구현 완료를 증명하는 결과가 아니다.

## 현재 Stage 6 구현 구조

Registry의 역할은 다음 파일로 분리되어 있다.

- `survivor_registry_core.py`: ROS에 의존하지 않는 candidate association, Tentative 및
  Confirmed 상태, 승격, LOST 전환, EMA, snapshot, reset을 담당한다.
- `survivor_registry_node.py`: map frame의 `PoseArray` 입력을 검증하고, ROS timestamp를
  core에 전달하며, transient-local typed track을 발행한다. 주기적인 lifecycle timer와
  reset service도 제공한다.
- `survivor_registry.launch.py`: YAML 기본값을 읽고 Registry와 visualizer의 모든 설정을
  launch argument로 노출한다.
- `survivor_registry.yaml`: launch가 사용하는 runtime 기본값을 정의한다.
- `survivor_registry_visualizer_node.py`: Confirmed/LOST typed track을 persistent marker로
  변환한다. confirmation, association, expiry 또는 LOST 시점은 결정하지 않는다.

Public 데이터 흐름은 다음과 같다.

```text
/leader/survivor/map_positions
        -> /leader/survivor_registry
        -> /leader/survivor/tracks
        -> /leader/survivor_registry_visualizer
        -> /leader/survivor/registry_markers
```

Tentative Track은 core 내부에만 존재하며 public Survivor ID와 marker가 없다.

## 현재 Tentative 의미와 동작

`_create_tentative()`는 다음 상태를 초기화한다.

- `first_seen_ns`: 처음 연결된 detection timestamp
- `last_seen_ns`: 생성 시점에는 `first_seen_ns`와 같은 timestamp
- `hit_count`: `1`
- raw position과 filtered position: 첫 detection 위치

이후 공간적으로 연결된 detection이 들어오면 raw/filtered position을 갱신하고,
`last_seen_ns`를 해당 detection timestamp로 바꾸며, `hit_count`를 1 증가시킨다.

현재 expiry 조건은 다음과 같다.

```text
timestamp_ns - track.first_seen_ns > tentative_timeout_sec
```

따라서 `tentative_timeout_sec=2.0`은 `last_seen`부터 측정하는 최대 detection gap이 아니라
`first_seen`부터 측정하는 전체 lifetime이다. detection이 계속 들어와도 lifetime은
연장되지 않는다. 비교 연산자가 strict `>`이므로 정확히 2.0초 경계에서는 Tentative가
유지되고, 이를 초과한 뒤 삭제된다.

`SurvivorRegistry.update()`는 Confirmed 또는 Tentative association을 수행하기 전에
`advance_time(detection_timestamp)`을 호출한다. 따라서 association 전에 expiry를
수행하는 구조 자체는 이미 존재한다. Stage 6.1에서는 Tentative expiry 기준을
`first_seen`에서 `last_seen`으로 변경하고, 그 의미를 명확히 나타내는 parameter 이름으로
바꾸어야 한다.

ROS node는 유효한 입력 message를 처리한 뒤 ROS clock의 현재 시각으로
`advance_time(now)`을 한 번 더 호출한다. 입력 callback과 독립적으로 동작하는 2 Hz timer도
`advance_time(now)`을 호출하고 snapshot을 발행한다. 따라서 detection message가 더 이상
들어오지 않아도 Tentative expiry와 Confirmed LOST 전환은 계속 수행된다.

## 현재 Confirm 조건

`_promote_confirmed()`에 존재하는 유일한 승격 조건은 다음과 같다.

```text
track.hit_count >= confirm_hits
```

기본값은 `confirm_hits=3`이다. 숨겨진 최소 관측 시간, frame 간격, confidence,
appearance 또는 velocity 조건은 없다. 승격 시 core는 다음 public Survivor ID를 할당하고,
Tentative의 first/last timestamp와 position을 복사하며, `observation_count`를 Tentative의
hit count로 설정한 뒤 Tentative Track을 삭제한다.

따라서 빠르게 연속된 detection 3개와 충분한 시간 동안 유지된 detection 3개를 동일하게
취급하는 것이 물리 테스트에서 관찰된 문제의 직접적인 원인임을 확인했다.

## 현재 Confirmed 및 LOST 동작

Visible Confirmed Track은 다음 조건에서 LOST가 된다.

```text
timestamp_ns - track.last_seen_ns > visible_timeout_sec
```

기본값은 `visible_timeout_sec=2.0`이다. 정확히 2.0초 경계에서는 track이 visible 상태로
유지된다. 경계를 초과한 뒤 실행되는 첫 update 또는 timer tick에서 `visible=false`,
`status=LOST`로 바뀐다. Registry는 track을 삭제하거나 저장된 raw/filtered position을
변경하지 않는다. 이후 LOST reassociation radius 안에 detection이 들어오면 같은 track을
visible/CONFIRMED로 복구하고 public ID도 유지한다.

주기적인 ROS timer가 입력 callback과 독립적으로 이 전환을 수행하므로, LOST 전환을 위해
새 detection array가 들어올 필요는 없다.

## 현재 parameter와 보호할 값

| Parameter | 현재 기본값 | 현재 역할 | Stage 6.1 방향 |
| --- | ---: | --- | --- |
| `confirm_hits` | `3` | 유일한 승격 threshold | 명시적인 최소 hit와 최소 관측 시간 조건으로 교체 |
| `tentative_timeout_sec` | `2.0 s` | `first_seen` 기준 전체 lifetime | `last_seen` 기준 최대 gap 의미로 교체 |
| `visible_timeout_sec` | `2.0 s` | 마지막 detection부터 LOST까지의 시간 | 시간 grace 증가, Phase A에서는 미구현 |
| `association_radius_m` | `0.50 m` | visible/Tentative XY gate | 보호 및 유지 |
| `reassociation_radius_m` | `0.75 m` | LOST Track XY gate | 보호 및 유지 |
| `position_ema_alpha` | `0.50` | XYZ EMA에서 새 sample의 가중치 | 보호 및 유지 |

Visible Confirmed Track은 raw position과 0.50 m gate를 사용한다. LOST Track은 filtered
position과 0.75 m gate를 사용한다. Tentative Track도 0.50 m gate를 사용한다. Candidate
pair를 정렬한 뒤 one-to-one으로 선택하므로 detection 하나가 여러 track을 갱신하거나 track
하나가 같은 array의 여러 detection을 소비할 수 없다.

## Parameter reference 전체 조사 결과

Phase A의 repository 전체 검색에서 runtime 또는 검증 reference가 발견된 위치는 다음과
같다.

- `rescue_robot_survivor/survivor_registry_core.py`: 기본값, validation, expiry,
  association, promotion, EMA 동작
- `rescue_robot_survivor/survivor_registry_node.py`: ROS parameter 선언과
  `RegistryConfig` 변환
- `launch/survivor_registry.launch.py`: float/int launch type과 override
- `config/survivor_registry.yaml`: 배포 기본값
- `test/test_survivor_registry_core.py`: lifecycle, association, filtering, reset,
  invalid config contract
- `test/test_survivor_registry_node.py`: 즉시 confirmation을 위해 `confirm_hits=1`을
  사용하는 callback/timer harness
- `test/test_survivor_registry_launch.py`: 정확한 YAML 기본값과 launch 노출 여부
- 기존 Stage 6 validation 문서: Stage 6 당시 동작과 검증 결과
- package README: association/reassociation tuning을 기존 다음 단계로 설명

Root README에는 Stage 6 architecture와 runtime 명령이 있지만 현재 여섯 Registry
parameter 이름을 직접 나열하지는 않는다.

## Phase B 변경 예상 범위와 보호 파일

다음 goal에서 구현 및 test 변경이 예상되는 파일은 다음과 같다.

- `survivor_registry_core.py`
- `survivor_registry_node.py`
- `config/survivor_registry.yaml`
- `launch/survivor_registry.launch.py`
- `test_survivor_registry_core.py`
- `test_survivor_registry_node.py`
- `test_survivor_registry_launch.py`
- 이 Stage 6.1 validation 문서와 관련 README/index link

이후 goal이 좁은 범위의 문서 link나 회귀 test 조정을 명시적으로 요구하지 않는 한 다음
동작과 파일은 보호한다.

- `survivor_registry_visualizer_node.py`와 marker semantics
- `SurvivorTrack.msg` 및 `SurvivorTrackArray.msg` wire contract
- Stage 5 raw map visualizer 구현과 test
- person detector, depth, camera XYZ, map transform, VSLAM, dual EKF, nvblox, RViz,
  robot control 코드
- association/reassociation radius 및 EMA algorithm/default
- topic 이름, QoS, reset service, persistent ID 할당, one-to-one matching, LOST Track 유지

기존 Stage 6 validation 문서는 최초 구현의 역사적 검증 자료다. 당시 사용한 old parameter
값을 현재 값인 것처럼 조용히 고쳐 쓰면 안 된다. 이후 documentation 단계에서는 기존
문서에서 이 Stage 6.1 후속 문서로 연결하고, Before/After 의미를 명확히 분리해야 한다.

## Phase A 결론

Phase A 조사 결과, 보고된 false positive 문제는 hit count만 사용하는 승격 조건에서
직접 발생하며, 현재 Tentative timeout은 `first_seen`을 기준으로 한다. 입력이 없을 때도
expiry를 수행할 독립 timer 위치는 이미 존재한다. 또한 수정 전 전체 회귀 기준선으로
96개 test가 모두 통과함을 확인했다.

Stage 6.1 구현은 아직 시작하지 않았다. Phase A에서는 source, launch, config, test 또는
runtime parameter를 변경하지 않았다.

## Phase B 구현

Phase B에서는 기존 spatial association, one-to-one matching, EMA, public message, marker
구조를 유지하고 Tentative confirmation과 detection loss 시간 정책만 변경했다.

### 기존 알고리즘과 새 알고리즘

기존 알고리즘은 detection 사이의 시간 간격이나 전체 관측 시간을 확인하지 않았다.

```text
기존:
candidate
  -> Tentative
  -> hit_count >= 3
  -> Confirmed Survivor ID 발급
```

새 알고리즘은 association 전에 마지막 detection 기준의 continuity를 먼저 확인하고,
최소 hit와 최소 관측 시간을 모두 만족할 때만 승격한다.

```text
변경 후:
candidate
  -> Tentative
  -> 현재 시각 - last_seen > 0.8초이면 Tentative 삭제
  -> 유효한 Tentative와 spatial association
  -> hit_count >= 4 확인
  -> last_seen - first_seen >= 2.0초 확인
  -> 두 조건을 모두 만족하면 Confirmed Survivor ID 발급
```

`hit_count`와 관측 시간은 AND 조건이다. 둘 중 하나만 만족하는 Tentative는 Confirmed로
승격하지 않는다. 또한 두 detection 사이의 gap이 0.8초를 초과하면 기존 Tentative를 먼저
삭제하므로, 오랫동안 끊긴 두 관측이 2초 이상의 duration으로 잘못 합쳐지지 않는다.

### 새 parameter와 기본값

| Parameter | 기본값 | 의미 |
| --- | ---: | --- |
| `confirm_min_duration_sec` | `2.0 s` | `last_seen - first_seen`으로 계산하는 최소 관측 기간 |
| `confirm_min_hits` | `4` | 승격 전 필요한 최소 detection 수 |
| `tentative_max_gap_sec` | `0.8 s` | Tentative의 마지막 detection 이후 허용하는 최대 공백 |
| `visible_timeout_sec` | `4.0 s` | Confirmed Track을 LOST로 바꾸기 전 허용하는 detection 공백 |
| `association_radius_m` | `0.50 m` | 기존 visible/Tentative XY gate, 변경 없음 |
| `reassociation_radius_m` | `0.75 m` | 기존 LOST XY gate, 변경 없음 |
| `position_ema_alpha` | `0.50` | 기존 XYZ EMA 가중치, 변경 없음 |

`tentative_timeout_sec`와 `confirm_hits`는 deprecated alias 없이 각각
`tentative_max_gap_sec`와 `confirm_min_hits`로 완전 migration했다. Core, ROS node, YAML,
launch 및 test의 runtime key를 동시에 변경했으며 두 이름이 서로 다른 의미로 함께 적용되는
상태를 만들지 않았다. 기존 이름은 Stage 6 당시 동작을 설명하는 역사 문맥과 이 migration
설명에서만 언급한다.

### 처리 순서와 경계 조건

`SurvivorRegistry.update()`는 기존과 마찬가지로 association보다 먼저
`advance_time(detection_timestamp)`을 호출한다. `advance_time()`의 Tentative 기준을
`first_seen`에서 `last_seen`으로 바꿨으므로 stale Tentative는 새 detection과 연결되기 전에
삭제된다. ROS node의 독립적인 2 Hz timer도 같은 `advance_time(now)` 경로를 사용하므로
입력 callback이 멈춰도 stale Tentative가 남지 않는다.

경계 조건은 다음과 같이 고정했다.

- `last_seen - first_seen >= confirm_min_duration_sec`: 정확히 2.0초일 때 승격 허용
- `now - last_seen > tentative_max_gap_sec`: 정확히 0.8초일 때 유지, 초과하면 삭제
- `now - last_seen > visible_timeout_sec`: 정확히 4.0초일 때 visible 유지, 초과하면 LOST

LOST 전환은 기존과 동일하게 Track, public ID, raw position, filtered position을 삭제하거나
이동시키지 않는다. 이후 기존 0.75 m gate 안에서 detection되면 같은 ID로 복구된다.

### Parameter validation

현재 project의 기존 convention대로 잘못된 설정은 `RegistryConfig` 생성 중 `ValueError`로
거부한다.

- `confirm_min_duration_sec`: finite이며 0 이상
- `confirm_min_hits`: bool이 아닌 정수이며 1 이상
- `tentative_max_gap_sec`, `visible_timeout_sec`: finite이며 0보다 큼
- `association_radius_m`, `reassociation_radius_m`: finite이며 0보다 큼
- `position_ema_alpha`: finite이며 `0 < alpha <= 1`

`confirm_min_duration_sec=0`은 명시된 계약에 따라 허용한다. 기본 runtime은 2.0초를
사용하며, 0은 confirmation delay와 무관한 association 단위 test에서만 명시적으로 사용한다.

### Logging 결정

Registry core는 ROS import와 logger가 없는 순수 Python 모듈이며, 현재 `UpdateResult`도
상태 전이 event를 외부로 노출하지 않는다. Phase B에서는 logging만을 위해 core public
interface나 association loop에 event plumbing을 추가하지 않았다. 상태 전이는 deterministic
core test와 `/leader/survivor/tracks`의 typed 상태로 확인한다. 과도한 INFO log도 추가하지
않았다.

### Phase B 자동 검증

다음 명령으로 수정된 두 package를 다시 build하고 전체 test를 실행했다.

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

결과는 **114 tests, 0 errors, 0 failures, 0 skipped**다. 기존 96개 test를 삭제하거나
skip하지 않았고 다음 Stage 6.1 contract를 추가로 확인했다.

- 1초 이내 4-hit와 2초 미만의 다수 hit는 Confirmed로 승격하지 않음
- duration만 만족하고 hit가 부족하면 승격하지 않음
- hit와 정확히 2.0초 duration을 모두 만족하면 승격
- 정확히 0.8초 gap은 유지하고 0.8초 초과 gap은 association 전에 expiry
- Tentative expiry가 `first_seen`이 아닌 `last_seen` 기준임
- 긴 시간 간격으로 continuity가 끊긴 두 detection은 승격하지 않음
- 4.0초 경계까지 visible 유지, 4.0초 초과 시 LOST
- LOST position 유지와 기존 ID 재연결
- 0.50/0.75 radius와 EMA 0.50 기본값 유지
- 기존 two-track, input order, one-to-one association, reset test 유지
- 입력 callback이 없어도 ROS timer가 stale Tentative를 삭제
- Stage 5 raw visualizer 및 Registry visualizer test 유지

설치된 launch의 `--show-args`에서도 다음 기본값을 확인했다.

```text
confirm_min_duration_sec = 2.0
confirm_min_hits = 4
tentative_max_gap_sec = 0.8
visible_timeout_sec = 4.0
association_radius_m = 0.5
reassociation_radius_m = 0.75
position_ema_alpha = 0.5
```

이 결과는 Phase B source 및 자동 회귀 검증 PASS를 의미한다. Jetson + D435 실물 검증은
아직 수행하지 않았으며 별도 후속 단계에서 확인해야 한다.

## Phase C 자동 테스트 상세

Phase C에서는 새 시간 정책의 각 failure mode와 기존 Stage 1~6 기능 회귀를 명시적으로
대조했다. Core timestamp는 0보다 커야 하므로 TEST A의 `0.0/0.3/0.6/0.9초`는 동일한
상대 간격을 유지한 `1.0/1.3/1.6/1.9초`로 실행했다. 관측 duration과 gap은 원래 재현
조건과 동일하다.

| 구분 | 재현하는 failure mode와 필요한 이유 | 기대 결과 | 실제 결과 |
| --- | --- | --- | --- |
| TEST A | 0.9초 안에 4-hit가 들어온 의자 등의 짧은 false positive. 기존 hit-only 정책의 실제 문제 재현 | public ID 생성 금지, Tentative 유지 | PASS: Confirmed 없음, Tentative 1개 |
| TEST B | 2초 duration은 만족하지만 `confirm_min_hits`가 부족한 경우. duration 단독 승격 방지 | Confirm 금지 | PASS: `confirm_min_hits=5`에서 4-hit 미승격 |
| TEST C | 모든 gap이 0.8초 이하이고 duration 2초와 4-hit를 모두 만족하는 실제 사람 조건 | Confirmed ID 1개 | PASS: 정확히 2.0초 경계에서 ID 1 발급 |
| TEST D | detection gap이 `0.8초 + 1ns`인 끊긴 관측. 오래된 Tentative 재사용 방지 | association 전에 old Tentative 삭제, 새 Tentative로 시작 | PASS: Confirmed 없음, 새 Tentative만 1개 |
| TEST E | gap이 정확히 0.8초인 경계 | 기존 Tentative 유지 | PASS: 두 hit가 같은 Track에 누적되어 승격 |
| TEST F | first_seen 이후 2초가 넘었지만 last_seen gap은 0.7초인 연속 관측 | Tentative 유지 | PASS: Tentative 1개 유지 |
| TEST G | Confirmed Track의 마지막 detection 후 3.9초 | LOST 금지 | PASS: visible/CONFIRMED 유지 |
| TEST H | visible timeout 정확히 4.0초인 경계 | strict `>` 정책에 따라 visible 유지 | PASS: visible/CONFIRMED 유지 |
| TEST I | visible timeout보다 1ns 초과 | LOST 전환 | PASS: `visible=false`, `status=LOST` |
| TEST J | LOST 전환 과정에서 영구 정보가 지워지거나 움직이는 회귀 | ID 및 raw/filtered position 유지 | PASS: ID와 두 position 모두 동일 |
| TEST K | LOST 위치에서 0.75 m 이내 재검출 | 새 ID가 아닌 기존 ID 복구 | PASS: ID 1, visible/CONFIRMED, count 증가 |
| TEST L | 시간 정책 수정 중 spatial tuning이 바뀌는 회귀 | 0.50/0.75 유지 | PASS: default와 launch 모두 일치 |
| TEST M | position filtering 값이 바뀌는 회귀 | EMA alpha 0.50 유지 | PASS: default 0.50 및 XYZ EMA 계산 test 통과 |
| TEST N | 두 정지 Track의 detection 배열 순서가 바뀌는 association 회귀 | 위치별 기존 ID와 one-to-one 유지 | PASS: 두 ID 유지, 한 detection의 다중 Track 갱신 없음 |
| TEST O | reset 뒤 state나 ID counter가 남는 회귀 | 모든 Track 삭제, next ID 1 | PASS: Tentative/Confirmed/timestamp guard 초기화 |
| TEST P | NaN/Inf position, invalid/duplicate timestamp, 잘못된 parameter | 안전하게 skip 또는 명확한 validation error | PASS: valid detection 보존, invalid 입력 거부 |

특히 TEST A의 **“4 hits in <1 sec should NOT confirm”**은 Stage 6.1을 시작하게 만든
짧은 오검출의 자동 재현 test다. Hit 수가 이미 4개여도 duration이 0.9초뿐이므로 public
Survivor ID가 만들어지지 않음을 확인했다.

### 전체 package 회귀 결과

일반적인 증분 build를 사용했으며 사용자 파일을 지우는 clean이나 destructive reset은
사용하지 않았다. `rescue_robot_interfaces`와 `rescue_robot_survivor`를 build한 뒤 전체
test를 실행했다.

```text
총 test: 114
통과: 114
실패: 0
오류: 0
skip: 0
```

전체 결과에는 detection/depth/geometry, person detector launch와 node, Camera XYZ,
Map XYZ/TF2 transform, Stage 5 raw marker, Stage 6 Registry core/node/launch 및 Registry
marker formatting test가 모두 포함된다. 기존 test를 삭제하거나 skip하지 않았다.
VSLAM, EKF, nvblox는 이번 package test의 직접 실행 대상은 아니지만 관련 source/config는
diff에서 변경되지 않았다.

### 실제 launch parameter 검증

하드웨어 없이 Registry launch만 짧게 실행해 `/leader/survivor_registry`에서
`ros2 param dump`를 확인했다.

```yaml
/leader/survivor_registry:
  ros__parameters:
    association_radius_m: 0.5
    confirm_min_duration_sec: 2.0
    confirm_min_hits: 4
    position_ema_alpha: 0.5
    reassociation_radius_m: 0.75
    tentative_max_gap_sec: 0.8
    visible_timeout_sec: 4.0
```

Registry와 Registry visualizer는 정상 기동했으며 Ctrl+C 후 두 process 모두 clean
종료했다. 이 검증은 YAML/launch/node parameter 전달과 launch syntax에 대한 PASS다.
Camera, YOLO, VSLAM, EKF, nvblox 및 물리 환경 동작을 검증한 결과로 해석하지 않는다.

### Parameter 검색 결과

`tentative_timeout_sec`, `tentative_max_gap_sec`, `confirm_min_duration_sec`,
`confirm_min_hits`, `visible_timeout_sec`를 repository 전체에서 검색했다. Core, node,
config, launch, test에는 새 parameter만 존재한다. `tentative_timeout_sec`와 기존
`confirm_hits`는 과거 Stage 6 validation 기록, 이 문서의 Before 설명 및 migration
설명에서만 남아 있으며 현재 runtime key로 사용되지 않는다.

## 1. Stage 6.1 개요

Stage 6의 persistent Registry를 보존하면서 confirmation과 일시적인 detection loss의
시간 semantics만 보강한 최소 수정 단계다.

## 2. 왜 보완 개발이 필요했는가

3-hit-only 승격과 2초 visible grace가 실제 물리 환경에서 너무 공격적이었다.

## 3. Stage 6 초기 설계

Raw map detection을 spatially association하고 Tentative를 3-hit 후 Confirmed로 바꾸는
설계였다.

## 4. 초기 Parameter

초기값은 `confirm_hits=3`, `tentative_timeout_sec=2.0`, `visible_timeout_sec=2.0`,
`association_radius_m=0.50`, `reassociation_radius_m=0.75`, `position_ema_alpha=0.50`이었다.

## 5. 실제 테스트에서 발견된 문제

짧은 사람 오검출과 일시적인 검출 중단이 persistent ID lifecycle을 불안정하게 만들었다.

## 6. 3명의 실제 사람인데 더 많은 Persistent ID가 누적된 현상

과거 false positive로 생성된 ID가 Registry에 유지되어 실제 사람 수보다 많은 ID가 RViz에
표시된 사례가 있었다.

## 7. 약 1초 의자 false positive가 persistent ID로 남은 현상

의자가 약 1초 동안 3회 이상 person으로 검출되면 duration과 무관하게 ID가 발급되었다.

## 8. moving survivor + temporary detection loss에서 ID split 가능성

움직이는 사람이 잠시 detection에서 사라진 뒤 gate 밖에서 재검출되면 새 ID가 생길 수 있었다.

## 9. 문제 재현 과정

물리 테스트에서 ID 누적, 의자 오검출, dropout 후 재연결 가능성을 관찰하고 자동 sequence로
각 시간 조건을 분리해 재현했다.

## 10. 초기 가설

초기에는 spatial association radius가 작아 동일 인물이 분리된다는 가설도 검토했다.

## 11. Association radius가 원인인지 검토

`0.50/0.75 m` gate와 one-to-one matching이 실제 ID 유지에 충분한지 별도 재테스트했다.

## 12. 정지 2인 + 이동 로봇 재테스트

사람 #1/#2를 정지시키고 로봇만 이동하는 조건에서 VSLAM/map 좌표 변화와 ID를 관찰했다.

## 13. 재테스트에서 0.50/0.75 m association이 정상적으로 ID를 유지한 결과

두 사람 모두 기존 ID를 유지했고 새 ID로 잘못 분리되지 않았다. 이 결과는 수정 전 물리
관찰이며 Stage 6.1 후속 물리 회귀 test는 아직 실행하지 않았다.

## 14. 따라서 association radius를 우선 유지하기로 한 이유

radius를 키우면 가까운 다른 사람을 잘못 연결할 위험이 있어, 원인이 확인된 시간 정책부터
변경하기로 했다.

## 15. 실제 root cause 분석

핵심 원인은 hit 수만 보는 promotion predicate, `first_seen` lifetime expiry, 2초 grace였다.

## 16. 3-hit-only confirmation의 문제

3회가 짧은 시간에 발생해도 실제 사람인지 구분하지 못했다.

## 17. first_seen 기반 tentative timeout의 문제

정상 detection이 계속되어도 전체 lifetime이 끝나므로 continuity 판단과 의미가 달랐다.

## 18. 2초 visible timeout의 한계

짧은 miss나 부분 가림만으로 Confirmed Track이 빠르게 LOST가 될 수 있었다.

## 19. 고려했던 해결안들

시간 기반 confirmation, gap 기반 expiry, visible grace 연장을 선택지로 검토했다.

## 20. association radius 확대를 즉시 적용하지 않은 이유

재테스트에서 기존 gate가 정상적으로 ID를 유지했으므로 원인 분리와 오연결 위험을 위해
변경하지 않았다.

## 21. velocity prediction을 이번 단계에서 제외한 이유

이번 문제는 우선 temporal lifecycle로 완화 가능하며, 속도 추정은 다음 보완 단계로 보류했다.

## 22. adaptive gate를 이번 단계에서 제외한 이유

gate 변화가 association 오류와 ID split을 혼합할 수 있어 이번 최소 수정 범위에서 제외했다.

## 23. promotion duplicate suppression을 보류한 이유

중복 merge 정책은 별도 identity 문제를 만들 수 있어 현재 confirmation 수정의 후속 과제로 남겼다.

## 24. 최종 선택한 해결 방안

`confirm_min_duration_sec=2.0`, `confirm_min_hits=4`, `tentative_max_gap_sec=0.8`,
`visible_timeout_sec=4.0`을 적용했다.

## 25. Time-based confirmation

hit 조건과 duration 조건을 AND로 적용해 짧은 false positive가 public Registry에 들어가는
것을 막는다.

## 26. confirm_min_duration_sec

단위는 초이며 `last_seen - first_seen >= 2.0`일 때만 duration 조건을 만족한다.

## 27. confirm_min_hits

최소 detection 수는 4회이며 duration만으로는 승격하지 않는다.

## 28. tentative_max_gap_sec

Tentative에서 마지막 detection 이후 0.8초를 초과하면 continuity가 끊긴 것으로 판단한다.

## 29. last_seen 기반 gap semantics

expiry는 `now - last_seen > tentative_max_gap_sec`이고, 정상 detection이 들어오면 last_seen이
갱신되어 track이 유지된다.

## 30. visible timeout 4초

Confirmed Track은 마지막 detection으로부터 4초까지 visible을 유지하고 초과 시 LOST가 된다.

## 31. 변경하지 않은 association/EMA 값

association radius는 0.50/0.75 m, EMA alpha는 0.50으로 유지했다.

## 32. Before vs After Parameter 표

### 문제 → 원인 → 해결 → 검증 통합 표

| 문제 | 실제 관찰 | 원인 | 해결 | 자동 검증 | 물리 검증 상태 |
| --- | --- | --- | --- | --- | --- |
| 약 1초 동안 발생한 의자 false positive가 persistent Survivor ID로 등록됨 | 의자 등의 사물이 짧게 person으로 오검출되어도 3-hit를 만족하면 ID와 marker가 Registry에 계속 남았음 | 기존에는 `confirm_hits=3`만 확인하고 관측 duration을 검사하지 않았음 | `confirm_min_duration_sec=2.0`과 `confirm_min_hits=4`를 AND 조건으로 적용 | TEST A PASS: 0.9초 안에 4-hit가 발생해도 Confirmed Survivor가 생성되지 않음. TEST B/C에서 duration과 hit가 모두 필요함을 추가 확인 | **NOT RUN** — 자연스러운 physical false positive가 재현되면 확인하며, 강제로 위험하거나 비현실적인 환경을 만들지 않음 |
| Tentative continuity의 의미가 불명확하고 오래된 관측이 이어질 가능성 | 기존 timeout은 마지막 정상 detection 이후의 공백이 아니라 Tentative가 처음 생성된 시점부터의 전체 lifetime을 제한했음 | `tentative_timeout_sec`가 `first_seen` 기준으로 구현되어 detection continuity와 다른 의미였음 | `tentative_max_gap_sec=0.8`로 migration하고 `now - last_seen > tentative_max_gap_sec`일 때 association 전에 삭제 | TEST D/E/F PASS: 0.8초 초과 시 old Tentative 삭제, 정확히 0.8초에서는 유지, first_seen에서 오래 지나도 last_seen gap이 짧으면 유지. ROS timer expiry test도 PASS | **NOT RUN** — 실제 detector dropout sequence는 사용자 물리 검증에서 확인 예정 |
| 짧은 detection dropout 뒤 Confirmed Survivor가 너무 빨리 LOST로 전환됨 | YOLO miss, 부분 가림, 로봇 회전, invalid depth 또는 bbox drop이 약 2초를 넘으면 LOST가 될 수 있었음 | `visible_timeout_sec=2.0`이 실제 물리 환경의 일시적인 detection loss를 흡수하기에 짧았음 | `visible_timeout_sec=4.0`으로 늘리고 Track, ID, raw/filtered position 유지 semantics는 보존 | TEST G/H/I/J/K PASS: 3.9초와 정확히 4.0초에서는 visible 유지, 4초 초과 시 LOST, 위치와 ID 유지, 근처 재검출 시 같은 ID 복구 | **NOT RUN** — 짧은 가림, 4초 이상 FOV 이탈, 재진입을 사용자가 순서대로 확인 예정 |
| 동일 인물의 duplicate ID가 association radius 부족 때문에 발생할 가능성 | 초기 물리 테스트에서는 moving survivor dropout 뒤 ID split 가능성이 있었으나, 별도 재테스트에서는 정지한 사람 #1/#2와 이동 로봇 조건에서 두 ID가 안정적으로 유지됨 | 모든 문제를 spatial gate 하나의 원인으로 단정할 근거가 부족했고, 시간 정책 문제가 코드에서 직접 확인됨 | `association_radius_m=0.50`, `reassociation_radius_m=0.75`, `position_ema_alpha=0.50`을 유지하고 temporal lifecycle만 우선 수정 | TEST L/M/N PASS: 기본 radius와 EMA 유지, two-track 및 reversed detection order의 one-to-one association 유지 | 수정 전 재테스트 **PASS**, Stage 6.1 수정 후 physical regression은 **NOT RUN** |

이 표의 자동 검증 PASS는 deterministic core/node test 결과를 의미한다. Jetson + D435
물리 검증 결과로 확대 해석하지 않으며, 사용자가 TEST A~G를 수행한 뒤 마지막 열만 실제
관찰값으로 갱신한다.

| 항목 | Before | After | 이유 |
| --- | --- | --- | --- |
| 승격 hit | `confirm_hits=3` | `confirm_min_hits=4` | duration과 함께 짧은 오검출 억제 |
| 승격 시간 | 없음 | `confirm_min_duration_sec=2.0 s` | 최소 관측 지속성 확보 |
| Tentative timeout | `first_seen` 기준 2.0 s | `last_seen` 기준 `tentative_max_gap_sec=0.8 s` | continuity 판단 |
| visible grace | 2.0 s | 4.0 s | 일시 dropout 흡수 |
| spatial gate | 0.50/0.75 m | 0.50/0.75 m | 정상 동작 재확인 |
| EMA | 0.50 | 0.50 | filtering 원인과 시간 정책 분리 |

## 33. Old State Machine

```text
Raw Detection -> Tentative -> hit_count >= 3 -> Confirmed
```

## 34. New State Machine

```text
Raw Detection -> Tentative
  -> stale gap check
  -> hits >= 4 AND duration >= 2.0 s
  -> Confirmed
```

Confirmed는 4초 초과 dropout에서 LOST가 되며 Registry track은 삭제되지 않는다.

## 35. Algorithm pseudocode

```text
before association:
    delete tentative when now - last_seen > tentative_max_gap_sec
associate visible/lost confirmed tracks using existing gates
associate remaining detections to remaining tentative tracks
create tentative tracks for unmatched detections
promote only when hits >= confirm_min_hits
             and last_seen - first_seen >= confirm_min_duration_sec
timer:
    apply the same expiry and visible-loss checks
```

## 36. 코드 변경 파일

Core, ROS node, YAML, launch, Registry core/node/launch tests와 이 검증 문서를 변경했다.

## 37. 각 파일 변경 이유

Core는 lifecycle semantics, node는 parameter 전달, YAML/launch는 runtime defaults, tests는
경계와 회귀 계약, 문서는 개발 과정과 검증 결과를 담당한다.

## 38. Parameter migration

runtime에서는 `confirm_hits`와 `tentative_timeout_sec` alias를 제공하지 않는다. 새 이름을
config/launch/test에 동시에 적용했으며 과거 문서의 명칭은 역사적 설명으로만 남겼다.

## 39. 자동 테스트 설계

Core timestamp를 재현 가능하게 정수 nanosecond로 변환하고, default lifecycle과 즉시
confirmation이 필요한 association test를 명시적 config로 분리했다.

## 40. false-positive regression test

TEST A가 0.9초 4-hit를 실행해 public ID가 생기지 않음을 검증한다.

## 41. tentative gap test

TEST D/E/F가 초과, 정확히 경계, last_seen 갱신의 세 가지 gap semantics를 검증한다.

## 42. visible grace test

TEST G/H/I가 3.9초, 정확히 4.0초, 4초 초과를 각각 검증한다.

## 43. 기존 association regression test

two-track, reversed input order, one-to-one matching, LOST reassociation test가 유지된다.

## 44. Build 결과

일반 `colcon build --symlink-install`이 두 package에서 PASS했다. clean build나 사용자 파일
삭제는 수행하지 않았다.

## 45. 전체 실행 방법

아래 순서는 Stage 6에서 사용한 전체 pipeline을 유지하며 Registry branch만 시간 정책을
개선한 것이다.

## 46. Terminal별 수동 실행 명령

```bash
# Terminal 1: VSLAM + nvblox + RViz
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh

# Terminal 2: Camera preprocessing
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py

# Terminal 3: YOLO
cd ~/damgc_robot
./scripts/run_survivor_detector.sh

# Terminal 4: Camera -> Map transform
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py

# Terminal 5: Stage 5 Raw visualizer
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py

# Terminal 6: Registry + Registry visualizer
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_registry.launch.py
```

## 47. Runtime parameter 확인 명령

```bash
ros2 node list | grep survivor
ros2 param get /leader/survivor_registry confirm_min_duration_sec
ros2 param get /leader/survivor_registry confirm_min_hits
ros2 param get /leader/survivor_registry tentative_max_gap_sec
ros2 param get /leader/survivor_registry visible_timeout_sec
ros2 param get /leader/survivor_registry association_radius_m
ros2 param get /leader/survivor_registry reassociation_radius_m
ros2 param get /leader/survivor_registry position_ema_alpha
```

실제 node 이름이 다르면 `ros2 node list` 결과의 Registry node 이름으로 대체한다.

## 48. Registry reset 방법

과거 ID가 새 검증 결과를 오염시키지 않도록 사람을 배치하기 전에 다음을 실행한다.

```bash
ros2 service call /leader/survivor/registry/reset \
  std_srvs/srv/Trigger "{}"
ros2 topic echo --once /leader/survivor/tracks \
  --qos-durability transient_local
```

reset 직후 `tracks`가 empty인지 확인한 뒤 physical test를 시작한다.

## 49. 물리 검증 TEST A — 실제 사람 Confirm delay

사람을 카메라 앞에 두고 raw detection 시작 시각과 Confirmed ID 발급 시각을 기록한다.
Stage 6.1 후속 물리 실행은 아직 하지 않았으므로 상태는 **NOT RUN**이다.

## 50. 물리 검증 TEST B — 지속 관측

같은 사람을 계속 관찰하며 새 ID가 증가하지 않고 observation_count만 증가하는지 확인한다.
상태: **NOT RUN**.

## 51. 물리 검증 TEST C — 짧은 detection dropout

안전하게 부분 가림 또는 시야 경계 dropout을 만들 수 있을 때 4초 이내 ID 유지 여부를
확인한다. 상태: **NOT RUN**.

## 52. 물리 검증 TEST D — FOV 완전 이탈

사람을 4초 이상 FOV 밖으로 이동해 LOST, marker 유지, 마지막 위치 유지를 확인한다.
상태: **NOT RUN**.

## 53. 물리 검증 TEST E — 재진입

마지막 위치 근처로 같은 사람이 재진입할 때 #2가 아니라 기존 #1로 복구되는지 확인한다.
상태: **NOT RUN**.

## 54. 물리 검증 TEST F — 정지 2인 + 로봇 이동

사람 #1/#2를 정지시키고 로봇만 안전하게 천천히 이동한다. Codex는 모터 명령을 보내지
않는다. 수정 전 재테스트는 기존 0.50/0.75 gate의 ID 유지 PASS였지만, Stage 6.1 후속
regression은 **NOT RUN**이다.

## 55. 실제 관찰 결과

Stage 6.1 이후 물리 관찰 결과는 아직 없다. 사용자가 제공한 수정 전 문제 관찰과 수정 전
association 재테스트는 2~18장에 기록했으며, 자동 검증 결과와 섞지 않았다.

## 56. 개선 전/후 비교

개선 전에는 3-hit만으로 약 1초 false positive가 ID를 받았고 Tentative lifetime이
first_seen 기준이었다. 개선 후에는 4-hit와 2초 duration을 모두 요구하며 last_seen gap
0.8초를 초과한 Tentative를 제거한다. Confirmed grace는 2초에서 4초로 늘었다.

## 57. VSLAM 회귀 확인

Phase C에서는 VSLAM source/config를 변경하지 않았고 package test가 통과했다. 실제
`/visual_slam/tracking/odometry` runtime 회귀는 Stage 6.1 물리 검증에서 확인할 항목이며
현재 상태는 **NOT RUN**이다.

## 58. EKF 회귀 확인

local/global EKF source/config는 변경하지 않았다. 실제 odometry runtime 회귀 확인은
**NOT RUN**이다.

## 59. nvblox 회귀 확인

nvblox source/config/RViz map은 변경하지 않았다. mesh/ESDF runtime 확인은 **NOT RUN**이다.

## 60. Known Limitations

현재 Registry는 map-frame spatial association과 시간 정책에 기반하며 process restart나
map session을 넘어 public ID를 저장하지 않는다.

## 61. 아직 해결하지 않은 moving-person long-occlusion 문제

4초를 초과하는 가림이나 큰 위치 변화에서는 기존 fixed reassociation gate 밖에서 새 ID가
생길 수 있다. Stage 6.1은 이를 완전히 해결한다고 주장하지 않는다.

## 62. spatial-only association 한계

appearance, velocity, body identity feature를 사용하지 않으므로 가까운 사람이 교차하거나
장시간 가려지는 조건은 별도 검토가 필요하다.

## 63. Future Improvements

필요할 경우 velocity prediction, adaptive gate, promotion duplicate suppression, track
merge, appearance Re-ID, ByteTrack, BoT-SORT를 다음 단계에서 검토한다.

## 64. Stage 6.1 PASS/FAIL

자동 검증 기준 Stage 6.1은 **PASS**다. 전체 114 test와 build, launch parameter 검증이
통과했다. 물리 validation은 Codex가 수행하지 않았으므로 전체 hardware PASS로 확대하지
않으며, 실물 최종 판정은 사용자의 TEST A–G 실행 후 갱신한다.

## 65. Manual Validation Quick Reference

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

./scripts/run_vslam_mapping.sh
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py
./scripts/run_survivor_detector.sh
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py
ros2 launch rescue_robot_survivor survivor_registry.launch.py

ros2 topic echo /leader/survivor/tracks \
  --qos-durability transient_local
ros2 topic echo /leader/survivor/map_positions
ros2 topic echo /leader/survivor/registry_markers \
  --qos-durability transient_local
ros2 topic info -v /leader/survivor/tracks
ros2 node list
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /leader/odometry/local
ros2 topic hz /leader/odometry/global
ros2 topic info -v /nvblox_node/mesh
ros2 service list | rg /nvblox_node/get_esdf_and_gradient
```

## 66. Rollback

commit과 push는 수행하지 않았다. rollback이 필요하면 먼저 `git status`와 `git diff`로
Stage 6.1 파일을 확인하고 해당 파일만 선택적으로 복원한다. broad `git reset --hard`,
`git restore .`, `git checkout .`은 사용하지 않는다. commit 후에는 다음 권장 방식으로
되돌린다.

```bash
git revert <stage-6.1-commit>
```

권장 commit message는 다음과 같다.

```bash
git commit -m "Improve survivor registry temporal confirmation"
```

## 67. 다음 개발 단계

먼저 사용자가 reset 후 실제 TEST A–G를 실행하고 raw detection, tracks, marker, VSLAM,
EKF, nvblox 관찰 결과를 기록한다. moving-person long-occlusion 또는 ID split이 실제로
남는 경우에만 다음 단계에서 velocity/adaptive gate/appearance 기반 보완을 검토한다.

## Phase D 최종 결과 요약

```text
Stage 6.1 자동 검증: PASS
Starting commit: 9a71d99191a9f23e887a176ac76730953a2814f6
Problem reproduced: PASS (자동 TEST A 및 기존 물리 관찰 기록)
Root cause: 3-hit-only promotion, first_seen lifetime expiry, 2초 visible grace
Build: PASS
Tests: 114 / 114 passed
Physical single-person confirmation: NOT RUN
Short dropout: NOT RUN
LOST/re-entry physical test: NOT RUN
Two stationary people + moving robot post-change test: NOT RUN
False positive physical test: NOT RUN
VSLAM regression: source/config 변경 없음, runtime NOT RUN
EKF regression: source/config 변경 없음, runtime NOT RUN
nvblox regression: source/config/RViz 변경 없음, runtime NOT RUN
Documentation: 이 문서에 67개 필수 chapter, 자동 test 표, 명령, reset, rollback 기록 완료
```

따라서 Phase D의 코드·자동 검증·문서화 범위는 PASS이며, 실제 장비에서의 최종 물리
판정은 사용자 실행 후에만 갱신할 수 있다.
