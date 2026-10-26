# 리더 단독 탐색·파지·생존자 옆 배달

```bash
cd ~/damgc_robot
bash scripts/run_solo_delivery.sh
```

실물 시작 위치와 방향을 맞추고, 출발지 뒤에 태그가 보이는 상자를 둔다.
서브 로봇은 이 태스크에 참여하지 않는다. 이 명령은 실제로 주행·파지한다.

## 자동 실행 순서

1. 기존 지도를 불러오지 않고 시작점을 고정한다. 2D RViz에 3m 전방 × 3m 우측 탐색 영역을 표시한다.
2. 처음 한 바퀴 회전한 뒤 짧은 경유점으로 탐색한다. 각 도착점에서 새 영상 3장만 사람 인식에 사용한다.
3. 출발지로 복귀하고 최초 방향의 반대쪽을 본다. 복귀 완료 파일이 생긴 뒤에만 파지·운반 구성요소를 시작한다.
4. 확정 생존자가 없으면 파지·운반을 건너뛴다. 있으면 지도와 시각 위치 보정을 고정한다.
5. 매핑 카메라·cuVSLAM·nvblox·YOLO를 종료하고, 호스트에서 RGB만 640×480, 30fps로 다시 켠다. 준비 확인 후 태그 정렬·파지를 수행한다.
6. 집게 명령 확인과 닫힘 대기가 끝나면 파지 위치에 고정된 상자 영역만 지도에서 한 번 제외한다. 지도 격자 경계에 걸친 칸도 포함하며, 미확인 칸과 다른 위치는 유지한다.
7. RGB 카메라를 끄고 운반용 Nav2와 로봇+상자 윤곽을 적용한다. 위치는 기존 바퀴·IMU odom을 계속 사용한다.
8. 생존자 주변 양쪽과 여러 도착 방향을 검사해 상자 기준점이 생존자 좌표 옆 55cm에 오는 경로를 고른다. 선택한 도착점은 운반 중 바꾸지 않는다.
9. 전체 경로가 후진이고 로봇·상자 윤곽이 알려진 빈 공간을 통과하는지 검사한다. 한 번에 최대 60cm, 속도 최대 3cm/s로 이동한다.
10. 마지막 위치와 방향을 확인한 뒤 STOP. 집게는 잡은 상태를 유지하며 들어 올리기·놓기는 하지 않는다.

상자 기준점은 base_link 전방 35cm로 근사한다. 사람의 몸 방향을 인식하는 것은 아니며,
저장한 생존자 좌표 옆에 상자가 오도록 로봇 방향을 맞춘다. 실제 배치·오도메트리 오차에 따라
최종 거리는 달라진다. 확인된 시연의 추정 상자 거리는 약 62cm였다.

## 실행·복구 기준

- 기존에 잘 된 매핑 절차를 그대로 실행하고, 운반 처리는 복귀 후에만 추가한다.
- 매핑 30fps, 도착 시에만 사람 인식, rosbag 끔. I²C 주기는 변경하지 않는다.
- 벽 근처 경로 비용을 넓게 준다: inflation 0.80m, decay 3.0, planner cost penalty 3.5.
  길 중앙을 선호하지만 중앙 주행이나 고정된 최소 벽 거리를 보장하지 않는다.
- 파지 내부 서비스 준비가 늦으면 시작 요청을 재시도한다. 이미 시작한 파지 명령을 반복하지 않는다.
- 운반 Nav2 준비 응답이 오래된 연결에 묶이면 요청을 정리하고 재조회한다.
- 운반 중에는 새 지도를 그리지 않는다. 저장 지도 검사는 실시간 후방 장애물 감지가 아니다.
- 파지 이후 실패하면 STOP하고 집게를 유지한다. 자동으로 집게를 다시 열거나 재파지하지 않는다.
- `--transport-only`는 완료된 파지 기록을 확인한 뒤 운반만 재연결하는 수동 복구용이다.
- 중복 실행은 잠금으로 막는다. 종료할 때는 이번 로그 폴더의 PID와 실제 실행 명령을 확인해 해당 프로세스 그룹만 종료한다.
- Ctrl-C는 전체 태스크 종료다. 재주행은 실물 시작 배치를 다시 맞춘 뒤 새 명령으로 시작한다.

로그: `log/vslam_mapping_날짜_시간/`. `return_context.json`, `post_return.log`,
`solo_grasp.log`, `delivery_nav2.log`, `solo_delivery_result.json`에서 단계와 결과를 확인한다.
사람이 없으면 결과는 `NO_SURVIVOR`이고 파지·운반을 시작하지 않는다.

## 검증 범위

실물에서 탐색·인식·반대 방향 복귀·파지·후진 운반·생존자 옆 정렬을 확인했다.
실물 시험 당시에는 호스트 RGB 카메라 시작과 상자 경계 겹침 보정에 수동 개입이 있었다.
이를 현재 자동 순서에 반영했으며, 정리 후 전체 실물 태스크를 새로 재실행하지는 않았다.
격리된 ROS 테스트는 HOLD→RGB 준비→파지→운반 순서와 RGB 준비 실패 시 주행 금지를 검증한다.
아래는 이전 시험 기록이다.

## Physical validation status at user-requested stop (2026-10-27)

The user stopped the integrated trial before exploration started. No physical
exploration→reverse-home→grasp→survivor delivery was completed for this new entry
point. The following startup observations are evidence, not a confirmed cause:

| Session | Camera rate | `Out of frame resources` log matches | Result |
| --- | --- | --- | --- |
| 02:29 checkpoint trial | 30fps | 0 | Initial scan, seven exploration stops and one recorded person; return needed manual recovery after a video gap |
| 03:22 solo trial | 30fps | 18 | Insufficient visual-odometry samples; no scan or motion started |
| 03:26 solo retry | 15fps | 0 before stop | User stopped during Nav2 preparation; reduced-rate operation not validated |

All three sessions logged a Nav2 lifecycle startup abort; that symptom also occurred
in the earlier successful trial, so it alone does not explain the regression.
The solo launch adds one map relay to the original mapping launch (19 child
process starts versus 18). No controlled comparison isolates its resource impact.
At final shutdown the leader mapping, camera, bridge, manual keyboard and mission
processes were gone. This does not retrospectively prove absence of overlap during
all startup moments. Frame-pool pressure, DDS/CPU scheduling and startup sequencing
remain investigation items; orphan processes have not been established as the cause.

Pre-push validation: 187 focused unit tests and three isolated ROS tests passed.
Bringup builds passed. Pair grasp and manual straight movement were independently
physically demonstrated, but they do not validate this combined solo delivery.


The former upfront `MAPPING_SOLO_DELIVERY=1` path is retained for investigation,
but the public solo-delivery entry point no longer uses it. The staged entry point
always requests camera 30fps and never triggers paired/follower operation.

## Staged physical trial: 2026-10-27 04:43

Session: `log/vslam_mapping_20261027_044323`. Fresh 30fps mapping completed
initial scan, checkpoint person detection, return and opposite heading without a
manual return override. Person 1 was saved at odom (2.25336, -1.45867). Gripper
completion was reported by both host and sequence. RGB-only startup initially
failed because RealSense is installed on the host rather than in the mapping
container; the camera was started manually on the host during this trial. The
next-run supervisor now handles the host camera request and startup acknowledgement.
This host-camera handshake has not yet been exercised by a fresh complete run.

After grasp, frozen-map overlap at the pickup prevented transport. The operator
confirmed this obstacle was the attached box and requested starting despite that
specific overlap. Seven known occupied cells intersecting the measured payload
rectangle plus half-cell diagonal were cleared once at the fixed pickup pose.
Unknown cells and cells outside this small pickup mask remain unchanged. The
boundary-aware removal is now in the normal post-grasp freeze function; the live
trial used `resume_delivery_slice.py --confirmed-box-overlap`, followed by
transport-only restart, with no reopening of the gripper.

The operator also requested routes farther from walls. Both live costmaps now
use inflation radius 0.80m and decay factor 3.0; Hybrid-A* cost penalty is 3.5.
RPP's inflation factor is kept consistent at 3.0. All six live parameter updates
were accepted and read back. This is a clearance preference, not a guarantee of
an exact corridor centre or a new hard minimum wall distance.

Transport completed five reverse steps at <=0.03m/s. Final odom was
(1.59488, -0.98877), 0.80895m from the recorded person. Selector STOP confirmed;
gripper remains closed, no lift/release. This demonstrates the physical phases
with the interventions above, not a single uninterrupted automatic run.
Five focused geometry/payload tests passed, including boundary-cell removal,
unknown-cell preservation and retaining a nearby outside obstacle.

### Side-of-person delivery continuation

At the user's request, the same held-box session continued from the original
stand-off to a side-delivery pose. The payload reference point is modelled 0.35m
in front of base_link; desired lateral separation from the saved person is 0.55m.
Both sides and several arrival headings are evaluated using full loaded-footprint
checks, known-space checks and reverse-only validation. The first accepted side
is locked for subsequent steps. The final step uses the existing general goal
checker and completion verifies heading error <=0.25rad, position error <=0.10m.
This is alignment beside a saved coordinate; no person's body orientation is
inferred, and the box reference point is an estimate rather than a fresh visual
measurement. No release is requested or performed.

Physical continuation: `side_delivery.log` reported DELIVERED after one short
reverse arc. Final base odom (1.94649,-0.91134), yaw -2.31562; estimated box centre
(1.70924,-1.16866), 0.61658m from saved person. Selector STOP confirmed. Six focused
geometry/payload tests passed. Public staged launch now requests side delivery;
that full entry still awaits a fresh uninterrupted trial with the host-camera fix.
