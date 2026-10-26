# Solo exploration, return, grasp and reverse delivery

```bash
cd ~/damgc_robot
bash scripts/run_solo_delivery.sh
```

Physically restore the robot's original start and heading, put the tagged supply
box behind that start, and separate the follower before this single-robot trial.
The entry point starts a fresh map, initial scan, checkpoint-only person detection,
short-step exploration and return to the original start facing backward.

After a successful home return, the same coordinator selects the confirmed survivor
nearest the saved home. With no confirmed survivor it stops without picking up.
It drains home actions, holds the navigation map, explicitly blocks visual global
localization corrections, enters mapping HOLD, and only after those acknowledgements
requests the existing validated leader grasp task. Camera/TF/selector/bridge are
reused; no duplicate drive hardware is started. The grasp launch creates its
existing tag processing, approach controller, guard and motor sequence on demand.
DONE means motor ACK plus close wait, not a grip-force measurement.

Nav2 already uses the local wheel/IMU-only odom->base_link transform. That filter
is not reset at handoff. Visual map-pose corrections to the separate global filter
are blocked until the next fresh session. Stereo tracking may still run, but it
is not required for carrying localization. RGB remains available for tag alignment.
HOLD stops all filtered-depth integration from grasp-mode entry onward. Frozen
mapping does not provide live rear obstacle sensing.

After confirmed grasp, a one-time navigation-map edit removes known occupied cells
inside the measured attached payload region at the actual pickup pose (x=.10..40m,
half width .05m). Unknown cells and cells outside that anchored region are unchanged;
no clearing follows the robot along the route. The original recorded map/person
coordinates remain available. The loaded footprint includes robot body and payload.

The delivery goal is .7m short of the saved survivor point along the direction from
pickup. Smac plans with the loaded footprint. Complete paths are checked to require
reverse-only travel and a known, free loaded footprint, densely sampled through the
path. At most .6m of the verified path is sent directly to FollowPath per step, so a
NavigateToPose behavior tree cannot silently replace it with a forward path. The
existing Nav2 collision checker remains enabled. Forward commands are rejected;
accepted reverse commands are curvature-preserving scaled to at most .03m/s and
.15rad/s, then sent through the existing selector/bridge. Position/TF loss, motor
fault, operator STOP, action failure or an inadmissible path stops delivery.

Completion stops near the survivor and holds the gripper target; this task neither
lifts nor releases and sends no follower commands. Wheel/IMU dead reckoning can
drift; saved-map collision checking is not live obstacle detection. Individual
phases may stop instead of completing if the known reverse route is unavailable.

New geometry and visual-hold unit checks pass. An isolated ROS test verifies that
map/visual HOLD precedes the grasp request, and fake Nav2 actions exercise reverse
short steps to the survivor stand-off. Physical integrated results are recorded in
the session logs under `log/vslam_mapping_*/solo_delivery_result.json` when present.

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
