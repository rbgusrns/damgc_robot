# Carrying mode and home key

In the existing arrow-key terminal:

| Key | Action |
| --- | --- |
| P | Stop current navigation/manual motion and enter HOLD; pause depth integration |
| M | Stop current navigation/manual motion and resume normal depth mapping |
| H | Send NavigateToPose to the saved home x/y/yaw in odom |
| L | Reload this session’s saved nvblox map; keep current odometry |
| Space | Stop and cancel navigation through the command selector |
| Arrow/Q/W/A/S | Manual takeover, cancelling the Nav2 goal |

The keyboard publishes `/leader/mapping/control`; the Docker-side `mapping_mode_manager.py` owns mode state and the Nav2 action client. It sends no goals on startup. `/leader/mapping/mode` and `/leader/mapping/status` are latched for late subscribers. Repeated key events are suppressed until a one-second release gap. Nav2 goals select NAV2 through the existing action-status selector.

## HOLD

`robot_self_filter.py` drops depth images in HOLD. Carried objects therefore are not newly integrated into the static environment, and local wheel + calibrated gyro odometry stays running. This first implementation pauses **all** depth mapping, including any remaining visible region. It does not claim live obstacle sensing while holding. Static TSDF decay is disabled so a carrying-mode pause retains existing geometry. Existing radius-based clearing is still configured at 5 m, so this mode is intended for the known 2 m exploration/return area.

The carried footprint uses a rectangle covering the configured forward extent, rear base extent and lateral width; it is published to both Nav2 costmaps. The base footprint is restored in MAPPING. Configure measured extents in `config/carrying_mode.yaml`:

- `payload_front_m`: base_link to furthest front edge, metres.
- `payload_half_width_m`: half of the total loaded robot/object width, metres, assuming centred grasp. Asymmetric loads need a polygon extension.

Both default to zero (unknown). P can pause the depth stream immediately, but HOME in HOLD is refused until both are positive. No object dimensions have been guessed. Automatic Nav2 command selection is also disabled in HOLD until the dimensions are configured; RViz may still show a requested goal, but it will not select the motor command source. Manual driving remains available. Opening/closing the gripper does not automatically switch modes: X/Z are actuator commands, not confirmed possession state. Use P after grasping and M after releasing.

## Home anchor and frame matching

The manager reads `manifest.json` from the restored snapshot, or from `data/maps/latest` if starting a new map. HOME requires either the restored directory to match the active saved map, or the manifest's `mapping_session_id` to match the current session. It does not send a previous map's coordinates into an unrelated fresh odom frame. The goal includes the saved heading; rotation to that heading at the end is expected.

A snapshot may contain `home_anchor` separately from its `anchor`/`initial_state`. This supports restarting the current live stack at its current estimated pose while preserving the originally marked physical home as the H goal. Normal `run_saved_mapping.sh` uses the original `data/maps/latest` anchor. `data/.mapping_session_id` is written by the launcher and included by future save operations.

Live feature deployment resumes `data/maps/carry_mode_resume_20261004` at the saved current pose and uses the original `home_anchor_20261004_162000` home position/heading. The map and pose were saved before stopping the old stack. No home-return or carrying physical test is performed as part of deployment; the operator triggers those with keys.

## Deployment result (2026-10-04)

Session `20261004_164513` restored the current map archive successfully; nvblox logged `Loaded map`, Nav2 lifecycle nodes became active, the new P/M/H keyboard menu was shown, and recorder subscriptions to mode/control/status were confirmed. Manager anchor directory points to `carry_mode_resume_20261004`; original marked home is preserved in its `home_anchor`. Initial source is STOP and no navigation goal was sent. Docker and host bringup builds completed. Physical HOME/HOLD behavior has not been exercised during deployment. Object extents remain unset pending operator input.

An initial attempt to disable TSDF decay with factor 1.0 was rejected by nvblox (`value < 1.0`). The final configuration retains factor 0.9999 and sets `decay_tsdf_rate_hz: 0.0`, which disables the decay task through nvblox's rate check. The command selector now accepts runtime changes to the boolean `enable_nav2_goal_selection` in addition to source_mode so mode transitions can update goal selection. Other runtime selector parameters remain rejected.

## L: reload the saved map

LOAD_MAP validates that the selected archive shares the current map frame, publishes STOP to cancel navigation, confirms STOP through the selector service, pauses incoming depth while nvblox replaces the layers, and then restores the preceding MAPPING/HOLD mode. Current EKF pose is not reset and no motor goal is sent. Nav2 automatic source selection is disabled during loading. Duplicate mode/home/load requests while loading are ignored. Status messages report success or failure; motor source remains STOP after completion. Reload restores the saved checkpoint, so subsequent unsaved mapped changes are replaced. It does not recover localization after the robot is picked up or placed elsewhere.

Feature deployment snapshot: `data/maps/reload_key_resume_20261004`, with the original marked home stored separately. The `.mapping_session_id` marker now lives in the shared repository’s ignored data directory so the container’s admin user can update it across restarts.

L deployment result: session `20261004_165546` restored `reload_key_resume_20261004/map.nvblx` and displayed the L key menu. Live terminal subsequently received HOLD and LOAD_MAP commands and reported `Saved map reloaded; current pose preserved, motors remain STOP`, confirming an actual reload completed. The prior HOLD mode was retained. Loaded-object dimensions are still unset. No automatic movement goal was sent by deployment.

Operator supplied payload dimensions: base_link to front edge 0.40 m, object width 0.10 m (half-width 0.05 m). These are saved in `config/carrying_mode.yaml` and were applied to the live manager. Nav2 goal selection became enabled in HOLD. Effective rectangular footprint is x=[-0.06,0.40], y=[-0.145,0.145] before costmap padding; the narrower object does not reduce the robot body width. Existing rejected/blocked goals should be sent again.

## Manual cooperative transport

B/N uses a separate two-robot readiness/start gate, without grasp or lift.
See [follower contract](../../../../docs/COOP_TRANSPORT_FOLLOWER_HANDOFF.md)
and [leader response](../../../../docs/COOP_TRANSPORT_LEADER_RESPONSE.md).
