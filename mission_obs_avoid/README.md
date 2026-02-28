# mission_obs_avoid

Mission-mode obstacle interception package for PX4 + MAVROS.

## What it does

- Monitors hazards from `/scan_horizontal` while vehicle is in `AUTO.MISSION`.
- Temporarily switches to `OFFBOARD` for a deterministic detour sequence:
  - brake,
  - sidestep,
  - forward-clear,
  - clear verification.
- Requests `AUTO.MISSION` resume when path is clear.
- Uses `RESUME_GUARD` to prevent immediate ping-pong.
- Falls back to `AUTO.LOITER` latch on repeated failures or immediate retrigger.

## State machine

`mission_interceptor_node` states:

1. `MONITOR`
2. `HAZARD_CONFIRM`
3. `OFFBOARD_WARMUP`
4. `OFFBOARD_REQUEST`
5. `BRAKE`
6. `SIDESTEP`
7. `FORWARD_CLEAR`
8. `CLEAR_VERIFY`
9. `MISSION_RESUME_REQUEST`
10. `RESUME_GUARD`
11. `COOLDOWN`
12. `LOITER_LATCH`

Resume-guard policy:

- If hazard retriggers in `RESUME_GUARD`, node requests `AUTO.LOITER` and latches.
- Latch exits only when operator restores `AUTO.MISSION` and hazard remains clear for `latch_clear_hold_sec`.

## Nodes

- `mission_interceptor_node`
- `cmd_vel_arbiter_node`

## Command arbitration

`cmd_vel_arbiter_node` is the only publisher to MAVROS cmd-vel when enabled:

- input A: `/offboard_stack/cmd_vel`
- input B: `/mission_interceptor/cmd_vel`
- selector: `/mission_interceptor/override_active`
- output: `/mavros/setpoint_velocity/cmd_vel`

When this package is active, other direct cmd-vel publishers should remap:

- `/mavros/setpoint_velocity/cmd_vel -> /offboard_stack/cmd_vel`

## Launch

Build first:

```bash
cd ~/ros2_ws
COLCON_LOG_PATH=/tmp/colcon_log_mission \
colcon build --base-paths src/obs_avoid/mission_obs_avoid \
  --packages-select mission_obs_avoid
source install/setup.bash
```

Then launch:

```bash
ros2 launch mission_obs_avoid mission_obs_avoid.launch.py use_sim_time:=true
```

or

```bash
./scripts/start_mission_obs_avoid.sh
```

## Config

Default parameters are in:

- `config/mission_obs_avoid.yaml`

Key params:

- Interceptor: `hazard_distance`, `emergency_distance`, `front_fov_deg`, `hazard_confirm_sec`,
  `offboard_warmup_sec`, `setpoint_hz`, `set_mode_retry_sec`, `max_set_mode_retries`,
  `brake_sec`, `sidestep_speed`, `sidestep_min_dist`, `sidestep_max_sec`,
  `forward_speed`, `forward_clear_dist`, `clear_distance`, `clear_hold_sec`,
  `max_intercept_sec`, `resume_grace_sec`, `cooldown_sec`, `latch_clear_hold_sec`,
  `sensor_timeout_sec`.
- Arbiter: `arbiter_rate_hz`, `offboard_cmd_timeout_sec`, `interceptor_cmd_timeout_sec`,
  `publish_zero_on_stale`.

## Output topics

- `/mission_obs_avoid/state`
- `/mission_obs_avoid/status`

## SITL checklist

1. Normal mission path without obstacle: no interception.
2. Obstacle on mission path: OFFBOARD detour then mission resume.
3. Immediate retrigger after resume: enters `LOITER_LATCH`.
4. Stop scan/odom during intercept: enters `LOITER_LATCH`.
5. Confirm only one writer to `/mavros/setpoint_velocity/cmd_vel` (arbiter output).
