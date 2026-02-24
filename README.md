# Local Planners (DWA + Sector) for PX4 Drone in ROS2

Last updated: 2026-02-24 18:35:00 +07  
Added: `spiral_mapping_mode` as an OFFBOARD spiral area-coverage alternative to `user_ctrl`  
Removed: forced auto-start of `user_ctrl` in stack startup layout (user now chooses mode in blank pane)

## 1) What This Package Is

This package (`obs_avoid`) contains five ROS2 nodes:

- `local_planner_mode_a`: A **local obstacle avoidance planner** (Dynamic Window Approach, DWA).
- `local_planner_sector_mode`: A **local obstacle avoidance planner** using classic **sector selection**.
- `local_planner_hybrid_mode`: A **hybrid local obstacle avoidance planner** (sector guidance + DWA scoring).
- `user_ctrl`: A small helper node for user/offboard control (goal input flow support).
- `spiral_mapping_mode`: OFFBOARD mapping mode that generates a square spiral mission from user input and returns to center.

The two planners are made to be compared:

- `local_planner_mode_a` (DWA): trajectory-sampling planner.
- `local_planner_sector_mode` (sector): direction-sector selection planner.
- `local_planner_hybrid_mode` (hybrid): sector picks preferred heading, DWA optimizes collision-free trajectory around it.

In simple words:

- You give the drone a goal point (`/drone_goal`).
- The planner reads LiDAR (`/scan`) and odometry (`/mavros/local_position/odom`).
- It continuously chooses a safe velocity command.
- It publishes planned velocity to `user_ctrl` (`/planner_cmd_vel`), and `user_ctrl` forwards active commands to MAVROS (`/mavros/setpoint_velocity/cmd_vel`).

So this node is the “brain” for short-range obstacle avoidance.

## 2) Why This Package Exists

If you command a drone directly toward a goal, it may:

- hit walls,
- get stuck in corners,
- spin in place,
- or move too slowly/jitter too much.

This package solves that by combining:

- an offboard helper (`user_ctrl`) to safely enter OFFBOARD/ARM and publish goals, and
- a local planner (`local_planner_mode_a` or `local_planner_sector_mode`) to avoid obstacles in real-time.

## 3) Package Workflow (End-to-End)

Typical end-to-end flow:

1. `user_ctrl` starts zero-velocity heartbeat commands to MAVROS.
2. After warmup, `user_ctrl` requests:
   - mode = `OFFBOARD`
   - arm = `true`
3. A planner node (`DWA` or `sector`) reads:
   - `/scan`
   - `/mavros/local_position/odom`
   - `/drone_goal`
4. The planner continuously publishes safe velocity commands to:
   - `/planner_cmd_vel`
5. `user_ctrl` forwards either:
   - planner command (`/planner_cmd_vel`) when planner is active, or
   - internal goal-tracking command (standalone mode),
   to `/mavros/setpoint_velocity/cmd_vel`.
6. You set goals either:
   - from `user_ctrl` terminal input, or
   - by publishing to `/drone_goal` manually.

## 4) `user_ctrl` Node (Offboard Helper)

Source: `src/offboard_node_user_ctrl.cpp`

Purpose:

- Keep PX4 OFFBOARD link alive with heartbeat velocity commands.
- Transition vehicle to OFFBOARD mode and ARM with retry logic.
- Optionally read goal inputs from terminal and publish to `/drone_goal`.

### `user_ctrl` I/O

Subscribed:

- `/mavros/local_position/pose` (`geometry_msgs/msg/PoseStamped`)
- `/planner_cmd_vel` (`geometry_msgs/msg/TwistStamped`)

Published:

- `/mavros/setpoint_velocity/cmd_vel` (`geometry_msgs/msg/TwistStamped`)
- `/drone_goal` (`geometry_msgs/msg/Point`)

Service clients:

- `/mavros/set_mode` (`mavros_msgs/srv/SetMode`)
- `/mavros/cmd/arming` (`mavros_msgs/srv/CommandBool`)

### `user_ctrl` parameters

- `setpoint_hz` (20.0): heartbeat publish rate
- `warmup_sec` (2.0): heartbeat warmup before OFFBOARD request
- `state_check_hz` (2.0): state machine update frequency
- `request_retry_sec` (1.0): retry interval for OFFBOARD/ARM service requests
- `ask_goal_on_start` (true): enable terminal goal input loop
- `goal_input_poll_hz` (10.0): terminal input polling rate
- `print_input_help_on_start` (true): print CLI help at startup
- `publish_heartbeat` (true): continue publishing zero-velocity setpoints after OFFBOARD+ARM

### `user_ctrl` terminal commands

If `ask_goal_on_start=true`, you can type:

- `x y z` (example: `10 0 5`) -> publishes `/drone_goal`
- `h` / `help` / `?` -> show input help
- `q` / `quit` / `exit` -> disable CLI goal input

## 5) Planner Behavior (Easy Explanation)

Every control cycle (about 20 Hz), the planner does this:

1. Read current drone position and speed.
2. Read LiDAR obstacles.
3. Read goal point.
4. Generate many candidate motions (`vx`, `vy`, `yaw_rate`) inside dynamic limits.
5. Simulate each motion for a short future horizon.
6. Reject motions that would collide.
7. Score remaining motions (goal progress, safety, smoothness, speed).
8. Send the best velocity command to PX4.

Extra logic:

- **Goal hold mode**: when very close, hold position.
- **Terminal mode**: near goal, slow down and align yaw.
- **Stall watchdog + escape mode**: if stuck for too long near obstacles, perform recovery maneuvers.

## 6) Topics Used by Planner Nodes

### Subscribed

- `/mavros/local_position/odom` (`nav_msgs/msg/Odometry`)
  - Drone position, orientation, and velocity estimate.
- `/scan` (`sensor_msgs/msg/LaserScan`)
  - 2D LiDAR obstacle measurements.
- `/drone_goal` (`geometry_msgs/msg/Point`)
  - Goal in ENU coordinates (`x`, `y`, `z`).

### Published

- `/planner_cmd_vel` (`geometry_msgs/msg/TwistStamped`)
  - Planner velocity command in ENU frame (consumed by `user_ctrl`).
- `/dwa/best_rollout_path` (`nav_msgs/msg/Path`)
  - Current short-horizon intended trajectory selected by DWA.
- `/dwa/full_path` (`nav_msgs/msg/Path`)
  - Accumulated full trajectory from odometry for RViz visualization.

## Trajectory Compare Tool (Test B)

Use this helper while SLAM+mapping is running to compare trajectories:

- Odom path: `/mapping/debug/path_odom`
- Map path (from TF `map -> base_link`): `/mapping/debug/path_map`
- Ground truth path (optional): `/mapping/debug/path_ground_truth`

Run:

```bash
cd ~/ros2_ws/src
./obs_avoid/scripts/start_trajectory_compare.sh --ros-args -p use_sim_time:=true
```

In RViz:

- Add `Path` display for each topic above.
- Set fixed frame to `map` for direct map-vs-odom visual comparison.

## 7) Frames and Motion Assumptions

- Planner runs in **2D XY + yaw**.
- Z is handled separately (simple vertical correction).
- LiDAR is used for obstacle avoidance in plane.
- Velocity commands are sent in ENU via MAVROS.

## 8) Dynamic Window Approach (DWA) in This Package

This implementation is practical and safety-oriented:

- Dynamic window from current state + acceleration limits.
- Forward simulation over `horizon_sec`.
- Collision check with safety radius.
- Weighted cost:
  - heading alignment,
  - goal progress,
  - obstacle clearance,
  - speed preference,
  - smoothness/switching penalties.

Important anti-crawl features:

- Progress normalization uses `progress_speed_ref` (not `v_max`).
- Obstacle cost is relaxed when already safely clear.
- Open-space anti-crawl penalty encourages normal speed when front is free.

## 9) Sector Select Method (Classic, `local_planner_sector_mode`)

This planner uses a simpler, very interpretable strategy:

1. Divide LiDAR front field-of-view into angular sectors (for example 15 sectors over 270 degrees).
2. For each sector, compute two robust distances from LiDAR samples:
   - `clear_r`: clearance estimate (from a percentile like median)
   - `occ_r`: occupancy estimate (from a lower percentile)
3. Mark each sector as free/occupied using `occ_dist`.
4. Score candidate sectors with a weighted cost:
   - how close sector direction is to the goal direction
   - switch penalty (avoid jumping too far from current sector)
   - clearance preference (prefer larger clear sectors)
5. Apply stability logic:
   - hysteresis (`hold_margin`) keeps current sector unless new sector is clearly better
   - short commitment (`commit_time_sec`) locks selected sector for a short time unless it becomes occupied
6. Near-wall override:
   - if front clearance is small, temporarily prefer max-clear sector
7. Control output:
   - yaw target points toward selected sector center (with yaw deadband/filter)
   - forward speed depends on yaw alignment + sector clearance
   - turn-then-go gate can stop translation when yaw error is too large
   - front-safety clamp enforces slow-down and hard stop near obstacles

Why this method is useful:

- Very easy to debug from logs (you can see each sector clearance and selected index).
- Usually stable in corridors and wall-follow-like behavior.
- Easier to reason about than full trajectory sampling.

Main tradeoff:

- It is less globally optimal than DWA in complex clutter, but often easier to tune quickly.

### Key sector parameters to tune first

- `sectors_n`: number of sectors (higher = finer direction choices, but noisier).
- `fov_deg`: FOV used for sectorization.
- `occ_dist`: occupancy threshold per sector.
- `w_goal`, `w_switch`, `w_clear`: sector cost balance.
- `hold_margin`, `commit_time_sec`: anti-oscillation stability.
- `turn_only_deg`: turn-first threshold.
- `safety_stop_dist`, `safety_slow_dist`: front safety envelope.
- `near_wall_override_dist`: when to force max-clear sector choice.

### Sector selection math (detailed)

For each sector `i`, the planner builds a set of LiDAR ranges that fall inside that angular bin.

It then computes:

- `clear_r[i] = percentile(samples_i, sector_clear_percentile)`
- `occ_r[i] = percentile(samples_i, sector_occ_percentile)`

Default behavior:

- `sector_clear_percentile = 0.50` -> **median** clearance (not mean, not min).
- `sector_occ_percentile = 0.20` -> conservative occupancy estimate.

Sector `i` is considered free when:

- `occ_r[i] > occ_dist`

For each free candidate sector, the planner computes cost:

`cost = w_goal * d_goal + w_switch * d_switch + w_clear * d_clear`

Where:

- `d_goal` = angular error between goal direction and sector center
- `d_switch` = angular jump from current selected sector
- `d_clear` = normalized inverse clearance (`1 - clamp(clear_r/prefer_dist, 0, 1)`)

Selection stabilization:

- **Hysteresis** (`hold_margin`): keep current sector unless new sector is clearly better.
- **Commitment** (`commit_time_sec`): keep selected sector for a short time unless it becomes occupied.
- **Near-wall override** (`near_wall_override_dist`): if front is tight, temporarily choose max-clear sector.

### Full sector planner parameter reference

Source of defaults:

- `src/local_planner_sector_mode.cpp`

Motion:

- `control_dt` (0.05): control loop period [s]
- `v_fwd_max` (4.2): max forward speed [m/s]
- `v_fwd_min` (0.20): minimum forward/creep speed [m/s]
- `vz_max` (1.0): vertical speed clamp [m/s]
- `k_yaw` (1.0): yaw proportional gain
- `yaw_rate_max` (0.8): yaw-rate clamp [rad/s]
- `yaw_deadband_deg` (2.0): zero small yaw errors
- `yaw_smooth_alpha` (0.20): low-pass filter on yaw-rate command
- `turn_only_deg` (30.0): if yaw error exceeds this, translation is suppressed

Sectorization and obstacle interpretation:

- `sectors_n` (15): number of sectors across FOV
- `fov_deg` (270.0): sectorized LiDAR field-of-view [deg]
- `max_use_range` (12.0): max LiDAR range used [m]
- `sector_clear_percentile` (0.50): percentile used for sector clearance
- `sector_occ_percentile` (0.20): percentile used for occupancy decision
- `occ_dist` (2.6): occupied threshold [m]
- `prefer_dist` (7.0): distance used to normalize clearance preference [m]

Sector choice stability and behavior:

- `w_goal` (1.35): weight for goal-direction alignment
- `w_switch` (0.30): weight for switching penalty
- `w_clear` (0.30): weight for clearance preference
- `hold_margin` (0.12): hysteresis margin for switching
- `commit_time_sec` (0.9): short sector lock duration [s]
- `near_wall_override_dist` (2.8): trigger distance for max-clear override [m]

Safety and stopping:

- `safety_stop_dist` (1.4): hard stop distance in front [m]
- `safety_slow_dist` (4.8): start slowing down distance [m]

Near-goal behavior:

- `goal_hold_enter` (0.50): enter goal hold below this distance [m]
- `goal_hold_exit` (0.90): exit hold above this distance [m]
- `terminal_radius` (1.0): activate terminal mode below this distance [m]
- `terminal_k_v` (0.6): terminal translational gain
- `terminal_v_max` (1.0): terminal max speed [m/s]
- `terminal_k_yaw` (1.5): terminal yaw gain
- `terminal_turn_only_deg` (20.0): terminal turn-first threshold [deg]
- `yaw_hold_enter` (1.2): start yaw hold near goal [m]
- `yaw_hold_exit` (1.8): release yaw hold [m]
- `yaw_stop_radius` (0.8): disable yaw correction very near goal [m]

Debug:

- `debug_hz` (2.0): status log frequency [Hz]

## 10) Modes Inside the DWA Planner

### Normal DWA mode

Main planner operation. Picks best candidate each cycle.

### Terminal mode (near goal)

When close enough to goal and front is clear:

- reduce translational speed,
- align yaw toward goal direction,
- avoid orbiting around goal.

### Goal hold mode

Very near goal: hold position with hysteresis to avoid flickering.

### Escape mode

If the planner repeatedly fails or stagnates near obstacles:

- triggers an escape maneuver (strafe/back + yaw),
- flips side direction after a timer,
- returns to DWA when valid motion resumes.

## 11) DWA Main Parameters (Default Block)

All defaults are in one place in source:

- `src/local_planner_mode_a.cpp` in `DwaParamDefaults` (`kDwaDefaults`).

### Kinematics and limits

- `v_max` (5.0): max XY speed [m/s]
- `yaw_rate_max` (1.2): max yaw rate [rad/s]
- `ax_max` (6.0): max vx acceleration window [m/s²]
- `ay_max` (6.0): max vy acceleration window [m/s²]
- `yaw_accel_max` (3.5): max yaw accel window [rad/s²]
- `vz_max` (1.0): z speed clamp [m/s]

### Timing and rollout

- `control_dt` (0.05): control period [s]
- `sim_dt` (0.10): rollout integration step [s]
- `horizon_sec` (2.2): prediction horizon [s]
- `vx_samples` (11): vx sample count
- `vy_samples` (1): vy sample count (1 means fixed vy)
- `w_samples` (19): yaw-rate sample count
- `lateral_speed_limit` (0.0): absolute vy cap [m/s]

### LiDAR and geometry

- `max_use_range` (12.0): max LiDAR range used [m]
- `safety_radius` (0.40): collision radius around drone [m]
- `clearance_norm_dist` (2.4): obstacle-cost normalization distance [m]
- `front_cone_deg` (20.0): front cone half-angle [deg]
- `scan_stride` (4): use every Nth LiDAR ray
- `obstacle_cloud_range` (8.0): ignore far obstacle points [m]

### Cost weights and anti-crawl

- `w_heading` (0.90)
- `w_distance` (3.00)
- `w_obstacle` (0.60)
- `w_velocity` (0.90)
- `w_smooth` (0.25)
- `w_switch` (0.35)
- `progress_speed_ref` (1.5)
- `obstacle_relax_dist` (2.0)
- `obstacle_relax_scale` (0.2)
- `open_space_speed_bias_dist` (4.0)
- `open_space_min_speed` (0.8)
- `open_space_crawl_penalty` (1.0)
- `min_plan_speed_far_goal` (0.6)
- `crawl_penalty` (0.4)

### Goal behavior

- `goal_hold_enter` (0.35)
- `goal_hold_exit` (0.70)
- `terminal_radius` (1.1)
- `terminal_k_v` (1.0)
- `terminal_v_max` (1.3)
- `terminal_k_yaw` (1.4)
- `terminal_turn_only_deg` (18.0)

### Stall/escape

- `stall_window_sec` (1.2)
- `stall_min_progress` (0.45)
- `stall_goal_dist_min` (2.0)
- `stall_trigger_count` (2)
- `stall_front_trigger_dist` (2.4)
- `stall_min_plan_speed` (0.45)
- `escape_fail_threshold` (2)
- `escape_yaw_rate` (0.9)
- `escape_strafe_speed` (1.2)
- `escape_back_speed` (0.4)
- `escape_flip_sec` (0.9)

### Output smoothing

- `cmd_smooth_alpha` (0.25)
- `cmd_xy_accel_max` (2.8)
- `cmd_yaw_accel_max` (2.5)
- `use_cmd_state_for_window` (true)

### Debug

- `debug_hz` (2.0)

## 12) Build and Run (Step by Step)

Assume workspace is `~/ros2_ws` and this package is at `~/ros2_ws/src/obs_avoid`.

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select obs_avoid
source install/setup.bash
```

### Recommended startup order

1. Start PX4 + MAVROS + simulator.
2. Start `user_ctrl` (handles warmup, OFFBOARD, ARM, optional CLI goals).
3. Start **one** planner node:
   - `local_planner_mode_a` (DWA), or
   - `local_planner_sector_mode` (sector)
4. Send goal(s) and observe behavior.

Important:

- Both planners publish to the same planner command topic (`/planner_cmd_vel`).
- Do **not** run both planners at the same time unless you remap one output topic.

### Simulation startup commands (exact sequence)

Run these in separate terminals in this order:

1. Start PX4 SITL with your world:

```bash
PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d
```
2. Start QGround Control
```bash

cd Download/ ./QGroundControl-x86_64.AppImage 
```

2. Start MAVROS:

```bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580 use_sim_time:=true
```

3. Bridge Gazebo LiDAR to ROS2:

```bash
ros2 run ros_gz_bridge parameter_bridge /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -p use_sim_time:=true
```

4. Relay LiDAR topic to `/scan` (used by planners):

```bash
ros2 run topic_tools relay /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan /scan
```

5. Publish static transform for LiDAR link:

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id link --ros-args -p use_sim_time:=true
```

### One-command startup (Terminator + tmux)

If you want a single command that opens Terminator and runs the full PX4/MAVROS/bridge stack in panes:

```bash
cd ~/ros2_ws/src/obs_avoid
./scripts/start_sim_stack_terminator.sh
```

Notes:

- It creates/uses a tmux session named `obs_avoid_stack`.
- `obs_avoid` window pane-0 is intentionally left idle so you can choose mode manually.
- You can override defaults:
  - `SESSION`
  - `ROS_WS`
  - `ROS_SETUP`
  - `PX4_DIR`
  - `WORLD_NAME`
  - `MODEL_NAME`
  - `FCU_URL`

### Run `user_ctrl`

```bash
ros2 run obs_avoid user_ctrl
```

### Run spiral mapping mode (alternative to `user_ctrl`)

```bash
ros2 run obs_avoid spiral_mapping_mode --ros-args -p use_sim_time:=true
```

or

```bash
./scripts/start_spiral_mapping_mode.sh
```

### Run planner (DWA)

```bash
ros2 run obs_avoid local_planner_mode_a
```

### Run planner (Sector method)

```bash
ros2 run obs_avoid local_planner_sector_mode
```

### Publish a goal manually

```bash
ros2 topic pub /drone_goal geometry_msgs/msg/Point "{x: 10.0, y: 0.0, z: 5.0}" -1
```

### Check inputs are alive

```bash
ros2 topic echo /mavros/local_position/odom
ros2 topic echo /scan
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

### A/B comparison workflow (DWA vs Sector)

1. Run the exact same map and start pose.
2. Run DWA planner only, record behavior.
3. Stop DWA planner.
4. Run sector planner only, record behavior.
5. Compare:
   - time-to-goal
   - min obstacle clearance
   - stuck/spin events
   - path smoothness

## 13) Change Parameters Without Editing Code

You can override parameters at runtime:

```bash
ros2 run obs_avoid local_planner_mode_a --ros-args \
  -p w_distance:=3.2 \
  -p w_obstacle:=0.5 \
  -p cmd_smooth_alpha:=0.22 \
  -p open_space_min_speed:=1.0
```

For permanent settings, use a ROS2 YAML parameter file.

Example for sector planner:

```bash
ros2 run obs_avoid local_planner_sector_mode --ros-args \
  -p sectors_n:=15 \
  -p occ_dist:=2.6 \
  -p turn_only_deg:=30.0 \
  -p commit_time_sec:=0.9
```

## 14) Quick Tuning Guide for Common Problems

### Problem: Drone crawls too slowly

- Increase:
  - `w_distance`
  - `progress_speed_ref`
  - `open_space_crawl_penalty`
- Decrease:
  - `w_obstacle` (carefully)

### Problem: Drone jitters/shakes

- Lower `cmd_smooth_alpha` slightly (for stronger filtering).
- Lower `cmd_xy_accel_max` and `cmd_yaw_accel_max`.
- Increase `scan_stride` (less LiDAR noise influence).

### Problem: Drone gets too close to obstacles

- Increase `w_obstacle`.
- Increase `safety_radius`.
- Decrease `obstacle_relax_dist` or increase `obstacle_relax_scale`.

### Problem: Escape triggers too often

- Increase `stall_trigger_count`.
- Increase `stall_min_progress` tolerance carefully.
- Reduce sensitivity with `stall_front_trigger_dist`.

## 15) Limitations You Should Know

- This is a **local** planner, not a global map planner.
- It does not build a full environment map.
- In very complex dead-ends, local methods may still need carefully tuned escape behavior.
- Assumes LiDAR quality is good and odometry is stable.

## 16) File Locations

- DWA planner source: `src/local_planner_mode_a.cpp`
- Sector planner source: `src/local_planner_sector_mode.cpp`
- User control helper: `src/offboard_node_user_ctrl.cpp`
- Build config: `CMakeLists.txt`
- ROS package manifest: `package.xml`

## 17) Summary in One Sentence

`obs_avoid` provides an offboard helper (`user_ctrl`) plus two local planners (DWA and sector-based), so you can safely arm/switch to OFFBOARD and compare obstacle-avoidance performance in the same environment.
