# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Source

```bash
cd ~/Documentos/ros2_ws/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

After adding new messages or services to `fleet_msgs`, always rebuild before running other packages.

## Running the System

Three terminals are the minimum for a working simulation:

**Terminal 1 — Gazebo + Nav2:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```

**Terminal 2 — Fleet nodes:**
```bash
source install/setup.bash
ros2 launch fleet_bringup fleet.launch.py single_robot_sim:=true
```

**Terminal 3 — UI backend:**
```bash
source install/setup.bash
./fleet_ui/backend/run.sh
# Frontend: cd fleet_ui/frontend && npm run dev  →  http://localhost:5173
```

## Experiment CLI

```bash
# Record a route (collection ON automatically)
python3 scripts/experiment_repeatability.py record \
  --single-robot --route percurso1 \
  --points "0.5,0,0;1.0,0,0" \
  --export run_record.json

# Replay the route
python3 scripts/experiment_repeatability.py replay \
  --single-robot --route percurso1 \
  --export run_replay_01.json

# Analyze repeatability across bags
python3 scripts/analyze_runs.py \
  collections/default/<baseline_bag> \
  collections/default/<replay_01_bag> \
  --labels baseline replay_01 \
  --output-dir analysis_result/
```

## Tests

```bash
python3 scripts/test_fleet_cases.py      # service-level integration tests
python3 scripts/test_roles_automatic.py  # role-based access control tests
```
Both require the fleet nodes to be running.

## Architecture

### Packages

| Package | Type | Purpose |
|---|---|---|
| `fleet_orchestrator` | ament_python | Main orchestration node: route record/replay via Nav2 |
| `fleet_data_collector` | ament_python | rosbag2 (MCAP) recording per robot |
| `fleet_msgs` | ament_cmake | ROS 2 message and service definitions |
| `fleet_bringup` | ament_python | Launch files for single/multi-robot |
| `route_tool` | ament_python | Deprecated standalone recorder |

### Robot ID Convention

Single-robot mode uses `robot_id = ""` (empty string). This maps to:
- Routes directory: `routes/default/`
- Collections directory: `collections/default/`
- Topics: `/scan`, `/odom` (no prefix)
- TF frames: `map`, `base_link`

Multi-robot mode uses `robot_id = "tb1"`, `"tb2"`, etc.:
- Routes/collections: `routes/tb1/`, `collections/tb1/`
- Topics: `/tb1/scan`, `/tb1/odom`
- TF frames: `tb1/map`, `tb1/base_link`

### Data Flow

**Record mode:**
1. `enable_collection` → rosbag2 starts writing MCAP
2. `start_record` → TF listener polls `map→base_link` @ 5 Hz, saves poses above distance/yaw threshold
3. `go_to_point` loop → Nav2 `NavigateToPose` action per waypoint
4. `stop_record` → saves poses to `routes/{robot_id}/{route_name}.yaml`
5. `disable_collection` → closes bag

**Replay mode:**
1. `enable_collection` → rosbag2 starts
2. `play_route` → loads YAML, calls Nav2 `FollowWaypoints` action
3. `disable_collection` → closes bag

**Analysis:**
- `analyze_runs.py` reads MCAP bags via rosbag2_py, extracts `/odom` trajectory, computes pairwise RMSE and endpoint error, outputs `summary.json` + `trajectory_overlay.png`.

### Key Design Decisions

**QoS policies:** Sensors (scan, odom, imu) use `BEST_EFFORT` for Gazebo compatibility. AMCL pose uses `RELIABLE + TRANSIENT_LOCAL`. TF uses `RELIABLE, KEEP_LAST depth=100`. Mixing these up causes silent topic drop.

**Nav2 availability:** Orchestrator waits up to 3 seconds for Nav2 action servers on startup; returns `NAV2_UNAVAILABLE` error code if not found.

**Role system:** `config/roles.yaml` maps `robot_id → role` (MUUT / FUUT / SU). Only MUUT robots can record/replay (motion). FUUT/SU can only collect data. Role checks happen inside the orchestrator service handlers.

**Route storage format:** YAML files contain a list of poses with `x`, `y`, and quaternion `qx qy qz qw`. Yaw is converted to/from quaternion on read/write.

**Bag format:** MCAP (rosbag2 modern format), not SQLite3. Reading bags requires `rosbag2_py` with the MCAP storage plugin.

**`use_sim_time`:** Must be `true` for all nodes when running in Gazebo, or TF lookups fail silently due to timestamp mismatch.

### fleet_ui

- **Backend:** FastAPI (`fleet_ui/backend/main.py`) — wraps ROS 2 service calls via `rclpy`. Requires workspace to be sourced before launch.
- **Frontend:** React + Vite (`fleet_ui/frontend/`) — calls backend REST endpoints. Dev server on port 5173.
- The backend exposes endpoints that mirror the ROS 2 services (record, replay, go_to_point, status, etc.).

### Config Files

- `src/fleet_orchestrator/config/single_robot_sim.yaml` — robot list, routes/collections paths, recording parameters for single-robot mode.
- `src/fleet_orchestrator/config/roles.yaml` — role assignments per robot_id.
- `src/fleet_bringup/params/` — Nav2 parameter files per robot (tb1, tb2).
