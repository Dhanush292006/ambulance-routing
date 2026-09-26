# Smart Emergency Ambulance — Digital Twin Control Center

This is a ROS 2 Jazzy / Gazebo Harmonic research demonstrator for adaptive emergency ambulance routing. It contains a complete, executable decision stack: a synchronized digital twin, constant-velocity obstacle prediction, **localized** graph updates, energy-aware routing, green-corridor decisions, experiment logging, and a Streamlit control center.

## Quick start

```bash
./setup.sh
./build.sh
./run.sh --demo
# new terminal (optional dashboard)
./dashboard.sh
```

The included `--demo` mode runs without Gazebo and records values measured by the simulation engine—not static dashboard fixtures. It writes `results/latest/{metrics,events,route_history}.json` and is the fastest way to inspect all research layers. Gazebo/ROS launch is available with `./run.sh --ros` after installing Nav2 and Gazebo Harmonic ROS integration.

For the full Gazebo demonstration, follow the ROS / Gazebo steps below. The `--demo` mode is a separate algorithm demonstration that does not launch Gazebo.

## Architecture

`physical simulation → digital twin → prediction → affected-edge update → DT-AAGR route selection → motion / metrics → twin`

The road graph has four meaningful alternatives from `STATION` to `HOSPITAL`. Edges carry distance, speed, traffic, risk, energy and blocked state. DT-AAGR only alters edges within the prediction radius; `full_graph_rebuild_ms` is deliberately measured separately for comparison and is never used as a fabricated metric.

## Scenarios

`normal`, `heavy_traffic`, `road_blocked`, `dynamic_obstacles`, `accident`, `pedestrian`, `multiple_blocks`, `green_corridor`, `low_battery`, `high_dynamic_traffic`. In research demo mode a primary road becomes blocked at 20 simulation seconds and a pedestrian enters the alternative corridor later.

```bash
python3 -m ambulance_core.demo --scenario road_blocked --planner dt-aagr
python3 -m ambulance_core.experiments --scenario accident
```

## ROS / Gazebo

Build with `./build.sh`, then start the integrated simulation and dashboard in separate terminals:

```bash
./gazebo.sh
```

The Gazebo world is a Chennai-inspired 1.5 km × 1 km city grid with a Marina-style coast, 140 mixed-use blocks, 18 looping traffic vehicles, eight looping ambulances, and six parked ambulance fleet units. The full-size matching Nav2 map is installed with the bringup package.

`./gazebo.sh` starts the Gazebo city, ROS bridges, AMCL, Nav2, the DT-AAGR digital twin, and the mission manager. It loads the included road map and uses Gazebo odometry aligned to the map frame. `./gazebo.sh headless:=true` runs the server without a window while keeping GPU LiDAR rendering enabled. In a second terminal, `./dashboard.sh` opens the live mission dashboard at `http://localhost:8501`.

The dashboard sends destination and mission commands to ROS 2. DT-AAGR selects graph waypoints, and the mission manager sends each waypoint to Nav2. Nav2 plans, controls, and avoids local obstacles; its velocity commands pass through the collision monitor and Gazebo bridge. Gazebo odometry supplies the dashboard pose and measured path length. `/scan` drives the local/global costmaps and the obstacle detector. The simulated mission can be run to each of the five facilities and returned to the station.

## White ambulance teleoperation

The final `ambulance_01` model is a white body with red emergency stripes, blue light bar, collision geometry, two driven wheels, LiDAR, camera, IMU, and GPS-like sensor. Start Gazebo first, then in a second terminal:

```bash
./teleop.sh
```

Use `i` / `,` to increase/decrease forward speed, `j` / `l` to steer, and `k` to stop. The command follows `/ambulance/cmd_vel` through `ros_gz_bridge` into Gazebo's differential-drive system. Keep the teleoperation terminal focused while driving.

### Obstacle handling

The world includes a central roadblock, traffic, and bypass corridors. LiDAR scans feed Nav2 costmaps and an obstacle detector; the digital twin can update affected graph edges and produce a new route. Static roadblock detection depends on sensor visibility and the vehicle approach, so inspect the live obstacle and replan counters in the dashboard during a run.

### Emergency mission demo

1. Run `./build.sh`.
2. Start `./gazebo.sh` (add `headless:=true` on machines without a display).
3. In another terminal, run `./dashboard.sh`.
4. Select a hospital and press **START EMERGENCY MISSION**. The ambulance starts at the station, follows DT-AAGR waypoints under Nav2 control, and publishes its measured pose and mission result to the dashboard.
5. Watch the live pose, route, mission state, and replan counter in the dashboard. Mission status and completed-run metrics are also written under `results/latest/`.

Pause, resume, cancel, return-to-station, and RViz2 are wired to their ROS 2 actions or processes. Nav2 can also be tested directly with `./nav_goal.sh --x 145 --y 0 --yaw 0`.

`ambulance_description` provides the vehicle URDF; `ambulance_gazebo/worlds/smart_city.sdf` provides a Gazebo Harmonic city with three road corridors, cross-links, hospital emergency entrance, ambulance station, urban districts, signals, construction zone, vehicles, and native LiDAR/camera/IMU/GPS topics. Install the Jazzy Nav2, `ros_gz`, robot-state-publisher, and RViz packages required by the launch file if they are not already installed. Then run `./setup.sh` and `./build.sh` from the workspace.

## Data and caveats

All generated plots/summary values are derived from the current run's route and timing measurements. This repository does not claim hardware validation or provide commercial city assets. Replace the primitive buildings in the supplied SDF with licensed Gazebo Fuel models for a production-quality visual city.

## Topics

The integration uses `/clock`, `/scan`, `/ambulance/odom`, `/ambulance/cmd_vel`, `/ambulance/obstacles`, `/digital_twin/status`, `/digital_twin/route`, `/digital_twin/destination`, and `/dashboard/mission_command`. See the launch file and node sources for message types.


## Dashboard mission controls

With the ROS stack running via `./gazebo.sh`, launch `./dashboard.sh`. The dashboard publishes destination and mission commands on `/dashboard/mission_command`; `ambulance_mission_manager` translates start, pause, resume, and cancel into Nav2 `NavigateToPose` actions. Nav2 remains responsible for vehicle motion. Mission state is written to `results/latest/mission_status.json`. Rebuild with `./build.sh` after changing the ROS package.

The Gazebo city includes five hospital facilities with road-side goal poses. The real-world Chennai panel requires an authorized Google Maps Platform key and remains visibly unavailable until configured. The Chennai reference panel requires an authorized Google Maps Platform integration and key, which are not configured in this workspace.
