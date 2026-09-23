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

For the full automatic demonstration, run `./auto_demo.sh`. It builds the workspace, runs the measured emergency scenario, opens Gazebo, and starts the dashboard. Press `Ctrl+C` in its terminal to stop both processes.

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

Build with `colcon build --symlink-install`, source `install/setup.bash`, then:

```bash
./gazebo.sh
```

`gazebo.sh` starts the Gazebo city and the ROS digital-twin node. In another terminal, run `./run.sh --demo` to generate the deterministic emergency scenario telemetry, then `./dashboard.sh` to show it. Gazebo is started as a visual/sensor physical layer; use `ros_gz_bridge` to bridge native Gazebo sensor topics into ROS for a hardware-in-the-loop extension.

### Live SLAM, Nav2, and dashboard data flow

`./gazebo.sh` now also starts SLAM Toolbox and Nav2. The launch bridges Gazebo `/clock`, `/ambulance/scan`, `/ambulance/odom`, and `/ambulance/cmd_vel` to ROS; `ambulance_odom_tf` publishes the odometry-derived TF chain required by SLAM: `odom → base_link → base_footprint / lidar_link`. SLAM Toolbox publishes `/map`; Nav2 consumes `/map`, `/scan`, `/odom`, and TF for localization/costmaps. The digital-twin node persists its live ROS state to `results/latest/metrics.json`, which the Streamlit dashboard reads on refresh.

## White ambulance teleoperation

The final `ambulance_01` model is a white body with red emergency stripes, blue light bar, collision geometry, two driven wheels, LiDAR, camera, IMU, and GPS-like sensor. Start Gazebo first, then in a second terminal:

```bash
./teleop.sh
```

Use `i` / `,` to increase/decrease forward speed, `j` / `l` to steer, and `k` to stop. The command follows `/ambulance/cmd_vel` through `ros_gz_bridge` into Gazebo's differential-drive system. Keep the teleoperation terminal focused while driving.

### Obstacle-driven replanning

The central arterial now contains a LiDAR-visible orange `replan_barrier` and three traffic cones around x=90 m. `lidar_obstacle_detector` uses the live `/scan` and `/ambulance/odom` topics to publish real obstacle positions to `/ambulance/obstacles`. The digital twin updates only the affected road edges and records the replanning event in `results/latest/events.json`; Nav2 simultaneously adds the scan return to its local costmap for collision avoidance. Drive toward the central obstruction in teleoperation, then use the north or south bypass.

### Nav2 goal on the SLAM map

After driving enough to map the corridor, send a goal in the live `map` frame:

```bash
./nav_goal.sh --x 145 --y 0 --yaw 0
```

This invokes Nav2's `navigate_to_pose` action, not a timed motion script. Nav2 uses the current SLAM map and live LiDAR costmap to plan and avoid the barrier. Use an explored location first (for example `--x 25 --y 0`) before sending the hospital goal.

`ambulance_description` provides the vehicle URDF; `ambulance_gazebo/worlds/smart_city.sdf` provides a Gazebo Harmonic city with three road corridors, cross-links, hospital emergency entrance, ambulance station, urban districts, signals, construction zone, vehicles, and native LiDAR/camera/IMU/GPS topics. Install the standard Jazzy packages appropriate to your OS: `ros-jazzy-nav2-bringup`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-ros-gz-sim`, and `ros-jazzy-rviz2`. The core node intentionally has no Nav2 API dependency; it publishes a route plan that a Nav2 bridge can consume.

## Data and caveats

All generated plots/summary values are derived from the current run's route and timing measurements. This repository does not claim hardware validation or provide commercial city assets. Replace the primitive buildings in the supplied SDF with licensed Gazebo Fuel models for a production-quality visual city.

## Topics

The core node publishes JSON state on `/digital_twin/status`, `/digital_twin/route`, `/digital_twin/obstacles`, and `/ambulance/status`; it subscribes to `/ambulance/obstacles` (`std_msgs/String` JSON list) and `/ambulance/emergency`. A simulator integration should also supply `/ambulance/scan`, `/ambulance/camera/image_raw`, `/ambulance/imu`, `/ambulance/odom`, and `/ambulance/gps`.
