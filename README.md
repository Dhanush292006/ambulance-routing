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
ros2 launch ambulance_bringup ambulance_system.launch.py use_gazebo:=true rviz:=true
```

`ambulance_description` provides the vehicle URDF; `ambulance_gazebo/worlds/smart_city.sdf` is an urban-world starter with roads, blocks, signals, emergency station and hospital. Install the standard Jazzy packages appropriate to your OS: `ros-jazzy-nav2-bringup`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-ros-gz-sim`, and `ros-jazzy-rviz2`. The core node intentionally has no Nav2 API dependency; it publishes a route plan that a Nav2 bridge can consume.

## Data and caveats

All generated plots/summary values are derived from the current run's route and timing measurements. This repository does not claim hardware validation or provide commercial city assets. Replace the primitive buildings in the supplied SDF with licensed Gazebo Fuel models for a production-quality visual city.

## Topics

The core node publishes JSON state on `/digital_twin/status`, `/digital_twin/route`, `/digital_twin/obstacles`, and `/ambulance/status`; it subscribes to `/ambulance/obstacles` (`std_msgs/String` JSON list) and `/ambulance/emergency`. A simulator integration should also supply `/ambulance/scan`, `/ambulance/camera/image_raw`, `/ambulance/imu`, `/ambulance/odom`, and `/ambulance/gps`.
