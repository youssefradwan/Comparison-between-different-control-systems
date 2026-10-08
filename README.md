# ARL Autonomous Vehicle Control Track

**Autotronics Research Lab (ARL) — Ain Shams University**  
*Course: Autonomous Vehicles & Drive-by-Wire Systems | Individual Project*

<p align="center">
  <img src="assets/demo.gif" alt="Autonomous Vehicle Simulation Demo" width="100%" />
</p>

---

## 🧠 What Is This Project About?

In this project you will build the control system for a self-driving car in a ROS 2 simulation. The car drives around a racetrack and your job is to make it stay on the path, control its speed, and complete laps as fast and accurately as possible.

You will work through a series of milestones, each building on the last:

1. **Explore the system** — Learn what topics the car publishes and subscribes to.
2. **Bring the car to life** — Implement the physics equations that describe how the car moves.
3. **Drive it manually** — Build a keyboard teleoperation node to drive the car yourself.
4. **Add cruise control** — Implement a PID speed controller so the car holds a steady speed.
5. **Make it autonomous** — Implement three different steering controllers (Lateral PID, Pure Pursuit, and MPC) so the car drives itself around the track.
6. **Monitor performance** — Build a lap analyzer that logs lap times, tracking error, and shows live graphs and 3D overlays in RViz.
7. **Report your results** — Compare your controllers and document your findings.

The car model is realistic: it has velocity as a state (not a direct input), meaning it accelerates and decelerates due to drag and friction — just like a real vehicle.



---



---

## 🚀 Quickstart

### 1. Build the Workspace
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install build, simulation, and controller dependencies
sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro

# Build the workspace (bicycle_sim, bicycle_control, track_environment)
cd /path/to/bicycle_gym-main
colcon build --symlink-install
source install/setup.bash
```

In every new terminal, source the ROS distribution and built workspace:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/bicycle_gym-main
source install/setup.bash
```

### 2. Launch Modes

| Mode | Launch Command | Section |
|---|---|---|
| **Base Simulation (CLI Testing)** | `ros2 launch bicycle_sim bicycle_sim.launch.py` | Milestones 1 & 2 |
| **Interactive Keyboard Teleop** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` | Milestones 3 & 4 |
| **Lateral PID (Reactive)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid` | Milestone 5.2 |
| **Pure Pursuit (Geometric Preview)**| `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit` | Milestone 5.3 |
| **Extended Kinematic MPC (Optimal Preview)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc` | Milestone 5.4 |

For keyboard teleoperation, start the keyboard driver in a second sourced terminal:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To request closed-loop cruise control in teleoperation mode:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true
```

The launch file also supports `rviz:=false` and `analyzer:=false` to disable those nodes. For example:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit rviz:=false
```

### 3. Inspecting the ROS Graph

Run these commands from a second sourced terminal while the simulation is running:

```bash
ros2 node list
ros2 topic list
ros2 topic info /state
ros2 topic info /throttle
ros2 topic info /steer
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show std_msgs/msg/Float32
ros2 interface show geometry_msgs/msg/Twist
ros2 topic echo /state
```

### 4. Direct Actuator Testing

With the base simulation running, send actuator commands from another sourced terminal. Throttle/brake uses `[-1.0, 1.0]`; steering is in radians, with positive values turning left.

```bash
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: 0.5}"
ros2 topic pub --once /steer std_msgs/msg/Float32 "{data: 0.30}"
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: -1.0}"
```

### 5. Real-Time Telemetry & Graphing
```bash
# Install plotting and telemetry visualization tools
sudo apt update && sudo apt install -y ros-humble-plotjuggler-ros rqt-plot

# Inspect live signals in rqt_plot:
ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed

# Or launch PlotJuggler for multi-topic time-series analysis:
ros2 run plotjuggler plotjuggler
```

Some telemetry topics are only available after the corresponding analyzer work is complete.

---

## 🎯 Milestones at a Glance

- **Milestone 1**: Topic Discovery, Graph Inspection & Telemetry Plotting (`ros2 topic list / info`, `rqt_plot`, `plotjuggler`)
- **Milestone 2**: Extended Kinematic Bicycle Model & Euler Integration (`src/bicycle_sim/bicycle_sim/bicycle_model.py`)
- **Milestone 3**: Teleoperation Bridge & Open-Loop Driving (`src/bicycle_control/bicycle_control/teleop_bridge.py`)
- **Milestone 4**: Low-Level Powertrain Cruise Control (`src/bicycle_control/bicycle_control/longitudinal_pid.py`)
- **Milestone 5**: Autonomous Path Tracking — It's Time to Get the Car to Drive Autonomously!
  - **5.1**: High-Level Velocity Profiler & Path Curvature (`src/bicycle_control/bicycle_control/velocity_profiler.py`)
  - **5.2**: Steer Using Reactive Feedback (`src/bicycle_control/bicycle_control/lateral_pid.py`)
  - **5.3**: Steer Using Geometric Preview (`src/bicycle_control/bicycle_control/pure_pursuit.py`)
  - **5.4**: Steer Using Constrained Optimal Preview (Extended Kinematic MPC) (`src/bicycle_control/bicycle_control/mpc.py`)
  - **5.5**: Real-Time Telemetry, Graphing & RViz Dashboard Engineering (`src/track_environment/track_environment/lap_analyzer.py`)
- **Milestone 6**: Free Exploration & Reference Resources (Ackermann Kinematics, 3D Simulation, Nav2 MPPI)
- **Milestone 7**: Deliverable 1 — Repository Documentation (`README.md` Benchmark Report)
- **Milestone 8**: Deliverable 2 — Technical Video Walkthrough (3–5 Minute Demo)

---

## 🏆 Telemetry Benchmark Leaderboard

*(To be completed by the student as part of Milestone 7)*

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| **Manual Teleoperation** | — | — | — | — | — | — |
| **Lateral PID (Reactive)** | — | — | — | — | — | — |
| **Pure Pursuit (Preview)** | — | — | — | — | — | — |
| **Extended Kinematic MPC (Optimal)** | — | — | — | — | — | — |
# Comparison-between-different-control-systems
