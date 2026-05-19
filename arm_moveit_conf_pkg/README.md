# arm_moveit_conf_pkg

A MoveIt 2 configuration package for the **3-DOF robotic arm** defined in `robotic_arm_control`. Provides motion planning, collision avoidance, IK solving, and trajectory execution via `ros2_control` — for both mock/demo mode and real ESP8266 hardware.

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Demo Mode](#1-demo-mode-no-gazebo-no-hardware)
  - [With Gazebo Simulation](#2-with-gazebo-simulation)
  - [Real Hardware](#3-real-hardware-esp8266-over-usb)
  - [Setting Goals in RViz](#4-setting-goals-in-rviz)
  - [Controlling Execution Speed](#5-controlling-execution-speed)
  - [Inspecting a Planned Trajectory](#6-inspecting-a-planned-trajectory)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [Known Fixes Applied](#known-fixes-applied)

---

## Overview

`arm_moveit_conf_pkg` provides the MoveIt 2 layer on top of `robotic_arm_control`. It handles:

- Semantic robot description (SRDF) — planning groups, end-effectors, disabled collisions
- KDL IK solver configuration for a 3-DOF arm (`position_only_ik: true`)
- Joint velocity and acceleration limits for trajectory parameterization
- `ros2_control` controller configuration at 100 Hz (real hardware) and 50 Hz (Gazebo)
- MoveIt controller interface linking `move_group` to `arm_controller` via `FollowJointTrajectory`
- RViz configuration with the MoveIt Motion Planning plugin pre-loaded
- `real_arm.launch.py` — real hardware launch with mixed timer/event sequencing (see Known Issues)

---

## Package Structure

```
arm_moveit_conf_pkg/
├── config/
│   ├── URDF.srdf                    # Semantic robot description
│   ├── joint_limits.yaml            # Velocity/acceleration limits (must be floats)
│   ├── kinematics.yaml              # KDL IK solver (position_only_ik: true)
│   ├── moveit_controllers.yaml      # MoveIt → ros2_control bridge
│   ├── moveit.rviz                  # RViz layout with MoveIt Motion Planning plugin
│   ├── pilz_cartesian_limits.yaml   # Cartesian velocity limits for Pilz planner
│   └── ros2_controllers.yaml        # Controller manager config (100 Hz, real hardware)
├── launch/
│   ├── real_arm.launch.py           # Real hardware launch (timer + OnProcessExit hybrid)
│   ├── demo.launch.py               # Mock hardware — MoveIt + RViz, no physical arm
│   ├── move_group.launch.py         # move_group node only
│   ├── moveit_rviz.launch.py        # RViz only (connect to existing move_group)
│   ├── rsp.launch.py                # Robot State Publisher only
│   ├── spawn_controllers.launch.py  # Controller spawner only
│   ├── static_virtual_joint_tfs.launch.py
│   └── warehouse_db.launch.py       # MoveIt warehouse database (optional)
├── .setup_assistant
├── CMakeLists.txt
└── package.xml
```

---

## Dependencies

**ROS 2 distribution:** Jazzy

| Package | Purpose |
|---|---|
| `moveit` | Core MoveIt 2 framework |
| `moveit_ros_planning_interface` | `MoveGroupInterface` API |
| `moveit_kinematics` | KDL IK solver plugin |
| `moveit_planners_ompl` | OMPL planning pipeline |
| `pilz_industrial_motion_planner` | PTP/LIN/CIRC planners |
| `ros2_control` | Controller manager infrastructure |
| `joint_trajectory_controller` | Position-controlled trajectory execution |
| `joint_state_broadcaster` | Publishes `/joint_states` from hardware |
| `mock_components` | Fake hardware for demo mode |
| `robot_state_publisher` | Publishes TF from URDF |

---

## Installation

1. Ensure `robotic_arm_control` and `arm_hardware_interface` are already built.

2. Build this package:

   ```bash
   cd ~/arm_system
   colcon build --packages-select arm_moveit_conf_pkg
   source install/setup.bash
   ```

---

## Usage

### 1. Demo Mode (no Gazebo, no hardware)

Uses `mock_components/GenericSystem` — arm moves in RViz only. Requires that plugin to be active in the URDF.

```bash
ros2 launch arm_moveit_conf_pkg demo.launch.py
```

Starts `robot_state_publisher`, `move_group`, `ros2_control_node` (mock), and `rviz2` simultaneously. Wait for:

```
You can start planning now!
```

before interacting with RViz.

### 2. With Gazebo Simulation

Launch Gazebo from `robotic_arm_control` first, then bring up `move_group` and RViz:

```bash
# Terminal 1
ros2 launch robotic_arm_control gazebo.launch.py

# Terminal 2
ros2 launch arm_moveit_conf_pkg move_group.launch.py

# Terminal 3
ros2 launch arm_moveit_conf_pkg moveit_rviz.launch.py
```

> Do not run `demo.launch.py` alongside Gazebo — they use conflicting hardware plugins.

### 3. Real Hardware (ESP8266 over USB)

Requires `arm_hardware_interface/ArmHardwareInterface` active in the URDF and the ESP8266 flashed and connected.

```bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch arm_moveit_conf_pkg real_arm.launch.py
```

**Startup sequence:**

```
ros2_control_node starts
    └─ TimerAction(3s) → joint_state_broadcaster spawner
                             └─ OnProcessExit → arm_controller spawner
    └─ TimerAction(5s) → move_group
    └─ TimerAction(6s) → rviz2
```

`arm_controller` is correctly chained off `joint_state_broadcaster` via `OnProcessExit`. However, `move_group` and `rviz2` start on fixed timers independent of whether `arm_controller` has become active — on a slow machine or with serial port init delay this can cause `move_group` to see `time=0.000000` on `/joint_states` and abort execution with `"couldn't receive full current joint state within 1s"`. See [Known Issues](#known-issues) for the planned fix.

**Important serial monitor warning:** Never open the Arduino serial monitor while ROS is connected — opening it asserts DTR which resets the ESP mid-session. To inspect the raw STATE stream without interfering:

```bash
stty -F /dev/ttyUSB0 raw 115200 && cat /dev/ttyUSB0
```

### 4. Setting Goals in RViz

#### Method A — Joints Tab (most reliable)

`MotionPlanning → Joints` tab → drag sliders for `base_joint`, `shoulder_joint`, `elbow_joint`. Bypasses IK entirely.

#### Method B — Interactive Marker (Cartesian drag)

Enable `MotionPlanning → Planning Request → Query Goal State` in the Displays panel. An orange ghost arm appears. Press **Interact** in the toolbar, then drag the end-effector ball.

> If the marker snaps back, IK failed — target is outside the reachable workspace (max ≈ 0.69 m from shoulder) or `position_only_ik: true` is missing from `kinematics.yaml`.

#### Method C — Named Poses

Use the **Goal State** dropdown in the Planning tab to select `home` (all joints at 0°).

#### Executing

| Button | Action |
|---|---|
| **Plan** | Compute and visualize trajectory (no movement) |
| **Execute** | Execute the last computed plan |
| **Plan & Execute** | Plan then immediately execute |
| **Stop** | Abort execution mid-motion |

### 5. Controlling Execution Speed

In the Planning tab before clicking Plan:

- **Velocity Scaling** — fraction of max velocity (default `0.1`). Set to `0.5` for faster motion.
- **Acceleration Scaling** — same for acceleration (default `0.1`).
- **Planning Time** — seconds budget. Increase to `10.0` for complex configurations.

To make defaults permanent, edit `config/joint_limits.yaml`:

```yaml
default_velocity_scaling_factor: 0.5
default_acceleration_scaling_factor: 0.5
```

### 6. Inspecting a Planned Trajectory

After clicking **Plan**, scrub frame-by-frame before executing:

```
RViz menu → Panels → Trajectory - Trajectory Slider
```

---

## Launch Files

### `real_arm.launch.py`

The primary launch file for real hardware. Key design decisions:

- `joint_state_broadcaster` is correctly sequenced via `OnProcessExit` off `ros2_control_node`'s 3 s timer. `arm_controller` is then chained via `OnProcessExit` off `joint_state_broadcaster` — so by the time `arm_controller` starts, `/joint_states` is guaranteed to be publishing.
- `move_group` and `rviz2` start on fixed `TimerAction` delays (5 s and 6 s respectively), independent of controller state. This is a known limitation — see [Known Issues](#known-issues).
- `ros2_control_node` will fail and exit if `/dev/ttyUSB0` is not available — this is intentional. Do not launch in real hardware mode without the ESP connected.
- If you see `"Could not contact service /controller_manager"`, raise the `joint_state_broadcaster` timer from `3.0` to `5.0`.

### `demo.launch.py`

Uses `mock_components/GenericSystem`. All nodes start simultaneously. Only for planning visualization — the arm does not move physically.

---

## Configuration Files

### `config/URDF.srdf`

```xml
<group name="arm">
    <chain base_link="rover_link" tip_link="tip_link"/>
</group>

<group_state name="home" group="arm">
    <joint name="base_joint" value="0"/>
    <joint name="shoulder_joint" value="0"/>
    <joint name="elbow_joint" value="0"/>
</group_state>

<end_effector name="tip" parent_link="link2_Link" group="arm"/>

<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="rover_link"/>
```

### `config/kinematics.yaml`

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1
  position_only_ik: true    # required for 3-DOF arm
```

### `config/joint_limits.yaml`

All values must be **floats** (e.g. `1.0`, not `1`). Integer values crash `AddTimeOptimalParameterization`.

### `config/ros2_controllers.yaml`

Controller manager at 100 Hz for real hardware. Includes `allow_nonzero_velocity_at_trajectory_end: true` to prevent the controller from aborting trajectories where the arm arrives at the goal with a small residual velocity.

---

## Known Fixes Applied and Known Issues

### Fix 1 — Hardware Plugin Mismatch for Demo Mode

**Problem:** `demo.launch.py` uses mock hardware but the URDF had `gz_ros2_control/GazeboSimSystem`, which cannot load outside Gazebo. Controller manager spammed `Waiting for robot_description` indefinitely.

**Fix:** Switch the URDF plugin to `mock_components/GenericSystem` for demo mode. See `robotic_arm_control/README.md` for the full plugin switching table.

### Fix 2 — Joint Limits Must Be Floats

**Problem:** Setup Assistant wrote `max_velocity: 1` as integers. MoveIt's `AddTimeOptimalParameterization` requires floats and crashes with `FAILURE` on every plan attempt.

**Fix:** All values in `joint_limits.yaml` changed to `1.0`.

### Fix 3 — SRDF End-Effector Circular Reference

**Problem:** Generated SRDF had `parent_group="arm"` on the end-effector whose group is also `arm` — circular reference. MoveIt rejected it.

**Fix:** Removed `parent_group` attribute. Also changed `parent_link` from `tip_link` to `link2_Link` because `tip_link` is a massless geometry-less frame — MoveIt cannot attach the interactive drag marker to it.

### Fix 4 — KDL Position-Only IK

**Problem:** KDL attempted full 6-DOF IK (position + orientation) for a 3-DOF arm. Mathematically impossible — every IK call failed silently.

**Fix:** Added `position_only_ik: true` to `kinematics.yaml`.

### Fix 5 — KDL Solver Timeout

**Problem:** Default timeout of `0.005 s` caused IK to time out before finding a valid solution even for reachable targets.

**Fix:** Increased to `kinematics_solver_timeout: 0.1`.

### Fix 6 — `ros2_controllers.yaml` Missing `command_interfaces`

**Problem:** Generated file was missing `command_interfaces`. Spawner crashed with `parameter 'command_interfaces': No parameter value set`. The `follow_joint_trajectory` action server never came up.

**Fix:** Added the full `arm_controller` block with `command_interfaces`, `state_interfaces`, and `joints`.

### Known Issue — `real_arm.launch.py` Timer-Based `move_group` Start

**Problem:** `move_group` starts on a fixed 5 s `TimerAction` independent of whether `arm_controller` has reached `active` state. MoveIt's trajectory validator checks the `/joint_states` timestamp on execution — if `move_group` starts before controllers are fully active, it sees `time=0.000000` and aborts with `"couldn't receive full current joint state within 1s"`.

**Workaround:** On most machines the 5 s timer is enough margin. If you consistently hit this abort, raise `period=5.0` to `period=8.0` in `real_arm.launch.py`.

**Planned fix:** Replace the `TimerAction` for `move_group` and `rviz2` with `OnProcessExit` handlers chained off `arm_controller_spawner`, so each stage is guaranteed to start only after the previous controller reaches `active`. This is tracked in the workspace to-do list.

---

## License

MIT — see `package.xml` for details.
