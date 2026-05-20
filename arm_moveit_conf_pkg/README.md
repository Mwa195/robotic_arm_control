# arm_moveit_conf_pkg

A MoveIt 2 configuration package for the **3-DOF robotic arm** defined in `robotic_arm_control`. Provides motion planning, collision avoidance, IK solving, and trajectory execution via `ros2_control` — for demo mode, Gazebo simulation, and real ESP32 hardware over **serial (USB) or WiFi**.

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Demo Mode](#1-demo-mode-no-gazebo-no-hardware)
  - [With Gazebo Simulation](#2-with-gazebo-simulation)
  - [Real Hardware — Serial](#3-real-hardware-serial-usb)
  - [Real Hardware — WiFi](#4-real-hardware-wifi)
  - [Setting Goals in RViz](#5-setting-goals-in-rviz)
  - [Controlling Execution Speed](#6-controlling-execution-speed)
  - [Inspecting a Planned Trajectory](#7-inspecting-a-planned-trajectory)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [Known Fixes Applied and Known Issues](#known-fixes-applied-and-known-issues)

---

## Overview

`arm_moveit_conf_pkg` provides the MoveIt 2 layer on top of `robotic_arm_control`. It handles:

- Semantic robot description (SRDF) — planning groups, end-effectors, disabled collisions
- KDL IK solver configuration for a 3-DOF arm (`position_only_ik: true`)
- Joint velocity and acceleration limits for trajectory parameterization
- `ros2_control` controller configuration at 100 Hz (real hardware) and 50 Hz (Gazebo)
- MoveIt controller interface linking `move_group` to `arm_controller` via `FollowJointTrajectory`
- RViz configuration with the MoveIt Motion Planning plugin pre-loaded
- `real_arm.launch.py` — real hardware launch with selectable serial/WiFi transport and correct controller sequencing

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
│   ├── real_arm.launch.py           # Real hardware launch — serial or WiFi selectable
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

---

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

---

### 3. Real Hardware — Serial (USB)

Requires `arm_hardware_interface/ArmHardwareInterface` active in the URDF and the ESP32 flashed and connected via USB.

```bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch arm_moveit_conf_pkg real_arm.launch.py
```

Serial is the default transport — no extra arguments needed. To use a different port:

```bash
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=serial serial_port:=/dev/ttyUSB1
```

**Important:** Never open the Arduino serial monitor while ROS is connected — it asserts DTR which resets the ESP mid-session. To inspect the raw STATE stream without interfering:

```bash
stty -F /dev/ttyUSB0 raw 115200 && cat /dev/ttyUSB0
```

---

### 4. Real Hardware — WiFi

Requires the ESP32 flashed with the WiFi firmware and connected to the same network as your laptop.

```bash
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=192.168.1.105
```

To use a non-default port:

```bash
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=192.168.1.105 esp_port:=9999
```

**Before launching, verify the ESP is reachable:**

```bash
ping 192.168.1.105
nc -zv 192.168.1.105 8888
```

To find the ESP's IP, check your router's DHCP table. Assigning a static DHCP lease by MAC address is recommended so the IP never changes between sessions.

> WiFi latency at 100 Hz: USB serial has near-zero latency. WiFi on a 2.4 GHz network can introduce 5–20 ms jitter, which may trigger occasional `"No complete STATE line this cycle"` warnings and slightly noisier velocity estimates. `TCP_NODELAY` is already set in the hardware interface to mitigate this. If jitter is consistently bad, drop the control rate to 50 Hz in `ros2_controllers.yaml`.

---

### Startup Sequence (both transports)

```
immediately:  robot_state_publisher   — publishes TF from URDF
              ros2_control_node       — loads ArmHardwareInterface, opens serial or TCP
                   │
after 3s:     joint_state_broadcaster — reads position_state_[] → publishes /joint_states
                   │ (on exit)
              arm_controller          — JointTrajectoryController, receives MoveIt goals

after 5s:     move_group              — MoveIt planning server
after 6s:     rviz2                   — visualization + interactive planning
```

`arm_controller` waits for `joint_state_broadcaster` to finish spawning via `OnProcessExit` — so `/joint_states` is guaranteed to be publishing before the trajectory controller comes online. `move_group` and `rviz2` start on fixed timers after that.

If you see `"Could not contact service /controller_manager"`, raise the `joint_state_broadcaster` timer from `3.0` to `5.0` in `real_arm.launch.py`.

---

### 5. Setting Goals in RViz

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

---

### 6. Controlling Execution Speed

In the Planning tab before clicking Plan:

- **Velocity Scaling** — fraction of max velocity (default `0.1`). Set to `0.5` for faster motion.
- **Acceleration Scaling** — same for acceleration (default `0.1`).
- **Planning Time** — seconds budget. Increase to `10.0` for complex configurations.

To make defaults permanent, edit `config/joint_limits.yaml`:

```yaml
default_velocity_scaling_factor: 0.5
default_acceleration_scaling_factor: 0.5
```

---

### 7. Inspecting a Planned Trajectory

After clicking **Plan**, scrub frame-by-frame before executing:

```
RViz menu → Panels → Trajectory - Trajectory Slider
```

---

## Launch Files

### `real_arm.launch.py`

The primary launch file for real hardware. Accepts four launch arguments:

| Argument | Default | Description |
|---|---|---|
| `transport` | `serial` | `"serial"` for USB, `"wifi"` for TCP |
| `serial_port` | `/dev/ttyUSB0` | Serial device (used when `transport=serial`) |
| `esp_ip` | `192.168.1.105` | ESP IP address (used when `transport=wifi`) |
| `esp_port` | `8888` | ESP TCP port (used when `transport=wifi`) |

At launch time, the file patches the URDF string in memory — replacing the four `<param>` values with whatever was passed on the command line — before handing it to `ros2_control_node` and `robot_state_publisher`. The URDF file on disk is never modified.

This means the transport selection is entirely a launch-time decision. No recompilation, no URDF editing, no separate launch files for serial vs WiFi — one command with one argument.

`ros2_control_node` will fail and exit if the connection cannot be established — serial port not found, or ESP unreachable over WiFi. This is intentional. Do not launch in real hardware mode without the ESP connected and reachable.

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

### Known Issue — `move_group` and `rviz2` Start on Fixed Timers

**Problem:** `move_group` starts on a fixed 5 s `TimerAction` and `rviz2` on 6 s, independent of whether `arm_controller` has reached `active` state. On a slow machine or with a slow WiFi connection, `arm_controller` may not be active yet when `move_group` starts — causing it to see `time=0.000000` on `/joint_states` and abort execution with `"couldn't receive full current joint state within 1s"`.

**Workaround:** On most machines the 5 s timer is enough margin. If you consistently hit this abort, raise `period=5.0` to `period=8.0` in `real_arm.launch.py`. With WiFi transport this is slightly more likely than with serial due to connection setup time.

**Planned fix:** Replace the `TimerAction` for `move_group` and `rviz2` with `OnProcessExit` handlers chained off `arm_controller_spawner`, so each stage is guaranteed to start only after the previous controller reaches `active`.

---

## License

MIT — see `package.xml` for details.