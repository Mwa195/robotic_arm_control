# robotic_arm_control

A ROS 2 package for simulating and controlling a **3-DOF robotic arm** mounted on a rover platform. The package provides the robot URDF, STL meshes, a Gazebo Harmonic simulation environment, and Python forward/inverse kinematics solvers.

---

## Table of Contents

- [Overview](#overview)
- [Robot Description](#robot-description)
- [Package Structure](#package-structure)
- [URDF Hardware Plugins](#urdf-hardware-plugins)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Visualize in RViz2](#1-visualize-in-rviz2)
  - [Launch Gazebo Simulation](#2-launch-gazebo-simulation)
  - [Forward Kinematics Node](#3-forward-kinematics-node)
  - [Inverse Kinematics Node](#4-inverse-kinematics-node)
  - [Simulation Control Testing](#5-simulation-control-testing)
- [Kinematics](#kinematics)
  - [Forward Kinematics](#forward-kinematics)
  - [Inverse Kinematics](#inverse-kinematics)
- [Joint Limits](#joint-limits)
- [Configuration](#configuration)

---

## Overview

`robotic_arm_control` is the base package for the arm system. It contains:

- A fully described URDF robot model with inertial, visual, and collision properties
- STL mesh files for all links
- Three `<ros2_control>` hardware plugin blocks in the URDF — one for each operating mode (see [URDF Hardware Plugins](#urdf-hardware-plugins))
- Gazebo Harmonic simulation with `gz_ros2_control`
- A `JointTrajectoryController` for position control
- A Python FK node that subscribes to `/joint_states` and reports end-effector position in real time
- A Python IK node that solves for joint angles given a Cartesian target and sends the trajectory to the arm controller
- A test script for running predefined motion sequences

---

## Robot Description

The arm is a **3-DOF revolute chain** mounted on a static rover base:

| Link | Mass (kg) | Description |
|---|---|---|
| `rover_link` | 3.524 | Fixed rover body |
| `base_Link` | 1.235 | Rotating base (yaw) |
| `link1_Link` | 0.358 | Upper arm (shoulder pitch) |
| `link2_Link` | 0.239 | Forearm (elbow pitch) |
| `tip_link` | — | Fixed end-effector frame |

| Joint | Type | Parent → Child | Axis | Range |
|---|---|---|---|---|
| `base_joint` | revolute | rover_link → base_Link | Z | ±180° |
| `shoulder_joint` | revolute | base_Link → link1_Link | −X | ±90° |
| `elbow_joint` | revolute | link1_Link → link2_Link | +X | ±120° |
| `tip_joint` | fixed | link2_Link → tip_link | — | — |

**Key geometric parameters (derived from URDF):**

- Shoulder height above rover frame: **0.16 m**
- Link 1 effective length (L1): **≈ 0.3809 m**
- Link 2 length (L2): **0.3087 m**
- Total max reach: **≈ 0.690 m**
- Structural tilt of link1 at zero config (α₁): **≈ 30.44°**

---

## Package Structure

```
robotic_arm_control/
├── config/
│   ├── arm_controllers.yaml      # ros2_control controller configuration (Gazebo mode)
│   ├── display.rviz              # RViz2 display configuration
│   └── joint_names_URDF.yaml     # Joint name list
├── launch/
│   ├── gazebo.launch.py          # Gazebo Harmonic simulation launch
│   └── display.launch            # RViz2 visualization launch
├── meshes/
│   ├── base_Link.STL
│   ├── link1_Link.STL
│   ├── link2_Link.STL
│   └── rover_link.STL
├── scripts/
│   ├── arm_fk.py                 # Forward kinematics ROS 2 node
│   ├── arm_ik.py                 # Inverse kinematics ROS 2 node
│   └── sim_control_testing.py    # Predefined motion test sequence
├── urdf/
│   ├── URDF.urdf                 # Robot description — contains all three hardware plugin blocks
│   └── URDF.csv                  # SolidWorks-exported link/joint data
├── CMakeLists.txt
└── package.xml
```

---

## URDF Hardware Plugins

The URDF contains plugin declarations for **all three operating modes**. Only one hardware plugin is active at a time — switch by editing `urdf/URDF.urdf` and setting the `<plugin>` tag in the `<ros2_control>` block:

| Mode | Plugin | When to use |
|---|---|---|
| MoveIt demo | `mock_components/GenericSystem` | Planning and visualization only, no physical hardware or Gazebo |
| Gazebo simulation | `gz_ros2_control/GazeboSimSystem` | Physics simulation in Gazebo Harmonic |
| Real hardware | `arm_hardware_interface/ArmHardwareInterface` | Actual ESP32 over UART serial or Wifi |

--> To switch modes, open urdf/URDF.urdf and replace the <plugin> tag in the <ros2_control> block with the appropriate value from the table above. Only one plugin can be active at a time.

The `<gazebo>` plugin block for `gz_ros2_control` is always present in the URDF and harmless when not in Gazebo mode — Gazebo simply won't be running to load it.

**Real hardware URDF params** (used by `ArmHardwareInterface`):

```xml
<ros2_control name="ArmSystem" type="system">
  <hardware>
    <plugin>arm_hardware_interface/ArmHardwareInterface</plugin>
    <!-- Transport selection: "serial" or "wifi"
         Override at launch time:
           ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi
           ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=serial -->
    <param name="transport">serial</param>
    <!-- Serial transport params -->
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
    <!-- WiFi transport params (used when transport=wifi) -->
    <param name="esp_ip">192.168.1.105</param>
    <param name="esp_port">8888</param>
  </hardware>
  ...
</ros2_control>
```

---

## Dependencies

**ROS 2 distribution:** Jazzy (or compatible)

| Package | Purpose |
|---|---|
| `robot_state_publisher` | Publishes TF from URDF |
| `joint_state_publisher` / `joint_state_publisher_gui` | Manual joint control in RViz2 |
| `controller_manager` | ros2_control infrastructure |
| `joint_trajectory_controller` | Position-controlled arm trajectory execution |
| `joint_state_broadcaster` | Publishes `/joint_states` from hardware/simulation |
| `gazebo_ros2_control` | Gazebo Harmonic ↔ ros2_control bridge |
| `ros_gz_bridge` | ROS 2 ↔ Gazebo topic bridge |
| `ros_gz_sim` | Robot spawning in Gazebo |
| `xacro` | URDF processing |

**Python dependencies:**

```
numpy
scipy        # required only for ik_refined mode in arm_ik.py
rclpy
```

---

## Installation

1. Clone the workspace and build:

   ```bash
   cd ~/arm_system
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Install ROS 2 dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

---

## Usage

### 1. Visualize in RViz2

```bash
ros2 launch robotic_arm_control display.launch
```

Starts `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`. Use the sliders to interactively drive each joint.

### 2. Launch Gazebo Simulation

Ensure the URDF has `gz_ros2_control/GazeboSimSystem` as the active plugin, then:

```bash
ros2 launch robotic_arm_control gazebo.launch.py
```

Starts Gazebo Harmonic, spawns the robot, and brings up `joint_state_broadcaster` and `arm_controller`. Controllers are sequenced with `OnProcessExit` event handlers to guarantee correct startup order.

### 3. Forward Kinematics Node

With any `/joint_states` publisher running:

```bash
ros2 run robotic_arm_control arm_fk.py
```

Subscribes to `/joint_states` and prints formatted end-effector information on every update:

```
───────────────────────────────────────────────────────
  Joint angles:  t1= +89.64°  t2= +77.11°  t3= +59.62°
  Shoulder:  (+0.0000, +0.0000, +0.1600) m
  Elbow:     (-0.1146, +0.0067, +0.5223) m
  Tip:       (+0.1799, +0.0049, +0.6151) m
  Reach:     Horizontal r=0.1799 m  Vertical z=0.6151 m  Distance from shoulder to tip D=0.4894 m
───────────────────────────────────────────────────────
```

Joint angles are looked up by name from the `/joint_states` message, so ordering in the message does not matter.

### 4. Inverse Kinematics Node

```bash
# Elbow-up (default)
ros2 run robotic_arm_control arm_ik.py 0.3 -0.3 0.5

# Elbow-down
ros2 run robotic_arm_control arm_ik.py 0.3 -0.3 0.5 False
```

Solves IK for the given `x y z` target (metres, rover frame), checks joint limits, verifies via FK, and sends a `FollowJointTrajectory` action goal to `/arm_controller/follow_joint_trajectory`.

### 5. Simulation Control Testing

```bash
ros2 run robotic_arm_control sim_control_testing.py
```

Runs a predefined sequence: home → base 90° → shoulder 45° → elbow −57° → home.

---

## Kinematics

### Forward Kinematics

Joint axes:
- `base_joint` → rotates around **+Z** (`Rz(t1)`)
- `shoulder_joint` → rotates around **−X** (`Rx(-t2)`)
- `elbow_joint` → rotates around **+X** (`Rx(t3)`)

Computation chain:

```
R_link1 = Rz(t1) @ Rx(-t2)
elbow   = shoulder_origin + R_link1 @ elbow_offset_in_link1
R_link2 = R_link1 @ Rx(t3)
tip     = elbow + R_link2 @ [0, -L2, 0]
```

### Inverse Kinematics

Two modes:

**Analytical** (`use_refined=False`) — closed-form, ~6 mm accuracy:
1. Decouple base: `t1 = atan2(x, -y)`
2. Project target into the arm's 2D plane
3. Law of cosines for the 2-link planar problem
4. Convert back via the structural offset α₁

**Refined** (`use_refined=True`, default) — Newton iteration via `scipy.optimize.fsolve`, sub-millimetre accuracy. Uses the analytical solution as the initial guess and compensates for the 6 mm X-offset at the elbow joint.

Both elbow-up and elbow-down configurations are supported.

---

## Joint Limits

| Joint | Lower | Upper |
|---|---|---|
| `base_joint` | −180° (−π rad) | +180° (+π rad) |
| `shoulder_joint` | −90° (−1.57 rad) | +90° (+1.57 rad) |
| `elbow_joint` | −120° (−2.094 rad) | +120° (+2.094 rad) |

Limits are enforced in both the URDF and the ESP32 firmware (`clamp()` function). MoveIt also respects them via `joint_limits.yaml` in `arm_moveit_conf_pkg`.

---

## Configuration

**`config/arm_controllers.yaml`** — ros2_control for Gazebo mode:
- Update rate: 50 Hz
- Command interface: `position`
- State interfaces: `position`, `velocity`

For real hardware the controller config lives in `arm_moveit_conf_pkg/config/ros2_controllers.yaml` at 100 Hz.

---

## License

MIT — see `package.xml` for details.
