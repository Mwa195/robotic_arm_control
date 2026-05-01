# arm_training

A ROS 2 package for simulating and controlling a **3-DOF robotic arm** mounted on a rover platform. The package provides a full Gazebo Harmonic simulation environment, forward and inverse kinematics solvers, and trajectory control via `ros2_control`.

---

## Table of Contents

- [Overview](#overview)
- [Robot Description](#robot-description)
- [Package Structure](#package-structure)
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

`arm_training` is designed as a learning and development platform for robotic arm kinematics and simulation in ROS 2. It includes:

- A fully described URDF robot model with inertial, visual, and collision properties
- STL mesh files for all links
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
arm_training/
├── config/
│   ├── arm_controllers.yaml      # ros2_control controller configuration
│   ├── display.rviz              # RViz2 display configuration
│   └── joint_names_URDF.yaml     # Joint name list
├── launch/
│   ├── gazebo.launch.py          # Gazebo Harmonic simulation launch
│   ├── display.launch            # RViz2 visualization launch
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
│   ├── URDF.urdf                 # Robot description (URDF)
│   └── URDF.csv                  # SolidWorks-exported link/joint data
├── CMakeLists.txt
└── package.xml
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
scipy        # required only for ik_refined mode
rclpy
```

---

## Installation

1. **Clone the repository** into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-url> arm_training
   ```

2. **Install ROS 2 dependencies:**

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**

   ```bash
   colcon build --packages-select arm_training
   source install/setup.bash
   ```

---

## Usage

### 1. Visualize in RViz2

Launch the robot model with the joint state publisher GUI for interactive joint control:

```bash
ros2 launch arm_training display.launch
```

This starts:
- `robot_state_publisher` — publishes the robot TF tree
- `joint_state_publisher_gui` — provides sliders to manually drive each joint
- `rviz2` — visualizes the robot model

### 2. Launch Gazebo Simulation

Start the full Gazebo Harmonic simulation with ros2_control:

```bash
ros2 launch arm_training gazebo.launch.py
```

This starts Gazebo, spawns the robot, and brings up both the `joint_state_broadcaster` and `arm_controller`. Controllers are sequenced with appropriate delays to allow the controller manager to initialize fully.

### 3. Forward Kinematics Node

With the simulation (or any `/joint_states` publisher) running, start the FK node:

```bash
ros2 run arm_training arm_fk.py
```

The node subscribes to `/joint_states` and prints formatted end-effector information on every update:

```
───────────────────────────────────────────────────
  Joint angles:  t1=  0.00°  t2= 45.00°  t3=-30.00°
  Shoulder:  (+0.0000, +0.0000, +0.1600) m
  Elbow:     (+0.0000, -0.2792, +0.4625) m
  Tip:       (+0.0000, -0.5446, +0.3156) m
  Reach:     Horizontal r=0.5446 m  Vertical z=0.3156 m  Distance from shoulder to tip D=0.6527 m
───────────────────────────────────────────────────
```

### 4. Inverse Kinematics Node

Move the arm tip to a specific Cartesian position (in the rover frame, metres):

```bash
# Elbow-up configuration (default)
ros2 run arm_training arm_ik.py 0.3 -0.3 0.5

# Elbow-down configuration
ros2 run arm_training arm_ik.py 0.3 -0.3 0.5 False
```

The node will:
1. Solve IK for the target position
2. Check joint limits
3. Verify the solution via FK
4. Send a `FollowJointTrajectory` action goal to `/arm_controller/follow_joint_trajectory`

### 5. Simulation Control Testing

Run a predefined test sequence that moves the arm through several positions and returns it home:

```bash
ros2 run arm_training sim_control_testing.py
```

The test sequence moves through: home → base rotation (90°) → shoulder tilt (45°) → elbow bend (−57°) → home.

---

## Kinematics

### Forward Kinematics

The FK solver (`arm_fk.py`) computes the positions of the shoulder, elbow, and tip given the three joint angles using direct rotation matrix composition.

The arm's joint axes are:
- **base_joint** → rotates around **+Z** (`Rz(t1)`)
- **shoulder_joint** → rotates around **−X** (`Rx(-t2)`)
- **elbow_joint** → rotates around **+X** (`Rx(t3)`)

The computation chain:

```
R_link1 = Rz(t1) @ Rx(-t2)
elbow   = shoulder_origin + R_link1 @ elbow_offset_in_link1
R_link2 = R_link1 @ Rx(t3)
tip     = elbow + R_link2 @ [0, -L2, 0]
```

### Inverse Kinematics

The IK solver (`arm_ik.py`) supports two modes:

**Analytical** (`use_refined=False`) — closed-form solution in ~6 mm accuracy:

1. Decouple the base joint: `t1 = atan2(x, -y)`
2. Project the target into the arm's 2D plane: `r = sqrt(x²+y²)`, `h = z − 0.16`
3. Apply the law of cosines to solve the 2-link planar problem
4. Convert planar angles back to joint angles via the structural offset α₁

**Refined** (`use_refined=True`, default) — Newton iteration via `scipy.optimize.fsolve`:

Uses the analytical solution as the initial guess and refines to sub-millimetre accuracy (< 0.01 mm), compensating for the 6 mm X-offset at the elbow joint in the URDF.

Both **elbow-up** and **elbow-down** configurations are supported. The solver returns `None` for unreachable targets or solutions that violate joint limits.

---

## Joint Limits

| Joint | Lower | Upper |
|---|---|---|
| `base_joint` | −180° (−π rad) | +180° (+π rad) |
| `shoulder_joint` | −90° (−1.57 rad) | +90° (+1.57 rad) |
| `elbow_joint` | −120° (−2.094 rad) | +120° (+2.094 rad) |

---

## Configuration

**`config/arm_controllers.yaml`** — ros2_control parameters:

- Controller manager update rate: **50 Hz**
- State/command publish rate: **50 Hz**
- Command interface: `position`
- State interfaces: `position`, `velocity`

**`config/display.rviz`** — pre-configured RViz2 layout for robot model visualization.

**`launch/gazebo.launch.py`** — injects the controller config path into the URDF at launch time and sets `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` for Gazebo Harmonic compatibility.

---

## License

MIT — see `package.xml` for details.
