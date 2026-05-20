# arm_system — 3-DOF Robotic Arm Workspace

A ROS 2 Jazzy workspace for a **3-DOF robotic arm** controlled over serial (USB or WiFi) via an ESP32 microcontroller. The system supports Gazebo Harmonic simulation, MoveIt 2 motion planning, real hardware execution, and live forward/inverse kinematics.

---

## Workspace Structure

```
arm_system/
├── robotic_arm_control/       # URDF, meshes, kinematics scripts, Gazebo launch
├── arm_moveit_conf_pkg/       # MoveIt 2 config, real hardware launch
├── arm_hardware_interface/    # ros2_control plugin (serial ↔ ESP32)
└── arm_esp_controller/        # ESP32 firmware (PlatformIO project)
```

---

## Package Roles

| Package | Role |
|---|---|
| `robotic_arm_control` | Robot model (URDF + STL meshes), FK/IK Python scripts, Gazebo simulation launch |
| `arm_moveit_conf_pkg` | MoveIt 2 configuration (SRDF, kinematics, planners), real hardware launch file |
| `arm_hardware_interface` | `ros2_control` hardware plugin — opens serial port, sends `CMD`, reads `STATE` |
| `arm_esp_controller` | ESP32 firmware — parses `CMD`, clamps joint limits, controls servos, streams `STATE` back |

---

## Communication Protocol

All communication between the ROS side and the ESP32 uses plain ASCII over a serial link (USB-UART at 115200 baud):

```
ROS → ESP:   CMD <base> <shoulder> <elbow>\n        (radians, 6 decimal places)
ESP → ROS:   STATE <base> <shoulder> <elbow>\n     (radians, 4 decimal places)
```

The ESP only begins streaming `STATE` after it receives the first valid `CMD`. Before that, the serial line is silent - this is the intended behaviour.

---

## Quick Start

### Prerequisites

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gazebo-ros2-control ros-jazzy-ros-gz-sim
pip install numpy scipy
```

### Build

```bash
cd ~/arm_system
colcon build --symlink-install
source install/setup.bash
```

### Mode 1 — MoveIt demo (no hardware, no Gazebo)

Uses `mock_components/GenericSystem` — arm moves in RViz only. Requires `mock_components` plugin active in URDF.

```bash
ros2 launch arm_moveit_conf_pkg demo.launch.py
```

### Mode 2 — Gazebo simulation

Requires `gz_ros2_control/GazeboSimSystem` plugin active in URDF.

```bash
# Terminal 1
ros2 launch robotic_arm_control gazebo.launch.py

# Terminal 2
ros2 launch arm_moveit_conf_pkg move_group.launch.py

# Terminal 3
ros2 launch arm_moveit_conf_pkg moveit_rviz.launch.py
```

### Mode 3 — Real hardware (ESP32 over USB)

Requires `arm_hardware_interface/ArmHardwareInterface` plugin active in URDF.

```bash
# Flash ESP first (see arm_esp_controller/README.md)
sudo chmod 666 /dev/ttyUSB0
ros2 launch arm_moveit_conf_pkg real_arm.launch.py
```

### Mode 4 - Real hardware (ESP32 over Wifi)

Requires `arm_hardware_interface/ArmHardwareInterface` plugin active in URDF.
```bash
# Replace ***.***.*.** with esp's wifi ip address
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=***.***.*.**
```

### FK / IK tools (any mode with /joint_states running)

```bash
# Live end-effector position
ros2 run robotic_arm_control arm_fk.py

# Move to Cartesian target
ros2 run robotic_arm_control arm_ik.py 0.3 -0.3 0.5
```

---

## URDF Hardware Plugin — Switch by Mode

The URDF contains three `<ros2_control>` / `<gazebo>` blocks. Only one hardware plugin can be active at a time. Edit `robotic_arm_control/urdf/URDF.urdf` and set the `<plugin>` tag accordingly:

| Mode | Plugin |
|---|---|
| Demo / MoveIt only | `mock_components/GenericSystem` |
| Gazebo simulation | `gz_ros2_control/GazeboSimSystem` |
| Real hardware | `arm_hardware_interface/ArmHardwareInterface` |

---

## Debugging Tips

| Symptom | Likely cause | Fix |
|---|---|---|
| `No complete STATE line` warnings | ESP not connected or `got_cmd` gate not passed | Check USB connection; send a trajectory first |
| `time=0.000000` on `/joint_states` | Controllers not yet active when `move_group` started | Use the fixed `real_arm.launch.py` with full `OnProcessExit` chain |
| `Could not contact /controller_manager` | `ros2_control_node` crashed (serial port missing) | Check `/dev/ttyUSB0` exists and has read/write permission |
| IK marker snaps back | Target outside workspace or `position_only_ik` missing | Max reach ≈ 0.69 m; verify `kinematics.yaml` |
| Arduino serial monitor freezes ROS | DTR signal resets ESP on monitor open | Use `stty -F /dev/ttyUSB0 raw 115200 && cat /dev/ttyUSB0` instead |

---

## To-Do

- [✓] **WiFi adaptation** — replace serial transport with TCP socket in `arm_hardware_interface`; add matching WiFi TCP server firmware to `arm_esp_controller`; update URDF params from `serial_port`/`baud_rate` to `esp_ip`/`esp_port`
- [✓] **Real servo testing** — connect physical servo motors to ESP32 PWM outputs; validate that commanded joint angles produce correct physical motion; tune scaling between radians and servo pulse widths; verify joint limit clamping prevents mechanical over-travel
- [ ] **Launch sequence hardening** — handle graceful failure when ESP is not connected at launch time; add a hardware-check node that verifies `/dev/ttyUSB0` exists and is readable before `ros2_control_node` starts, and falls back to mock mode automatically if it is absent

---

## License

MIT
