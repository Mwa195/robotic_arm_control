# arm_hardware_interface

A `ros2_control` hardware plugin that bridges the ROS 2 controller manager to an **ESP8266 microcontroller** over a serial connection. Implements the `hardware_interface::SystemInterface` lifecycle and handles all serial I/O for the 3-DOF robotic arm.

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Communication Protocol](#communication-protocol)
- [How It Works](#how-it-works)
  - [Lifecycle](#lifecycle)
  - [read() — 100 Hz](#read----100-hz)
  - [write() — 100 Hz](#write----100-hz)
  - [readLine() — persistent buffer](#readline----persistent-buffer)
- [URDF Integration](#urdf-integration)
- [Dependencies](#dependencies)
- [Building](#building)
- [Debugging](#debugging)
- [Known Issues and Design Decisions](#known-issues-and-design-decisions)

---

## Overview

`arm_hardware_interface` is a `pluginlib`-registered `SystemInterface` plugin. When loaded by `ros2_control_node`, it:

1. Opens `/dev/ttyUSB0` (or any port specified in the URDF) as a raw serial file descriptor
2. Every 100 Hz cycle, sends `CMD <base> <shoulder> <elbow>\n` to the ESP
3. Drains all available `STATE` lines from the serial buffer and commits the freshest one to `position_state_[]`
4. Exports `position_state_` and `velocity_state_` to `joint_state_broadcaster` → `/joint_states`
5. Reads `position_command_[]` written by `arm_controller` and sends it as the next `CMD`

There are no real encoders on this arm. The ESP echoes commanded positions back as state. The hardware interface is designed to be transparent about this — it never fabricates state data and correctly holds the last known position when no valid STATE line arrives.

---

## Package Structure

```
arm_hardware_interface/
├── include/arm_hardware_interface/
│   └── hardware_translator.hpp      # Class declaration
├── src/
│   └── hardware_translator.cpp      # Full implementation
├── arm_hardware_interface_plugin.xml  # pluginlib registration
├── CMakeLists.txt
└── package.xml
```

---

## Communication Protocol

```
ROS → ESP:    CMD <t1> <t2> <t3>\n
              e.g. "CMD 1.570796 0.500000 -0.300000\n"

ESP → ROS:    STATE <t1> <t2> <t3>\n
              e.g. "STATE 1.5708 0.5000 -0.3000\n"
```

- All values are joint angles in **radians**
- `CMD` uses 6 decimal places; `STATE` uses 4
- Lines are terminated with `\n` only (no `\r`)
- The ESP only begins sending `STATE` after the first valid `CMD` is received — before that, the serial line is silent. This is expected — the hardware interface will log `"No complete STATE line this cycle"` until the first trajectory executes.

---

## How It Works

### Lifecycle

The plugin follows the standard `ros2_control` lifecycle:

| Callback | Action |
|---|---|
| `on_init()` | Reads `serial_port` and `baud_rate` from URDF params; validates 3-joint config |
| `on_configure()` | Opens and configures the serial port (8N1, raw, non-blocking via `O_NONBLOCK`) |
| `on_activate()` | Seeds `position_command_[]` from `position_state_[]` to prevent jump-on-activate |
| `on_deactivate()` | Closes the serial file descriptor |

If `on_configure()` fails (e.g. `/dev/ttyUSB0` does not exist or lacks permissions), the controller manager exits. **Do not launch in real hardware mode without the ESP connected.**

### `read()` — 100 Hz

Called every 10 ms by the controller manager. The critical design decision here is **draining the full kernel buffer every cycle**:

```
while (readLine() returns a complete line):
    parse it
    if valid STATE: overwrite p0, p1, p2 (keep only freshest)
commit freshest p0, p1, p2 to position_state_[]
estimate velocity_state_[] = Δposition / Δtime
```

**Why drain all lines?** The ESP sends STATE at 100 Hz independently on its own `millis()` timer. The Linux scheduler does not guarantee that `read()` fires in perfect 10 ms lockstep — it can be delayed by a few milliseconds. When it is, the kernel serial buffer accumulates 2–4 complete STATE lines. Processing only the oldest one (as a naive single-`readLine()` implementation would do) causes state lag and eventually kernel buffer overflow — which drops bytes mid-line and produces malformed STATE strings that fail parsing, leaving `position_state_[]` stuck at the last known value.

Draining all lines and keeping only the freshest eliminates this entirely.

### `write()` — 100 Hz

Reads `position_command_[0..2]` (set by `arm_controller`) and sends:

```
CMD <t1> <t2> <t3>\n
```

Returns `ERROR` if the `write()` syscall fails (e.g. ESP disconnected mid-session).

### `readLine()` — persistent buffer

`readLine()` uses a `serial_buffer_` member (persistent across calls) rather than a fresh local string each cycle. This correctly handles the case where a STATE line is split across two 10 ms cycles — which happens when the scheduler fires `read()` while the ESP is mid-transmission. The partial line accumulates in `serial_buffer_` and completes on the next cycle.

Serial configuration:
- `O_NONBLOCK` on `open()` — `read()` returns `EAGAIN` immediately when no data is available instead of blocking
- `VMIN=0, VTIME=0` in termios — consistent with non-blocking mode
- `CRTSCTS` disabled — no hardware flow control
- Raw mode — no line buffering, no echo, no signal generation

---

## URDF Integration

Add this block to the `<robot>` element in `robotic_arm_control/urdf/URDF.urdf`:

```xml
<ros2_control name="ArmSystem" type="system">
  <hardware>
    <plugin>arm_hardware_interface/ArmHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
  </hardware>

  <joint name="base_joint">
    <command_interface name="position">
      <param name="min">-3.14159</param>
      <param name="max">3.14159</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="shoulder_joint">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="elbow_joint">
    <command_interface name="position">
      <param name="min">-2.094</param>
      <param name="max">2.094</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Only one `<ros2_control>` block can be active at a time — see `robotic_arm_control/README.md` for the full plugin switching table (mock / Gazebo / real hardware).

---

## Dependencies

| Package | Purpose |
|---|---|
| `rclcpp` | ROS 2 logging and clock |
| `hardware_interface` | `SystemInterface` base class |
| `pluginlib` | Plugin registration macros |
| `rclcpp_lifecycle` | Lifecycle state types |

All are standard `ros2_control` dependencies — no extra installation needed beyond a full Jazzy desktop install.

---

## Building

```bash
cd ~/arm_system
colcon build --packages-select arm_hardware_interface
source install/setup.bash
```

The `pluginlib` export is registered via `arm_hardware_interface_plugin.xml`. After building, verify the plugin is discoverable:

```bash
ros2 run pluginlib_tutorials list_plugins arm_hardware_interface
```

---

## Debugging

**Check serial port permissions:**
```bash
ls -la /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
# or permanently:
sudo usermod -aG dialout $USER   # then log out and back in
```

**Watch raw STATE stream without resetting the ESP:**
```bash
stty -F /dev/ttyUSB0 raw 115200 && cat /dev/ttyUSB0
```
> Never use the Arduino serial monitor while ROS is connected — opening it asserts DTR, which resets the ESP and corrupts the ROS session.

**Verify the plugin loads:**
```bash
ros2 control list_hardware_interfaces
```

**Watch `/joint_states` live:**
```bash
ros2 topic echo /joint_states
```
If positions are always 0.0, the ESP has not received a CMD yet (correct before first trajectory) or STATE lines are not parsing correctly (check for malformed warnings in `ros2_control_node` output).

---

## Known Issues and Design Decisions

**No real encoders.** The ESP echoes commanded values as state. This means:
- The arm always assumes home position (0, 0, 0) on boot
- `position_state_[]` reflects what was commanded, not what the arm physically did
- Velocity is estimated from the delta between successive commanded positions, not from actual motion

**STATE silence before first CMD.** The ESP does not stream STATE until it receives a valid CMD. This is intentional (prevents zero-flood on startup) but means the `"No complete STATE line this cycle"` warning appears continuously until the first trajectory executes. This is not an error.

**Single TCP connection (future WiFi mode).** The current implementation uses POSIX serial fd. The WiFi adaptation (TCP socket) reuses the same `serial_fd_` member and the same `readLine()`/`writeLine()` logic unchanged — only `openSerialPort()` is replaced with `openTcpSocket()`. See the workspace README for the full WiFi to-do item.

---

## License

MIT — see `package.xml` for details.
