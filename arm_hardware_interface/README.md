# arm_hardware_interface

A `ros2_control` hardware plugin that bridges the ROS 2 controller manager to an **ESP32 microcontroller** over either a **serial (USB)** or **WiFi (TCP)** connection. Implements the `hardware_interface::SystemInterface` lifecycle and handles all I/O for the 3-DOF robotic arm.

The transport is selected at launch time — no recompilation needed, and the serial path is fully preserved as the default.

---

## Table of Contents
- [Overview](#overview)
- [Package Structure](#package-structure)
- [Communication Protocol](#communication-protocol)
- [Transport Modes](#transport-modes)
- [How It Works](#how-it-works)
  - [Lifecycle](#lifecycle)
  - [read() — 100 Hz](#read----100-hz)
  - [write() — 100 Hz](#write----100-hz)
  - [readLine() — persistent buffer](#readline----persistent-buffer)
- [URDF Integration](#urdf-integration)
- [Launching](#launching)
- [Dependencies](#dependencies)
- [Building](#building)
- [Debugging](#debugging)
- [Known Issues and Design Decisions](#known-issues-and-design-decisions)

---

## Overview

`arm_hardware_interface` is a `pluginlib`-registered `SystemInterface` plugin. When loaded by `ros2_control_node`, it:

1. Reads the `transport` param from the URDF (`"serial"` or `"wifi"`) and opens the appropriate connection
2. Every 100 Hz cycle, sends `CMD <base> <shoulder> <elbow>\n` to the ESP
3. Drains all available `STATE` lines from the buffer and commits the freshest one to `position_state_[]`
4. Exports `position_state_` and `velocity_state_` to `joint_state_broadcaster` → `/joint_states`
5. Reads `position_command_[]` written by `arm_controller` and sends it as the next `CMD`

Once the connection is open, `serial_fd_` is a standard POSIX file descriptor regardless of transport — `read()`, `write()`, and `readLine()` are identical for both.

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
- The ESP only begins sending `STATE` after the first valid `CMD` is received — before that, the line is silent. This is expected — the hardware interface will log `"No complete STATE line this cycle"` until the first trajectory executes.

This protocol is **identical** for both serial and WiFi transports.

---

## Transport Modes

| Mode | How it connects | When to use |
|---|---|---|
| `serial` (default) | Opens `/dev/ttyUSBx` as a raw termios fd | Bench work, debugging, reliable low-latency |
| `wifi` | Opens a TCP socket to the ESP's IP:port | Wireless / untethered operation |

### Serial
- Uses POSIX `open()` + termios 8N1 raw mode
- `O_NONBLOCK` on open, `VMIN=0 VTIME=0` — non-blocking reads
- 115200 baud by default

### WiFi (TCP)
- Blocking `connect()` to ESP IP, then switches to `O_NONBLOCK`
- `TCP_NODELAY` is set immediately — **this is critical**. Without it, the kernel's Nagle algorithm batches small writes and can delay them up to 200 ms, completely breaking 100 Hz timing
- After `openTcpSocket()` returns, `serial_fd_` is a normal non-blocking fd — `readLine()` and `writeLine()` need zero changes

### Latency note
USB serial at 115200 baud has near-zero latency. WiFi over 2.4 GHz can have 5–20 ms of jitter on a busy network. At 100 Hz (10 ms budget per cycle) this will cause occasional `"No complete STATE line this cycle"` warnings and slightly noisier velocity estimates. Mitigations in order of impact:
1. `TCP_NODELAY` (already set in `openTcpSocket()`)
2. Use a dedicated hotspot or 5 GHz network with no other devices
3. Drop the control rate to 50 Hz in `ros2_controllers.yaml` if jitter is consistently bad

---

## How It Works

### Lifecycle

| Callback | Action |
|---|---|
| `on_init()` | Reads `transport` param; reads `serial_port`+`baud_rate` or `esp_ip`+`esp_port` accordingly; validates 3-joint config |
| `on_configure()` | Calls `openSerialPort()` or `openTcpSocket()` depending on `transport_` |
| `on_activate()` | Seeds `position_command_[]` from `position_state_[]` to prevent jump-on-activate |
| `on_deactivate()` | Closes `serial_fd_` (works for both serial and socket) |

If `on_configure()` fails (port not found, ESP unreachable, wrong IP), the controller manager exits. **Do not launch in real hardware mode without the ESP connected and reachable.**

### `read()` — 100 Hz

Called every 10 ms by the controller manager. The critical design decision here is **draining the full kernel buffer every cycle**:

```
while (readLine() returns a complete line):
    parse it
    if valid STATE: overwrite p0, p1, p2 (keep only freshest)
commit freshest p0, p1, p2 to position_state_[]
estimate velocity_state_[] = Δposition / Δtime
```

**Why drain all lines?** The ESP sends STATE at 100 Hz independently on its own `millis()` timer. The Linux scheduler does not guarantee that `read()` fires in perfect 10 ms lockstep — it can be delayed by a few milliseconds. When it is, the buffer accumulates 2–4 complete STATE lines. Processing only the oldest one causes state lag and eventually buffer overflow — which drops bytes mid-line and produces malformed STATE strings, leaving `position_state_[]` stuck. Draining all lines and keeping only the freshest eliminates this entirely.

This applies equally to serial and WiFi — the TCP receive buffer behaves the same way.

### `write()` — 100 Hz

Reads `position_command_[0..2]` (set by `arm_controller`) and sends:
```
CMD <t1> <t2> <t3>\n
```
Returns `ERROR` if the `write()` syscall fails (e.g. ESP disconnected mid-session).

### `readLine()` — persistent buffer

`readLine()` uses a `serial_buffer_` member (persistent across calls) rather than a fresh local string each cycle. This correctly handles the case where a STATE line is split across two 10 ms cycles — which happens when the scheduler fires `read()` while the ESP is mid-transmission. The partial line accumulates in `serial_buffer_` and completes on the next cycle.

This works identically for both serial fd and TCP socket fd.

---

## URDF Integration

The `<ros2_control>` block now has four params — `transport` selects the mode, and only the matching pair of params is used:

```xml
<ros2_control name="ArmSystem" type="system">
  <hardware>
    <plugin>arm_hardware_interface/ArmHardwareInterface</plugin>

    <!-- "serial" or "wifi" — overridden at launch time by real_arm.launch.py -->
    <param name="transport">serial</param>

    <!-- Serial params (used when transport=serial) -->
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>

    <!-- WiFi params (used when transport=wifi) -->
    <param name="esp_ip">192.168.1.105</param>
    <param name="esp_port">8888</param>
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

The URDF values are static defaults. `real_arm.launch.py` patches them at launch time, so the transport and IP you pass on the command line always win over what's written in the file.

---

## Launching

```bash
# Serial (default) — uses /dev/ttyUSB0
ros2 launch arm_moveit_conf_pkg real_arm.launch.py

# Serial on a different port
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=serial serial_port:=/dev/ttyUSB1

# WiFi
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=192.168.1.105

# WiFi on a non-default port
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=192.168.1.105 esp_port:=9999
```

To find the ESP's IP, either check your router's DHCP table or temporarily add `Serial.println(WiFi.localIP())` to the ESP's `setup()`. Assigning a static DHCP lease by MAC address in your router is recommended so the IP never changes between sessions.

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

Verify the plugin is discoverable after building:
```bash
ros2 run pluginlib_tutorials list_plugins arm_hardware_interface
```

---

## Debugging

### Serial transport

**Check port permissions:**
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

### WiFi transport

**Verify the ESP is reachable before launching:**
```bash
ping 192.168.1.105
```

**Check the TCP port is open on the ESP:**
```bash
nc -zv 192.168.1.105 8888
```

**Watch the raw STATE stream over TCP:**
```bash
nc 192.168.1.105 8888
# then type: CMD 0.0 0.0 0.0   (ESP won't send STATE until first CMD)
```

### Both transports

**Verify the plugin loads:**
```bash
ros2 control list_hardware_interfaces
```

**Watch `/joint_states` live:**
```bash
ros2 topic echo /joint_states
```

If positions are always 0.0, the ESP has not received a CMD yet (correct before first trajectory) or STATE lines are not parsing correctly — check for malformed warnings in the `ros2_control_node` output.

---

## Known Issues and Design Decisions

**No real encoders.** The ESP echoes commanded values as state. This means:
- The arm always assumes home position (0, 0, 0) on boot
- `position_state_[]` reflects what was commanded, not what the arm physically did
- Velocity is estimated from the delta between successive commanded positions, not from actual motion

**STATE silence before first CMD.** The ESP does not stream STATE until it receives a valid CMD. This is intentional (prevents zero-flood on startup) but means `"No complete STATE line this cycle"` appears continuously until the first trajectory executes. This is not an error.

**WiFi latency at 100 Hz.** See the [Latency note](#latency-note) in the Transport Modes section. The warning threshold is the same for both transports — WiFi will simply trigger it more often on a noisy network.

**Single active connection.** The WiFi path opens one TCP connection in `on_configure()` and holds it for the session. If the ESP reboots mid-session, the socket will error on the next `write()`, `on_deactivate()` will fire, and you will need to re-launch. Automatic reconnection is not currently implemented.

---

## License

MIT — see `package.xml` for details.