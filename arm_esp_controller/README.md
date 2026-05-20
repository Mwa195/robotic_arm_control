# arm_esp_controller

ESP32 firmware for the 3-DOF robotic arm controller. Receives joint position commands from ROS 2 and streams joint state back. Supports two transports — **serial (USB)** and **WiFi (TCP)** — selectable at compile time via a single `#define`. Built with **PlatformIO** targeting the ESP32.

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Communication Protocol](#communication-protocol)
- [Transport Modes](#transport-modes)
- [Firmware Logic](#firmware-logic)
- [Servo Output](#servo-output)
- [Joint Limits](#joint-limits)
- [Setup and Flashing](#setup-and-flashing)
- [Monitoring](#monitoring)
- [Design Decisions](#design-decisions)

---

## Overview

The ESP32 receives joint position commands from ROS 2, clamps them to hardware limits, drives the physical servos, and streams the current position back at 100 Hz. It:

1. Listens for `CMD <base> <shoulder> <elbow>\n` lines over serial or WiFi TCP
2. Parses and clamps each joint angle to its hardware limits
3. Writes the clamped angles to the servo motors via PWM
4. Streams `STATE <base> <shoulder> <elbow>\n` back at 100 Hz — but **only after receiving the first valid CMD**

The startup silence is intentional: it prevents the ROS serial/TCP buffer from filling with zeros before any trajectory executes.

---

## Project Structure

```
arm_esp_controller/
├── src/
│   └── main.cpp          # Full firmware — setup(), loop(), protocol, servo output
├── include/              # (empty — no custom headers needed)
├── lib/                  # Managed by PlatformIO (ESP32Servo fetched automatically)
├── test/                 # PlatformIO test scaffold (unused)
├── platformio.ini        # Build config — ESP32, framework, port, library deps
└── .pio/                 # PlatformIO build artefacts (do not edit)
```

---

## Communication Protocol

```
Host → ESP:   CMD <t1> <t2> <t3>\n
              e.g.  "CMD 1.570796 0.500000 -0.300000\n"

ESP → Host:   STATE <t1> <t2> <t3>\n
              e.g.  "STATE 1.5708 0.5000 -0.3000\n"
```

- All values are joint angles in **radians**
- Lines are `\n`-terminated only (no `\r`)
- `CMD` values are parsed with `sscanf(..., "CMD %f %f %f", ...)` — extra whitespace is tolerated
- `STATE` values use 4 decimal places (`%.4f`)
- Unknown or malformed lines are silently discarded

This protocol is **identical** regardless of which transport is active.

---

## Transport Modes

Transport is selected at **compile time** with a single define at the top of `main.cpp`:

```cpp
#define USE_WIFI 1   // 1 = WiFi TCP,  0 = Serial USB
```

Reflash after changing this value.

### Serial (USE_WIFI 0)

- Communicates over USB-UART at 115200 baud
- `transportAvailable()`, `transportRead()`, `transportPrint()` call `Serial.*` directly
- Default for bench work and debugging

### WiFi TCP (USE_WIFI 1)

- Connects to the configured SSID on boot, starts a `WiFiServer` on port 8888
- Accepts one client connection at a time; reconnects automatically if the client drops
- Credentials and port are set at the top of `main.cpp`:

```cpp
const char* WIFI_SSID     = "your_ssid";
const char* WIFI_PASSWORD = "your_password";
const int   TCP_PORT      = 8888;
```

- The onboard LED blinks while connecting and stays solid once WiFi is up
- The assigned IP is printed to Serial on boot — use this to set `esp_ip` in the ROS launch command:

```bash
ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=<IP here>
```

Assigning a static DHCP lease by MAC address in your router is recommended so the IP never changes between sessions.

### Transport abstraction

All protocol logic (`inputBuffer` parsing, STATE sending) is written against three thin wrappers:

```cpp
int  transportAvailable()       // returns bytes available on whichever transport is active
char transportRead()            // reads one byte
void transportPrint(const char* buf)  // sends a string
```

Switching transport requires changing only the `#define` — the protocol and servo logic are untouched.

---

## Firmware Logic

### `setup()`

```cpp
Serial.begin(115200);
delay(500);
pinMode(LED_PIN, OUTPUT);

base_servo.attach(SERVO0_PIN, PULSE_MIN, PULSE_MAX);
shoulder_servo.attach(SERVO1_PIN, PULSE_MIN, PULSE_MAX);
elbow_servo.attach(SERVO2_PIN, PULSE_MIN, PULSE_MAX);

#if USE_WIFI
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // blink LED until connected, then print IP and start TCP server
#endif
```

### `loop()`

Two independent tasks run every iteration:

**Task 1 — Receive CMD (event-driven, non-blocking)**

```cpp
while (transportAvailable()) {
    char c = transportRead();
    if (c == '\n') {
        // parse complete line from inputBuffer
        if (starts with "CMD" and sscanf parses 3 floats):
            pos[0] = clamp(a0, base_joint_lim)
            pos[1] = clamp(a1, shoulder_joint_lim)
            pos[2] = clamp(a2, elbow_joint_lim)
            writeServo(0, pos[0])
            writeServo(1, pos[1])
            writeServo(2, pos[2])
            got_cmd = true
        inputBuffer = ""
    } else {
        inputBuffer += c
    }
}
```

`inputBuffer` is a global `String` that accumulates characters across `loop()` iterations — partial lines are held correctly if the host sends faster than the ESP reads.

**Task 2 — Send STATE (millis-based, non-blocking)**

```cpp
if (got_cmd && (millis() - lastSendTime > 10)) {
    snprintf(buf, "STATE %.4f %.4f %.4f\n", pos[0], pos[1], pos[2]);
    transportPrint(buf);
    lastSendTime = millis();
}
```

The 10 ms interval produces 100 Hz STATE output, matching the `ros2_control` update rate. The `got_cmd` gate ensures STATE is never sent before the first CMD arrives.

---

## Servo Output

Servos are **ASME-05B** units (380 kg·cm, 0–300° range, ±150° from home).

| Servo | Pin | Joint |
|---|---|---|
| `base_servo` | GPIO 18 | `base_joint` |
| `shoulder_servo` | GPIO 19 | `shoulder_joint` |
| `elbow_servo` | GPIO 21 | `elbow_joint` |

The home position is **150°** (mid-range), which maps to **0 rad** in the ROS frame. The `writeServo()` function converts radians to a microsecond PWM pulse:

```cpp
void writeServo(int id, float angle_rad) {
    float angle_deg = angle_rad * (180.0f / PI);
    int pulse = map(angle_deg, -150, 150, PULSE_MIN, PULSE_MAX);
    servo.writeMicroseconds(pulse);
}
```

| Parameter | Value |
|---|---|
| `PULSE_MIN` | 500 µs → 0° (−150° from home) |
| `PULSE_MAX` | 2500 µs → 300° (+150° from home) |

`writeMicroseconds()` is used instead of `write()` because `write()` is limited to 180° — these servos have a 300° range and need the full pulse width range to reach their limits.

The library used is **ESP32Servo** (`madhephaestus/ESP32Servo`) — the standard Arduino `Servo.h` does not support the ESP32's PWM hardware.

---

## Joint Limits

Limits are enforced on the ESP side as a hardware safety layer, independent of MoveIt's software limits:

| Joint | Lower (rad) | Upper (rad) | Degrees |
|---|---|---|---|
| `base_joint` | −2.616 | +2.616 | ±150° |
| `shoulder_joint` | −1.57 | +1.57 | ±90° |
| `elbow_joint` | −2.094 | +2.094 | ±120° |

The `clamp()` function silently saturates out-of-range values — no error is sent back to ROS.

> **Note:** `base_joint` limits here are ±150° (±2.616 rad) — wider than the ±180° in the URDF and MoveIt config. The ESP-side limit is the binding hardware constraint. If you change limits in one place, update all three: `URDF.urdf`, `joint_limits.yaml`, and `main.cpp`.

---

## Setup and Flashing

### Requirements

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- ESP32 development board (esp32dev / ESP-WROOM-32)
- USB cable connected to `/dev/ttyUSB0` (Linux) or `COM*` (Windows)

### Configure transport before flashing

Open `src/main.cpp` and set:

```cpp
#define USE_WIFI 0   // serial
// or
#define USE_WIFI 1   // WiFi — also set WIFI_SSID and WIFI_PASSWORD below
```

### Flash

```bash
cd arm_esp_controller
pio run --target upload
```

PlatformIO auto-detects `/dev/ttyUSB*`. To specify explicitly, edit `platformio.ini`:

```ini
upload_port = /dev/ttyUSB0
```

### `platformio.ini` summary

```ini
[env:esp12e]
platform  = espressif32
board     = esp32dev
framework = arduino
monitor_speed = 115200
upload_port   = /dev/ttyUSB*
lib_deps      = madhephaestus/ESP32Servo
```

---

## Monitoring

**Never open the PlatformIO serial monitor while ROS 2 is connected.** It asserts DTR, resets the ESP, and corrupts the ROS session.

To observe the raw STATE stream without interfering:

```bash
stty -F /dev/ttyUSB0 raw 115200 && cat /dev/ttyUSB0
```

To send a manual CMD for testing without ROS (serial mode):

```bash
echo "CMD 0.5 0.3 -0.4" > /dev/ttyUSB0
```

To test WiFi mode without ROS:

```bash
# Check the TCP port is open
nc -zv 192.168.x.x 8888

# Connect and send a manual CMD
nc 192.168.x.x 8888
CMD 0.5 0.3 -0.4
```

STATE lines will not appear until at least one valid CMD is sent.

---

## Design Decisions

**Why `got_cmd` gate instead of streaming from boot?**
Streaming `STATE 0.0000 0.0000 0.0000` from boot fills the ROS kernel buffer with zeros before any trajectory executes. When `read()` eventually drains this backlog, it consumes stale data instead of fresh position feedback. The gate eliminates this cleanly.

**Why `millis()` instead of `delay(10)`?**
`delay(10)` blocks the entire `loop()`. Incoming CMD bytes accumulate in the UART/TCP buffer during the delay, and if a CMD straddles the boundary the second half is processed a full cycle late with a stale first half still in `inputBuffer`. `millis()` keeps the loop spinning continuously so this never happens.

**Why character-by-character reading instead of `readStringUntil('\n')`?**
`readStringUntil()` is blocking — it waits for `\n` or a timeout, stalling the send side. Character-by-character reading with a persistent `inputBuffer` is non-blocking and lets both tasks interleave cleanly.

**Why compile-time transport selection instead of a runtime parameter?**
A runtime switch would require persistent storage (EEPROM/NVS) or a configuration protocol before the main protocol starts — added complexity with no real benefit. The transport is a physical wiring decision: either a USB cable is plugged in or it isn't. A recompile takes seconds and makes the active transport unambiguous.

**Why `writeMicroseconds()` instead of `write()`?**
The ASME-05B servos have a 300° range. Arduino's `write()` is capped at 180° and would only use part of the range. `writeMicroseconds()` with explicit `PULSE_MIN`/`PULSE_MAX` bounds gives full range and precise control.

---

## License

MIT