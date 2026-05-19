# arm_esp_controller

ESP8266 firmware for the 3-DOF robotic arm controller. Receives joint position commands from ROS 2 over serial and streams joint state back. Built with **PlatformIO** targeting the ESP8266 (NodeMCU v2 / ESP-12E).

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Communication Protocol](#communication-protocol)
- [Firmware Logic](#firmware-logic)
- [Joint Limits](#joint-limits)
- [Setup and Flashing](#setup-and-flashing)
- [Monitoring](#monitoring)
- [Design Decisions](#design-decisions)
- [Future Work](#future-work)

---

## Overview

The ESP8266 acts as a simple command-and-echo bridge between the ROS 2 hardware interface and the physical servos (not yet connected — see [Future Work](#future-work)). It:

1. Listens for `CMD <base> <shoulder> <elbow>\n` lines over serial at 115200 baud
2. Parses and clamps each joint angle to its hardware limits
3. Streams `STATE <base> <shoulder> <elbow>\n` back to ROS at 100 Hz — but **only after receiving the first valid CMD**

This last point is important: the ESP is silent on startup. This prevents the ROS serial buffer from filling with zeros before any trajectory executes, which would delay state feedback.

---

## Project Structure

```
arm_esp_controller/
├── src/
│   └── main.cpp          # Full firmware — setup(), loop(), protocol parsing
├── include/              # (empty — no custom headers needed)
├── lib/                  # (empty — no external libraries beyond Arduino framework)
├── test/                 # PlatformIO test scaffold (unused)
├── platformio.ini        # Build configuration — board, framework, port, baud
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
- The ESP does not respond to unknown or malformed lines — it silently discards them and continues

---

## Firmware Logic

### `setup()`

```cpp
Serial.begin(115200);
delay(500);              // wait for USB-UART chip to stabilise
pinMode(LED_PIN, OUTPUT);
```

### `loop()`

Two independent tasks run every iteration:

**Task 1 — Receive CMD (event-driven, no blocking)**

```cpp
while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
        // parse complete line from inputBuffer
        if (starts with "CMD" and sscanf parses 3 floats):
            pos[0] = clamp(a0, base_joint_lim)
            pos[1] = clamp(a1, shoulder_joint_lim)
            pos[2] = clamp(a2, elbow_joint_lim)
            got_cmd = true
        inputBuffer = ""
    } else {
        inputBuffer += c
    }
}
```

`inputBuffer` is a global `String` that accumulates characters across `loop()` iterations. This is important: if the ROS host sends the CMD line faster than the ESP reads it, characters are buffered correctly without loss.

**Task 2 — Send STATE (millis-based, non-blocking)**

```cpp
if (got_cmd && (millis() - lastSendTime > 10)) {
    snprintf(buf, "STATE %.4f %.4f %.4f\n", pos[0], pos[1], pos[2]);
    Serial.print(buf);
    lastSendTime = millis();
}
```

The 10 ms interval produces 100 Hz STATE output, matching the `ros2_control` update rate. Using `millis()` instead of `delay(10)` keeps the receive loop fully responsive — a `delay()` here would cause the ESP to miss incoming CMD bytes during the wait, leading to mid-line corruption on the ROS side.

The `got_cmd` gate ensures STATE is never sent before the first CMD arrives.

---

## Joint Limits

Limits are enforced on the ESP side as a hardware safety layer, independent of MoveIt's software limits:

| Joint | Lower (rad) | Upper (rad) | Degrees |
|---|---|---|---|
| `base_joint` | −3.14159 | +3.14159 | ±180° |
| `shoulder_joint` | −1.57 | +1.57 | ±90° |
| `elbow_joint` | −2.094 | +2.094 | ±120° |

The `clamp()` function silently saturates out-of-range values. No error is sent back to ROS — the clamped value is used instead.

These match the limits in `robotic_arm_control/urdf/URDF.urdf` and `arm_moveit_conf_pkg/config/joint_limits.yaml`. If you change limits in one place, update all three.

---

## Setup and Flashing

### Requirements

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- ESP8266 (NodeMCU v2, Wemos D1 Mini, or ESP-12E module)
- USB-UART cable connected to `/dev/ttyUSB0` (Linux) or `COM*` (Windows)

### Flash

```bash
cd arm_esp_controller
pio run --target upload
```

PlatformIO auto-detects `/dev/ttyUSB*`. If you have multiple serial devices, specify explicitly in `platformio.ini`:

```ini
upload_port = /dev/ttyUSB0
```

### Board variants

The default config targets `nodemcuv2`. For other boards, change `platformio.ini`:

```ini
; NodeMCU v2 (ESP-12E) — default
board = nodemcuv2

; Wemos D1 Mini
board = d1_mini

; Bare ESP-12E module
board = esp12e
```

---

## Monitoring

**Never open the Arduino/PlatformIO serial monitor while ROS 2 is connected.** Opening any serial monitor asserts the DTR line, which resets the ESP and corrupts the ROS session — all queued STATE lines are lost and the ROS hardware interface will log errors.

To observe the raw STATE stream without interfering:

```bash
stty -F /dev/ttyUSB0 raw 115200 && cat /dev/ttyUSB0
```

This reads the port in raw mode with no DTR assertion. You should see:

```
STATE 1.5708 0.5000 -0.3000
STATE 1.5708 0.5000 -0.3000
...
```

To send a manual CMD for testing (without ROS):

```bash
echo "CMD 0.5 0.3 -0.4" > /dev/ttyUSB0
```

---

## Design Decisions

**Why `got_cmd` gate instead of streaming from boot?**
Streaming `STATE 0.0000 0.0000 0.0000` from boot fills the ROS kernel serial buffer with zeros before any trajectory executes. When `read()` eventually drains this backlog, it consumes stale data instead of fresh position feedback. The gate eliminates this cleanly.

**Why `millis()` instead of `delay(10)`?**
`delay(10)` blocks the entire `loop()`, which means incoming CMD bytes sit in the UART hardware buffer during the delay. At 115200 baud the ROS host can send a 30-byte CMD line in ~2.6 ms — well within the 10 ms window. But if the scheduler on the ROS side delays a write slightly, the CMD can straddle two `loop()` iterations. With `delay()`, the second half of the CMD arrives while the ESP is blocked, gets buffered, and then gets processed in a later iteration with a stale first half still in `inputBuffer`. Using `millis()` keeps the loop spinning continuously so this never happens.

**Why character-by-character reading instead of `Serial.readStringUntil('\n')`?**
`readStringUntil()` is blocking — it waits until a `\n` arrives or a timeout expires. This stalls the send side. Character-by-character reading with a persistent `inputBuffer` is non-blocking and allows both tasks to interleave cleanly.

---

## Future Work

- **Real servo output** — map `pos[]` to PWM signals for physical servo motors; validate angle-to-pulse scaling; add per-joint power enable/disable
- **WiFi TCP mode** — replace `Serial` with a `WiFiServer`/`WiFiClient` pair; expose port 8888 for the ROS hardware interface TCP adaptation; see workspace README for full implementation plan
- **Encoder feedback** — replace echo-back state with actual encoder readings so `position_state_[]` reflects true arm position rather than commanded position

---

## License

MIT
