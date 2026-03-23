# Alicia-D Leader ROS2 Driver

> **⚠️ DEPRECATION NOTICE:** The original ROS1 version of this repository ([Alicia-D-Leader-ROS](https://github.com/Synria-Robotics/Alicia-D-Leader-ROS)) is **deprecated and should no longer be used**. It relied on an outdated serial protocol (wrong baud rate, incorrect frame format, passive streaming instead of request-response) and is incompatible with current Alicia-D hardware. This repository is the actively maintained ROS2 Humble replacement.

## Overview

ROS2 Humble driver for the **Alicia-D Leader** — a 6-DOF robot arm controller (teleoperation device) with a gripper. A human operates the physical device and the system reads joint angles in real time over USB serial, publishing them as ROS2 topics.

The driver uses a single Python node (`alicia_driver_node.py`) that handles serial communication, frame parsing, and state publishing. The protocol matches the official [Alicia-D-SDK](https://github.com/Synria-Robotics/Alicia-D-SDK).

## Requirements

- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **Python**: Python 3.10+
- **pyserial**: `sudo apt install python3-serial` (or `pip install pyserial`)

## Installation

### 1. Create a workspace and clone

```bash
mkdir -p ~/alicia_leader_ws/src
cd ~/alicia_leader_ws/src
git clone https://github.com/Synria-Robotics/Alicia-D-Leader-ROS.git .
```

### 2. Build

```bash
cd ~/alicia_leader_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alicia_duo_leader_driver
```

### 3. Serial port permissions

```bash
sudo usermod -a -G dialout $USER
```
**Log out and log back in for this to take effect.**

## Usage

### Launch the driver

```bash
source ~/alicia_leader_ws/install/setup.bash
ros2 launch alicia_duo_leader_driver serial_server.launch.py
```

The driver auto-detects the serial port. You can also specify parameters:

```bash
# Specify port
ros2 launch alicia_duo_leader_driver serial_server.launch.py port:=ttyACM0

# Enable debug logging
ros2 launch alicia_duo_leader_driver serial_server.launch.py debug_mode:=true

# All parameters
ros2 launch alicia_duo_leader_driver serial_server.launch.py port:=ttyACM0 baudrate:=1000000 query_rate:=200.0 debug_mode:=false
```

### Read joint states

Once the driver is running, subscribe to the main topic:

```bash
ros2 topic echo /arm_joint_state
```

Or run the demo script:

```bash
ros2 run alicia_duo_leader_driver arm_read_demo.py
```

### Zero calibration

```bash
ros2 topic pub --once /zero_calibrate std_msgs/msg/Bool "data: true"
```

### Torque enable/disable

```bash
# Enable torque
ros2 topic pub --once /torque_enable std_msgs/msg/Bool "data: true"

# Disable torque
ros2 topic pub --once /torque_enable std_msgs/msg/Bool "data: false"
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_state` | `alicia_duo_leader_driver/msg/ArmJointState` | **Main output.** 6 joint angles (radians), gripper value (0–1000), run status. Published at ~200 Hz. |
| `/servo_states_main` | `std_msgs/msg/Float32MultiArray` | Joint angles + gripper as a flat array `[j1, j2, j3, j4, j5, j6, gripper]`. |
| `/read_serial_data` | `std_msgs/msg/UInt8MultiArray` | Raw serial frames for debugging. |
| `/zero_calibrate` | `std_msgs/msg/Bool` | Send `True` to set current position as zero. |
| `/torque_enable` | `std_msgs/msg/Bool` | Send `True`/`False` to enable/disable torque. |

## Message: ArmJointState

```
std_msgs/Header header
float32 joint1    # radians, range [-π, π]
float32 joint2
float32 joint3
float32 joint4
float32 joint5
float32 joint6
int32   but1      # run status (0=idle, 1=locked, 0x10=sync, etc.)
int32   but2      # reserved
float32 gripper   # raw value 0–1000
float32 time      # reserved
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `""` (auto-detect) | Serial device name (e.g., `ttyACM0`). Auto-detects `ttyACM*`/`ttyUSB*` if empty. |
| `baudrate` | `1000000` | Serial baud rate. Must match hardware. |
| `query_rate` | `200.0` | Joint state query rate in Hz. |
| `debug_mode` | `false` | Enable verbose serial frame logging. |

## Serial Protocol

The driver uses a request-response protocol matching the Alicia-D-SDK:

- **Frame format**: `[0xAA, CMD, FUNC, LEN, DATA..., CRC8, 0xFF]`
- **Checksum**: CRC-32 of `[CMD, FUNC, LEN, DATA]`, lower 8 bits
- **Baud rate**: 1000000
- The device does **not** auto-stream. The driver sends a query command (`CMD=0x06, FUNC=0x00`) at the configured rate and parses the response.
- Joint angles are 16-bit little-endian values (0–4095), mapped to radians: `θ = (value / 4096) × 2π − π`

## Troubleshooting

- **No serial port found**: Check that the device is connected (`ls /dev/ttyACM* /dev/ttyUSB*`) and you have permissions (`groups $USER` should include `dialout`).
- **No data on topics**: Ensure the device is powered on. The driver will log "Connected to /dev/ttyACM0" when the port opens successfully.
- **Wrong baud rate**: The default is 1000000 (1 Mbps), matching the SDK. Older devices may use 921600.
- **Debug mode**: Launch with `debug_mode:=true` to see raw TX/RX frames in the console.
