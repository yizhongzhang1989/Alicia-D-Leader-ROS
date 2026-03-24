#!/usr/bin/env python3
# coding=utf-8

"""
Alicia-D Leader ROS2 Driver Node

Single node that handles serial communication with the Alicia-D leader arm,
periodically queries joint states, and publishes them as ROS2 topics.
Protocol matches the Alicia-D-SDK.

Frame format: [0xAA, CMD, FUNC, LEN, DATA..., CRC8, 0xFF]
  - CRC8 = CRC-32(CMD+FUNC+LEN+DATA) & 0xFF
  - Total frame length = LEN + 6
"""

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
import binascii
import math
import shutil
import time
import threading
import platform
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, Float32MultiArray, UInt8MultiArray
from alicia_duo_leader_driver.msg import ArmJointState

# Protocol constants
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0xFF
DEFAULT_OVERHEAD = 6  # header + cmd + func + len + crc + footer

# Pre-built query/control commands (from SDK INFO_COMMAND_MAP)
CMD_QUERY_JOINT = [0xAA, 0x06, 0x00, 0x01, 0xFE, 0x9A, 0xFF]
CMD_ZERO_CALI = [0xAA, 0x03, 0x00, 0x01, 0xFE, 0xA8, 0xFF]
CMD_TORQUE_ON = [0xAA, 0x05, 0x00, 0x01, 0x01, 0xF9, 0xFF]
CMD_TORQUE_OFF = [0xAA, 0x05, 0x00, 0x01, 0x00, 0x6F, 0xFF]
CMD_VERSION = [0xAA, 0x01, 0x00, 0x01, 0xFE, 0x23, 0xFF]


def crc32_checksum(payload):
    """Calculate CRC-32 of payload and return lower 8 bits."""
    crc = binascii.crc32(bytes(payload)) & 0xFFFFFFFF
    return crc & 0xFF


def bytes_to_radians(low_byte, high_byte):
    """Convert 2-byte little-endian servo value to radians. Range: [-pi, pi]."""
    value = (low_byte & 0xFF) | ((high_byte & 0xFF) << 8)
    value = max(0, min(4095, value))
    return (value / 4096.0) * (2 * math.pi) - math.pi


class AliciaDriverNode(Node):
    def __init__(self):
        super().__init__('alicia_driver_node')

        # Parameters
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('query_rate', 200.0)  # Hz
        self.declare_parameter('joint_config', '')

        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.query_rate = self.get_parameter('query_rate').value

        self.get_logger().info(f'Port: {self.port_name if self.port_name else "auto-detect"}')
        self.get_logger().info(f'Baudrate: {self.baudrate}')
        self.get_logger().info(f'Query rate: {self.query_rate} Hz')

        # Load joint configuration (direction, zero_offset, continuous)
        self._load_joint_config()

        # Angle unwrapping state for continuous joints
        self._prev_raw = [None] * 6       # previous raw angle per joint
        self._unwrap_turns = [0] * 6       # accumulated full turns

        # Publishers
        self.joint_state_pub = self.create_publisher(ArmJointState, '/arm_joint_state', 10)
        self.joint_state_raw_pub = self.create_publisher(ArmJointState, '/arm_joint_state_raw', 10)
        self.array_pub = self.create_publisher(Float32MultiArray, '/servo_states_main', 10)
        self.raw_pub = self.create_publisher(UInt8MultiArray, '/read_serial_data', 10)

        # Subscribers
        self.create_subscription(Bool, '/zero_calibrate', self.zero_calib_callback, 10)
        self.create_subscription(Bool, '/torque_enable', self.torque_callback, 10)

        # Serial state
        self.serial_port = None
        self._rx_buffer = bytearray()
        self._lock = threading.Lock()
        self._running = True

        # Connect
        if not self._connect():
            # Retry in background
            self._reconnect_timer = self.create_timer(2.0, self._reconnect_cb)
        else:
            self._reconnect_timer = None
            self._start_comm_thread()

        self.get_logger().info('Alicia driver node initialized')

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_comm_thread') and self._comm_thread.is_alive():
            self._comm_thread.join(timeout=2.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

    # ── Joint Configuration ──

    def _load_joint_config(self):
        """Load joint direction, zero offset, and continuity settings from YAML."""
        # Defaults: no flip, no offset, no unwrapping
        self._joint_direction = [1.0] * 6
        self._joint_zero_offset = [0.0] * 6
        self._joint_continuous = [False] * 6
        self._gripper_direction = 1.0
        self._gripper_zero_offset = 0.0

        config_path = self.get_parameter('joint_config').value
        if not config_path:
            try:
                pkg_share = get_package_share_directory('alicia_duo_leader_driver')
                template_cfg = os.path.join(pkg_share, 'config', 'joint_config_template.yaml')

                # Find workspace root by walking up from pkg_share looking for
                # a directory that contains config/joint_config_template.yaml
                ws_root = None
                d = os.path.dirname(pkg_share)
                for _ in range(6):
                    d = os.path.dirname(d)
                    if os.path.isfile(os.path.join(d, 'config', 'joint_config_template.yaml')):
                        ws_root = d
                        break

                if ws_root:
                    user_cfg = os.path.join(ws_root, 'config', 'joint_config.yaml')
                    ws_template = os.path.join(ws_root, 'config', 'joint_config_template.yaml')
                else:
                    # Fallback: place next to installed template
                    user_cfg = os.path.join(os.path.dirname(template_cfg), 'joint_config.yaml')
                    ws_template = template_cfg

                if os.path.isfile(user_cfg):
                    config_path = user_cfg
                else:
                    # Auto-copy template to create joint_config.yaml
                    src = ws_template if os.path.isfile(ws_template) else template_cfg
                    if os.path.isfile(src):
                        os.makedirs(os.path.dirname(user_cfg), exist_ok=True)
                        shutil.copy2(src, user_cfg)
                        self.get_logger().info(f'Created {user_cfg} from template — edit this file to configure joints')
                        config_path = user_cfg
                    else:
                        config_path = template_cfg
            except Exception:
                self.get_logger().warn('Could not find default joint config, using identity mapping')
                return

        if not os.path.isfile(config_path):
            self.get_logger().warn(f'Joint config not found: {config_path}, using identity mapping')
            return

        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)

            jc = cfg.get('joint_config', {})
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            for i, name in enumerate(joint_names):
                if name in jc:
                    j = jc[name]
                    self._joint_direction[i] = float(j.get('direction', 1.0))
                    self._joint_zero_offset[i] = float(j.get('zero_offset', 0.0))
                    self._joint_continuous[i] = bool(j.get('continuous', False))

            if 'gripper' in jc:
                g = jc['gripper']
                self._gripper_direction = float(g.get('direction', 1.0))
                self._gripper_zero_offset = float(g.get('zero_offset', 0.0))

            self.get_logger().info(f'Joint config loaded from {config_path}')
            self.get_logger().info(f'  directions: {self._joint_direction}')
            self.get_logger().info(f'  zero_offsets: {self._joint_zero_offset}')
            self.get_logger().info(f'  continuous: {self._joint_continuous}')
        except Exception as e:
            self.get_logger().error(f'Failed to load joint config: {e}')

    def _apply_joint_transform(self, raw_radians):
        """
        Apply direction, zero offset, and angle unwrapping to raw joint angles.
        Returns a new list of transformed angles.
        """
        result = [0.0] * 6
        for i in range(6):
            raw = raw_radians[i]

            # Angle unwrapping for continuous joints
            if self._joint_continuous[i]:
                if self._prev_raw[i] is not None:
                    delta = raw - self._prev_raw[i]
                    # Detect wrap: if jump > π, it wrapped backwards; if < -π, forwards
                    if delta > math.pi:
                        self._unwrap_turns[i] -= 1
                    elif delta < -math.pi:
                        self._unwrap_turns[i] += 1
                self._prev_raw[i] = raw
                raw = raw + self._unwrap_turns[i] * 2.0 * math.pi

            # Apply direction and zero offset
            result[i] = self._joint_direction[i] * raw + self._joint_zero_offset[i]

        return result

    # ── Serial Connection ──

    def _find_serial_port(self):
        """Find available serial port, prioritizing ttyACM and ttyUSB."""
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return ''

        # If user specified a port
        if self.port_name:
            name = self.port_name
            if not name.startswith('/dev/'):
                name = '/dev/' + name
            for p in ports:
                if p.device == name:
                    if os.access(p.device, os.R_OK | os.W_OK):
                        return p.device
            self.get_logger().warn(f'Specified port {name} not available, searching...')

        # Auto-detect by priority
        priorities = ['ttyACM', 'ttyUSB', 'ttyCH343USB', 'ttyCH341USB']
        for key in priorities:
            for p in ports:
                if key in p.device and os.access(p.device, os.R_OK | os.W_OK):
                    return p.device
        return ''

    def _connect(self):
        """Connect to serial port."""
        port = self._find_serial_port()
        if not port:
            self.get_logger().warn('No serial port found')
            return False

        try:
            self.get_logger().info(f'Connecting to {port} at {self.baudrate}...')
            self.serial_port = serial.Serial(
                port=port, baudrate=self.baudrate, timeout=0.1,
                write_timeout=1.0, xonxoff=False, rtscts=False, dsrdtr=False
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.serial_port.setDTR(True)
            self.serial_port.setRTS(False)
            self.get_logger().info(f'Connected to {port}')
            return True
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return False

    def _reconnect_cb(self):
        """Timer callback for reconnection attempts."""
        if self._connect():
            if self._reconnect_timer:
                self._reconnect_timer.cancel()
                self._reconnect_timer = None
            self._start_comm_thread()

    def _start_comm_thread(self):
        """Start the communication thread for query/response."""
        self._comm_thread = threading.Thread(target=self._comm_loop, daemon=True)
        self._comm_thread.start()

    # ── Communication Loop ──

    def _comm_loop(self):
        """Main loop: send joint query, read responses, parse and publish."""
        interval = 1.0 / self.query_rate
        self.get_logger().info(f'Communication thread started (interval={interval*1000:.1f}ms)')

        while self._running and rclpy.ok():
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(0.5)
                    continue

                # Send joint+gripper query
                self._send_raw(CMD_QUERY_JOINT)

                # Read and process available frames
                frames_read = 0
                max_frames = 10
                while frames_read < max_frames:
                    frame = self._read_frame()
                    if frame is None:
                        break
                    self._process_frame(frame)
                    frames_read += 1

                if frames_read == 0:
                    time.sleep(interval)
                else:
                    # Brief sleep to allow serial buffer to fill
                    time.sleep(max(0.001, interval - 0.002))

            except (OSError, serial.SerialException) as e:
                self.get_logger().error(f'Serial error: {e}')
                if self.serial_port:
                    try:
                        self.serial_port.close()
                    except Exception:
                        pass
                self.serial_port = None
                # Schedule reconnect
                if self._running:
                    self._reconnect_timer = self.create_timer(2.0, self._reconnect_cb)
                break
            except Exception as e:
                self.get_logger().error(f'Comm loop error: {e}')
                time.sleep(0.1)

    # ── Serial Read/Write ──

    def _send_raw(self, data):
        """Send raw byte list to serial port."""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        try:
            self.serial_port.write(bytes(data))
            self.serial_port.flush()
            if self.debug_mode:
                hex_str = ' '.join(f'{b:02X}' for b in data)
                self.get_logger().debug(f'TX: {hex_str}')
            return True
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            return False

    def _read_frame(self):
        """
        Read one complete frame from serial using SDK protocol.
        Frame: [0xAA, CMD, FUNC, LEN, DATA(LEN bytes), CRC8, 0xFF]
        Total length = LEN + 6
        Returns list of ints or None.
        """
        try:
            if not self.serial_port or not self.serial_port.is_open:
                return None

            try:
                available = self.serial_port.in_waiting
            except OSError:
                return None

            if available == 0:
                return None

            read_size = min(available, 80)
            try:
                self._rx_buffer += self.serial_port.read(read_size)
            except OSError:
                return None

            while len(self._rx_buffer) >= DEFAULT_OVERHEAD:
                # Discard buffer if too large (corrupted)
                if len(self._rx_buffer) > 200:
                    self._rx_buffer.clear()
                    continue

                # Sync to 0xAA header
                if self._rx_buffer[0] != FRAME_HEADER:
                    self._rx_buffer.pop(0)
                    continue

                # LEN is at index 3
                if len(self._rx_buffer) < 4:
                    break

                data_len = self._rx_buffer[3]
                frame_length = data_len + DEFAULT_OVERHEAD

                if len(self._rx_buffer) < frame_length:
                    break  # Wait for more data

                candidate = self._rx_buffer[:frame_length]

                # Check footer
                if candidate[-1] != FRAME_FOOTER:
                    self._rx_buffer.pop(0)
                    continue

                # Verify CRC-32 checksum
                payload = candidate[1:-2]  # CMD + FUNC + LEN + DATA
                expected_crc = crc32_checksum(payload)
                received_crc = candidate[-2]

                if expected_crc == received_crc:
                    self._rx_buffer = self._rx_buffer[frame_length:]
                    if self.debug_mode:
                        hex_str = ' '.join(f'{b:02X}' for b in candidate)
                        self.get_logger().debug(f'RX: {hex_str}')
                    return list(candidate)
                else:
                    if self.debug_mode:
                        self.get_logger().warn(
                            f'CRC mismatch: expected 0x{expected_crc:02X}, got 0x{received_crc:02X}')
                    self._rx_buffer.pop(0)

            return None

        except Exception as e:
            self.get_logger().error(f'Read error: {e}')
            return None

    # ── Frame Processing ──

    def _process_frame(self, frame):
        """Process a parsed frame based on CMD and FUNC codes."""
        cmd = frame[1]
        func = frame[2]

        # Publish raw frame for debugging/other nodes
        raw_msg = UInt8MultiArray()
        raw_msg.data = frame
        self.raw_pub.publish(raw_msg)

        if cmd == 0x06 and func == 0x00:
            self._handle_joint_data(frame)
        elif cmd == 0x01:
            self._handle_version_data(frame)
        elif cmd == 0xEE:
            self._handle_error_data(frame)
        elif self.debug_mode:
            self.get_logger().debug(f'Unhandled frame: CMD=0x{cmd:02X} FUNC=0x{func:02X}')

    def _handle_joint_data(self, frame):
        """Parse joint+gripper response and publish ArmJointState."""
        data_len = frame[3]
        data_start = 4
        data_bytes = frame[data_start:data_start + data_len]

        if len(data_bytes) < 15:
            self.get_logger().warn(f'Joint data too short: {len(data_bytes)} bytes')
            return

        # Parse 6 joint angles (2 bytes each, little-endian)
        raw_joint_values = []
        for i in range(6):
            idx = i * 2
            rad = bytes_to_radians(data_bytes[idx], data_bytes[idx + 1])
            raw_joint_values.append(rad)

        # Apply direction, zero offset, and angle unwrapping
        joint_values = self._apply_joint_transform(raw_joint_values)

        # Parse gripper (2 bytes, little-endian) - raw value 0-1000
        gripper_raw_value = (data_bytes[12] & 0xFF) | ((data_bytes[13] & 0xFF) << 8)
        gripper_value = max(0, min(1000, gripper_raw_value))
        gripper_value = self._gripper_direction * gripper_value + self._gripper_zero_offset

        # Parse run status
        run_status = data_bytes[14] if len(data_bytes) > 14 else 0

        now_stamp = self.get_clock().now().to_msg()

        # Publish raw (untransformed) joint state — for URDF visualization
        raw_msg = ArmJointState()
        raw_msg.header.stamp = now_stamp
        raw_msg.joint1 = raw_joint_values[0]
        raw_msg.joint2 = raw_joint_values[1]
        raw_msg.joint3 = raw_joint_values[2]
        raw_msg.joint4 = raw_joint_values[3]
        raw_msg.joint5 = raw_joint_values[4]
        raw_msg.joint6 = raw_joint_values[5]
        raw_msg.gripper = float(max(0, min(1000, gripper_raw_value)))
        raw_msg.but1 = run_status
        raw_msg.but2 = 0
        self.joint_state_raw_pub.publish(raw_msg)

        # Publish transformed joint state — for robot control
        msg = ArmJointState()
        msg.header.stamp = now_stamp
        msg.joint1 = joint_values[0]
        msg.joint2 = joint_values[1]
        msg.joint3 = joint_values[2]
        msg.joint4 = joint_values[3]
        msg.joint5 = joint_values[4]
        msg.joint6 = joint_values[5]
        msg.gripper = float(gripper_value)
        msg.but1 = run_status
        msg.but2 = 0
        self.joint_state_pub.publish(msg)

        # Publish backward-compatible array
        compat_msg = Float32MultiArray()
        compat_msg.data = joint_values + [float(gripper_value)]
        self.array_pub.publish(compat_msg)

    def _handle_version_data(self, frame):
        """Parse and log firmware version."""
        data_len = frame[3]
        data_bytes = frame[4:4 + data_len]
        if len(data_bytes) >= 24:
            serial_num = ''.join(chr(b) for b in data_bytes[0:16]).strip()
            self.get_logger().info(f'Device serial: {serial_num}')

    def _handle_error_data(self, frame):
        """Parse and log device error."""
        if len(frame) >= 7:
            error_code = frame[3]
            error_param = frame[4]
            error_types = {
                0x00: 'Header/footer or length error',
                0x01: 'Checksum error',
                0x02: 'Mode error',
                0x03: 'Invalid ID',
            }
            error_msg = error_types.get(error_code, f'Unknown (0x{error_code:02X})')
            self.get_logger().warn(f'Device error: {error_msg}, param=0x{error_param:02X}')

    # ── Control Callbacks ──

    def zero_calib_callback(self, msg):
        """Handle zero calibration request."""
        if msg.data:
            self.get_logger().info('Sending zero calibration command')
            self._send_raw(CMD_ZERO_CALI)

    def torque_callback(self, msg):
        """Handle torque enable/disable."""
        if msg.data:
            self.get_logger().info('Enabling torque')
            self._send_raw(CMD_TORQUE_ON)
        else:
            self.get_logger().info('Disabling torque')
            self._send_raw(CMD_TORQUE_OFF)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AliciaDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
