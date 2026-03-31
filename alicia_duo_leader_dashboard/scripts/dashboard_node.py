#!/usr/bin/env python3
"""
Alicia-D Leader Dashboard — Web-based URDF viewer + joint state display.
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from alicia_duo_leader_driver.msg import ArmJointState
from std_msgs.msg import Bool

import http.server
import json
import math
import mimetypes
import os
import threading
import time
import yaml
import xml.etree.ElementTree as ET
from functools import partial

# Register MIME types
mimetypes.add_type("application/octet-stream", ".stl")
mimetypes.add_type("application/octet-stream", ".STL")
mimetypes.add_type("application/xml", ".urdf")
mimetypes.add_type("application/javascript", ".js")

RAD_TO_DEG = 180.0 / math.pi


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    """Serves index.html at / and handles API endpoints. Static files from CWD."""

    def __init__(self, *args, dashboard_node=None, viewer_html=b'', **kwargs):
        self._dashboard = dashboard_node
        self._viewer_html = viewer_html
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(self._viewer_html)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(self._viewer_html)
        elif self.path == '/api/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self._dashboard.register_sse_client(self.wfile)
            try:
                while True:
                    time.sleep(1)
            except Exception:
                pass
            finally:
                self._dashboard.unregister_sse_client(self.wfile)
        elif self.path == '/api/state':
            state = self._dashboard.get_latest_state()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(state or {}).encode())
        elif self.path == '/api/config':
            config = self._dashboard.get_config()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(config).encode())
        else:
            super().do_GET()

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(length)) if length else {}
        if self.path == '/api/publish_enable':
            enabled = bool(body.get('enabled', True))
            self._dashboard.set_publish_enabled(enabled)
            resp = json.dumps({'success': True, 'enabled': enabled}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        if len(args) >= 2 and '404' in str(args[1]):
            super().log_message(format, *args)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('alicia_duo_leader_dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')

        web_port = self.get_parameter('port').value
        web_host = self.get_parameter('host').value

        self._latest_state = None
        self._lock = threading.Lock()
        self._sse_clients = []
        self._sse_lock = threading.Lock()
        self._device_connected = None  # None = unknown, True/False from driver

        self.create_subscription(ArmJointState, '/arm_joint_state', self._cb, 10)
        self.create_subscription(ArmJointState, '/arm_joint_state_raw', self._cb_raw, 10)
        self.create_subscription(Bool, '/alicia/device_connected', self._device_status_cb, 10)

        # Publisher for enabling/disabling arm_joint_state publishing
        self._publish_enable_pub = self.create_publisher(Bool, '/alicia/publish_enable', 10)
        self._publish_enabled = True

        self._latest_raw = None

        # Serve from static dir — same pattern as working reference viewer
        pkg_share = get_package_share_directory('alicia_duo_leader_dashboard')
        static_dir = os.path.join(pkg_share, 'static')
        os.chdir(static_dir)
        self.get_logger().info(f'Serving static files from: {static_dir}')

        # Load HTML from file
        html_path = os.path.join(static_dir, 'index.html')
        with open(html_path, 'rb') as f:
            viewer_html = f.read()

        handler = partial(DashboardHandler, dashboard_node=self, viewer_html=viewer_html)
        self._httpd = http.server.ThreadingHTTPServer((web_host, web_port), handler)
        self._web_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._web_thread.start()

        self.get_logger().info(f'Dashboard: http://localhost:{web_port}')

        # Load joint config and URDF limits
        self._joint_continuous = [False] * 6
        self._urdf_limits_deg = [[-180, 180]] * 6  # [lower, upper] in degrees
        self._load_joint_config()
        self._load_urdf_limits(static_dir)

    def _load_joint_config(self):
        """Load continuous flags from joint config."""
        try:
            driver_share = get_package_share_directory('alicia_duo_leader_driver')
            ws_root = None
            d = os.path.dirname(driver_share)
            for _ in range(6):
                d = os.path.dirname(d)
                if os.path.isfile(os.path.join(d, 'config', 'joint_config_template.yaml')):
                    ws_root = d
                    break
            candidates = []
            if ws_root:
                candidates.append(os.path.join(ws_root, 'config', 'joint_config.yaml'))
                candidates.append(os.path.join(ws_root, 'config', 'joint_config_template.yaml'))
            candidates.append(os.path.join(driver_share, 'config', 'joint_config.yaml'))
            candidates.append(os.path.join(driver_share, 'config', 'joint_config_template.yaml'))
            for path in candidates:
                if os.path.isfile(path):
                    with open(path, 'r') as f:
                        cfg = yaml.safe_load(f)
                    jc = cfg.get('joint_config', {})
                    for i, name in enumerate(['joint1','joint2','joint3','joint4','joint5','joint6']):
                        if name in jc:
                            self._joint_continuous[i] = bool(jc[name].get('continuous', False))
                    self.get_logger().info(f'Joint continuous: {self._joint_continuous}')
                    return
        except Exception as e:
            self.get_logger().warn(f'Could not load joint config: {e}')

    def _load_urdf_limits(self, static_dir):
        """Load joint limits from URDF file."""
        try:
            urdf_path = os.path.join(static_dir, 'robot', 'model.urdf')
            if not os.path.isfile(urdf_path):
                return
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
            for joint_el in root.findall('joint'):
                name = joint_el.get('name')
                if name in joint_names:
                    idx = joint_names.index(name)
                    limit_el = joint_el.find('limit')
                    if limit_el is not None:
                        lower = float(limit_el.get('lower', '-3.14159'))
                        upper = float(limit_el.get('upper', '3.14159'))
                        self._urdf_limits_deg[idx] = [round(lower * RAD_TO_DEG, 1), round(upper * RAD_TO_DEG, 1)]
            self.get_logger().info(f'URDF joint limits (deg): {self._urdf_limits_deg}')
        except Exception as e:
            self.get_logger().warn(f'Could not load URDF limits: {e}')

    def get_config(self):
        return {
            'continuous': self._joint_continuous,
            'urdf_limits_deg': self._urdf_limits_deg,
        }

    def _cb_raw(self, msg):
        with self._lock:
            self._latest_raw = {
                'joints': [round(v * RAD_TO_DEG, 2) for v in [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]],
                'joints_rad': [round(v, 4) for v in [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]],
                'gripper': round(msg.gripper, 1),
            }

    def _device_status_cb(self, msg):
        with self._lock:
            self._device_connected = msg.data
        # Push device status immediately via SSE (independent of joint data)
        data_str = json.dumps({'device_connected': msg.data})
        self._broadcast_sse(data_str)

    def _cb(self, msg):
        state = {
            'joints': [round(v * RAD_TO_DEG, 2) for v in [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]],
            'joints_rad': [round(v, 4) for v in [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]],
            'gripper': round(msg.gripper, 1),
            'status': msg.but1,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        with self._lock:
            self._latest_state = state
            combined = dict(state)
            if self._latest_raw:
                combined['raw'] = self._latest_raw
            if self._device_connected is not None:
                combined['device_connected'] = self._device_connected

        data_str = json.dumps(combined)
        self._broadcast_sse(data_str)

    def _broadcast_sse(self, data_str):
        with self._sse_lock:
            dead = []
            for wf in self._sse_clients:
                try:
                    wf.write(f'data: {data_str}\n\n'.encode())
                    wf.flush()
                except Exception:
                    dead.append(wf)
            for d in dead:
                self._sse_clients.remove(d)

    def get_latest_state(self):
        with self._lock:
            return self._latest_state

    def register_sse_client(self, wfile):
        with self._sse_lock:
            self._sse_clients.append(wfile)

    def unregister_sse_client(self, wfile):
        with self._sse_lock:
            if wfile in self._sse_clients:
                self._sse_clients.remove(wfile)

    def set_publish_enabled(self, enabled):
        self._publish_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self._publish_enable_pub.publish(msg)
        self.get_logger().info(f'Publish /arm_joint_state: {"enabled" if enabled else "disabled"}')

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DashboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
