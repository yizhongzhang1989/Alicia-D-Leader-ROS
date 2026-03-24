#!/usr/bin/env python3
"""
Web dashboard node for Alicia-D arm joint states.
Subscribes to /arm_joint_state and serves a web UI with real-time updates via SSE.
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from alicia_duo_leader_driver.msg import ArmJointState

import json
import math
import mimetypes
import os
import threading
import time
import yaml
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial

RAD_TO_DEG = 180.0 / math.pi

# Register MIME types for robot files
mimetypes.add_type('application/octet-stream', '.stl')
mimetypes.add_type('application/octet-stream', '.STL')
mimetypes.add_type('application/xml', '.urdf')
mimetypes.add_type('application/javascript', '.js')


class DashboardNode(Node):
    def __init__(self):
        super().__init__('arm_joint_state_dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')

        self.web_port = self.get_parameter('port').value
        self.web_host = self.get_parameter('host').value

        self._latest_state = None
        self._latest_raw_state = None
        self._lock = threading.Lock()
        self._sse_clients = []
        self._sse_lock = threading.Lock()

        self.create_subscription(
            ArmJointState, '/arm_joint_state', self._joint_state_cb, 10)
        self.create_subscription(
            ArmJointState, '/arm_joint_state_raw', self._joint_state_raw_cb, 10)

        # Serve static files from package share directory
        pkg_share = get_package_share_directory('arm_joint_state_dashboard')
        static_dir = os.path.join(pkg_share, 'static')

        # Start web server in a background thread
        handler = partial(DashboardHandler, static_dir, self)
        self._httpd = HTTPServer((self.web_host, self.web_port), handler)
        self._web_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._web_thread.start()

        self.get_logger().info(f'Dashboard running at http://localhost:{self.web_port}')

        # Load joint config to know which joints are continuous
        self._joint_continuous = [False] * 6
        self._load_joint_config()

        # Load URDF from alicia_d_description package for web serving
        self._urdf_web_content = None
        self._mesh_dir = None
        self._load_urdf_for_web()

    def _load_joint_config(self):
        """Load continuous flags from joint config."""
        try:
            driver_share = get_package_share_directory('alicia_duo_leader_driver')
            # Search order: workspace root config/, then installed share
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
                    self.get_logger().info(f'Joint continuous flags: {self._joint_continuous}')
                    return
        except Exception as e:
            self.get_logger().warn(f'Could not load joint config for dashboard: {e}')

    def get_joint_config(self):
        return {'continuous': self._joint_continuous}

    def _load_urdf_for_web(self):
        """Load URDF from alicia_d_description and prepare for web serving."""
        try:
            desc_share = get_package_share_directory('alicia_d_description')
            urdf_path = os.path.join(desc_share, 'urdf', 'Alicia_D_v5_6_leader_ur.urdf')
            mesh_dir = os.path.join(desc_share, 'meshes', 'leader_ur')

            if not os.path.isfile(urdf_path):
                self.get_logger().warn(f'URDF not found: {urdf_path}')
                return

            with open(urdf_path, 'r') as f:
                urdf_content = f.read()

            # Replace package:// URIs with relative web paths
            # package://alicia_d_description/meshes/leader_ur/X.STL -> meshes/X.STL
            urdf_content = urdf_content.replace(
                'package://alicia_d_description/meshes/leader_ur/', 'meshes/')

            # Remove mujoco tag (has stale meshdir paths, not needed for web viewer)
            import re
            urdf_content = re.sub(r'\s*<mujoco>.*?</mujoco>\s*', '\n', urdf_content, flags=re.DOTALL)

            self._urdf_web_content = urdf_content
            self._mesh_dir = mesh_dir
            self.get_logger().info(f'URDF loaded for web from {urdf_path}')
            self.get_logger().info(f'Mesh dir: {mesh_dir}')
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF for web: {e}')

    def get_urdf_content(self):
        return self._urdf_web_content

    def get_mesh_path(self, filename):
        """Return full path to a mesh file, or None."""
        if not self._mesh_dir:
            return None
        path = os.path.join(self._mesh_dir, filename)
        if os.path.isfile(path):
            return path
        return None

    def _joint_state_raw_cb(self, msg):
        raw = {
            'joints': [
                round(msg.joint1 * RAD_TO_DEG, 2),
                round(msg.joint2 * RAD_TO_DEG, 2),
                round(msg.joint3 * RAD_TO_DEG, 2),
                round(msg.joint4 * RAD_TO_DEG, 2),
                round(msg.joint5 * RAD_TO_DEG, 2),
                round(msg.joint6 * RAD_TO_DEG, 2),
            ],
            'joints_rad': [
                round(msg.joint1, 4),
                round(msg.joint2, 4),
                round(msg.joint3, 4),
                round(msg.joint4, 4),
                round(msg.joint5, 4),
                round(msg.joint6, 4),
            ],
            'gripper': round(msg.gripper, 1),
        }
        with self._lock:
            self._latest_raw_state = raw

    def _joint_state_cb(self, msg):
        state = {
            'joints': [
                round(msg.joint1 * RAD_TO_DEG, 2),
                round(msg.joint2 * RAD_TO_DEG, 2),
                round(msg.joint3 * RAD_TO_DEG, 2),
                round(msg.joint4 * RAD_TO_DEG, 2),
                round(msg.joint5 * RAD_TO_DEG, 2),
                round(msg.joint6 * RAD_TO_DEG, 2),
            ],
            'joints_rad': [
                round(msg.joint1, 4),
                round(msg.joint2, 4),
                round(msg.joint3, 4),
                round(msg.joint4, 4),
                round(msg.joint5, 4),
                round(msg.joint6, 4),
            ],
            'gripper': round(msg.gripper, 1),
            'status': msg.but1,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

        with self._lock:
            self._latest_state = state
            combined = dict(state)
            if self._latest_raw_state:
                combined['raw'] = self._latest_raw_state

        # Push to SSE clients
        data_str = json.dumps(combined)
        with self._sse_lock:
            dead = []
            for client_wfile in self._sse_clients:
                try:
                    client_wfile.write(f'data: {data_str}\n\n'.encode())
                    client_wfile.flush()
                except Exception:
                    dead.append(client_wfile)
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

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, static_dir, dashboard_node, *args, **kwargs):
        self._static_dir = static_dir
        self._dashboard = dashboard_node
        super().__init__(*args, directory=static_dir, **kwargs)

    def do_GET(self):
        if self.path == '/api/state':
            state = self._dashboard.get_latest_state()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(state or {}).encode())
        elif self.path == '/api/config':
            config = self._dashboard.get_joint_config()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(config).encode())
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
        elif self.path == '/robot/model.urdf':
            # Serve URDF with web-friendly mesh paths
            content = self._dashboard.get_urdf_content()
            if content:
                data = content.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/xml')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404, 'URDF not loaded')
        elif self.path.startswith('/robot/meshes/'):
            # Serve mesh files from alicia_d_description package
            filename = self.path.split('/robot/meshes/')[-1]
            # Sanitize: only allow simple filenames (no path traversal)
            if '/' in filename or '..' in filename:
                self.send_error(403)
                return
            mesh_path = self._dashboard.get_mesh_path(filename)
            if mesh_path:
                with open(mesh_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404, f'Mesh not found: {filename}')
        else:
            super().do_GET()

    def log_message(self, format, *args):
        pass  # Suppress HTTP access logs


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
