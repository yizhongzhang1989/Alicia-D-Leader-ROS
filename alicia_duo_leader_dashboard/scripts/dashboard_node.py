#!/usr/bin/env python3
"""
Alicia-D Leader Dashboard — Web-based URDF viewer + joint state display.
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from alicia_duo_leader_driver.msg import ArmJointState

import http.server
import json
import math
import mimetypes
import os
import threading
import time
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
        else:
            super().do_GET()

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

        self.create_subscription(ArmJointState, '/arm_joint_state', self._cb, 10)
        self.create_subscription(ArmJointState, '/arm_joint_state_raw', self._cb_raw, 10)

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

    def _cb_raw(self, msg):
        with self._lock:
            self._latest_raw = {
                'joints': [round(v * RAD_TO_DEG, 2) for v in [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]],
                'joints_rad': [round(v, 4) for v in [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]],
                'gripper': round(msg.gripper, 1),
            }

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

        data_str = json.dumps(combined)
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
