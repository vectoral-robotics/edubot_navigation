"""Hardware-free Humble smoke test. Run in an isolated ROS container after colcon build."""

import http.client
import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__("localhost", timeout=40)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def wait_for(predicate, seconds=45):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        try:
            value = predicate()
            if value:
                return value
        except (OSError, http.client.HTTPException):
            pass
        time.sleep(0.25)
    raise AssertionError("Timed out waiting for ROS state")


def main():
    rclpy.init()
    node = Node("navigation_test_sensors")
    scan_pub = node.create_publisher(LaserScan, "/scan", 10)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    broadcaster = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    laser_tf = TransformStamped()
    laser_tf.header.frame_id, laser_tf.child_frame_id = "base_link", "laser"
    laser_tf.transform.rotation.w = 1.0
    static.sendTransform(laser_tf)
    velocities = []
    node.create_subscription(Twist, "/cmd_vel", lambda message: velocities.append(message), 10)

    def sensors():
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id, transform.child_frame_id = "odom", "base_link"
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "laser"
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2 * math.pi / 359
        scan.range_min, scan.range_max = 0.05, 10.0
        scan.scan_time = 0.1
        # A stationary robot at the center of a four-metre square room.
        scan.ranges = [
            2
            / max(
                abs(math.cos(scan.angle_min + i * scan.angle_increment)),
                abs(math.sin(scan.angle_min + i * scan.angle_increment)),
            )
            for i in range(360)
        ]
        scan_pub.publish(scan)

    node.create_timer(0.1, sensors)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        with tempfile.TemporaryDirectory(prefix="navigation-integration-") as directory:
            root = Path(directory)
            log = (root / "manager.log").open("w")
            env = {**os.environ, "EDUBOT_NAVIGATION_STATE": directory}
            process = None

            def start():
                return subprocess.Popen(
                    ["ros2", "run", "edubot_navigation", "navigation_manager"],
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )

            def request(command=None):
                connection = UnixConnection(str(root / "control.sock"))
                try:
                    connection.request(
                        "POST" if command else "GET",
                        "/command" if command else "/status",
                        json.dumps(command) if command else None,
                        {"Content-Type": "application/json"},
                    )
                    response = connection.getresponse()
                    data = json.loads(response.read())
                    if response.status != 200:
                        raise AssertionError(data)
                    return data
                finally:
                    connection.close()

            try:
                process = start()
                wait_for(lambda: request()["sensorsReady"])
                request({"action": "mapping"})
                wait_for(lambda: request()["mapReady"])
                print("PASS: live sensors -> SLAM map", flush=True)
                request({"action": "save", "name": "test-room"})
                request({"action": "activate", "name": "test-room"})
                wait_for(lambda: request()["mapReady"])
                time.sleep(3)
                request({"action": "pose", "x": 0, "y": 0, "yaw": 0})
                wait_for(lambda: request()["ready"], 60)
                print("PASS: saved map -> AMCL and Nav2 ready", flush=True)
                request({"action": "goal", "x": 0.4, "y": 0, "yaw": 0})
                wait_for(lambda: any(abs(v.linear.x) > 0.001 for v in velocities), 15)
                assert all(v.linear.y == 0 for v in velocities)
                request({"action": "cancel"})
                count = len(velocities)
                time.sleep(1)
                assert all(v.linear.x == 0 and v.angular.z == 0 for v in velocities[count:])
                print(
                    "PASS: real NavigateToPose action, differential output and cancellation",
                    flush=True,
                )
                request({"action": "stop"})
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=20)
                process = start()
                wait_for(lambda: request()["loadedMap"] == "test-room")
                assert request()["mode"] == "navigation"
                print("PASS: active map restored after manager restart", flush=True)
            except Exception:
                for path in [root / "manager.log", root / "session.log"]:
                    print(f"\n{path.name}:\n{path.read_text()[-18000:]}", flush=True)
                raise
            finally:
                if process and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=25)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                log.close()
    finally:
        rclpy.shutdown()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()
