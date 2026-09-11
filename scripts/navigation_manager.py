#!/usr/bin/env python3
"""Robot-owned mapping/navigation session, exposed only through a shared Unix socket."""

import contextlib
import fcntl
import json
import math
import os
import re
import shutil
import signal
import socketserver
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler
from pathlib import Path

import yaml


class RequestError(Exception):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def map_name(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", value):
        raise RequestError(
            "Use 1-64 letters, digits, underscores or hyphens for the map name", 400
        )
    return value


def finite_number(body, key, limit):
    value = body.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise RequestError(f"{key} must be a number", 400)
    if not math.isfinite(value) or abs(value) > limit:
        raise RequestError(f"{key} is out of range", 400)
    return float(value)


class MapStore:
    def __init__(self, root):
        self.root = Path(root)
        self.maps = self.root / "maps"
        self.maps.mkdir(parents=True, exist_ok=True)
        self.selection = self.root / "active-map.json"

    def path(self, name):
        result = self.maps / map_name(name) / "map.yaml"
        if not result.is_file() or not (result.parent / "map.pgm").is_file():
            raise RequestError("Saved map not found", 404)
        return result

    def list(self):
        return sorted(
            p.parent.name
            for p in self.maps.glob("*/map.yaml")
            if not p.parent.name.startswith(".") and (p.parent / "map.pgm").is_file()
        )

    def active(self):
        if not self.selection.exists():
            return None
        name = json.loads(self.selection.read_text())["name"]
        self.path(name)
        return name

    def activate(self, name):
        self.path(name)
        temporary = self.selection.with_suffix(".tmp")
        with temporary.open("w") as stream:
            json.dump({"name": name}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.selection)

    def save(self, name, run=subprocess.run):
        name = map_name(name)
        destination = self.maps / name
        if destination.exists():
            raise RequestError("A map with this name already exists; choose a new name")
        temporary = Path(tempfile.mkdtemp(prefix=".saving-", dir=self.maps))
        try:
            result = run(
                [
                    "ros2",
                    "run",
                    "nav2_map_server",
                    "map_saver_cli",
                    "-f",
                    str(temporary / "map"),
                    "--fmt",
                    "pgm",
                    "--ros-args",
                    "-p",
                    "save_map_timeout:=10.0",
                    "-p",
                    "map_subscribe_transient_local:=true",
                ],
                capture_output=True,
                text=True,
                timeout=25,
                check=False,
            )
            if result.returncode != 0:
                raise RequestError("Map saving failed; check the LiDAR and SLAM log")
            metadata_path = temporary / "map.yaml"
            metadata = yaml.safe_load(metadata_path.read_text())
            if not (temporary / "map.pgm").is_file() or not metadata.get("resolution", 0) > 0:
                raise RequestError("Map saver produced an incomplete map")
            # map_saver may write an absolute image path. Keep the pair relocatable.
            metadata["image"] = "map.pgm"
            metadata_path.write_text(yaml.safe_dump(metadata))
            temporary.rename(destination)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)


class Session:
    def __init__(self, store, ros, popen=subprocess.Popen):
        self.store, self.ros, self.popen = store, ros, popen
        self.process = None
        self.log = None
        self.mode = "idle"
        self.loaded_map = None
        self.error = None
        self.lock = threading.Lock()

    def status(self):
        if self.lock.acquire(blocking=False):
            try:
                if self.process is not None and self.process.poll() is not None:
                    self.ros.halt()
                    self.mode = "error"
                    self.ros.mode = "idle"
                    self.error = (
                        "SLAM/Nav2 exited. Stop the session and check session.log on the robot."
                    )
            finally:
                self.lock.release()
        try:
            active = self.store.active()
        except (ValueError, KeyError, OSError, RequestError):
            active = None
            self.error = "The active map is missing or invalid; activate a saved map."
        return {
            "ok": True,
            "mode": self.mode,
            "activeMap": active,
            "loadedMap": self.loaded_map,
            "maps": self.store.list(),
            "error": self.error,
            "busy": self.lock.locked(),
            **self.ros.status(),
        }

    def stop(self):
        self.ros.halt()
        if self.process is not None:
            # A process group also covers child nodes if ros2 launch itself has exited.
            for sig, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                with contextlib.suppress(subprocess.TimeoutExpired):
                    self.process.wait(timeout=timeout)
                try:
                    os.killpg(self.process.pid, 0)
                except ProcessLookupError:
                    break
            self.process = None
        if self.log:
            self.log.close()
            self.log = None
        self.ros.reset()
        self.mode, self.loaded_map = "idle", None

    def start(self, mode, name=None):
        path = self.store.path(name) if mode == "navigation" else None
        self.stop()
        self.error = None
        launch = "dashboard_navigation.launch.py" if path else "edubot_slam.launch.py"
        args = ["ros2", "launch", "edubot_navigation", launch, "use_sim_time:=false"]
        args += [f"map:={path}"] if path else ["use_rviz:=false"]
        self.log = (self.store.root / "session.log").open("w")
        try:
            self.process = self.popen(
                args, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True
            )
        except Exception:
            self.log.close()
            self.log = None
            raise
        self.mode, self.loaded_map = mode, name
        self.ros.mode = mode

    def command(self, body):
        action = body.get("action")
        # Stop the motor gate immediately, even while map saving holds the command lock.
        if action == "stop":
            self.ros.halt()
        # Status polling holds this lock only briefly. Allow it to finish without
        # rejecting a legitimate command; long mutations still return busy.
        if not self.lock.acquire(timeout=0.1):
            raise RequestError("Another navigation operation is in progress; retry shortly")
        try:
            if action == "mapping":
                if not self.ros.status()["sensorsReady"]:
                    raise RequestError("Waiting for live LiDAR and odometry")
                self.start("mapping")
            elif action == "save":
                if self.mode != "mapping" or not self.ros.map_received:
                    raise RequestError("Start mapping and wait for a map before saving")
                self.ros.halt()
                self.store.save(body.get("name"))
            elif action == "activate":
                name = map_name(body.get("name"))
                self.store.path(name)
                self.start("navigation", name)
                self.store.activate(name)
            elif action == "resume":
                name = self.store.active()
                if name is None:
                    raise RequestError("Save and activate a map first")
                self.start("navigation", name)
            elif action == "stop":
                self.stop()
            elif action == "cancel":
                self.ros.cancel()
            elif action in ("goal", "pose"):
                if self.mode != "navigation":
                    raise RequestError("Activate a map before setting a pose or goal")
                pose = [
                    finite_number(body, key, limit)
                    for key, limit in [("x", 10000), ("y", 10000), ("yaw", math.pi)]
                ]
                getattr(self.ros, action)(*pose)
            elif action == "drive":
                if self.mode != "mapping":
                    raise RequestError("Manual mapping controls are only available during mapping")
                self.ros.drive(
                    finite_number(body, "linear", 0.15), finite_number(body, "angular", 0.6)
                )
            else:
                raise RequestError("Unknown navigation action", 400)
            return self.status()
        finally:
            self.lock.release()


class RosBridge:
    def __init__(self):
        import rclpy
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener

        self.rclpy, self.Twist, self.Pose = rclpy, Twist, PoseWithCovarianceStamped
        self.NavigateToPose, self.CancelGoal = NavigateToPose, CancelGoal
        self.node = Node("edubot_navigation_manager")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.velocity = self.node.create_publisher(Twist, "/cmd_vel", 1)
        self.initial_pose = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        self.action = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.GetState = GetState
        self.lifecycle = self.node.create_client(GetState, "/bt_navigator/get_state")
        self.lifecycle_pending = False
        self.session_generation = 0
        self.cancel_client = self.node.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.last_scan = self.last_odom = 0
        self.motion_lock = threading.RLock()
        self.stop_generation = 0
        self.reset()
        self.node.create_subscription(
            LaserScan,
            "/scan",
            lambda _: setattr(self, "last_scan", time.monotonic()),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Odometry,
            "/odom",
            lambda _: setattr(self, "last_odom", time.monotonic()),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.node.create_subscription(Twist, "/navigation/cmd_vel", self.on_velocity, 1)
        self.node.create_timer(0.05, self.watchdog)
        self.node.create_timer(1.0, self.check_lifecycle)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def reset(self):
        with self.motion_lock:
            self.session_generation += 1
            self.nav_active = False
            self.lifecycle_pending = False
            self.mode = "idle"
            self.map_received = False
            self.pose_set = False
            self.goal_handle = None
            self.goal_status = "No active goal"
            self.moving = False
            self.manual_until = 0
            self.last_command = 0
            self.tf.clear()

    def check_lifecycle(self):
        if self.mode != "navigation" or self.lifecycle_pending:
            return
        if not self.lifecycle.service_is_ready():
            self.nav_active = False
            return
        self.lifecycle_pending = True
        generation = self.session_generation
        future = self.lifecycle.call_async(self.GetState.Request())

        def finished(result):
            self.lifecycle_pending = False
            if generation == self.session_generation:
                try:
                    self.nav_active = result.result().current_state.id == 3
                except Exception:
                    self.nav_active = False

        future.add_done_callback(finished)

    def on_map(self, message):
        if self.mode != "idle" and message.info.width > 0 and message.info.height > 0:
            self.map_received = True

    def transform_ready(self):
        try:
            transform = self.tf.lookup_transform("map", "base_link", self.rclpy.time.Time())
            age = (
                self.node.get_clock().now().nanoseconds
                - (transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec)
            ) / 1e9
            return -2 < age < 2
        except Exception:
            return False

    def status(self):
        sensors = time.monotonic() - min(self.last_scan, self.last_odom) < 1.0
        return {
            "sensorsReady": sensors,
            "mapReady": self.map_received,
            "localized": self.pose_set and self.transform_ready(),
            "ready": self.mode == "navigation"
            and sensors
            and self.pose_set
            and self.map_received
            and self.transform_ready()
            and self.action.server_is_ready()
            and self.nav_active,
            "goalStatus": self.goal_status,
        }

    def halt(self):
        with self.motion_lock:
            self.stop_generation += 1
            self.moving = False
            self.manual_until = 0
            self.velocity.publish(self.Twist())

    def watchdog(self):
        with self.motion_lock:
            now = time.monotonic()
            if self.manual_until:
                if now > self.manual_until or not self.status()["sensorsReady"]:
                    self.halt()
            elif self.moving and (not self.status()["ready"] or now - self.last_command > 1):
                self.halt()
                self.goal_status = "Stopped: sensor, localization or controller timeout"
                if self.cancel_client.service_is_ready():
                    self.cancel_client.call_async(self.CancelGoal.Request())

    def on_velocity(self, message):
        with self.motion_lock:
            if self.mode != "navigation" or not self.moving or not self.status()["ready"]:
                return
            output = self.Twist()
            output.linear.x = max(-0.15, min(0.2, message.linear.x))
            output.angular.z = max(-0.8, min(0.8, message.angular.z))
            # Explicitly keep all lateral/vertical commands zero for differential navigation.
            self.velocity.publish(output)
            self.last_command = time.monotonic()

    def drive(self, linear, angular):
        with self.motion_lock:
            if not self.status()["sensorsReady"]:
                self.halt()
                raise RequestError("Waiting for live LiDAR and odometry")
            message = self.Twist()
            message.linear.x, message.angular.z = linear, angular
            self.manual_until = time.monotonic() + 0.4
            self.velocity.publish(message)

    @staticmethod
    def wait(future, timeout=5):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise RequestError("ROS operation timed out; wait for Nav2 to become ready", 504)
        return future.result()

    def cancel(self):
        self.halt()
        if self.action.server_is_ready():
            if not self.cancel_client.wait_for_service(timeout_sec=2):
                raise RequestError("Nav2 cancellation service is unavailable")
            response = self.wait(self.cancel_client.call_async(self.CancelGoal.Request()))
            if response.return_code != 0:
                raise RequestError(
                    "Nav2 did not acknowledge cancellation; motor output is stopped"
                )
        self.goal_status = "Cancelled"
        self.goal_handle = None

    def pose(self, x, y, yaw):
        self.cancel()
        if self.initial_pose.get_subscription_count() == 0:
            raise RequestError("AMCL is still starting; retry setting the initial pose")
        message = self.Pose()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            math.sin(yaw / 2),
            math.cos(yaw / 2),
        )
        message.pose.covariance[0] = message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.tf.clear()
        self.initial_pose.publish(message)
        self.pose_set = True
        self.goal_status = "Initial pose set; waiting for localization"

    def goal(self, x, y, yaw):
        if not self.status()["ready"]:
            raise RequestError(
                "Wait for Nav2, live sensors and localization; set the initial pose on the map"
            )
        self.cancel()
        generation = self.stop_generation
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = (
            math.sin(yaw / 2),
            math.cos(yaw / 2),
        )
        handle = self.wait(self.action.send_goal_async(goal))
        if not handle.accepted:
            raise RequestError("Nav2 rejected the goal")
        with self.motion_lock:
            if generation != self.stop_generation:
                handle.cancel_goal_async()
                raise RequestError("Navigation was stopped while the goal was being accepted")
            self.goal_handle = handle
            self.goal_status = "Navigating"
            self.last_command = (
                time.monotonic() + 4
            )  # Allow initial planning before the first command.
            self.moving = True
        handle.get_result_async().add_done_callback(lambda future: self.on_result(handle, future))

    def on_result(self, handle, future):
        with self.motion_lock:
            if handle is self.goal_handle:
                self.halt()
                self.goal_status = {4: "Goal reached", 5: "Cancelled", 6: "Navigation failed"}.get(
                    future.result().status, "Navigation ended"
                )
                self.goal_handle = None


class ControlServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def respond(self, status, body):
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path != "/status":
            return self.respond(404, {"ok": False, "error": "Not found"})
        self.respond(200, self.server.session.status())

    def do_POST(self):
        try:
            if self.path != "/command":
                raise RequestError("Not found", 404)
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise RequestError("Invalid request size", 400)
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise RequestError("Expected an object", 400)
            self.respond(200, self.server.session.command(body))
        except RequestError as error:
            self.respond(error.status, {"ok": False, "error": str(error)})
        except (ValueError, TypeError) as error:
            self.respond(400, {"ok": False, "error": str(error)})
        except Exception as error:
            self.server.session.error = str(error)
            self.respond(500, {"ok": False, "error": str(error)})


def main():
    import rclpy

    root = Path(os.environ.get("EDUBOT_NAVIGATION_STATE", "/state/edubot/navigation"))
    store = MapStore(root)
    # Hold a lifetime lock before removing a stale socket or starting any ROS processes.
    with (root / "manager.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        socket = root / "control.sock"
        socket.unlink(missing_ok=True)
        rclpy.init()
        ros = RosBridge()
        session = Session(store, ros)
        with ControlServer(str(socket), Handler) as server:
            os.chmod(socket, 0o660)
            server.session = session
            server.timeout = 0.5
            running = True

            def shutdown(_signum, _frame):
                nonlocal running
                running = False
                ros.halt()

            signal.signal(signal.SIGTERM, shutdown)
            signal.signal(signal.SIGINT, shutdown)
            try:
                active = store.active()
                if active:
                    session.start("navigation", active)
            except Exception as error:
                session.error = f"Could not restore active map: {error}"
            try:
                while running:
                    server.handle_request()
                    session.status()
            finally:
                with session.lock:
                    session.stop()
                rclpy.shutdown()
                ros.thread.join(timeout=2)
                socket.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
