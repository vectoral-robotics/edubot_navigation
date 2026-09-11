"""Run without ROS: python3 -m unittest discover -s tests -v."""

import importlib.util
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import yaml

spec = importlib.util.spec_from_file_location(
    "manager", Path(__file__).parents[1] / "scripts/navigation_manager.py"
)
manager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager)


class MapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = manager.MapStore(self.temp.name)

    @staticmethod
    def saver(args, **_kwargs):
        output = Path(args[args.index("-f") + 1])
        output.with_suffix(".yaml").write_text(
            yaml.safe_dump(
                {"image": str(output.with_suffix(".pgm")), "resolution": 0.05, "origin": [0, 0, 0]}
            )
        )
        output.with_suffix(".pgm").write_bytes(b"P5\n1 1\n255\n\xff")
        return subprocess.CompletedProcess(args, 0)

    def test_saved_map_is_relocatable_and_active_selection_survives_restart(self):
        self.store.save("Room-1", self.saver)
        self.store.activate("Room-1")
        restored = manager.MapStore(self.temp.name)
        self.assertEqual(restored.active(), "Room-1")
        self.assertEqual(restored.list(), ["Room-1"])
        self.assertEqual(yaml.safe_load(restored.path("Room-1").read_text())["image"], "map.pgm")

    def test_failed_save_never_changes_active_map_or_leaves_partial_map(self):
        self.store.save("old", self.saver)
        self.store.activate("old")
        with self.assertRaises(manager.RequestError):
            self.store.save("new", lambda *a, **k: subprocess.CompletedProcess([], 1))
        self.assertEqual(self.store.active(), "old")
        self.assertEqual(list(self.store.maps.iterdir()), [self.store.maps / "old"])

    def test_names_cannot_escape_map_directory(self):
        for name in ["../oops", "/tmp/map", "a/b", "$(id)", "", None, "a" * 65]:
            with self.subTest(name=name), self.assertRaises(manager.RequestError):
                self.store.save(name, self.saver)

    def test_duplicates_are_not_overwritten(self):
        self.store.save("map", self.saver)
        with self.assertRaises(manager.RequestError):
            self.store.save("map", self.saver)

    def test_missing_activation_keeps_old_selection(self):
        self.store.save("old", self.saver)
        self.store.activate("old")
        with self.assertRaises(manager.RequestError):
            self.store.activate("missing")
        self.assertEqual(self.store.active(), "old")


class SessionTests(unittest.TestCase):
    saver = staticmethod(MapTests.saver)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = manager.MapStore(self.temp.name)
        self.ros = Mock()
        self.ros.status.return_value = {"sensorsReady": True}
        self.process = Mock(pid=123456)
        self.process.poll.return_value = None
        self.popen = Mock(return_value=self.process)
        self.session = manager.Session(self.store, self.ros, self.popen)
        self.addCleanup(lambda: self.session.log and self.session.log.close())

    def test_activation_launches_saved_map_and_restores_with_resume(self):
        self.store.save("room", self.saver)
        self.session.command({"action": "activate", "name": "room"})
        self.assertIn(f"map:={self.store.path('room')}", self.popen.call_args.args[0])
        self.assertEqual(self.store.active(), "room")
        self.assertEqual(self.session.mode, "navigation")
        self.assertEqual(self.ros.mode, "navigation")

    def test_mapping_requires_live_sensors(self):
        self.ros.status.return_value = {"sensorsReady": False}
        with self.assertRaises(manager.RequestError):
            self.session.command({"action": "mapping"})
        self.popen.assert_not_called()

    def test_modes_reject_inappropriate_commands(self):
        for command in [
            {"action": "save", "name": "new"},
            {"action": "goal", "x": 0, "y": 0, "yaw": 0},
            {"action": "drive", "linear": 0.1, "angular": 0},
        ]:
            with self.subTest(command=command), self.assertRaises(manager.RequestError):
                self.session.command(command)

    def test_command_waits_for_a_short_status_poll(self):
        self.session.lock.acquire()
        timer = threading.Timer(0.02, self.session.lock.release)
        timer.start()
        try:
            result = self.session.command({"action": "stop"})
            self.assertEqual(result["mode"], "idle")
        finally:
            timer.join()

    def test_stop_closes_motor_gate_even_when_save_is_busy(self):
        with self.session.lock, self.assertRaises(manager.RequestError):
            self.session.command({"action": "stop"})
        self.ros.halt.assert_called_once()

    def test_missing_map_does_not_stop_current_session(self):
        with self.assertRaises(manager.RequestError):
            self.session.command({"action": "activate", "name": "missing"})
        self.ros.halt.assert_not_called()

    def test_process_exit_stops_motion_and_reports_failure(self):
        self.session.process = self.process
        self.process.poll.return_value = 1
        self.assertEqual(self.session.status()["mode"], "error")
        self.ros.halt.assert_called()
        self.assertEqual(self.ros.mode, "idle")

    def test_stop_terminates_only_owned_process_group(self):
        self.session.process = self.process
        with patch.object(manager.os, "killpg", side_effect=[None, ProcessLookupError]) as kill:
            self.session.stop()
        self.assertEqual(kill.call_args_list[0].args, (123456, manager.signal.SIGINT))
        self.assertIsNone(self.session.process)


class MotionTests(unittest.TestCase):
    def bridge(self):
        bridge = manager.RosBridge.__new__(manager.RosBridge)
        bridge.motion_lock = threading.RLock()
        bridge.stop_generation = 0
        bridge.mode = "navigation"
        bridge.velocity = Mock()
        bridge.Twist = lambda: type(
            "Twist",
            (),
            {
                "linear": type("Vector", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                "angular": type("Vector", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
            },
        )()
        bridge.status = lambda: {"ready": True, "sensorsReady": True}
        bridge.moving = True
        bridge.manual_until = 0
        bridge.last_command = time.monotonic()
        bridge.cancel_client = Mock()
        bridge.CancelGoal = Mock()
        return bridge

    def test_navigation_never_publishes_lateral_motion_and_clamps_speed(self):
        bridge = self.bridge()
        command = bridge.Twist()
        command.linear.x, command.linear.y, command.angular.z = 5.0, 4.0, 3.0
        bridge.on_velocity(command)
        output = bridge.velocity.publish.call_args.args[0]
        self.assertEqual((output.linear.x, output.linear.y, output.angular.z), (0.2, 0.0, 0.8))

    def test_cancelled_session_discards_late_velocity(self):
        bridge = self.bridge()
        bridge.halt()
        bridge.velocity.reset_mock()
        bridge.on_velocity(bridge.Twist())
        bridge.velocity.publish.assert_not_called()

    def test_manual_command_expires_without_browser(self):
        bridge = self.bridge()
        bridge.manual_until = time.monotonic() - 1
        bridge.watchdog()
        self.assertEqual(bridge.manual_until, 0)
        self.assertEqual(bridge.velocity.publish.call_args.args[0].linear.x, 0)

    def test_sensor_loss_stops_and_cancels_navigation(self):
        bridge = self.bridge()
        bridge.status = lambda: {"ready": False, "sensorsReady": False}
        bridge.watchdog()
        self.assertFalse(bridge.moving)
        bridge.cancel_client.call_async.assert_called_once()

    def test_non_finite_motion_is_rejected(self):
        for value in [float("nan"), float("inf"), True, "0.1", 10]:
            with self.subTest(value=value), self.assertRaises(manager.RequestError):
                manager.finite_number({"linear": value}, "linear", 0.15)


if __name__ == "__main__":
    unittest.main()
