"""Controller-Datenfluss/Shutdown mit Fakes; keine echten Fahrbefehle."""

from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from src.exploration.explore import ExplorationSession, LIDAR_TOPIC, POSE_TOPIC, run_exploration
from src.memory.spatial import SpatialMemory
from test_localization import pose_message


def scan_message(timestamp=10, ranges=None):
    header = pose_message(timestamp).header
    return SimpleNamespace(header=header, HasField=lambda name: name == "header",
                           ranges=[5.0] * 361 if ranges is None else ranges,
                           range_min=0.1, range_max=10, angle_min=-math.pi,
                           angle_step=math.pi / 180)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.move = Mock()
        self.memory = SpatialMemory()
        self.session = ExplorationSession(self.memory, self.move, clock=lambda: self.now)

    def send_valid(self, stamp=10):
        self.session.on_pose(pose_message(stamp, x=0.1, y=0.1))
        self.session.on_scan(scan_message(stamp))

    def test_waits_for_both_sensors(self):
        self.session.tick()
        self.move.assert_called_with(0, 0)
        self.session.on_scan(scan_message())
        self.session.tick()
        self.move.assert_called_with(0, 0)
        self.session.on_pose(pose_message(x=0.1, y=0.1))
        self.session.tick()
        self.move.assert_called_with(0.15, 0)

    def test_memory_updates_from_pose_and_coverage(self):
        self.send_valid()
        self.session.on_pose(pose_message(10.1, x=0.6, y=0.1))
        self.assertEqual(self.memory.visited_cells, {(0, 0), (1, 0)})
        self.assertEqual(self.memory.coverage, 2 / 400)

    def test_invalid_scan_stops_old_command_and_recovers(self):
        self.send_valid()
        self.session.tick()
        self.session.on_scan(scan_message(10.1, []))
        self.move.assert_called_with(0, 0)
        self.session.tick()
        self.move.assert_called_with(0, 0)
        self.send_valid(10.2)
        self.session.tick()
        self.move.assert_called_with(0.15, 0)

    def test_nan_front_scan_stops(self):
        self.send_valid()
        values = [5.0] * 361
        values[180] = math.nan
        self.session.on_scan(scan_message(10.1, values))
        self.session.tick()
        self.move.assert_called_with(0, 0)

    def test_invalid_missing_and_out_of_bounds_pose_stop(self):
        for message in (pose_message(x=math.nan), pose_message(x=5),
                        pose_message(name="another_model")):
            self.send_valid()
            self.session.on_pose(message)
            self.move.assert_called_with(0, 0)
            self.session.tick()
            self.move.assert_called_with(0, 0)

    def test_scan_timeout_with_fresh_pose(self):
        self.send_valid()
        self.now = 1.01
        self.session.on_pose(pose_message(10.1, x=0.1, y=0.1))
        self.session.tick()
        self.move.assert_called_with(0, 0)

    def test_pose_timeout_with_fresh_scan(self):
        self.send_valid()
        self.now = 1.01
        self.session.on_scan(scan_message(10.1))
        self.session.tick()
        self.move.assert_called_with(0, 0)

    def test_repeated_old_messages_do_not_extend_timeout(self):
        self.send_valid()
        self.now = 1.01
        self.send_valid()
        self.session.tick()
        self.move.assert_called_with(0, 0)

    def test_simulation_time_mismatch_stops(self):
        self.send_valid()
        self.session.on_pose(pose_message(11, x=0.1, y=0.1))
        self.session.tick()
        self.move.assert_called_with(0, 0)

    def test_time_reset_stops_and_requires_restart(self):
        self.send_valid()
        self.session.on_pose(pose_message(0))
        self.session.tick()
        self.move.assert_called_with(0, 0)
        self.assertIsNotNone(self.session.error)

    def test_close_stops_and_late_callbacks_cannot_restart(self):
        self.send_valid()
        self.session.tick()
        self.session.close()
        self.move.assert_called_with(0, 0)
        count = self.move.call_count
        cells = self.memory.visited_cells
        self.session.on_pose(pose_message(11, x=2, y=2))
        self.session.on_scan(scan_message(11))
        self.session.tick()
        self.assertEqual(self.move.call_count, count)
        self.assertEqual(self.memory.visited_cells, cells)

    def test_snapshot_is_independent_for_disk_writing(self):
        self.send_valid()
        snapshot, _, _, _ = self.session.snapshot()
        snapshot.visit(2, 2)
        self.assertEqual(self.memory.count, 1)
        self.assertEqual(snapshot.count, 2)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.filename = Path(self.directory.name) / "memory.json"

    def run_fake(self, subscribe_ok=True, invalid_file=False):
        node = Mock()
        move = Mock()
        callbacks = {}
        def subscribe(message_type, topic, callback):
            callbacks[topic] = callback
            if topic == POSE_TOPIC:
                callback(pose_message(x=0.1, y=0.1))
                return True
            callback(scan_message())
            return subscribe_ok
        node.subscribe.side_effect = subscribe
        modules = {"gz.transport13": SimpleNamespace(Node=lambda: node),
                   "gz.msgs10.pose_v_pb2": SimpleNamespace(Pose_V=object),
                   "gz.msgs10.laserscan_pb2": SimpleNamespace(LaserScan=object),
                   "src.navigation.movement": SimpleNamespace(move=move)}
        # Separat getestetes tick(); dieser Test prüft Runner-Lebenszyklus/Datei-I/O.
        thread = Mock(ident=None)
        real_save = SpatialMemory.save
        def save(memory, filename):
            self.assertEqual(move.call_args.args, (0, 0), "Stop muss vor finalem Speichern kommen")
            real_save(memory, filename)
        with patch.dict("sys.modules", modules), redirect_stdout(io.StringIO()), \
                patch("src.exploration.explore.threading.Thread", return_value=thread), \
                patch("src.exploration.explore.time.sleep", side_effect=KeyboardInterrupt), \
                patch.object(SpatialMemory, "save", autospec=True, side_effect=save):
            result = run_exploration(self.filename)
        return result, move, node, callbacks

    def test_loads_old_cells_ctrl_c_stops_and_saves_new_cells(self):
        memory = SpatialMemory()
        memory.visit(-1, -1)
        memory.save(self.filename)
        result, move, node, callbacks = self.run_fake()
        self.assertEqual(result.visited_cells, {(-2, -2), (0, 0)})
        self.assertEqual(SpatialMemory.load(self.filename).visited_cells, result.visited_cells)
        move.assert_called_with(0, 0)
        self.assertEqual(node.unsubscribe.call_count, 2)
        count = move.call_count
        callbacks[LIDAR_TOPIC](scan_message(11))
        self.assertEqual(move.call_count, count)

    def test_subscription_failure_still_stops(self):
        with self.assertRaisesRegex(RuntimeError, "abonniert"):
            self.run_fake(subscribe_ok=False)
        self.assertTrue(self.filename.exists())

    def test_corrupt_memory_not_overwritten(self):
        self.filename.write_text("broken json")
        with self.assertRaises(ValueError):
            self.run_fake()
        self.assertEqual(self.filename.read_text(), "broken json")

    def test_bad_duration_rejected(self):
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                run_exploration(self.filename, duration=value)


if __name__ == "__main__":
    unittest.main()
