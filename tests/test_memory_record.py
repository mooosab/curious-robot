"""Memory-Runner mit synthetischen Posen/Transport und kontrollierter Uhr."""

from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from src.memory.record import record_memory
from src.memory.spatial import SpatialMemory
from test_localization import pose_message


class MemoryRecorderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.filename = Path(self.directory.name) / "memory.json"

    def run_recorder(self, messages=(), steps=(), subscribe_ok=True):
        node = Mock()
        clock = [0.0]
        steps = iter(steps)
        saves = []
        real_save = SpatialMemory.save

        def subscribe(message_type, topic, callback):
            node.callback = callback
            if subscribe_ok:
                for message in messages:
                    callback(message)
            return subscribe_ok

        def sleep(seconds):
            try:
                clock[0], updates = next(steps)
            except StopIteration:
                raise KeyboardInterrupt
            for message in updates:
                node.callback(message)

        def save(memory, filename):
            saves.append(memory.count)
            real_save(memory, filename)

        node.subscribe.side_effect = subscribe
        modules = {"gz.transport13": SimpleNamespace(Node=lambda: node),
                   "gz.msgs10.pose_v_pb2": SimpleNamespace(Pose_V=object)}
        output = io.StringIO()
        with patch.dict("sys.modules", modules), redirect_stdout(output), \
                patch("src.memory.record.time.monotonic", side_effect=lambda: clock[0]), \
                patch("src.memory.record.time.sleep", side_effect=sleep), \
                patch.object(SpatialMemory, "save", autospec=True, side_effect=save):
            memory = record_memory(self.filename)
        node.advertise.assert_not_called()
        return memory, node, saves, output.getvalue()

    def test_ctrl_c_saves_and_restart_loads_existing_visits(self):
        memory, node, saves, _ = self.run_recorder([pose_message(x=-0.1, y=0.1)])
        self.assertEqual(memory.visited_cells, {(-1, 0)})
        self.assertEqual(saves, [1])
        node.unsubscribe.assert_called_once()
        _, _, saves, output = self.run_recorder([pose_message(x=1.1, y=0.1)])
        self.assertIn("geladen: 1", output)
        self.assertEqual(SpatialMemory.load(self.filename).visited_cells, {(-1, 0), (2, 0)})
        self.assertEqual(saves, [2])

    def test_periodic_saves_only_when_dirty_and_final_unsaved_changes(self):
        messages = [pose_message(x=0.1, y=0.1)] * 50
        steps = [(5, []), (10, messages), (11, [pose_message(x=0.6, y=0.1)])]
        _, _, saves, _ = self.run_recorder(messages, steps)
        self.assertEqual(saves, [1, 2])

    def test_invalid_outside_and_wrong_model_are_not_recorded(self):
        messages = [pose_message(x=math.nan), pose_message(x=math.inf),
                    pose_message(x=10), pose_message(name="chassis"),
                    pose_message(x=0.1, y=0.1)]
        memory, _, _, output = self.run_recorder(messages)
        self.assertEqual(memory.count, 1)
        self.assertIn("verworfen: 3", output)

    def test_late_callback_is_ignored(self):
        memory, node, _, _ = self.run_recorder([pose_message(x=0.1, y=0.1)])
        node.callback(pose_message(x=1.1, y=1.1))
        self.assertEqual(memory.count, 1)
        self.assertEqual(SpatialMemory.load(self.filename).count, 1)

    def test_no_pose_preserves_memory_and_reports_missing_reception(self):
        memory = SpatialMemory()
        memory.visit(1, 1)
        memory.save(self.filename)
        _, _, saves, output = self.run_recorder()
        self.assertEqual(saves, [])
        self.assertEqual(SpatialMemory.load(self.filename).count, 1)
        self.assertIn("Keine gültige Pose", output)

    def test_simulation_reset_keeps_cumulative_visits(self):
        memory, _, _, _ = self.run_recorder([pose_message(10, x=0.1, y=0.1),
                                            pose_message(0, x=1.1, y=0.1)])
        self.assertEqual(memory.count, 2)

    def test_failed_subscription_does_not_create_memory(self):
        with self.assertRaisesRegex(RuntimeError, "abonniert"):
            self.run_recorder(subscribe_ok=False)
        self.assertFalse(self.filename.exists())

    def test_corrupt_file_is_preserved(self):
        self.filename.write_text("bad json")
        with self.assertRaises(ValueError):
            self.run_recorder()
        self.assertEqual(self.filename.read_text(), "bad json")

    def test_invalid_intervals_are_rejected_before_transport(self):
        for value in (0, -1, math.nan, math.inf):
            for option in ("save_interval", "duration"):
                with self.subTest(option=option, value=value), self.assertRaises(ValueError):
                    record_memory(self.filename, **{option: value})


if __name__ == "__main__":
    unittest.main()
