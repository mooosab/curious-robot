"""Synthetische Pose-/Transport-Tests: Gazebo muss nicht installiert/laufend sein."""

import csv
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from contextlib import redirect_stdout

from src.localization.pose import Pose2D, extract_model_pose, quaternion_to_yaw
from src.localization.path import CSV_FIELDS, PathTracker, load_path, write_pose
from src.localization.record import POSE_TOPIC, record_path


def pose_message(timestamp=10.0, name="curious_robot", x=1.0, y=2.0, yaw=0.0):
    """Nur die verwendeten Protobuf-Felder nachbilden, inklusive Feldpräsenz."""
    def message(**fields):
        return SimpleNamespace(**fields, HasField=lambda field: field in fields)

    seconds = int(timestamp)
    stamp = message(sec=seconds, nsec=round((timestamp - seconds) * 1e9))
    pose = message(name=name, position=message(x=x, y=y, z=0.25),
                   orientation=message(x=0.0, y=0.0,
                                       z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    return message(header=message(stamp=stamp), pose=[pose])


class PoseTests(unittest.TestCase):
    def test_standard_headings(self):
        for yaw in (0, math.pi / 2, -math.pi / 2, math.pi):
            with self.subTest(yaw=yaw):
                self.assertAlmostEqual(
                    quaternion_to_yaw(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)), yaw)

    def test_normalizes_quaternion_and_accepts_equivalent_negative(self):
        for scale in (1, 2, -1, -3):
            self.assertAlmostEqual(quaternion_to_yaw(0, 0, scale, scale), math.pi / 2)

    def test_heading_with_roll_and_pitch(self):
        roll, pitch, yaw = 0.2, -0.3, 1.1
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        self.assertAlmostEqual(quaternion_to_yaw(
            sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy), yaw)

    def test_invalid_quaternions(self):
        for quaternion in ((0, 0, 0, 0), (0, 0, math.nan, 1),
                           (0, math.inf, 0, 1), (None, 0, 0, 1), (True, 0, 0, 1)):
            with self.subTest(quaternion=quaternion), self.assertRaises(ValueError):
                quaternion_to_yaw(*quaternion)

    def test_extracts_model_not_first_link_and_uses_simulation_time(self):
        message = pose_message(12.25, x=-2, y=3, yaw=math.pi / 2)
        message.pose.insert(0, pose_message(name="chassis", x=99).pose[0])
        pose = extract_model_pose(message)
        self.assertEqual((pose.timestamp, pose.x, pose.y), (12.25, -2, 3))
        self.assertAlmostEqual(pose.yaw, math.pi / 2)

    def test_origin_and_zero_time_are_valid(self):
        self.assertEqual(extract_model_pose(pose_message(0, x=0, y=0)), Pose2D(0, 0, 0, 0))

    def test_absent_or_duplicate_model(self):
        message = pose_message(name="another_robot")
        self.assertIsNone(extract_model_pose(message))
        self.assertEqual(extract_model_pose(message, "another_robot").x, 1)
        message = pose_message()
        message.pose.append(message.pose[0])
        with self.assertRaises(ValueError):
            extract_model_pose(message)

    def test_missing_fields_and_invalid_position(self):
        for target, field in (("message", "header"), ("header", "stamp"),
                              ("pose", "position"), ("pose", "orientation")):
            message = pose_message()
            objects = {"message": message, "header": message.header, "pose": message.pose[0]}
            objects[target].HasField = lambda name, missing=field: name != missing
            with self.subTest(field=field), self.assertRaises(ValueError):
                extract_model_pose(message)
        for value in (math.nan, math.inf, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                extract_model_pose(pose_message(x=value))
        with self.assertRaises(ValueError):
            extract_model_pose(SimpleNamespace())

    def test_invalid_timestamps_and_pose_values(self):
        for sec, nsec in ((-1, 0), (0, -1), (0, 1_000_000_000)):
            message = pose_message()
            message.header.stamp.sec, message.header.stamp.nsec = sec, nsec
            with self.assertRaises(ValueError):
                extract_model_pose(message)
        for values in ((-1, 0, 0, 0), (0, 0, 0, math.nan), (True, 0, 0, 0)):
            with self.assertRaises(ValueError):
                Pose2D(*values)


class PathTests(unittest.TestCase):
    def test_sampling_and_latest_pose(self):
        tracker = PathTracker(0.2)
        first, middle, sampled = (Pose2D(t, t, 0, 0) for t in (1, 1.1, 1.2))
        self.assertEqual(tracker.add(first), first)
        self.assertIsNone(tracker.add(middle))
        self.assertEqual(tracker.latest, middle)
        self.assertEqual(tracker.add(sampled), sampled)
        self.assertEqual(tracker.count, 2)

    def test_rotation_without_translation_is_recorded(self):
        tracker = PathTracker()
        tracker.add(Pose2D(0, 1, 2, 0))
        rotated = Pose2D(0.2, 1, 2, math.pi / 2)
        self.assertEqual(tracker.add(rotated), rotated)

    def test_duplicates_and_time_reset(self):
        tracker = PathTracker()
        first = Pose2D(10, 1, 2, 0)
        tracker.add(first)
        self.assertIsNone(tracker.add(first))
        with self.assertRaisesRegex(ValueError, "rückwärts"):
            tracker.add(Pose2D(0, 0, 0, 0))
        self.assertEqual(tracker.latest, first)
        self.assertEqual(tracker.count, 1)

    def test_finish_includes_last_point_once(self):
        tracker = PathTracker()
        self.assertIsNone(tracker.finish())
        tracker.add(Pose2D(0, 0, 0, 0))
        last = Pose2D(0.1, 1, 0, 0)
        tracker.add(last)
        self.assertEqual(tracker.finish(), last)
        self.assertIsNone(tracker.finish())
        self.assertEqual(tracker.count, 2)

    def test_invalid_interval_and_input(self):
        for interval in (0, -1, math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                PathTracker(interval)
        with self.assertRaises(ValueError):
            PathTracker().add(None)

    def test_csv_round_trip(self):
        points = [Pose2D(0, 1, 2, 0), Pose2D(0.2, -1, 3, -math.pi / 2)]
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.csv"
            with filename.open("w", newline="") as destination:
                writer = csv.writer(destination)
                writer.writerow(CSV_FIELDS)
                for point in points:
                    write_pose(writer, point)
            self.assertEqual(load_path(filename), points)

    def test_invalid_csv(self):
        header = "timestamp,x,y,yaw\n"
        contents = ["", header, "x,y\n1,2\n", header + "0,nan,0,0\n",
                    header + "0,1,2\n", header + "0,1,2,0,99\n",
                    header + "1,0,0,0\n0,0,0,0\n"]
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.csv"
            for content in contents:
                filename.write_text(content)
                with self.subTest(content=content), self.assertRaises(ValueError):
                    load_path(filename)


class RecorderTests(unittest.TestCase):
    def run_recorder(self, filename, messages, subscribe_ok=True):
        node = Mock()

        def subscribe(message_type, topic, callback):
            self.assertEqual(topic, POSE_TOPIC)
            node.callback = callback
            for message in messages:
                callback(message)
            return subscribe_ok

        node.subscribe.side_effect = subscribe
        modules = {"gz.transport13": SimpleNamespace(Node=lambda: node),
                   "gz.msgs10.pose_v_pb2": SimpleNamespace(Pose_V=object)}
        with patch.dict("sys.modules", modules), redirect_stdout(io.StringIO()), \
                patch("src.localization.record.time.sleep", side_effect=KeyboardInterrupt):
            count = record_path(filename)
        return count, node

    def test_receives_samples_saves_final_point_and_ignores_late_callback(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.csv"
            count, node = self.run_recorder(filename, [pose_message(t) for t in (10, 10.1, 10.2, 10.3)])
            self.assertEqual(count, 3)
            self.assertEqual([p.timestamp for p in load_path(filename)], [10, 10.2, 10.3])
            node.unsubscribe.assert_called_once_with(POSE_TOPIC)
            previous = filename.read_text()
            node.callback(pose_message(11))
            self.assertEqual(filename.read_text(), previous)

    def test_recovers_after_invalid_pose_and_ignores_other_models(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.csv"
            count, _ = self.run_recorder(filename, [pose_message(x=math.nan),
                                        pose_message(name="left_wheel"), pose_message()])
            self.assertEqual(count, 1)

    def test_reset_preserves_existing_recording_and_reports_error(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.csv"
            with self.assertRaisesRegex(RuntimeError, "rückwärts"):
                self.run_recorder(filename, [pose_message(10), pose_message(0)])
            self.assertEqual([p.timestamp for p in load_path(filename)], [10])

    def test_no_messages_and_failed_subscription_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            for subscribed, expected in ((True, "Keine gültige"), (False, "abonniert")):
                with self.subTest(subscribed=subscribed), self.assertRaisesRegex(RuntimeError, expected):
                    self.run_recorder(Path(directory) / f"{subscribed}.csv", [], subscribed)

    def test_existing_file_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.csv"
            filename.write_text("existing recording")
            with self.assertRaises(FileExistsError):
                self.run_recorder(filename, [pose_message()])
            self.assertEqual(filename.read_text(), "existing recording")


if __name__ == "__main__":
    unittest.main()
