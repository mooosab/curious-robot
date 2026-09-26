"""Entscheidungslogik testen, ohne Fahrbefehle oder Gazebo-Prozesse."""

import math
import io
import threading
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace
from unittest.mock import Mock, patch

from src.navigation.navigation import choose_motion, main
from src.perception.lidar import analyze_scan


def scan_results(front=5.0, left=5.0, right=5.0):
    ranges = [5.0] * 361
    ranges[158:203] = [front] * 45
    ranges[203:248] = [left] * 45
    ranges[113:158] = [right] * 45
    return analyze_scan(ranges, 0.1, 10)


class NavigationTests(unittest.TestCase):
    def test_clear_path_moves_forward(self):
        self.assertEqual(choose_motion(scan_results()), (0.15, 0.0, None))

    def test_blocked_front_turns_without_forward_motion(self):
        self.assertEqual(choose_motion(scan_results(0.9, 4, 2)), (0.0, 0.5, "left"))
        self.assertEqual(choose_motion(scan_results(0.9, 2, 4)), (0.0, -0.5, "right"))

    def test_one_meter_boundary(self):
        # Ohne laufendes Ausweichmanöver: nur unter 1.0 m ausweichen.
        self.assertEqual(choose_motion(scan_results(0.99)), (0.0, -0.5, "right"))
        self.assertEqual(choose_motion(scan_results(1.0)), (0.15, 0.0, None))
        self.assertEqual(choose_motion(scan_results(1.01)), (0.15, 0.0, None))

    def test_equal_sides_have_consistent_direction(self):
        self.assertEqual(choose_motion(scan_results(0.9, 3, 3)), (0.0, -0.5, "right"))

    def test_direction_is_retained_when_side_distances_change(self):
        self.assertEqual(choose_motion(scan_results(0.9, 2, 4), "left"),
                         (0.0, 0.5, "left"))

    def test_hysteresis_prevents_front_threshold_oscillation(self):
        direction = "left"
        for distance in [0.99, 1.0, 1.01, 0.98, 1.2, 1.29]:
            linear, angular, direction = choose_motion(scan_results(distance), direction)
            self.assertEqual((linear, angular, direction), (0.0, 0.5, "left"))
        self.assertEqual(choose_motion(scan_results(1.3), direction), (0.15, 0.0, None))

    def test_close_diagonal_blocks_forward_motion(self):
        self.assertEqual(choose_motion(scan_results(4, 0.4, 3)), (0.0, -0.5, "right"))
        self.assertEqual(choose_motion(scan_results(4, 3, 0.4)), (0.0, 0.5, "left"))

    def test_turn_does_not_end_with_diagonal_too_close(self):
        self.assertEqual(choose_motion(scan_results(4, 0.7, 3), "right"),
                         (0.0, -0.5, "right"))
        self.assertEqual(choose_motion(scan_results(4, 0.8, 3), "right"),
                         (0.15, 0.0, None))

    def test_unknown_front_or_diagonal_stops(self):
        for name in ("Vorne", "Vorne-links", "Vorne-rechts"):
            with self.subTest(sector=name):
                results = scan_results()
                results[name] = {"distance": None, "obstacle": None, "invalid_count": 45}
                self.assertEqual(choose_motion(results, "left"), (0.0, 0.0, "left"))

    def test_partial_invalid_data_stops_even_with_known_obstacle(self):
        results = scan_results(0.9, 4, 3)
        results["Vorne-links"]["invalid_count"] = 1
        self.assertEqual(choose_motion(results), (0.0, 0.0, None))

    def test_no_return_is_not_invalid(self):
        self.assertEqual(choose_motion(scan_results(math.inf, math.inf, math.inf)),
                         (0.15, 0.0, None))

    def run_live_loop(self, scan, clock_values):
        """Transport ersetzen, aber den echten Callback und die Schleife ausführen."""
        node = Mock()
        movement = SimpleNamespace(move=Mock(), stop=Mock())
        callbacks = []

        def subscribe(message_type, topic, callback):
            callbacks.append(callback)
            callback(scan)
            return True

        node.subscribe.side_effect = subscribe
        modules = {
            "gz.transport13": SimpleNamespace(Node=lambda: node),
            "gz.msgs10.laserscan_pb2": SimpleNamespace(LaserScan=object),
            "src.navigation.movement": movement,
        }
        with patch.dict("sys.modules", modules), redirect_stdout(io.StringIO()), \
                patch("src.navigation.navigation.time.monotonic", side_effect=clock_values), \
                patch("src.navigation.navigation.time.sleep", side_effect=[None, KeyboardInterrupt]):
            main()
        return movement, callbacks[0]

    def test_scan_timeout_and_shutdown_stop(self):
        scan = SimpleNamespace(ranges=[5.0] * 361, range_min=0.1, range_max=10,
                               angle_min=-math.pi, angle_step=math.pi / 180)
        movement, callback = self.run_live_loop(scan, [10.0, 11.1])
        movement.move.assert_called_once_with(0.15, 0.0)
        # Start, Timeout und finally senden jeweils Stopp.
        self.assertEqual(movement.stop.call_count, 3)
        callback(scan)
        movement.move.assert_called_once_with(0.15, 0.0)

    def test_invalid_scan_sends_stop_instead_of_leaving_old_command(self):
        scan = SimpleNamespace(ranges=[], range_min=0.1, range_max=10,
                               angle_min=-math.pi, angle_step=math.pi / 180)
        movement, _ = self.run_live_loop(scan, [10.0, 10.1])
        movement.move.assert_not_called()
        self.assertEqual(movement.stop.call_count, 3)

    def test_terminal_output_never_holds_command_lock(self):
        # Wenn stdout blockiert, muss die Timeout-Schleife den Lock bekommen.
        for ranges in ([5.0] * 361, []):
            command_lock = threading.Lock()

            def check_output(*args, **kwargs):
                acquired = command_lock.acquire(blocking=False)
                self.assertTrue(acquired, "Terminalausgabe blockiert den Timeout-Lock")
                if acquired:
                    command_lock.release()

            scan = SimpleNamespace(ranges=ranges, range_min=0.1, range_max=10,
                                   angle_min=-math.pi, angle_step=math.pi / 180)
            with self.subTest(valid=bool(ranges)), \
                    patch("src.navigation.navigation.threading.Lock", return_value=command_lock), \
                    patch("builtins.print", side_effect=check_output):
                self.run_live_loop(scan, [10.0, 11.1])

    def test_invalid_scan_values_preserve_turn_and_stop(self):
        for invalid in (math.nan, -math.inf, 0, None):
            with self.subTest(invalid=invalid):
                self.assertEqual(choose_motion(scan_results(left=invalid), "left"),
                                 (0.0, 0.0, "left"))
        # Nach zuverlässigen Daten dieselbe Richtung weiterfahren.
        self.assertEqual(choose_motion(scan_results(0.9, 2, 4), "left"),
                         (0.0, 0.5, "left"))


if __name__ == "__main__":
    unittest.main()
