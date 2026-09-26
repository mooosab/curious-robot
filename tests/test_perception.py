"""Tests ohne Gazebo: python3 -m unittest discover -s tests -v"""

import io
import math
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace

from src.perception.lidar import analyze_scan, evaluate_sector, format_scan
from src.perception.test import lidar_callback


class PerceptionTests(unittest.TestCase):
    def test_invalid_values_are_excluded(self):
        invalid = [math.nan, -math.inf, -1, 0, 0.09, 10.01, None, "1", True]
        self.assertEqual(evaluate_sector(invalid + [0.8], 0.1, 10),
                         (0.8, True, len(invalid)))

    def test_no_valid_values_is_unknown(self):
        for values in ([], [math.nan], [-math.inf, 0]):
            with self.subTest(values=values):
                distance, obstacle, _ = evaluate_sector(values, 0.1, 10)
                self.assertIsNone(distance)
                self.assertIsNone(obstacle)

    def test_missing_data_does_not_mean_clear(self):
        self.assertEqual(evaluate_sector([4, math.nan], 0.1, 10), (4, None, 1))
        self.assertEqual(evaluate_sector([math.inf, math.nan], 0.1, 10),
                         (math.inf, None, 1))

    def test_positive_infinity_means_no_return(self):
        self.assertEqual(evaluate_sector([math.inf] * 45, 0.1, 10),
                         (math.inf, False, 0))
        self.assertEqual(evaluate_sector([math.inf, 0.8], 0.1, 10), (0.8, True, 0))

    def test_threshold_and_sensor_boundaries(self):
        for distance, expected in [(0.1, True), (0.99, True), (1.0, False),
                                   (1.01, False), (10, False)]:
            with self.subTest(distance=distance):
                self.assertEqual(evaluate_sector([distance], 0.1, 10),
                                 (distance, expected, 0))

    def test_sector_edges_and_rear_wrap(self):
        cases = {
            "Vorne": [158, 202], "Vorne-links": [203, 247],
            "Links": [248, 292], "Hinten-links": [293, 337],
            "Hinten": [338, 360, 0, 22], "Hinten-rechts": [23, 67],
            "Rechts": [68, 112], "Vorne-rechts": [113, 157],
        }
        for name, indices in cases.items():
            for index in indices:
                with self.subTest(name=name, index=index):
                    values = [5.0] * 361
                    values[index] = 0.5
                    results = analyze_scan(values, 0.1, 10)
                    detected = [key for key, result in results.items() if result["obstacle"]]
                    self.assertEqual(detected, [name])

    def test_wrong_scan_size_is_rejected(self):
        for count in [0, 180, 360, 362]:
            with self.subTest(count=count):
                with self.assertRaises(ValueError):
                    analyze_scan([5] * count, 0.1, 10)

    def test_wrong_metadata_is_rejected(self):
        for minimum, maximum in [(0, 10), (math.nan, 10), (0.1, math.inf),
                                 (10, 0.1), (0.1, 0.5)]:
            with self.subTest(minimum=minimum, maximum=maximum):
                with self.assertRaises(ValueError):
                    analyze_scan([5] * 361, minimum, maximum)
        for start, step in [(0, math.pi / 180), (-math.pi, 0), (-math.pi, math.nan)]:
            with self.assertRaises(ValueError):
                analyze_scan([5] * 361, 0.1, 10, start, step)

    def test_output_shows_unknown_and_all_eight_sectors(self):
        results = analyze_scan([math.nan] * 361, 0.1, 10)
        output = format_scan(results)
        self.assertEqual(output.count("UNBEKANNT"), 8)
        self.assertNotIn("NEIN", output)
        for name in results:
            self.assertIn(name, output)
        self.assertIn("kein Treffer", format_scan(analyze_scan([math.inf] * 361, 0.1, 10)))

    def test_callback_recovers_after_invalid_scan(self):
        message = SimpleNamespace(ranges=[], range_min=0.1, range_max=10,
                                  angle_min=-math.pi, angle_step=math.pi / 180)
        output = io.StringIO()
        with redirect_stdout(output):
            lidar_callback(message)
            message.ranges = [0.8] * 361
            lidar_callback(message)
        self.assertIn("Scan nicht auswertbar", output.getvalue())
        self.assertEqual(output.getvalue().count("| JA"), 8)


if __name__ == "__main__":
    unittest.main()
