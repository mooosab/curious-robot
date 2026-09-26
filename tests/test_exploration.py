"""Lokale Exploration ohne Gazebo: Geometrie, Score, Zustand und Safety."""

import math
import unittest

from src.exploration.basic import (
    BasicExplorer, CANDIDATE_ANGLES, candidate_scores, lookahead_cells,
    lookahead_points, normalize_angle, novelty_score, preferred_direction,
)
from src.localization.pose import Pose2D
from src.memory.spatial import SpatialMemory
from src.perception.lidar import analyze_scan


def clear_scan(**distances):
    results = analyze_scan([5.0] * 361, 0.1, 10)
    for name, distance in distances.items():
        results[name] = {"distance": distance, "obstacle": distance < 1,
                         "invalid_count": 0}
    return results


def mark_direction(memory, pose, angle):
    for cell in lookahead_cells(memory, pose, angle):
        memory.visit_cell(*cell)


class LookaheadTests(unittest.TestCase):
    def setUp(self):
        self.pose = Pose2D(1, 0.1, 0.1, 0)
        self.memory = SpatialMemory()

    def test_yaw_zero_and_relative_rotation(self):
        self.assertEqual(lookahead_points(self.pose, 0), [(0.6, 0.1), (1.1, 0.1), (1.6, 0.1)])
        point = lookahead_points(self.pose, math.pi / 4)[0]
        self.assertAlmostEqual(point[0], 0.1 + math.sqrt(0.125))
        self.assertAlmostEqual(point[1], 0.1 + math.sqrt(0.125))

    def test_yaw_plus_ninety(self):
        pose = Pose2D(1, 0, 0, math.pi / 2)
        self.assertEqual(lookahead_points(pose, 0), [(0, 0.5), (0, 1), (0, 1.5)])

    def test_negative_heading_and_coordinates(self):
        pose = Pose2D(1, -1, -1, -math.pi / 2)
        self.assertEqual(lookahead_points(pose, 0)[0], (-1, -1.5))
        self.assertEqual(lookahead_cells(self.memory, pose, 0), {(-2, -3), (-2, -4), (-2, -5)})

    def test_normalization_and_pi_axis(self):
        self.assertAlmostEqual(normalize_angle(3 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-5 * math.pi / 2), -math.pi / 2)
        pose = Pose2D(1, 0, 0, math.pi)
        self.assertEqual(lookahead_cells(self.memory, pose, 0), {(-1, 0), (-2, 0), (-3, 0)})
        self.assertEqual(lookahead_points(self.pose, 2 * math.pi), lookahead_points(self.pose, 0))

    def test_invalid_angles_and_distances(self):
        for value in (math.nan, math.inf, None):
            with self.assertRaises(ValueError):
                normalize_angle(value)
        for distance in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                lookahead_points(self.pose, 0, (distance,))

    def test_duplicate_cells_count_once_and_current_cell_is_excluded(self):
        cells = lookahead_cells(self.memory, self.pose, 0, (0.1, 0.5, 0.6, 0.7))
        self.assertEqual(cells, {(1, 0)})
        self.assertEqual(novelty_score(self.memory, [(1, 0), (1, 0)]), 1)

    def test_unknown_partial_and_known_scores(self):
        cells = lookahead_cells(self.memory, self.pose, 0)
        self.assertEqual(novelty_score(self.memory, cells), 3)
        self.memory.visit_cell(1, 0)
        self.assertEqual(novelty_score(self.memory, cells), 2)
        mark_direction(self.memory, self.pose, 0)
        self.assertEqual(novelty_score(self.memory, cells), 0)

    def test_world_boundary_clips_lookahead(self):
        pose = Pose2D(1, 4.2, 0.1, 0)
        self.assertEqual(lookahead_cells(self.memory, pose, 0), {(9, 0)})
        self.assertEqual(lookahead_cells(self.memory, Pose2D(1, 4.8, 0, 0), 0), set())

    def test_no_scores_behind_lidar_obstacle(self):
        results = clear_scan(Vorne=1.4)
        self.assertEqual(candidate_scores(results, self.pose, self.memory)[0], 1)

    def test_deterministic_tie_break(self):
        scores = candidate_scores(clear_scan(), self.pose, self.memory)
        self.assertEqual(list(scores), list(CANDIDATE_ANGLES))
        self.assertEqual(preferred_direction(scores), 0)
        self.assertEqual(preferred_direction({math.pi / 4: 3, -math.pi / 4: 3}), math.pi / 4)
        self.assertIsNone(preferred_direction({}))


class DecisionTests(unittest.TestCase):
    def setUp(self):
        self.pose = Pose2D(1, 0.1, 0.1, 0)
        self.memory = SpatialMemory()
        self.explorer = BasicExplorer()

    def test_clear_unknown_world_starts_forward(self):
        self.assertEqual(self.explorer.command(clear_scan(), self.pose, self.memory), (0.15, 0))

    def test_memory_changes_repeated_route_to_unknown_right(self):
        mark_direction(self.memory, self.pose, 0)
        mark_direction(self.memory, self.pose, math.pi / 4)
        self.assertEqual(self.explorer.command(clear_scan(), self.pose, self.memory), (0, -0.5))

    def test_unknown_left_wins(self):
        mark_direction(self.memory, self.pose, 0)
        mark_direction(self.memory, self.pose, -math.pi / 4)
        self.assertEqual(self.explorer.command(clear_scan(), self.pose, self.memory), (0, 0.5))

    def test_blocked_unknown_right_loses_to_known_safe_left(self):
        mark_direction(self.memory, self.pose, math.pi / 4)
        results = clear_scan(**{"Vorne": 0.9, "Vorne-rechts": 0.9})
        self.assertEqual(self.explorer.command(results, self.pose, self.memory), (0, 0.5))

    def test_all_blocked_stops(self):
        self.assertEqual(self.explorer.command(analyze_scan([0.5] * 361, 0.1, 10),
                                              self.pose, self.memory), (0, 0))

    def test_invalid_front_or_diagonal_always_stops(self):
        for name in ("Vorne", "Vorne-links", "Vorne-rechts"):
            for value in (None, math.nan, -math.inf):
                results = clear_scan()
                results[name]["distance"] = value
                with self.subTest(name=name, value=value):
                    self.assertEqual(self.explorer.command(results, self.pose, self.memory), (0, 0))
            results = clear_scan()
            results[name]["invalid_count"] = 1
            self.assertEqual(self.explorer.command(results, self.pose, self.memory), (0, 0))

    def test_no_return_is_allowed(self):
        self.assertEqual(self.explorer.command(analyze_scan([math.inf] * 361, 0.1, 10),
                                              self.pose, self.memory), (0.15, 0))

    def test_one_meter_boundary_and_diagonal_stop(self):
        # Ohne Neuigkeitsgewinn bleiben die ursprünglichen Vorwärtsschwellen wirksam.
        for angle in CANDIDATE_ANGLES:
            mark_direction(self.memory, self.pose, angle)
        for front, expected_linear in ((0.99, 0), (1.0, 0.15), (1.01, 0.15)):
            explorer = BasicExplorer()
            self.assertEqual(explorer.command(clear_scan(Vorne=front), self.pose, self.memory)[0],
                             expected_linear)
        self.assertEqual(self.explorer.command(clear_scan(**{"Vorne-links": 0.59}),
                                              self.pose, self.memory), (0, 0))

    def test_avoidance_direction_retained_and_hysteresis_preserved(self):
        self.explorer.command(clear_scan(Vorne=0.9), self.pose, self.memory)
        direction = self.explorer.turn_direction
        for front in (1.0, 1.2, 1.29):
            result = self.explorer.command(clear_scan(Vorne=front), self.pose, self.memory)
            self.assertEqual(result[0], 0)
            self.assertEqual(self.explorer.turn_direction, direction)
        self.assertEqual(self.explorer.command(clear_scan(Vorne=1.3), self.pose, self.memory), (0.15, 0))
        self.assertIsNone(self.explorer.turn_direction)

    def test_rear_invalid_or_close_prevents_exploration_rotation(self):
        mark_direction(self.memory, self.pose, 0)
        results = clear_scan(Hinten=0.6)
        self.assertEqual(self.explorer.command(results, self.pose, self.memory), (0.15, 0))
        results["Hinten"]["distance"] = None
        self.assertEqual(self.explorer.command(results, self.pose, self.memory), (0.15, 0))

    def test_heading_target_does_not_chase_changing_scores(self):
        mark_direction(self.memory, self.pose, 0)
        mark_direction(self.memory, self.pose, -math.pi / 4)
        self.explorer.command(clear_scan(), self.pose, self.memory)
        target = self.explorer.target_yaw
        rotated = Pose2D(2, 0.1, 0.1, 0.2)
        mark_direction(self.memory, rotated, math.pi / 4)
        self.assertEqual(self.explorer.command(clear_scan(), rotated, self.memory), (0, 0.5))
        self.assertEqual(self.explorer.target_yaw, target)
        completed = Pose2D(3, 0.1, 0.1, target)
        self.assertEqual(self.explorer.command(clear_scan(), completed, self.memory), (0.15, 0))
        self.assertIsNone(self.explorer.target_yaw)

    def test_safety_interrupts_exploration_target(self):
        mark_direction(self.memory, self.pose, 0)
        self.explorer.command(clear_scan(), self.pose, self.memory)
        command = self.explorer.command(clear_scan(Vorne=0.9), self.pose, self.memory)
        self.assertEqual(command[0], 0)
        self.assertIsNone(self.explorer.target_yaw)
        self.assertIsNotNone(self.explorer.turn_direction)

    def test_no_new_exploration_turn_until_half_meter_progress(self):
        self.explorer.command(clear_scan(), self.pose, self.memory)
        mark_direction(self.memory, self.pose, 0)
        self.assertEqual(self.explorer.command(clear_scan(), self.pose, self.memory), (0.15, 0))

    def test_world_boundaries_prevent_forward_even_with_clear_scan(self):
        pose = Pose2D(1, 4.8, 4.8, math.pi / 4)
        self.assertEqual(self.explorer.command(clear_scan(), pose, self.memory), (0, 0))


if __name__ == "__main__":
    unittest.main()
