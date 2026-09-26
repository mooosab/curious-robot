"""Visited Cells und Persistenz ohne Gazebo testen."""

import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from src.memory.spatial import DEFAULT_BOUNDS, SpatialMemory


class SpatialTests(unittest.TestCase):
    def test_positive_coordinates(self):
        memory = SpatialMemory()
        for position, cell in [((0, 0), (0, 0)), ((0.49, 1.1), (0, 2)),
                               ((0.5, 1.5), (1, 3)), ((4.8, 4.8), (9, 9))]:
            with self.subTest(position=position):
                self.assertEqual(memory.world_to_cell(*position), cell)

    def test_negative_coordinates_use_floor(self):
        memory = SpatialMemory()
        for position, cell in [((-0.01, -0.49), (-1, -1)), ((-0.5, -0.5), (-1, -1)),
                               ((-0.51, -1.01), (-2, -3)), ((-4.9, -4.9), (-10, -10))]:
            with self.subTest(position=position):
                self.assertEqual(memory.world_to_cell(*position), cell)

    def test_same_cell_and_different_cells(self):
        memory = SpatialMemory()
        self.assertEqual(memory.world_to_cell(0.01, 0.1), memory.world_to_cell(0.49, 0.4))
        self.assertNotEqual(memory.world_to_cell(0.49, 0.1), memory.world_to_cell(0.5, 0.1))

    def test_visits_queries_and_duplicates(self):
        memory = SpatialMemory()
        self.assertFalse(memory.was_visited(-0.2, 0.1))
        self.assertTrue(memory.visit(-0.2, 0.1))
        self.assertTrue(memory.was_visited(-0.4, 0.3))
        self.assertFalse(memory.visit(-0.4, 0.3))
        self.assertEqual(memory.count, 1)
        self.assertEqual(memory.visited_cells, {(-1, 0)})
        self.assertTrue(memory.visit(0, 0))
        self.assertEqual(memory.count, 2)

    def test_direct_cell_api(self):
        memory = SpatialMemory()
        self.assertTrue(memory.visit_cell(-10, 9))
        self.assertTrue(memory.was_cell_visited(-10, 9))
        self.assertTrue(memory.was_visited(-4.9, 4.8))
        self.assertFalse(memory.visit_cell(-10, 9))

    def test_read_only_configuration_and_cells(self):
        memory = SpatialMemory()
        for field, value in (("cell_size", 1), ("bounds", (0, 1, 0, 1)),
                             ("visited_cells", {(0, 0)})):
            with self.assertRaises(AttributeError):
                setattr(memory, field, value)
        with self.assertRaises(AttributeError):
            memory.visited_cells.add((0, 0))

    def test_default_coverage(self):
        memory = SpatialMemory()
        self.assertEqual(memory.total_cells, 400)
        self.assertEqual(memory.coverage, 0)
        memory.visit(0, 0)
        self.assertEqual(memory.coverage, 1 / 400)
        memory.visit(0.1, 0.1)
        self.assertEqual(memory.coverage, 1 / 400)

    def test_full_coverage_stays_between_zero_and_one(self):
        memory = SpatialMemory(bounds=(-1, 1, -1, 1))
        self.assertEqual(memory.total_cells, 16)
        for x in range(-2, 2):
            for y in range(-2, 2):
                memory.visit_cell(x, y)
                self.assertTrue(0 <= memory.coverage <= 1)
        self.assertEqual(memory.coverage, 1)
        with self.assertRaises(ValueError):
            memory.visit_cell(2, 2)
        self.assertEqual(memory.coverage, 1)

    def test_partial_edge_cells_count_once(self):
        memory = SpatialMemory(bounds=(-0.1, 0.6, -0.1, 0.6))
        self.assertEqual(memory.total_cells, 9)
        for x in (-0.1, 0.1, 0.55):
            for y in (-0.1, 0.1, 0.55):
                memory.visit(x, y)
        self.assertEqual(memory.coverage, 1)

    def test_bounds_are_lower_inclusive_upper_exclusive(self):
        memory = SpatialMemory()
        self.assertEqual(memory.world_to_cell(-4.9, -4.9), (-10, -10))
        self.assertEqual(memory.world_to_cell(math.nextafter(4.9, 0), 0), (9, 0))
        for x, y in ((4.9, 0), (0, 4.9), (-4.91, 0), (0, -4.91), (100, 100)):
            with self.subTest(x=x, y=y), self.assertRaises(ValueError):
                memory.visit(x, y)
        self.assertEqual(memory.count, 0)

    def test_invalid_positions_and_cells(self):
        memory = SpatialMemory()
        for value in (math.nan, math.inf, -math.inf, None, "0", True):
            for position in ((value, 0), (0, value)):
                with self.subTest(position=position), self.assertRaises(ValueError):
                    memory.world_to_cell(*position)
        for cell in ((0.0, 1), (True, 0), (0, "1"), (-11, 0), (10, 0)):
            with self.subTest(cell=cell), self.assertRaises(ValueError):
                memory.visit_cell(*cell)

    def test_invalid_configuration(self):
        for size in (0, -0.5, math.nan, math.inf, True, "0.5"):
            with self.assertRaises(ValueError):
                SpatialMemory(cell_size=size)
        for bounds in ((0, 0, 0, 1), (1, 0, 0, 1), (0, 1, 0, 0),
                       (0, math.inf, 0, 1), (0, True, 0, 1), (0, 1), None):
            with self.assertRaises(ValueError):
                SpatialMemory(bounds=bounds)

    def test_custom_cell_size(self):
        memory = SpatialMemory(cell_size=1, bounds=(-2, 2, -2, 2))
        self.assertEqual(memory.world_to_cell(-0.1, 1.9), (-1, 1))
        self.assertEqual(memory.total_cells, 16)

    def test_default_bounds_match_actual_world_wall_collisions(self):
        root = Path(__file__).resolve().parents[1]
        world = ET.parse(root / "simulation/worlds/basic_world.sdf").getroot().find("world")
        def inner_face(name, axis, direction):
            model = world.find(f"model[@name='{name}']")
            center = float(model.findtext("pose").split()[axis])
            width = float(model.findtext("link/collision/geometry/box/size").split()[axis])
            return center + direction * width / 2
        self.assertEqual(DEFAULT_BOUNDS, (
            inner_face("wall_left", 0, 1), inner_face("wall_right", 0, -1),
            inner_face("wall_front", 1, 1), inner_face("wall_back", 1, -1)))


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.filename = Path(self.directory.name) / "data/memory.json"

    def test_save_structure_and_round_trip(self):
        memory = SpatialMemory()
        memory.visit(-0.1, 0.1)
        memory.visit(1.1, 2.1)
        memory.save(self.filename)
        data = json.loads(self.filename.read_text())
        self.assertEqual(data["version"], 1)
        self.assertEqual(data["cell_size"], 0.5)
        self.assertEqual(data["bounds"], list(DEFAULT_BOUNDS))
        self.assertEqual(data["visited_cells"], [[-1, 0], [2, 4]])
        loaded = SpatialMemory.load(self.filename)
        self.assertIsNot(loaded, memory)
        self.assertTrue(loaded.was_visited(-0.1, 0.1))
        self.assertEqual(loaded.visited_cells, memory.visited_cells)
        self.assertEqual(loaded.coverage, memory.coverage)

    def test_missing_file_returns_empty_memory_without_writing(self):
        self.assertEqual(SpatialMemory.load(self.filename).count, 0)
        self.assertFalse(self.filename.exists())

    def test_empty_memory_round_trip(self):
        SpatialMemory().save(self.filename)
        self.assertEqual(SpatialMemory.load(self.filename).count, 0)

    def test_custom_configuration_and_mismatch(self):
        memory = SpatialMemory(cell_size=1, bounds=(-2, 2, -3, 3))
        memory.visit(-1, -2)
        memory.save(self.filename)
        with self.assertRaisesRegex(ValueError, "Konfiguration"):
            SpatialMemory.load(self.filename)
        loaded = SpatialMemory.load(self.filename, cell_size=1, bounds=(-2, 2, -3, 3))
        self.assertTrue(loaded.was_visited(-1, -2))
        with self.assertRaises(ValueError):
            SpatialMemory.load(self.filename, cell_size=1, bounds=(-2, 2, -2, 2))

    def test_corrupt_json_is_not_silently_replaced(self):
        self.filename.parent.mkdir()
        for content in (b"{broken", b"", b"\xff", b"[]", b"null",
                        b'{"version": 2, "version": 1}'):
            self.filename.write_bytes(content)
            with self.assertRaises(ValueError):
                SpatialMemory.load(self.filename)
            self.assertEqual(self.filename.read_bytes(), content)

    def test_invalid_schema_cells_and_version(self):
        SpatialMemory().save(self.filename)
        valid = json.loads(self.filename.read_text())
        for key, value in (("version", 2), ("version", True), ("format", "other"),
                           ("cell_size", True), ("bounds", [0, 0, 0, 0]),
                           ("visited_cells", {}), ("visited_cells", [[0, 0], [0, 0]]),
                           ("visited_cells", [[True, 0]]), ("visited_cells", [[0.1, 0]]),
                           ("visited_cells", [[10, 0]]), ("visited_cells", [[0]])):
            self.filename.write_text(json.dumps({**valid, key: value}))
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                SpatialMemory.load(self.filename)
        del valid["bounds"]
        self.filename.write_text(json.dumps(valid))
        with self.assertRaises(ValueError):
            SpatialMemory.load(self.filename)

    def test_failed_atomic_replace_preserves_original(self):
        memory = SpatialMemory()
        memory.save(self.filename)
        old = self.filename.read_bytes()
        memory.visit(1, 1)
        with patch("src.memory.spatial.os.replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                memory.save(self.filename)
        self.assertEqual(self.filename.read_bytes(), old)
        self.assertEqual(list(self.filename.parent.iterdir()), [self.filename])


if __name__ == "__main__":
    unittest.main()
