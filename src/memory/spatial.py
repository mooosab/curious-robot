"""Einfaches Welt-Raster mit besuchten Zellen und JSON-Persistenz."""

import json
import math
import os
from pathlib import Path
import tempfile

from src.localization.pose import require_finite


DEFAULT_CELL_SIZE = 0.5
# basic_world.sdf: Wandmitten +/-5 m, Wandstärke 0.2 m -> innen +/-4.9 m.
# Reihenfolge: xmin, xmax, ymin, ymax. Obere Grenzen sind exklusiv.
DEFAULT_BOUNDS = (-4.9, 4.9, -4.9, 4.9)
FORMAT = "curious-robot-visited-cells"


def _unique_json_fields(pairs):
    data = {}
    for key, value in pairs:
        if key in data:
            raise ValueError(f"Doppeltes JSON-Feld: {key}")
        data[key] = value
    return data


class SpatialMemory:
    """Zellen sind am Weltursprung ausgerichtet, nicht an der unteren Grenze.

    Coverage zählt alle Zellen, die das definierte Rechteck schneiden, auch
    teilweise enthaltene Randzellen. Hindernisse/Robotergröße werden nicht
    abgezogen. Konfiguration und visited_cells sind von außen nur lesbar.
    """

    def __init__(self, cell_size=DEFAULT_CELL_SIZE, bounds=DEFAULT_BOUNDS):
        require_finite(cell_size)
        if cell_size <= 0:
            raise ValueError("Zellgröße muss positiv sein")
        if not isinstance(bounds, (tuple, list)) or len(bounds) != 4:
            raise ValueError("Grenzen benötigen xmin, xmax, ymin, ymax")
        require_finite(*bounds)
        xmin, xmax, ymin, ymax = bounds
        if xmin >= xmax or ymin >= ymax:
            raise ValueError("Weltgrenzen müssen ein nichtleeres Rechteck bilden")
        scaled = [value / cell_size for value in bounds]
        require_finite(*scaled)
        self._cell_size = cell_size
        self._bounds = tuple(bounds)
        self._min_cell = (math.floor(scaled[0]), math.floor(scaled[2]))
        self._max_cell = (math.ceil(scaled[1]) - 1, math.ceil(scaled[3]) - 1)
        self._visited_cells = set()

    @property
    def cell_size(self):
        return self._cell_size

    @property
    def bounds(self):
        return self._bounds

    @property
    def visited_cells(self):
        return frozenset(self._visited_cells)

    @property
    def count(self):
        return len(self._visited_cells)

    @property
    def total_cells(self):
        return ((self._max_cell[0] - self._min_cell[0] + 1)
                * (self._max_cell[1] - self._min_cell[1] + 1))

    @property
    def coverage(self):
        return self.count / self.total_cells

    def world_to_cell(self, x, y):
        require_finite(x, y)
        xmin, xmax, ymin, ymax = self.bounds
        if not (xmin <= x < xmax and ymin <= y < ymax):
            raise ValueError("Position liegt außerhalb der Weltgrenzen")
        return math.floor(x / self.cell_size), math.floor(y / self.cell_size)

    def _validate_cell(self, cell_x, cell_y):
        if type(cell_x) is not int or type(cell_y) is not int:
            raise ValueError("Zellindizes müssen ganze Zahlen sein")
        if not (self._min_cell[0] <= cell_x <= self._max_cell[0]
                and self._min_cell[1] <= cell_y <= self._max_cell[1]):
            raise ValueError("Zelle liegt außerhalb des definierten Rasters")

    def visit_cell(self, cell_x, cell_y):
        """True genau beim ersten Besuch; doppelte Besuche ändern nichts."""
        self._validate_cell(cell_x, cell_y)
        cell = (cell_x, cell_y)
        if cell in self._visited_cells:
            return False
        self._visited_cells.add(cell)
        return True

    def was_cell_visited(self, cell_x, cell_y):
        self._validate_cell(cell_x, cell_y)
        return (cell_x, cell_y) in self._visited_cells

    def visit(self, x, y):
        return self.visit_cell(*self.world_to_cell(x, y))

    def was_visited(self, x, y):
        return self.was_cell_visited(*self.world_to_cell(x, y))

    def save(self, filename):
        """Erst temporär vollständig schreiben, dann die Zieldatei ersetzen.

        Nur ein schreibender Prozess pro Datei wird unterstützt.
        """
        filename = Path(filename)
        filename.parent.mkdir(parents=True, exist_ok=True)
        data = {"format": FORMAT, "version": 1, "cell_size": self.cell_size,
                "bounds": self.bounds, "visited_cells": sorted(self._visited_cells)}
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8",
                                             dir=filename.parent, prefix=filename.name + ".",
                                             suffix=".tmp", delete=False) as destination:
                temporary = Path(destination.name)
                json.dump(data, destination, indent=2, allow_nan=False)
                destination.write("\n")
                destination.flush()
                os.fsync(destination.fileno())
            os.replace(temporary, filename)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    @classmethod
    def load(cls, filename, cell_size=DEFAULT_CELL_SIZE, bounds=DEFAULT_BOUNDS):
        """Fehlende Datei -> leeres Memory; falsche Datei -> sichtbarer Fehler.

        Erwartete Konfiguration explizit angeben, falls sie vom Standard
        abweicht. Vorhandene Zellen werden niemals still neu interpretiert.
        """
        memory = cls(cell_size, bounds)
        try:
            with open(filename, encoding="utf-8") as source:
                data = json.load(source, object_pairs_hook=_unique_json_fields)
        except FileNotFoundError:
            return memory
        except (json.JSONDecodeError, UnicodeError) as error:
            raise ValueError(f"Beschädigte Memory-Datei: {error}") from error

        expected_keys = {"format", "version", "cell_size", "bounds", "visited_cells"}
        if not isinstance(data, dict) or set(data) != expected_keys:
            raise ValueError("Ungültiges Memory-Dateiformat")
        if data["format"] != FORMAT or type(data["version"]) is not int or data["version"] != 1:
            raise ValueError("Inkompatibles Memory-Dateiformat oder Version")
        saved_config = cls(data["cell_size"], data["bounds"])
        if saved_config.cell_size != memory.cell_size or saved_config.bounds != memory.bounds:
            raise ValueError("Gespeicherte Zellgröße/Weltgrenzen passen nicht zur Konfiguration")
        if not isinstance(data["visited_cells"], list):
            raise ValueError("visited_cells muss eine Liste sein")
        for cell in data["visited_cells"]:
            if not isinstance(cell, list) or len(cell) != 2:
                raise ValueError("Ungültiger Zelleintrag")
            if not memory.visit_cell(*cell):
                raise ValueError("Doppelter Zelleintrag in der Memory-Datei")
        return memory
