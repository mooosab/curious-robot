"""Kleine deterministische Exploration, ohne Gazebo oder Fahrbefehle."""

import math

from src.localization.pose import require_finite
from src.navigation.navigation import (
    choose_motion, TURN_SPEED, FRONT_CLEAR_DISTANCE, DIAGONAL_CLEAR_DISTANCE,
)


# Reihenfolge ist zugleich Tie-Break: geradeaus, links, rechts.
CANDIDATE_ANGLES = (0.0, math.pi / 4, -math.pi / 4)
LOOKAHEAD_DISTANCES = (0.5, 1.0, 1.5)
DECISION_DISTANCE = 0.5
HEADING_TOLERANCE = math.radians(5)
# Scanursprung ist 0.125 m vor Modellursprung; mit Körperradius konservativ runden.
LOOKAHEAD_MARGIN = 0.45
# Drehung um die Radachse x=-0.10: Chassisecke r=hypot(0.35,0.20)=0.403 m.
# LiDAR-Abstand zur Achse 0.225 m -> 0.628 m; 0.70 m inklusive Reserve.
ROTATION_CLEARANCE = 0.70
SECTORS = ("Vorne", "Vorne-links", "Links", "Hinten-links", "Hinten",
           "Hinten-rechts", "Rechts", "Vorne-rechts")


def normalize_angle(angle):
    require_finite(angle)
    return (angle + math.pi) % (2 * math.pi) - math.pi


def lookahead_points(pose, relative_angle, distances=LOOKAHEAD_DISTANCES):
    angle = normalize_angle(pose.yaw + relative_angle)
    cosine, sine = math.cos(angle), math.sin(angle)
    # Exakte Achsenrichtungen nicht durch sin(pi)-Rundungsreste in Nachbarzellen legen.
    cosine = 0.0 if abs(cosine) < 1e-12 else cosine
    sine = 0.0 if abs(sine) < 1e-12 else sine
    points = []
    for distance in distances:
        require_finite(distance)
        if distance <= 0:
            raise ValueError("Lookahead-Distanzen müssen positiv sein")
        points.append((pose.x + distance * cosine, pose.y + distance * sine))
    return points


def lookahead_cells(memory, pose, relative_angle, distances=LOOKAHEAD_DISTANCES):
    """Einzigartige Zellen; keine Bewertung jenseits der Weltgrenze."""
    cells = set()
    current = memory.world_to_cell(pose.x, pose.y)
    for x, y in lookahead_points(pose, relative_angle, distances):
        try:
            cell = memory.world_to_cell(x, y)
        except ValueError:
            break
        if cell != current:
            cells.add(cell)
    return cells


def novelty_score(memory, cells):
    """Ein Punkt pro unterschiedlicher unbesuchter Zelle, sonst null."""
    return sum(not memory.was_cell_visited(*cell) for cell in set(cells))


def valid_sector(sector):
    distance = sector.get("distance")
    return (isinstance(distance, (int, float)) and not isinstance(distance, bool)
            and distance > 0 and not math.isnan(distance)
            and sector.get("invalid_count") == 0
            and sector.get("obstacle") is not None)


def rotation_clear(results):
    # Auch Rückseite/Seiten werden durch eine Drehung bewegt.
    return all(valid_sector(results.get(name, {}))
               and results[name]["distance"] >= ROTATION_CLEARANCE for name in SECTORS)


def direction_sectors(results, angle):
    index = int(math.floor(normalize_angle(angle) / (math.pi / 4) + 0.5)) % 8
    return [results.get(SECTORS[i % 8], {}) for i in (index, index + 1, index - 1)]


def direction_clear(results, angle):
    front, left, right = direction_sectors(results, angle)
    return (all(valid_sector(s) for s in (front, left, right))
            and front["distance"] >= FRONT_CLEAR_DISTANCE
            and min(left["distance"], right["distance"]) >= DIAGONAL_CLEAR_DISTANCE)


def candidate_scores(results, pose, memory, angles=CANDIDATE_ANGLES):
    scores = {}
    can_rotate = rotation_clear(results)
    for angle in angles:
        if not direction_clear(results, angle) or (angle != 0 and not can_rotate):
            continue
        distance = direction_sectors(results, angle)[0]["distance"]
        distances = tuple(d for d in LOOKAHEAD_DISTANCES if d + LOOKAHEAD_MARGIN <= distance)
        cells = lookahead_cells(memory, pose, angle, distances)
        if cells:
            scores[angle] = novelty_score(memory, cells)
    return scores


def preferred_direction(scores):
    # max behält bei Gleichstand den ersten Eintrag der festen Kandidatenreihenfolge.
    return max(scores, key=scores.get) if scores else None


class BasicExplorer:
    """Nur lokaler Zustand: Safety-Drehrichtung, kurzer Heading-Wechsel, Fortschritt."""

    def __init__(self):
        self.turn_direction = None
        self.target_yaw = None
        self.last_decision_position = None
        self.scores = {}
        self.reason = "warte auf Daten"

    def command(self, results, pose, memory):
        previous_turn = self.turn_direction
        # Die bestehende Entscheidung hat VOR jeder Exploration Vorrang.
        front_valid = all(valid_sector(results.get(name, {}))
                          for name in ("Vorne", "Vorne-links", "Vorne-rechts"))
        if not front_valid:
            self.reason = "Stopp: ungültiger Frontscan"
            return 0.0, 0.0
        linear, angular, direction = choose_motion(results, self.turn_direction)
        self.turn_direction = direction
        if angular:
            self.target_yaw = None
            if not rotation_clear(results):
                self.reason = "Stopp: kein ausreichender Drehfreiraum"
                return 0.0, 0.0
            if previous_turn is None:
                self.scores = candidate_scores(results, pose, memory,
                                               angles=CANDIDATE_ANGLES[1:])
                preferred = preferred_direction(self.scores)
                if preferred is not None:
                    self.turn_direction = "left" if preferred > 0 else "right"
            self.reason = f"Safety-Ausweichen {self.turn_direction}"
            return 0.0, TURN_SPEED if self.turn_direction == "left" else -TURN_SPEED
        if not linear:
            self.reason = "Safety-Stopp"
            return 0.0, 0.0
        if previous_turn is not None:
            self.last_decision_position = (pose.x, pose.y)

        if self.target_yaw is not None:
            error = normalize_angle(self.target_yaw - pose.yaw)
            if abs(error) <= HEADING_TOLERANCE:
                self.target_yaw = None
                self.last_decision_position = (pose.x, pose.y)
            elif rotation_clear(results) and direction_clear(results, error):
                self.reason = "Exploration: gewählten Heading-Wechsel beibehalten"
                return 0.0, math.copysign(TURN_SPEED, error)
            else:
                self.target_yaw = None
                self.reason = "Stopp: Exploration-Drehung nicht mehr frei"
                return 0.0, 0.0

        # Keine Fahrt über konfigurierte Grenzen, auch wenn ein Testscan +inf liefert.
        forward = lookahead_points(pose, 0, (DECISION_DISTANCE,))[0]
        try:
            memory.world_to_cell(*forward)
            forward_in_bounds = True
        except ValueError:
            forward_in_bounds = False
        progressed = (self.last_decision_position is None or math.hypot(
            pose.x - self.last_decision_position[0],
            pose.y - self.last_decision_position[1]) >= DECISION_DISTANCE)
        if progressed or not forward_in_bounds:
            self.scores = candidate_scores(results, pose, memory)
            preferred = preferred_direction(self.scores)
            self.last_decision_position = (pose.x, pose.y)
            if preferred is not None and preferred != 0 and self.scores[preferred] > 0:
                self.target_yaw = normalize_angle(pose.yaw + preferred)
                self.reason = f"Exploration: {math.degrees(preferred):+.0f}° | Scores {self.scores}"
                return 0.0, math.copysign(TURN_SPEED, preferred)
        if not forward_in_bounds:
            self.reason = "Stopp: Weltgrenze"
            return 0.0, 0.0
        self.reason = "Safety-freie Vorwärtsfahrt"
        return linear, 0.0
