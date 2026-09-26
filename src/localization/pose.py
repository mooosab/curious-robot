"""Gazebo-Weltpose als x/y/yaw; keine Gazebo-Imports für die Auswertung nötig."""

from dataclasses import dataclass
import math


def require_finite(*values):
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError("Pose-Werte müssen endliche Zahlen sein")


@dataclass(frozen=True)
class Pose2D:
    timestamp: float  # Simulationszeit in Sekunden
    x: float          # Weltkoordinaten in Metern
    y: float
    yaw: float        # Radiant, positiv gegen den Uhrzeigersinn um +Z

    def __post_init__(self):
        require_finite(self.timestamp, self.x, self.y, self.yaw)
        if self.timestamp < 0:
            raise ValueError("Die Simulationszeit darf nicht negativ sein")


def quaternion_to_yaw(x, y, z, w):
    """Heading: 0 zeigt nach Welt-+X, +pi/2 nach Welt-+Y.

    Gazebo liefert Quaternion-Komponenten x/y/z/w. Normalisieren verhindert,
    dass kleine Abweichungen von der Einheitslänge den Winkel verfälschen.
    """
    require_finite(x, y, z, w)
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Quaternion hat keine gültige Länge")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def extract_model_pose(message, model_name="curious_robot"):
    """Liest genau das oberste Modell aus SceneBroadcasters Pose_V.

    In basic_world liegt curious_robot direkt in der Welt. Link-Posen im
    selben Topic sind relativ zu ihrem Elternmodell und werden NICHT benutzt.
    Kein Treffer -> None; fehlende/ungültige Daten -> ValueError.
    """
    try:
        matches = [pose for pose in message.pose if pose.name == model_name]
        if not matches:
            return None
        if len(matches) != 1:
            raise ValueError("Modellname ist im Pose-Topic nicht eindeutig")
        pose = matches[0]
        # Protobuf liefert bei fehlenden Unterfeldern sonst stille Nullwerte.
        if not (message.HasField("header") and message.header.HasField("stamp")
                and pose.HasField("position") and pose.HasField("orientation")):
            raise ValueError("Zeitstempel, Position oder Orientierung fehlen")
        stamp = message.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nsec < 1_000_000_000:
            raise ValueError("Ungültiger Simulationszeitstempel")
        position, rotation = pose.position, pose.orientation
        require_finite(position.x, position.y, position.z)
        return Pose2D(
            stamp.sec + stamp.nsec * 1e-9, position.x, position.y,
            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )
    except (AttributeError, TypeError) as error:
        raise ValueError("Unvollständige Pose-Nachricht") from error
