"""Zeitliches Sampling und einfache CSV-Dateien, ohne Gazebo oder Matplotlib."""

import csv

from .pose import Pose2D, require_finite


CSV_FIELDS = ("timestamp", "x", "y", "yaw")


class PathTracker:
    """Behält nur aktuelle/zuletzt gespeicherte Pose; CSV speichert den Pfad.

    add() liefert nur dann einen Punkt, wenn er geschrieben werden soll.
    Rückwärts laufende Simulationszeit beendet eine Aufnahme beim Aufrufer,
    damit nach einem Welt-Reset keine falsche Verbindung im Pfad entsteht.
    """

    def __init__(self, interval=0.2):
        require_finite(interval)
        if interval <= 0:
            raise ValueError("Sampling-Intervall muss größer als null sein")
        self.interval = interval
        self.latest = None
        self.last_sample = None
        self.count = 0

    def add(self, pose):
        if not isinstance(pose, Pose2D):
            raise ValueError("Eine gültige Pose2D wird benötigt")
        if self.latest is not None:
            if pose.timestamp < self.latest.timestamp:
                raise ValueError("Simulationszeit läuft rückwärts; neue Aufnahme starten")
            if pose.timestamp == self.latest.timestamp:
                return None
        self.latest = pose
        if (self.last_sample is None
                or pose.timestamp - self.last_sample.timestamp >= self.interval - 1e-9):
            self.last_sample = pose
            self.count += 1
            return pose
        return None

    def finish(self):
        """Auch den letzten empfangenen Punkt speichern, falls noch ungesampelt."""
        if self.latest is not None and self.latest != self.last_sample:
            self.last_sample = self.latest
            self.count += 1
            return self.latest
        return None


def write_pose(writer, pose):
    writer.writerow((pose.timestamp, pose.x, pose.y, pose.yaw))


def load_path(filename):
    """CSV für den Offline-Plot laden und fehlerhafte Daten klar ablehnen."""
    points = []
    with open(filename, newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames != list(CSV_FIELDS):
            raise ValueError("CSV erwartet die Spalten timestamp,x,y,yaw")
        for line, row in enumerate(reader, start=2):
            try:
                if None in row:
                    raise ValueError("Zu viele Spalten")
                pose = Pose2D(*(float(row[field]) for field in CSV_FIELDS))
                if points and pose.timestamp <= points[-1].timestamp:
                    raise ValueError("Zeitstempel müssen aufsteigend sein")
            except (ValueError, TypeError) as error:
                raise ValueError(f"Ungültige CSV-Zeile {line}: {error}") from error
            points.append(pose)
    if not points:
        raise ValueError("Die Aufnahme enthält noch keine Pose-Daten")
    return points
