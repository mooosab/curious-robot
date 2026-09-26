"""Nur Pose empfangen: python3 -m src.localization.record --output recordings/run.csv"""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import threading
import time

from .path import CSV_FIELDS, PathTracker, write_pose
from .pose import extract_model_pose, require_finite


POSE_TOPIC = "/world/basic_world/dynamic_pose/info"


def record_path(output, interval=0.2, duration=None, topic=POSE_TOPIC,
                model_name="curious_robot"):
    """CSV direkt schreiben: konstanter RAM-Bedarf, keine Fahrbefehle.

    Sampling nutzt Simulationszeit; die optionale Laufzeit nutzt Wanduhrzeit.
    Ungültige Nachrichten werden gezählt und übersprungen. Ein Zeitrücksprung
    oder Schreibfehler beendet die Aufnahme mit Fehlermeldung.
    """
    tracker = PathTracker(interval)
    if duration is not None:
        require_finite(duration)
        if duration <= 0:
            raise ValueError("Dauer muss größer als null sein")

    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V

    node = Node()
    lock = threading.Lock()
    running = True
    failure = None
    rejected = 0
    last_received = None
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Vorhandene Fahrten niemals still überschreiben.
    with output.open("x", newline="", encoding="utf-8") as destination:
        writer = csv.writer(destination)
        writer.writerow(CSV_FIELDS)
        destination.flush()

        def callback(message):
            nonlocal failure, rejected, last_received
            with lock:
                if not running or failure is not None:
                    return
                try:
                    pose = extract_model_pose(message, model_name)
                except ValueError:
                    rejected += 1
                    return
                if pose is None:
                    return
                try:
                    sampled = tracker.add(pose)
                    last_received = time.monotonic()
                    if sampled is not None:
                        write_pose(writer, sampled)
                        destination.flush()
                except (ValueError, OSError) as error:
                    failure = str(error)

        subscribed = False
        start = time.monotonic()
        try:
            subscribed = node.subscribe(Pose_V, topic, callback)
            if not subscribed:
                raise RuntimeError(f"Pose-Topic konnte nicht abonniert werden: {topic}")
            print(f"Aufnahme: {output}\nTopic: {topic}\nBeenden mit Ctrl+C.")
            next_status = start
            while duration is None or time.monotonic() - start < duration:
                now = time.monotonic()
                with lock:
                    current, count = tracker.latest, tracker.count
                    age = None if last_received is None else now - last_received
                    error, invalid = failure, rejected
                if error is not None:
                    raise RuntimeError(error)
                if now >= next_status:
                    if current is None:
                        print(f"Warte auf gültige Modellpose ... verworfen: {invalid}")
                    else:
                        print(f"t={current.timestamp:.3f} s (Simulation) | "
                              f"x={current.x:.3f} m y={current.y:.3f} m "
                              f"yaw={math.degrees(current.yaw):.1f}° | "
                              f"{count} Punkte | Empfang vor {age:.1f} s | "
                              f"verworfen: {invalid}")
                    next_status = now + 1
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                with lock:
                    running = False
                    if failure is None:
                        final_pose = tracker.finish()
                        if final_pose is not None:
                            write_pose(writer, final_pose)
                        destination.flush()
            finally:
                # Außerhalb des Locks: unsubscribe kann auf Callbacks warten.
                if subscribed:
                    node.unsubscribe(topic)

    if failure is not None:
        raise RuntimeError(failure)
    if tracker.count == 0:
        raise RuntimeError("Keine gültige Modellpose empfangen; CSV enthält nur den Header")
    print(f"{tracker.count} Punkte gespeichert: {output}")
    return tracker.count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("recordings") /
                        f"path-{datetime.now():%Y%m%d-%H%M%S-%f}.csv")
    parser.add_argument("--interval", type=float, default=0.2,
                        help="Sampling in Simulationssekunden (Standard: 0.2)")
    parser.add_argument("--duration", type=float, help="Laufzeit in Wanduhrsekunden")
    parser.add_argument("--topic", default=POSE_TOPIC)
    parser.add_argument("--model", default="curious_robot")
    args = parser.parse_args()
    try:
        record_path(args.output, args.interval, args.duration, args.topic, args.model)
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        parser.exit(1, f"Aufnahme fehlgeschlagen: {error}\n")


if __name__ == "__main__":
    main()
