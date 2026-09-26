"""Visited Cells live speichern: python -m src.memory.record"""

import argparse
from pathlib import Path
import threading
import time

from src.localization.pose import extract_model_pose, require_finite
from src.localization.record import POSE_TOPIC
from .spatial import DEFAULT_BOUNDS, DEFAULT_CELL_SIZE, SpatialMemory


DEFAULT_FILE = Path("data/spatial_memory.json")


def record_memory(filename=DEFAULT_FILE, cell_size=DEFAULT_CELL_SIZE,
                  bounds=DEFAULT_BOUNDS, save_interval=5.0, duration=None,
                  topic=POSE_TOPIC, model_name="curious_robot"):
    """Bestehende Localization auswerten; keinerlei Fahrbefehle senden.

    Jede empfangene Pose markiert ihre Zelle. Speichern höchstens alle fünf
    Wanduhrsekunden, falls neue Zellen vorliegen, und final bei Ctrl+C.
    Besuche bleiben auch über einen Simulationszeit-Reset hinweg erhalten.
    """
    require_finite(save_interval)
    if save_interval <= 0:
        raise ValueError("Speicherintervall muss positiv sein")
    if duration is not None:
        require_finite(duration)
        if duration <= 0:
            raise ValueError("Dauer muss positiv sein")
    filename = Path(filename)
    # Defekte/inkompatible Dateien vor dem Abonnieren ablehnen, niemals leeren.
    memory = SpatialMemory.load(filename, cell_size, bounds)
    dirty = not filename.exists()

    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V

    node = Node()
    lock = threading.Lock()
    running = True
    rejected = 0
    received = 0
    last_received = None

    def callback(message):
        nonlocal dirty, rejected, received, last_received
        with lock:
            if not running:
                return
            try:
                pose = extract_model_pose(message, model_name)
                if pose is None:
                    return
                new_cell = memory.visit(pose.x, pose.y)
            except ValueError:
                rejected += 1
                return
            dirty = dirty or new_cell
            received += 1
            last_received = time.monotonic()

    start = time.monotonic()
    next_save = start + save_interval
    next_status = start
    subscribed = False
    print(f"Memory: {filename} | geladen: {memory.count} besuchte Zellen")
    try:
        subscribed = node.subscribe(Pose_V, topic, callback)
        if not subscribed:
            raise RuntimeError(f"Pose-Topic konnte nicht abonniert werden: {topic}")
        while duration is None or time.monotonic() - start < duration:
            now = time.monotonic()
            with lock:
                if now >= next_save:
                    if dirty:
                        memory.save(filename)
                        dirty = False
                    next_save = now + save_interval
                count, coverage, invalid = memory.count, memory.coverage, rejected
                age = None if last_received is None else now - last_received
            if now >= next_status:
                reception = "warte auf Pose" if age is None else f"Empfang vor {age:.1f} s"
                print(f"Visited cells: {count}/{memory.total_cells} | "
                      f"Coverage: {coverage:.2%} | {reception} | verworfen: {invalid}")
                next_status = now + 2.0
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            with lock:
                running = False
                if subscribed and dirty:
                    memory.save(filename)
        finally:
            if subscribed:
                node.unsubscribe(topic)
    print(f"Memory gespeichert: {memory.count} Zellen, Coverage {memory.coverage:.2%}")
    if received == 0:
        print("Keine gültige Pose empfangen; vorhandene Besuche bleiben erhalten.")
    return memory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE)
    parser.add_argument("--cell-size", type=float, default=DEFAULT_CELL_SIZE)
    parser.add_argument("--bounds", type=float, nargs=4, default=DEFAULT_BOUNDS,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    parser.add_argument("--save-interval", type=float, default=5.0)
    parser.add_argument("--duration", type=float, help="Optional: Wanduhrsekunden bis zum Ende")
    parser.add_argument("--topic", default=POSE_TOPIC)
    parser.add_argument("--model", default="curious_robot")
    args = parser.parse_args()
    try:
        record_memory(args.file, args.cell_size, args.bounds, args.save_interval,
                      args.duration, args.topic, args.model)
    except (ValueError, OSError, RuntimeError, ImportError) as error:
        parser.exit(1, f"Spatial Memory fehlgeschlagen: {error}\n")


if __name__ == "__main__":
    main()
