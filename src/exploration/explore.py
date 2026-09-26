"""Einziger Fahrcontroller: python -m src.exploration.explore"""

import argparse
import copy
from pathlib import Path
import threading
import time

from src.localization.pose import extract_model_pose, require_finite
from src.localization.record import POSE_TOPIC
from src.memory.record import DEFAULT_FILE
from src.memory.spatial import DEFAULT_BOUNDS, DEFAULT_CELL_SIZE, SpatialMemory
from src.navigation.navigation import SCAN_TIMEOUT
from src.perception.lidar import analyze_scan
from .basic import BasicExplorer


LIDAR_TOPIC = "/model/curious_robot/lidar"
POSE_TIMEOUT = 1.0
MAX_SENSOR_SKEW = 0.25  # Simulationssekunden zwischen Scan und Pose
CONTROL_INTERVAL = 0.1
SAVE_INTERVAL = 5.0


def scan_timestamp(message):
    try:
        if not (message.HasField("header") and message.header.HasField("stamp")):
            raise ValueError("Scan-Zeitstempel fehlt")
        stamp = message.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nsec < 1_000_000_000:
            raise ValueError("Scan-Zeitstempel ungültig")
        result = stamp.sec + stamp.nsec * 1e-9
        require_finite(result)
        return result
    except (AttributeError, TypeError) as error:
        raise ValueError("Scan-Zeitstempel fehlt") from error


class ExplorationSession:
    """Daten und Befehle unter einem Lock; keine Datei-/Terminal-I/O im Lock.

    tick() ist ohne Gazebo testbar. Der Runner ruft es unabhängig von langsamen
    JSON-/Terminalzugriffen alle 0.1 Wanduhrsekunden auf.
    """

    def __init__(self, memory, move, clock=time.monotonic):
        self.memory = memory
        self.move = move
        self.clock = clock
        self.lock = threading.Lock()
        self.active = True
        self.explorer = BasicExplorer()
        self.pose = None
        self.results = None
        self.pose_time = None
        self.scan_time = None
        self.pose_stamp = None
        self.scan_stamp = None
        self.first_stamp = None
        self.error = None
        self.status = "warte auf Scan und Pose"

    def _stop(self, reason):
        self.status = reason
        self.move(0.0, 0.0)

    def on_pose(self, message):
        with self.lock:
            if not self.active:
                return
            try:
                pose = extract_model_pose(message)
                if pose is None:
                    raise ValueError("Modellpose fehlt")
                self.memory.world_to_cell(pose.x, pose.y)
            except ValueError as error:
                self.pose = None
                self._stop(f"Stopp: {error}")
                return
            if self.pose_stamp is not None:
                if pose.timestamp < self.pose_stamp:
                    self.error = "Pose-Zeit läuft rückwärts; Controller neu starten"
                    self._stop(self.error)
                    return
                if pose.timestamp == self.pose_stamp:
                    return  # Wiederholte alte Daten verlängern den Timeout nicht.
            self.pose = pose
            self.pose_stamp = pose.timestamp
            self.pose_time = self.clock()
            if self.first_stamp is None:
                self.first_stamp = pose.timestamp
            self.memory.visit(pose.x, pose.y)

    def on_scan(self, message):
        with self.lock:
            if not self.active:
                return
            try:
                stamp = scan_timestamp(message)
                results = analyze_scan(message.ranges, message.range_min, message.range_max,
                                       message.angle_min, message.angle_step)
            except (ValueError, TypeError, AttributeError) as error:
                self.results = None
                self._stop(f"Stopp: ungültiger Scan ({error})")
                return
            if self.scan_stamp is not None:
                if stamp < self.scan_stamp:
                    self.error = "Scan-Zeit läuft rückwärts; Controller neu starten"
                    self._stop(self.error)
                    return
                if stamp == self.scan_stamp:
                    return
            self.results = results
            self.scan_stamp = stamp
            self.scan_time = self.clock()

    def tick(self):
        with self.lock:
            if not self.active:
                return
            now = self.clock()
            if (self.error or self.results is None or self.pose is None
                    or now - self.scan_time > SCAN_TIMEOUT
                    or now - self.pose_time > POSE_TIMEOUT
                    or abs(self.scan_stamp - self.pose_stamp) > MAX_SENSOR_SKEW):
                self._stop("Stopp: fehlende, alte oder zeitlich unpassende Sensordaten")
                return
            linear, angular = self.explorer.command(self.results, self.pose, self.memory)
            self.move(linear, angular)
            self.status = self.explorer.reason

    def snapshot(self):
        with self.lock:
            elapsed = 0 if self.pose_stamp is None else self.pose_stamp - self.first_stamp
            return copy.deepcopy(self.memory), self.status, elapsed, self.error

    def close(self):
        with self.lock:
            self.active = False
            self._stop("Controller beendet")


def run_exploration(filename=DEFAULT_FILE, duration=None, sim_duration=None,
                    cell_size=DEFAULT_CELL_SIZE, bounds=DEFAULT_BOUNDS):
    for value in (duration, sim_duration):
        if value is not None:
            require_finite(value)
            if value <= 0:
                raise ValueError("Dauer muss positiv sein")
    filename = Path(filename)
    memory = SpatialMemory.load(filename, cell_size, bounds)
    loaded_count = memory.count

    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_v_pb2 import Pose_V
    from src.navigation.movement import move

    session = ExplorationSession(memory, move)
    node = Node()
    finished = threading.Event()
    worker_error = []

    def control_loop():
        try:
            while not finished.is_set():
                session.tick()
                finished.wait(CONTROL_INTERVAL)
        except Exception as error:
            # Fehler im Thread dürfen keinen alten Fahrbefehl aktiv lassen.
            worker_error.append(error)
            session.close()
            finished.set()

    worker = threading.Thread(target=control_loop, name="exploration-control", daemon=True)
    subscriptions = []
    saved_count = memory.count if filename.exists() else -1
    start = time.monotonic()
    next_save, next_status = start + SAVE_INTERVAL, start
    try:
        move(0.0, 0.0)
        for message_type, topic, callback in (
                (Pose_V, POSE_TOPIC, session.on_pose), (LaserScan, LIDAR_TOPIC, session.on_scan)):
            if not node.subscribe(message_type, topic, callback):
                raise RuntimeError(f"Topic konnte nicht abonniert werden: {topic}")
            subscriptions.append(topic)
        worker.start()
        print(f"Exploration: {filename} | geladen: {loaded_count} Zellen\n"
              "Einziger Fahrcontroller; kein paralleles navigation.py oder Memory-Schreiber!")
        while duration is None or time.monotonic() - start < duration:
            snapshot, status, elapsed, error = session.snapshot()
            if worker_error:
                raise RuntimeError(f"Steuerung fehlgeschlagen: {worker_error[0]}")
            if error:
                raise RuntimeError(error)
            if sim_duration is not None and elapsed >= sim_duration:
                break
            now = time.monotonic()
            if now >= next_save:
                if snapshot.count != saved_count:
                    snapshot.save(filename)
                    saved_count = snapshot.count
                next_save = now + SAVE_INTERVAL
            if now >= next_status:
                print(f"Simulation: {elapsed:.1f} s | Visited cells: {snapshot.count} | "
                      f"Coverage: {snapshot.coverage:.2%} | {status}")
                next_status = now + 2.0
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        finished.set()
        try:
            # Stop vor Dateizugriff, Terminalausgabe und Abmelden der Subscriber.
            session.close()
        finally:
            if worker.ident is not None:
                worker.join(timeout=2)
            try:
                for topic in subscriptions:
                    node.unsubscribe(topic)
            finally:
                snapshot, _, elapsed, error = session.snapshot()
                if snapshot.count != saved_count:
                    snapshot.save(filename)
    if worker_error:
        raise RuntimeError(f"Steuerung fehlgeschlagen: {worker_error[0]}")
    if error:
        raise RuntimeError(error)
    print(f"Exploration beendet: {snapshot.count} Zellen, {snapshot.coverage:.2%}, "
          f"{elapsed:.1f} Simulationssekunden")
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE)
    parser.add_argument("--duration", type=float, help="Maximale Wanduhrsekunden")
    parser.add_argument("--sim-duration", type=float, help="Sekunden ab erster Modellpose")
    parser.add_argument("--cell-size", type=float, default=DEFAULT_CELL_SIZE)
    parser.add_argument("--bounds", type=float, nargs=4, default=DEFAULT_BOUNDS,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    args = parser.parse_args()
    try:
        run_exploration(args.file, args.duration, args.sim_duration, args.cell_size, args.bounds)
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        parser.exit(1, f"Exploration fehlgeschlagen: {error}\n")


if __name__ == "__main__":
    main()
