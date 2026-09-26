"""Start im Projektordner: python3 -m src.navigation.navigation"""

import threading
import time

from src.perception.lidar import OBSTACLE_DISTANCE, analyze_scan


FORWARD_SPEED = 0.15
TURN_SPEED = 0.5
# Zusätzlicher Abstand verhindert Umschalten bei schwankenden Messwerten.
FRONT_CLEAR_DISTANCE = OBSTACLE_DISTANCE + 0.3
# Schräge Hindernisse dürfen beim Geradeausfahren nicht direkt an der Front liegen.
DIAGONAL_STOP_DISTANCE = 0.6
DIAGONAL_CLEAR_DISTANCE = 0.8
SCAN_TIMEOUT = 1.0  # Sekunden ohne neuen Scan -> Stopp


def choose_motion(results, turn_direction=None):
    """Liefert (Vorwärtsgeschwindigkeit, Drehgeschwindigkeit, Drehrichtung).

    Reine Entscheidung ohne Gazebo: dadurch mit Beispieldaten testbar.
    Die vordere Hindernisschwelle kommt aus der Perception (aktuell 1.0 m).
    """
    front = results["Vorne"]
    left = results["Vorne-links"]
    right = results["Vorne-rechts"]

    # Ein vorhandener Abstand allein reicht nicht: weitere Strahlen können fehlen.
    for sector in (front, left, right):
        if (sector["distance"] is None or sector["obstacle"] is None
                or sector["invalid_count"] > 0):
            return 0.0, 0.0, turn_direction

    diagonal_distance = min(left["distance"], right["distance"])
    if turn_direction is not None:
        # Erst mit zusätzlichem Freiraum wieder geradeaus fahren (Hysterese).
        if (front["distance"] >= FRONT_CLEAR_DISTANCE
                and diagonal_distance >= DIAGONAL_CLEAR_DISTANCE):
            return FORWARD_SPEED, 0.0, None
    elif (front["distance"] >= OBSTACLE_DISTANCE
          and diagonal_distance >= DIAGONAL_STOP_DISTANCE):
        return FORWARD_SPEED, 0.0, None
    else:
        # Nur zu Beginn des Manövers wählen, nicht bei jedem Scan neu.
        turn_direction = "left" if left["distance"] > right["distance"] else "right"

    # Nicht weiter ins Hindernis fahren: zunächst auf der Stelle drehen.
    angular_speed = TURN_SPEED if turn_direction == "left" else -TURN_SPEED
    return 0.0, angular_speed, turn_direction


def main():
    # Importieren dieses Moduls startet keine Navigation und sendet keine Befehle.
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    from src.navigation.movement import move, stop

    turn_direction = None
    last_scan_time = None
    running = True
    command_lock = threading.Lock()

    def lidar_callback(msg):
        nonlocal turn_direction, last_scan_time
        # Callback und Timeout-Prüfung dürfen keine widersprüchlichen Befehle senden.
        with command_lock:
            if not running:
                return
            last_scan_time = time.monotonic()
            try:
                results = analyze_scan(
                    msg.ranges, msg.range_min, msg.range_max,
                    msg.angle_min, msg.angle_step,
                )
            except ValueError as error:
                stop()
                status = f"Stopp: Scan nicht auswertbar ({error})"
            else:
                linear, angular, turn_direction = choose_motion(results, turn_direction)
                move(linear, angular)
                action = "STOPP" if linear == angular == 0 else (
                    "VORWÄRTS" if angular == 0 else f"DREHEN {turn_direction}"
                )
                status = (
                    f"{action} | Vorne: {results['Vorne']['distance']} | "
                    f"Vorne-links: {results['Vorne-links']['distance']} | "
                    f"Vorne-rechts: {results['Vorne-rechts']['distance']}"
                )
        # Ein blockiertes Terminal darf den Timeout-Stopp nicht blockieren.
        print(status)

    node = Node()
    try:
        stop()
        if not node.subscribe(LaserScan, "/model/curious_robot/lidar", lidar_callback):
            raise RuntimeError("LiDAR-Topic konnte nicht abonniert werden")
        print("Navigation gestartet; warte auf zuverlässige LiDAR-Daten.")
        while True:
            time.sleep(0.1)
            with command_lock:
                if last_scan_time is None or time.monotonic() - last_scan_time > SCAN_TIMEOUT:
                    stop()
    except KeyboardInterrupt:
        pass
    finally:
        with command_lock:
            running = False
            stop()
    print("\nNavigation beendet.")


if __name__ == "__main__":
    main()
