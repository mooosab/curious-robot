"""LiDAR live anzeigen: python3 src/perception/test.py"""

import time

if __package__:
    from .lidar import analyze_scan, format_scan
else:
    from lidar import analyze_scan, format_scan


def lidar_callback(msg):
    try:
        results = analyze_scan(
            msg.ranges, msg.range_min, msg.range_max,
            msg.angle_min, msg.angle_step,
        )
    except ValueError as error:
        print(f"\nScan nicht auswertbar: {error}. Hindernisstatus: UNBEKANNT.")
        return
    print(format_scan(results))


def main():
    # Gazebo wird nur für den Live-Empfang benötigt, nicht für die Tests.
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan

    node = Node()
    if not node.subscribe(LaserScan, "/model/curious_robot/lidar", lidar_callback):
        raise RuntimeError("LiDAR-Topic konnte nicht abonniert werden")

    print("Warte auf LiDAR-Daten ... Beenden mit Ctrl+C.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nLiDAR-Anzeige beendet.")


if __name__ == "__main__":
    main()
