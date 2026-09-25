from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan
import time


lidar_data = []


def lidar_callback(msg):
    global lidar_data

    # LiDAR-Daten in normale Python-Liste umwandeln
    lidar_data = list(msg.ranges)

    # Bestimmte Richtungen auslesen
    front = lidar_data[180]
    right = lidar_data[90]
    left = lidar_data[270]
    back = lidar_data[0]
    left_front = lidar_data[225]
    right_front = lidar_data[135]
    back_left = lidar_data[315]
    back_right = lidar_data[45]

    print(
        f"Vorne: {front:.2f} m | "
        f"Links: {left:.2f} m | "
        f"Rechts: {right:.2f} m | "
        f"Hinten: {back:.2f} m |\n"
        f"Links vorne: {left_front:.2f} m | "
        f"Rechts vorne: {right_front:.2f} m"
        f"Hinten links: {back_left:.2f} m | "
        f"Hinten rechts: {back_right:.2f} m"
    )


node = Node()

node.subscribe(
    LaserScan,
    "/model/curious_robot/lidar",
    lidar_callback
)


while True:
    time.sleep(1)