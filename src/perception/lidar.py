"""Auswertung und Darstellung unseres 361-Strahlen-LiDARs, ohne Gazebo."""

import math


OBSTACLE_DISTANCE = 1


def evaluate_sector(values, range_min, range_max):
    """Liefert Mindestdistanz, Hindernisstatus und Anzahl ungültiger Werte.

    Status: True = Hindernis, False = kein nahes Hindernis, None = unbekannt.
    +inf bedeutet kein Treffer innerhalb der Reichweite.
    """
    valid_values = []
    invalid_count = 0
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            invalid_count += 1
        elif value == math.inf:
            valid_values.append(value)
        elif math.isfinite(value) and range_min <= value <= range_max:
            valid_values.append(value)
        else:
            invalid_count += 1

    distance = min(valid_values) if valid_values else None
    if distance is not None and distance < OBSTACLE_DISTANCE:
        obstacle = True
    elif distance is None or invalid_count:
        obstacle = None
    else:
        obstacle = False
    return distance, obstacle, invalid_count


def analyze_scan(ranges, range_min, range_max, angle_min=-math.pi,
                 angle_step=math.pi / 180):
    """Wertet einen vollständigen Scan mit der bestehenden Sektoreinteilung aus."""
    values = list(ranges)
    if len(values) != 361:
        raise ValueError(f"361 Messwerte erwartet, {len(values)} erhalten")
    if not (math.isfinite(range_min) and math.isfinite(range_max)
            and 0 < range_min < OBSTACLE_DISTANCE <= range_max):
        raise ValueError(f"Ungültige Sensorgrenzen für die {OBSTACLE_DISTANCE:.1f}-m-Hindernisschwelle")
    # Feste Indizes passen nur zum Scan von -180 bis +180 Grad in 1-Grad-Schritten.
    if not (math.isclose(angle_min, -math.pi, abs_tol=1e-6)
            and math.isclose(angle_step, math.pi / 180, abs_tol=1e-6)):
        raise ValueError("Scanwinkel passen nicht zu den acht festen Bereichen")

    sectors = {
        "Vorne": values[158:203],
        "Vorne-links": values[203:248],
        "Links": values[248:293],
        "Hinten-links": values[293:338],
        "Hinten": values[338:361] + values[0:23],
        "Hinten-rechts": values[23:68],
        "Rechts": values[68:113],
        "Vorne-rechts": values[113:158],
    }
    results = {}
    for name, sector in sectors.items():
        distance, obstacle, invalid_count = evaluate_sector(sector, range_min, range_max)
        results[name] = {
            "distance": distance,
            "obstacle": obstacle,
            "invalid_count": invalid_count,
        }
    return results


def format_scan(results):
    """Erzeugt die Tabelle, ohne Messwerte zu verändern."""
    lines = [
        f"\nLiDAR-Auswertung — Hindernis bei Distanz < {OBSTACLE_DISTANCE:.2f} m",
        f"{'Bereich':<16} | {'Distanz':>12} | {'Hindernis':<10} | Ungültig",
        "-" * 65,
    ]
    for name, result in results.items():
        distance = result["distance"]
        if distance is None:
            distance_text = "--"
        elif distance == math.inf:
            distance_text = "kein Treffer"
        else:
            distance_text = f"{distance:.2f} m"
        status = {True: "JA", False: "NEIN", None: "UNBEKANNT"}[result["obstacle"]]
        lines.append(
            f"{name:<16} | {distance_text:>12} | {status:<10} | {result['invalid_count']}"
        )
    return "\n".join(lines)
