"""Pfad anzeigen: python3 -m src.localization.plot recordings/run.csv"""

import argparse
import math
from pathlib import Path

from .path import load_path


def plot_path(points):
    """Matplotlib erst hier importieren; Localization selbst benötigt es nicht."""
    import matplotlib.pyplot as plt

    if not points:
        raise ValueError("Keine Punkte zum Zeichnen")
    figure, axes = plt.subplots()
    axes.plot([point.x for point in points], [point.y for point in points],
              color="steelblue", label="Pfad")
    first, last = points[0], points[-1]
    axes.scatter(first.x, first.y, c="green", marker="o", s=80, label="Start", zorder=3)
    axes.scatter(last.x, last.y, c="red", marker="x", s=80, label="Ende", zorder=4)
    axes.quiver(last.x, last.y, math.cos(last.yaw) * 0.3, math.sin(last.yaw) * 0.3,
                angles="xy", scale_units="xy", scale=1, color="red")
    # Auch ein einzelner Punkt / eine reine Drehung bleibt gut sichtbar.
    xs, ys = [point.x for point in points], [point.y for point in points]
    axes.set_xlim(min(xs) - 0.5, max(xs) + 0.5)
    axes.set_ylim(min(ys) - 0.5, max(ys) + 0.5)
    axes.set_aspect("equal", adjustable="box")
    axes.set(xlabel="Welt-X [m]", ylabel="Welt-Y [m]",
             title=f"Curious Robot – {len(points)} Punkte, "
                   f"{last.timestamp - first.timestamp:.1f} s Simulationszeit")
    axes.grid(True, alpha=0.3)
    axes.legend()
    figure.tight_layout()
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output", type=Path, help="Optional als Bild speichern")
    parser.add_argument("--no-show", action="store_true", help="Kein Fenster öffnen")
    args = parser.parse_args()
    if args.no_show and args.output is None:
        parser.error("--no-show benötigt --output")
    try:
        if args.no_show:
            import matplotlib
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figure = plot_path(load_path(args.path))
        if args.output:
            figure.savefig(args.output, dpi=150)
            print(f"Pfadbild gespeichert: {args.output}")
        if not args.no_show:
            plt.show()
        plt.close(figure)
    except (OSError, ValueError, ImportError) as error:
        parser.exit(1, f"Darstellung fehlgeschlagen: {error}\n")


if __name__ == "__main__":
    main()
