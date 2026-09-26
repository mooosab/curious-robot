"""Optionaler Matplotlib-Test, ohne GUI-Fenster oder laufendes Gazebo."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

from src.localization.plot import plot_path
from src.localization.pose import Pose2D


@unittest.skipUnless(importlib.util.find_spec("matplotlib"), "Matplotlib nicht installiert")
class PlotTests(unittest.TestCase):
    def test_trajectory_markers_heading_and_export(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        points = [Pose2D(0, 0, 1, 0), Pose2D(1, 2, 3, math.pi / 2)]
        figure = plot_path(points)
        try:
            axes = figure.axes[0]
            self.assertEqual(list(axes.lines[0].get_xdata()), [0, 2])
            self.assertEqual(list(axes.lines[0].get_ydata()), [1, 3])
            self.assertEqual(axes.collections[0].get_offsets().tolist(), [[0, 1]])
            self.assertEqual(axes.collections[1].get_offsets().tolist(), [[2, 3]])
            heading = axes.collections[2]
            self.assertAlmostEqual(float(heading.U[0]), 0)
            self.assertAlmostEqual(float(heading.V[0]), 0.3)
            self.assertEqual(axes.get_aspect(), 1)
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "path.png"
                figure.savefig(output)
                self.assertTrue(output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
        finally:
            plt.close(figure)

    def test_single_point_is_visible(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure = plot_path([Pose2D(0, 0, 0, 0)])
        try:
            self.assertEqual(figure.axes[0].get_xlim(), (-0.5, 0.5))
            self.assertEqual(figure.axes[0].get_ylim(), (-0.5, 0.5))
        finally:
            plt.close(figure)


if __name__ == "__main__":
    unittest.main()
