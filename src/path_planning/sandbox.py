"""Sandbox environment."""


import numpy as np

from path_planning.omap import OMap


def main() -> None:
    """Main function within Sandbox."""
    test_omap = OMap(0.2, 0.2, 0.05)

    # points = np.array([[0, 0, 0], [0.1, 0, 0]])
    # test_omap.set_points_global(points)
    test_omap.set_rectangles(np.array([0, 0, 0]), np.array([0.1, 0.1, 0]))
    print(test_omap.map)
