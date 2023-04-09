"""Sandbox environment."""

import numpy as np
import plotly.graph_objects as go

from path_planning import viz
from path_planning.omap import OMap


def main() -> None:
    """Main function within Sandbox."""
    test_omap = OMap(
        30,
        20,
        6,
        np.array([0.0, 0.0, 0]),
        cell_size=0.25,
        bbox_tuple=(np.array([-10, -10, -20]), np.array([10, 10, 20])),
    )

    points = np.array([[0, 0, 0], [0.1, 0, 0]])
    test_omap.set_points_global(points)
    test_omap.set_rectangles(np.array([0, 3, 0]), np.array([15, 6, 5]))
    test_omap.set_rectangles(np.array([0, 9, 0]), np.array([22, 12, 5]))
    test_omap.set_rectangles(np.array([0, 15, 0]), np.array([25, 18, 5]))

    fig = go.Figure()
    omap_trace = viz.create_omap_trace(test_omap)
    traj = np.array([[0, 0, 0], [0, 0.3, 0]])
    traj_trace = viz.create_points_trace(traj)
    fig.add_trace(omap_trace)
    fig.add_trace(traj_trace)
    update_dictionary = viz.create_plot_update(test_omap)
    fig.update_layout(dict1=update_dictionary)
    # fig.show()
