"""Sandbox environment."""

import numpy as np

# import plotly.graph_objects as go

from path_planning.omap import OMap

# from .viz import create_omap_trace, create_points_trace, create_plot_update


def main() -> None:
    """Main function within Sandbox."""
    test_omap = OMap(0.4, 0.4, 0.05)

    # points = np.array([[0, 0, 0], [0.1, 0, 0]])
    # test_omap.set_points_global(points)
    test_omap.set_rectangles(np.array([0, 0, 0]), np.array([0.1, 0.1, 0]))

    # fig = go.Figure()
    # omap_trace = create_omap_trace(test_omap)
    # traj = np.array([[0, 0, 0], [0, 0.3, 0]])
    # print(traj.shape)
    # traj_trace = create_points_trace(traj)
    # fig.add_trace(omap_trace)
    # fig.add_trace(traj_trace)
    # update_dictionary = create_plot_update(test_omap)
    # fig.update_layout(dict1=update_dictionary)
    # fig.show()
