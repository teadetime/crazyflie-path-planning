from copy import copy
from operator import attrgetter
from queue import PriorityQueue
from typing import Dict, List, NamedTuple, Optional, Tuple

import cvxpy as cp
import numpy as np
from pyinstrument import Profiler

from .path_planner import Agent, AgentPaths, Goal, PathPlanner
from ..omap import OMap
from ..utils import Path, Point

from .gcs_lib.FreespacePolytopes import FreespacePolytopes
from .gcs_lib.GraphOfConvexSets import GraphOfConvexSets
from .gcs_lib.plotting import plot_obs, plot_zones

import plotly.graph_objects as go # TODO: Remove this once we are no longer in debug mode.
import plotly.express as px

class GCS(PathPlanner):
    """Wrapper of the underlying GraphOfConvexSets solver. 
    - Parses our standard library omap, goal, and start positions. 
    - Converts the output of GraphOfConvexSets to AgentPaths
    """

    @staticmethod
    def make_rect(dims, pos, theta):
        pos = np.atleast_2d(np.array(pos).flatten()).T
        theta = np.deg2rad(theta)

        square = 0.5 * np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]).T
        rect = np.diag(dims) @ square
        rotmat = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ])
        return rotmat @ rect + pos
    
    @staticmethod
    def generate(  # noqa: C901
        omap: OMap,
        goals: Dict[Agent, List[Goal]],
        override_starting_pos: None | Dict[Agent, Point] = None,
    ) -> AgentPaths:  # pyright: ignore[reportGeneralTypeIssues]

        profiler = Profiler()
        profiler.start()

        obs_cube1 = GCS.make_rect([np.sqrt(2), np.sqrt(2)], [0, 0], 45)
        obs_cube2 = GCS.make_rect([1, 1], [3.5, 4.5], 0)
        r_wall = GCS.make_rect([0.5, 14], [7.25, 0], 0)
        t_wall = GCS.make_rect([14, 0.5], [0, 7.25], 0)

        l_wall = np.diag([-1, 1]) @ r_wall
        b_wall = np.diag([1, -1]) @ t_wall

        obs = [obs_cube1, obs_cube2, r_wall, t_wall, l_wall, b_wall]
        polys = FreespacePolytopes(obs, n_regions=5, grid_dims=100)

        fig = go.Figure()
        plot_obs(obs, fig=fig)
        
        colors = px.colors.qualitative.Plotly[0:len(polys)]
        plot_zones(polys, colors, fig=fig)

        fig.update_layout(yaxis_scaleanchor="x")

        try:
            graph_of_convex_sets = GraphOfConvexSets(polys)
            # x_opt, vertices_opt = graph_of_convex_sets.solve(np.array([0, -5]), np.array([0, 5]))
            x_opt, vertices_opt = graph_of_convex_sets.solve(np.array([-5, -6]), np.array([5, 6]))
            fig.add_trace(go.Scatter(
                x=x_opt[0, :],
                y=x_opt[1, :],
                line=dict(color="darkred"),
                name="Shortest Path",
                text=[f"Zone: {v}" for v in vertices_opt]
            ))
        except Exception as e:
            import traceback
            traceback.print_exception(e)

        fig.show()

        profiler.stop()
        profiler.print()
        return None