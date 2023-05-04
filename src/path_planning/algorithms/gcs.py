from copy import copy
from operator import attrgetter
from queue import PriorityQueue
from typing import Dict, List, NamedTuple, Optional, Tuple

import cvxpy as cp
import numpy as np

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
    def generate(  # noqa: C901
        omap: OMap,
        goals: Dict[Agent, List[Goal]],
        override_starting_pos: None | Dict[Agent, Point] = None,
    ) -> AgentPaths:  # pyright: ignore[reportGeneralTypeIssues]

        # Example scene
        obs_cube1 = np.array([
            [1, 0],
            [0, 1],
            [-1, 0],
            [0, -1],
        ]).T
        obs_cube2 = np.array([
            [3,4],
            [4,4],
            [4,5],
            [3,5],
        ]).T
        r_wall = np.array([
            [7.5, 7],
            [7, 7],
            [7, -7],
            [7.5, -7],
        ]).T
        t_wall = np.array([
            [7, 7.5],
            [-7, 7.5],
            [-7, 7],
            [7, 7],
        ]).T

        l_wall = np.diag([-1, 1]) @ r_wall
        b_wall = np.diag([1, -1]) @ t_wall

        obs = [obs_cube1, obs_cube2, r_wall, t_wall, l_wall, b_wall]

        polys = FreespacePolytopes(obs)

        fig = go.Figure()
        plot_obs(obs, fig=fig)
        
        colors = px.colors.qualitative.Plotly[0:len(polys.As)]
        plot_zones(polys, colors, fig=fig)

        fig.update_layout(yaxis_scaleanchor="x")
        fig.show()

        graph_of_convex_sets = GraphOfConvexSets(polys)
        print(graph_of_convex_sets.mat_edges)

        return AgentPaths()