"""Wrappers for testing algorithms."""

import time
import numpy as np
from typing import Dict, List, Tuple
from random import uniform

from path_planning.algorithms.cbs import CBS, Constraint  # , Constraint
from path_planning.omap import OMap
from path_planning.utils import Agent, Goal
from .utils import Point


def choose_omap_point(omap: OMap):
    """choose random point in OMap"""
    dimensions = np.multiply(np.array(omap.map.shape), omap.cell_size)
    new_point = np.array(
        [
            uniform(0, dimensions[0] - 0.1),
            uniform(0, dimensions[1] - 0.1),
            0.0,
        ]
    )
    return new_point


def distance(point1: Point, point2: Point):
    """find distance between two points"""
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def generate_start_goal_pair(
    omap: OMap,
    min_dist: float = 0.0,
    constraint_start: list = [],
    constraint_goal: list = [],
):
    """choose non-equal, empty start and goal points in an OMap"""
    pair = []
    for i in range(2):
        point = choose_omap_point(omap=omap)
        cell = omap._glbl_pts_to_cells(point).astype(np.int64)
        while (
            bool(omap.map[cell].any())
            or (len(pair) > 0 and np.any(np.all(point == pair, axis=1)))
            or (len(pair) > 0 and distance(point, pair[0]) < min_dist)
            or (
                len(constraint_goal) > 0
                and len(pair) > 0
                and np.any(np.all(point == constraint_goal, axis=1))
            )
            or (
                len(constraint_start) > 0
                and len(pair) == 0
                and np.any(np.all(point == constraint_start, axis=1))
            )
        ):
            point = choose_omap_point(omap=omap)
            cell = omap._glbl_pts_to_cells(point).astype(np.int64)
        pair.append(point)
    return pair


def sweep_cbs(iterations: int, num_agents: int, omap: OMap, min_path_length: float):
    """Sweep cbs with random starts and ends"""
    agents = []
    for agent_idx in range(num_agents):
        agent = Agent(name=f"agent_{agent_idx}", pos=(0, 0, 0))
        agents.append(agent)
    results = []
    for i in range(iterations):
        goals = dict()
        for agent in agents:
            start, goal = generate_start_goal_pair(omap=omap, min_dist=min_path_length)
            agent.pos = np.array(start)
            goals[agent] = [Goal(np.array(goal, dtype=np.float64))]
        start_time = time.process_time()

        result = CBS.generate(
            omap,
            goals,
        )
        end_time = time.process_time()
        results.append((result[0], end_time - start_time))
    return results
