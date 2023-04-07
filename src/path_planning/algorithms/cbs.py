"""Conflict-Based-Search implementation using A* as a backbone."""


from typing import Dict, List, NamedTuple, Set, Tuple


import plotly.graph_objects as go

from .path_planner import Agent, AgentPaths, Goal, PathPlanner
from ..omap import OMap
from ..utils import Path, Point


class Constraint(NamedTuple):
    """Contraint for CBS."""

    agent: Agent
    vertex: Point
    time: int


class Node(NamedTuple):
    """Node for CBS."""

    constraints: set[Constraint]
    solution: AgentPaths
    cost: float


class CBS(PathPlanner):
    """Conflict Based Search PathPlanner."""

    @staticmethod
    def single_agent_astar() -> Tuple[Agent, Path, float]:
        """Run Astar for a single agent."""
        pass

    @staticmethod
    def multi_agent_astar() -> Tuple[AgentPaths, float]:
        """Run Astar for all agents."""
        pass

    @staticmethod
    def validate_path(
        omap: OMap, paths: AgentPaths
    ) -> Tuple[Set[Agent], Point, Point, int]:
        """Returns Conflicts within an AgentPaths."""
        pass

    @staticmethod
    def generate(
        omap: OMap,
        goals: Dict[Agent, List[Goal]],
        override_starting_pos: None | Dict[Agent, Point] = None,
    ) -> Tuple[AgentPaths, go.Figure]:
        """Conflict Base Search PathPlanner."""
        # Relies heavily on pseudocode:
        # https://www.sciencedirect.com/science/article/pii/S0004370214001386
        initial_constraint = {}
        initial_solution = CBS.multi_agent_astar()[0]
        initial_cost = CBS.multi_agent_astar()[1]
        initial_node = Node(initial_constraint, initial_solution, initial_cost)
        explored_node_set = (
            {}
        )  # Keep track so that we can avoid duplicates w/constant time
        explore_list: List[Node] = [initial_node]  # Maps to Open in paper

        while explore_list != []:
            # Sort the open list
            cur_node = explore_list.pop(-1)  # Pop last element (lowest cost)

            print(f"{cur_node}{explored_node_set}")
