"""Declares ABC for a trajectory generator."""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple

import plotly.graph_objects as go

from ..omap import OMap
from ..utils import Agent, Goal, Path, Point


AgentPaths = Dict[Agent, Path]


class PathPlanner(ABC):
    """PathPlanner ABC."""

    @staticmethod
    @abstractmethod
    def generate(
        omap: OMap,
        goals: dict[Agent, List[Goal]],
        override_starting_pos: None | Dict[Agent, Point] = None,
    ) -> Tuple[AgentPaths, go.Figure]:
        """Abstract Base Class for a PathPlanner."""
        pass


class AStar(PathPlanner):
    """AStar implementation."""

    @staticmethod
    def generate(
        omap: OMap, goals: dict[str, list[Goal]]
    ) -> Tuple[AgentPaths, go.Figure]:  # pyright:ignore
        """Execute A* to generate a path."""
        pass

    @staticmethod
    def build_figure() -> go.Figure:
        """Build Astar plotly figure."""
        pass
