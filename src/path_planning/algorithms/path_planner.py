"""Declares ABC for a trajectory generator."""

from abc import ABC, abstractmethod
from typing import Tuple

import plotly.graph_objects as go

from ..omap import OMap
from ..utils import Path, Point


class Goal:
    """Wrapper for goal position with priority."""

    def __init__(self, position: Point, priority: int, label: str = "") -> None:
        """Initialize Goal.

        Args:
            position: Position
            priority: Integer priority. 0 is highest priority
            label: Goal label for plotting
        """
        self.pos = position
        self.priority = priority
        self.label = label


class Agent:
    """Agent class."""

    def __init__(self, name: str) -> None:
        """Initialize Agent."""
        self.name = name
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.battery = None


class PathPlanner(ABC):
    """PathPlanner ABC."""

    @staticmethod
    @abstractmethod
    def generate(
        omap: OMap,
        goals: dict[Agent, list[Goal]],
        override_starting_pos: None | dict[Agent, Point] = None,
    ) -> Tuple[Path, go.Figure]:
        """Abstract Base Class for a PathPlanner."""
        pass


class AStar(PathPlanner):
    """AStar implementation."""

    @staticmethod
    def generate(
        omap: OMap, goals: dict[str, list[Goal]]
    ) -> Tuple[Path, go.Figure]:  # pyright:ignore
        """Execute A* to generate a path."""
        pass

    @staticmethod
    def build_figure() -> go.Figure:
        """Build Astar plotly figure."""
        pass
