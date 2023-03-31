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


class PathPlanner(ABC):
    """PathPlanner ABC."""

    @staticmethod
    @abstractmethod
    def generate(omap: OMap, goals: dict[str, list[Goal]]) -> Tuple[Path, go.Figure]:
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
