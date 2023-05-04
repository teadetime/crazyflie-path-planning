"""Declares ABC for a trajectory generator."""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple

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
    ) -> AgentPaths:
        """Abstract Base Class for a PathPlanner."""
        pass


class AStar(PathPlanner):
    """AStar implementation."""

    @staticmethod
    def generate(
        omap: OMap, goals: dict[str, list[Goal]]
    ) -> AgentPaths:  # pyright:ignore
        """Execute A* to generate a path."""
        pass
