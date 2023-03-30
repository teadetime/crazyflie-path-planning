"""Declares ABC for a trajectory generator."""

from abc import ABC, abstractmethod


class TrajectoryPlanner(ABC):
    """TrajectoryPlanner ABC."""

    @abstractmethod
    def __call__(self) -> None:
        """Abstract Base Class for a trajectory planner."""
        pass
