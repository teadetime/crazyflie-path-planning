"""Utilities used across the project."""

from typing import Annotated, Literal, TypeVar

import numpy as np
import numpy.typing as npt

Point = Annotated[
    npt.NDArray[TypeVar("DType", bound=np.number)], Literal[1, 3]
]  # 1 x (xyz)
Points = Annotated[
    npt.NDArray[TypeVar("DType", bound=np.number)], Literal["N", 3]
]  # N x (xyz)
Path = Annotated[
    npt.NDArray[TypeVar("DType", bound=np.number)], Literal["N", 4]
]  # N x (xyz)


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
        self.pos: Point = np.array((0, 0, 0))
        self.battery = None
