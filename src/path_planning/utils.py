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
