"""Define Occupancy Map."""

from math import ceil
from typing import Annotated, Literal, Tuple, TypeVar

import numpy as np
import numpy.typing as npt

from .utils import Point, Points

OmapGrid = Annotated[
    npt.NDArray[TypeVar("DType", bound=np.bool_)], Literal["L", "W", "H", "T"]
]

GridPoints = Annotated[npt.NDArray[TypeVar("DType", bound=np.bool_)], Literal["N", 3]]

Cells = Annotated[
    npt.NDArray[TypeVar("DType", bound=np.int_)], Literal["N", 3]
]  # N x (xyz)


class OMap:
    """Occupancy Map."""

    def __init__(
        self,
        x_size: float = 1.0,
        y_size: float = 1.0,
        z_size: float = 1.0,
        origin: Point | None = None,
        cell_size: float = 0.05,
        bbox_tuple: Tuple[Point, Point] | None = None,
    ) -> None:
        """Initialize Occupancy Map."""
        if origin is None:
            origin = np.array([0.0, 0.0, 0.0])

        if origin[0] >= x_size or origin[1] >= y_size or origin[2] >= z_size:
            raise ValueError("Specify an origin within the Occupancy Grid")

        self.bbox_tuple = bbox_tuple
        if bbox_tuple is not None:
            # TODO: enforce bottom left and top right?
            if len(bbox_tuple) != 2:
                ValueError(
                    f"Bbox tuple should have max two opposing corners to make\
                          a bounding box, got {len(bbox_tuple)} corners"
                )
            self.bbox = np.stack(
                [
                    bbox_tuple[0],
                    bbox_tuple[1],
                    np.array(
                        [
                            bbox_tuple[0][0],
                            bbox_tuple[0][1],
                            bbox_tuple[1][2],
                        ]
                    ),
                    np.array(
                        [
                            bbox_tuple[1][0],
                            bbox_tuple[1][1],
                            bbox_tuple[0][2],
                        ]
                    ),
                    np.array(
                        [
                            bbox_tuple[0][0],
                            bbox_tuple[1][1],
                            bbox_tuple[1][2],
                        ]
                    ),
                    np.array(
                        [
                            bbox_tuple[0][0],
                            bbox_tuple[1][1],
                            bbox_tuple[0][2],
                        ]
                    ),
                    np.array(
                        [
                            bbox_tuple[1][0],
                            bbox_tuple[0][1],
                            bbox_tuple[1][2],
                        ]
                    ),
                    np.array(
                        [
                            bbox_tuple[1][0],
                            bbox_tuple[0][1],
                            bbox_tuple[0][2],
                        ]
                    ),
                ]
            )
        else:
            self.bbox = None

        x_cells = ceil(x_size / cell_size)
        y_cells = ceil(y_size / cell_size)
        z_cells = ceil(z_size / cell_size)

        if (
            x_cells != x_size / cell_size
            or y_cells != y_size / cell_size
            or z_cells != z_size / cell_size
        ):
            raise ValueError("Specify an evenly divisible of grid")

        self.origin = origin
        self.cell_size = cell_size
        self.map = np.zeros((x_cells, y_cells, z_cells), dtype=bool)

    @property
    def min_max(self) -> Tuple[Point, Point]:
        """Return grid points in the global frame (Meters)."""
        return (
            -self.origin,
            np.array(self.map.shape) * self.cell_size - self.origin,
        )

    def _cells_to_glbl_pts(self, cells: Cells) -> Points:
        """Convert cells (Nx3) to global points."""
        return (cells * self.cell_size - self.origin).transpose()

    def _glbl_pts_to_cells(
        self, points: Points, within_grid: bool = True, trim_within_grid: bool = False
    ) -> Cells:
        """Conver global points to omap cells."""
        # TODO: Raise error on too big of query
        transformed = points + self.origin
        scaled = np.floor(transformed / self.cell_size).astype("int")

        if within_grid:
            if ((scaled >= self.map.shape) | (scaled < ([0, 0, 0]))).any():
                raise ValueError(
                    "Specified global points lay outside the occupancy map"
                )
        if trim_within_grid is True:
            scaled = np.clip(scaled, np.array([0, 0, 0]), np.array(self.map.shape))
        return scaled

    def obstacles_in_global(self) -> Points:
        """Return Obstacle map in Global Coordinates (Meters)."""
        obstacles = np.array(self.map.nonzero()).transpose()
        return self._cells_to_glbl_pts(obstacles)

    def set_points_global(self, points: Points, state: bool = True) -> None:
        """Set points in the global frame (relative to origin) to the omap."""
        cells = self._glbl_pts_to_cells(points, True).transpose()
        self.map[(cells[0], cells[1], cells[2])] = state

    def get_points_global(self, points: Points, use_bbox: bool = False) -> GridPoints:
        """Get global frame points occupancy."""
        if use_bbox and self.bbox is not None:
            query = np.concatenate([points + c for c in self.bbox])
        else:
            query = points
        cells = self._glbl_pts_to_cells(query).transpose()
        return self.map[(cells[0], cells[1], cells[2])]

    def set_rectangles(
        self, bottom_left_point: Point, top_right_point: Point, value: bool = True
    ) -> None:
        """Set subic volumes in environment in global frame."""
        bottom_left_cell = self._glbl_pts_to_cells(bottom_left_point)
        top_right_cell = self._glbl_pts_to_cells(top_right_point) + 1

        points = np.mgrid[
            bottom_left_cell[0] : top_right_cell[0],
            bottom_left_cell[1] : top_right_cell[1],
            bottom_left_cell[2] : top_right_cell[2],
        ]
        x = points[0].flatten()
        y = points[1].flatten()
        z = points[2].flatten()
        self.map[(x, y, z)] = value

    def check_line(
        self, start_pt: Point, end_pt: Point, density: int = 4, use_bbox: bool = True
    ) -> bool:
        """Check if line and bbox is occupied in omap."""
        length = np.linalg.norm(end_pt - start_pt)
        num_pts = int(length / (self.cell_size / density))
        if num_pts < 2:
            num_pts = 2
        points = np.linspace(
            start_pt, end_pt, num=num_pts, endpoint=True  # pyright: ignore
        )
        if not use_bbox or self.bbox is None:
            query = points
        else:
            query = np.concatenate([points + c for c in self.bbox])

        return not self.get_points_global(query).any()

    def _cells_around_point(self, point: Point) -> Points:
        """Use built in bounding box tuple to fill in cells around point."""
        if self.bbox is not None:
            one_corner_cell = self._glbl_pts_to_cells(
                point + self.bbox[0], False, trim_within_grid=True
            )
            opposite_corner_cell = (
                self._glbl_pts_to_cells(
                    point + self.bbox[1], False, trim_within_grid=True
                )
                + 1
            )
            print(f"one_corner: {one_corner_cell}")
            print(f"other_corner: {opposite_corner_cell}")
            points = np.mgrid[
                one_corner_cell[0] : opposite_corner_cell[0],
                one_corner_cell[1] : opposite_corner_cell[1],
                one_corner_cell[2] : opposite_corner_cell[2],
            ]

            # Turn into N x 3
            return points.reshape(3, (int(points.size / 3))).transpose()
        else:
            add_dim = np.expand_dims(point, 0)
            return self._glbl_pts_to_cells(add_dim, False, trim_within_grid=True)

    def point_in_collision_omap(self, point: Point) -> bool:
        """Check if points around a global point collide with obstacle map."""
        cells = self._cells_around_point(point)
        collision = self.map[(list(cells[:, 0]), list(cells[:, 1]), list(cells[:, 2]))]
        return bool(collision.any())

    def points_in_collision(self, pt_a: Point, pt_b: Point) -> bool:
        """Check if two points collide."""
        cells_a = self._cells_around_point(pt_a)
        cells_b = self._cells_around_point(pt_b)

        print(f"cells_a: {cells_a.shape} ", cells_a)
        print(f"cells_b: {cells_b.shape}", cells_b)
        return (cells_a == cells_b[:, None]).all(-1).any()
