"""Sandbox environment."""

import numpy as np

from path_planning.algorithms.cbs import CBS, Constraint
from path_planning.omap import OMap
from path_planning.utils import Agent, Goal


def main() -> None:
    """Main function within Sandbox."""
    test_omap = OMap(
        3,
        2,
        2,
        np.array([0.0, 0.0, 0]),
        cell_size=0.2,
        # bbox_tuple=(np.array([-.10, -.10, -.20]), np.array([.10, .10, .20])),
    )

    # points = np.array([[0, 0, 0], [0.1, 0, 0]])
    # test_omap.set_points_global(points)
    # test_omap.set_rectangles(np.array([0, 3, 0]), np.array([15, 6, 5]))
    # test_omap.set_rectangles(np.array([0, 9, 0]), np.array([22, 12, 5]))
    # test_omap.set_rectangles(np.array([0, 15, 0]), np.array([25, 18, 5]))

    # fig = go.Figure()
    # omap_trace = viz.create_omap_trace(test_omap)
    # traj = np.array([[0, 0, 0], [0, 0.3, 0]])
    # traj_trace = viz.create_points_trace(traj)
    # fig.add_trace(omap_trace)
    # fig.add_trace(traj_trace)
    # update_dictionary = viz.create_plot_update(test_omap)
    # fig.update_layout(dict1=update_dictionary)
    # fig.show()

    # point1 = np.array([0.2, 0.2, 0.1])
    # cells = test_omap._cells_around_point(point1)
    # print(cells.shape)
    # point1 = np.array([.2,.2,.1])
    # point2 = np.array([.55,.55,.20])

    # results = test_omap.points_in_collision(point1, point2)
    # print(results)

    # TESTING CBS AND ASTAR
    start_pt = np.array([0.0, 0.0, 0.1])

    start_cell = test_omap._glbl_pts_to_cells(start_pt).astype(np.int64)

    goal_pt = np.array([0.6, 0.6, 0.1])
    goal_cell = test_omap._glbl_pts_to_cells(goal_pt).astype(np.int64)
    print(f"Start: {start_cell}, End: {goal_cell}")
    goal = Goal(position=goal_cell)

    avoid_pt = (1, 1, 0)
    # test_omap.map[avoid_pt] = True

    constraint = Constraint(
        Agent(name="test", pos=np.array([0, 0, 0])), np.array(avoid_pt), 1
    )
    result = CBS.single_agent_astar(
        test_omap, start_cell, [goal], constraint=constraint, existing_path=None
    )

    ag1 = Agent(name="test1", pos=np.array([0, 0, 0]))  # global starting position
    ag2 = Agent(name="test2", pos=np.array([1, 2, 0]))

    # Goals in cells
    goals = {
        ag1: [Goal(np.array((2, 2, 2), dtype=np.int64))],
        ag2: [Goal(np.array((5, 3, 3), dtype=np.int64))],
    }
    result = CBS.generate(
        test_omap,
        goals,
    )
    if result:
        print(result[0])
    else:
        print("No path found!")
