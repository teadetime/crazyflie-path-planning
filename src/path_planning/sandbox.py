"""Sandbox environment."""

import numpy as np

from path_planning import viz
from path_planning.algorithms.cbs import CBS
from path_planning.omap import OMap
from path_planning.utils import Agent, Goal


def main() -> None:
    """Main function within Sandbox."""
    test_omap = OMap(
        10,
        10,
        2,
        np.array([0.0, 0.0, 0]),
        cell_size=0.25,
        # bbox_tuple=(np.array([-.10, -.10, -.20]), np.array([.10, .10, .20])),
    )

    test_omap.set_rectangles(np.array([1, 8, 0]), np.array([5, 9, 1.9]))
    test_omap.set_rectangles(np.array([6, 8, 0]), np.array([9, 9, 1.9]))
    test_omap.set_rectangles(np.array([1, 2, 0]), np.array([2, 8, 1.9]))
    test_omap.set_rectangles(np.array([8, 2, 0]), np.array([9, 8, 1.9]))
    test_omap.set_rectangles(np.array([2, 2, 0]), np.array([8, 3, 1.9]))

    ag1 = Agent(name="CF 1", pos=np.array([5, 5, 0.0]))  # global starting position
    ag2 = Agent(name="CF 2", pos=np.array([6, 5, 0.0]))  # global starting position
    ag3 = Agent(name="CF 3", pos=np.array([4, 5, 0.0]))  # global starting position
    ag4 = Agent(name="CF 4", pos=np.array([3, 5, 0.0]))  # global starting position
    ag5 = Agent(name="CF 5", pos=np.array([4, 4, 0.0]))  # global starting position
    ag6 = Agent(name="CF 6", pos=np.array([6, 4, 0.0]))  # global starting position
    ag7 = Agent(name="CF 7", pos=np.array([7, 5, 0.0]))  # global starting position
    ag8 = Agent(name="CF 8", pos=np.array([7, 7, 0.0]))  # global starting position
    # ag9 = Agent(name="CF 9", pos=np.array([7, 4, 0.0]))  # global starting position
    # ag10 = Agent(name="CF 10", pos=np.array([5.5, 4, 0.0]))  # global starting position

    # Goals in cells
    goals = {
        ag1: [Goal(np.array([5, 1, 0.0]))],
        ag2: [Goal(np.array([6, 1, 0.0]))],
        ag3: [Goal(np.array([7, 1, 0.0]))],
        ag4: [Goal(np.array([4, 1, 0.0]))],
        ag5: [Goal(np.array([1, 1, 0.0]))],
        ag6: [Goal(np.array([2, 1, 0.0]))],
        ag7: [Goal(np.array([3, 1, 0.0]))],
        ag8: [Goal(np.array([3.5, 0.7, 0.0]))],
        # ag9: [Goal(np.array([1.5, 1, 0.0]))],
        # ag10: [Goal(np.array([3, 1, 0.0]))],
    }

    agent_paths, _figure = CBS.generate(
        test_omap,
        goals,
    )
    if agent_paths:
        print(agent_paths)

        animation = viz.build_animation(test_omap, agent_paths)
        animation.show()
