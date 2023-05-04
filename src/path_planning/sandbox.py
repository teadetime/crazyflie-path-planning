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

    test_omap.set_rectangles(np.array([3, 3, 0]), np.array([6, 6, 1.9]))

    ag1 = Agent(
        name="CF 1", pos=np.array([1.05901491, 7.13232832, 0.5328699])
    )  # global starting position
    ag2 = Agent(name="CF 2", pos=np.array([9.55325931, 6.58183465, 1.35118168]))

    # Goals in cells
    goals = {
        ag1: [Goal(np.array([1.78599858, 2.51949983, 0.13095248]))],
        ag2: [Goal(np.array([2.99585621, 0.36188631, 0.2105532]))],
    }

    agent_paths = CBS.generate(
        test_omap,
        goals,
    )
    if agent_paths:
        print(agent_paths)

        animation = viz.build_animation(test_omap, agent_paths)
        animation.show()
