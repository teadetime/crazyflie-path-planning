"""Sandbox environment."""

import numpy as np
import plotly.graph_objects as go

from path_planning import viz
from path_planning.algorithms.cbs import CBS
from path_planning.omap import OMap
from path_planning.utils import Agent, Goal



from .sweeps import sweep_cbs

def main() -> None:
    """Main function within Sandbox."""
    test_omap = OMap(
        10,
        10,
        0.25,
        np.array([0.0, 0.0, 0]),
        cell_size=0.25,
        # bbox_tuple=(np.array([-.10, -.10, -.20]), np.array([.10, .10, .20])),
    )

    test_omap.set_rectangles(np.array([4, 4, 0]), np.array([5, 5, 0.24]))

    times = []
    agent_nums = [i for i in range(2, 3)]

    for agent_num in agent_nums:
        output = sweep_cbs(
            iterations=20,
            num_agents=agent_num,
            omap=test_omap,
            min_path_length=min(test_omap.map.shape) * test_omap.cell_size / 2.1,
        )
        output = np.array(output)
        time_avg = np.average(np.transpose(output)[1])
        print(time_avg, agent_num)
        times.append(time_avg)

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=agent_nums, y=times))
    fig.update_layout(
        title="Agent nums vs time",
        xaxis_title="Number of agents",
        yaxis_title="Average runtime",
    )
    fig.show()
