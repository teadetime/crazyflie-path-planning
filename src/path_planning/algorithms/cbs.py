"""Conflict-Based-Search implementation using A* as a backbone."""


from typing import Dict, List, NamedTuple, Set, Tuple


import plotly.graph_objects as go

from .path_planner import Agent, AgentPaths, Goal, PathPlanner
from ..omap import OMap
from ..utils import Path, Point

Solution = Dict[Agent, Tuple[Path, float]]


class Conflict(NamedTuple):
    """Conflict for CBS."""

    agent_set: Set[Agent]
    vertex: Point
    time: int


class Constraint(NamedTuple):
    """Contraint for CBS."""

    agent: Agent
    vertex: Point
    time: int


class Node(NamedTuple):
    """Node for CBS."""

    constraint_set: set[Constraint]
    solution: Solution
    cost: float


class CBS(PathPlanner):
    """Conflict Based Search PathPlanner."""

    @staticmethod
    def single_agent_astar(
        omap: OMap, goals: List[Goal]
    ) -> Tuple[Path, float]:  # pyright: ignore[reportGeneralTypeIssues]
        """Run Astar for a single agent, returning path and cost."""
        pass

    @staticmethod
    def validate_solution(omap: OMap, solution: Solution) -> Conflict | None:
        """Returns Conflicts within an AgentPaths."""
        agent_paths_cells = {}
        for agent, path in solution.items():
            agent_paths_cells[agent] = omap._glbl_pts_to_cells(path[0])

        agents = solution.keys()
        curr_timestep = 0
        max_len = max(agent_paths_cells.values(), key=len)
        while curr_timestep < max_len:
            # TODO: add support for Points dictionary and then stack all the values
            # use: any(np.equal(a,[1,2]).all(1)) to look for match in numpy array

            coordinates: Dict[Agent, Point] = {}
            for agent in agents:
                path = agent_paths_cells[agent]
                if len(path) < curr_timestep + 1:
                    pt = path[-1]
                else:
                    pt = path[curr_timestep]
                # TODO: points = omap.(get points) and use points instead of pt
                existing_values = list(coordinates.values())
                if pt in existing_values:
                    # TODO: Create new constraints with agent and other agent
                    other_agent = list(coordinates.keys())[existing_values.index(pt)]

                    return Conflict({agent, other_agent}, pt, curr_timestep)
                else:
                    coordinates[agent] = pt
        # No conflicts if this point reached
        return None

    @staticmethod
    def generate(
        omap: OMap,
        goals: Dict[Agent, List[Goal]],
        override_starting_pos: None | Dict[Agent, Point] = None,
    ) -> Tuple[AgentPaths, go.Figure]:  # pyright: ignore[reportGeneralTypeIssues]
        """Conflict Base Search PathPlanner."""

        def _get_cost(soln: Solution) -> float:
            """Sum costs of paths."""
            return sum(j for _, j in soln.values())

        # Relies heavily on pseudocode:
        # https://www.sciencedirect.com/science/article/pii/S0004370214001386
        initial_constraint: Set[Constraint] = set()
        initial_solution: Solution = {}
        for agent, goal_list in goals.items():
            initial_solution[agent] = CBS.single_agent_astar(omap, goal_list)

        initial_cost = _get_cost(initial_solution)
        initial_node = Node(initial_constraint, initial_solution, initial_cost)
        explored_node_set: Set[Node] = set()  # avoid duplicates
        explore_list: List[Node] = [initial_node]  # "OPEN" in paper

        while explore_list != []:
            # Sort the open list
            cur_node = explore_list.pop(-1)  # Pop last element (lowest cost)
            explored_node_set.add(cur_node)

            conflict = CBS.validate_solution(omap, cur_node.solution)

            if conflict is None:  # Solution had been found
                # Create agentPaths and Figure
                agent_paths = {k: v[0] for k, v in cur_node.solution.items()}
                return (agent_paths, go.Figure())
            for agent in conflict.agent_set:
                constraint_set = {
                    Constraint(agent, conflict.vertex, conflict.time)
                }  # + cur_node.constraints
                solution = cur_node.solution
                solution[agent] = CBS.single_agent_astar(omap, goals[agent])
                cost = _get_cost(solution)
                explore_list.append(Node(constraint_set, solution, cost))

            explore_list.sort(key=lambda a: a.cost)
