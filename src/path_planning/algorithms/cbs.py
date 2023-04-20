"""Conflict-Based-Search implementation using A* as a backbone."""


from queue import PriorityQueue
from typing import Dict, FrozenSet, List, NamedTuple, Optional, Set, Tuple

import numpy as np
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


class CBSNode(NamedTuple):
    """Node for CBS."""

    constraint_set: frozenset[Constraint]
    solution: Solution
    cost: float


class CBS(PathPlanner):
    """Conflict Based Search PathPlanner."""

    @staticmethod
    def single_agent_astar(  # noqa: C901
        omap: OMap,
        start: Point,
        goals: List[Goal],
        existing_path: Path | None = None,
        constraint: Constraint | None = None,
    ) -> Tuple[Path, float] | None:
        """Run Astar for a single agent, returning path and cost."""
        came_from: dict[bytes, Optional[bytes]] = {}
        cost_so_far: dict[bytes, float] = {}
        time: dict[bytes, int] = {}

        def get_neighbors_cells(
            time: int, pt: Point, cnst: Constraint | None = None
        ) -> List[Point]:
            # Cell is the size of a the drone doing a very simple astar
            points = []
            pt_int = pt.astype(np.int64)
            for x in range(-1, 2):
                for y in range(-1, 2):
                    for z in range(-1, 2):
                        point = pt_int + np.array([x, y, z], dtype=np.int64)
                        if ((point >= omap.map.shape) | (point < ([0, 0, 0]))).any():
                            continue
                        if cnst is not None and time == cnst.time:
                            if not np.array((point - cnst.vertex), dtype=bool).any():
                                continue
                        if not bool(omap.map[(point[0], point[1], point[2])].any()):
                            points.append(point.astype(np.float64))
            return points

        def heuristic(pos: Point, goal: Point) -> float:
            return float(np.linalg.norm(goal - pos))

        def reconstruct_path(start: Point, goal: Point) -> Path | None:
            current = goal.tobytes()
            start_bytes: bytes = start.tobytes()
            path: Path = np.ones((1, 4))
            if goal.tobytes() not in came_from:  # no path was found
                return None
            while current != start_bytes and current is not None:
                path = np.vstack(
                    (
                        np.append(
                            np.frombuffer(current).astype(np.int64),
                            cost_so_far[current],
                        ),
                        path,
                    )
                )
                current = came_from[current]

            path = np.vstack((np.append(start, cost_so_far[start_bytes]), path[:-1, :]))
            return path

        if existing_path is None:
            start_byte = start.tobytes()
            came_from[start_byte] = None
            cost_so_far[start_byte] = 0
            time[start_byte] = 0

            frontier: PriorityQueue = PriorityQueue()
            frontier.put((0, start_byte))

            goal = goals[0]

            while not frontier.empty():
                _, current_bytes = frontier.get()
                current_point = np.frombuffer(current_bytes)
                curr_time = time[current_bytes]
                if not np.array((current_point - goal.pos), dtype=bool).any():
                    path = reconstruct_path(start, current_point)
                    if path is not None:
                        return (path, path[-1])
                    break

                new_neighbors = get_neighbors_cells(
                    curr_time + 1, current_point, constraint
                )  # constraint should be None

                for next in new_neighbors:
                    new_cost = cost_so_far[current_bytes] + float(
                        np.linalg.norm((next, current_point))
                    )
                    next_bytes = next.tobytes()
                    if (
                        next_bytes not in cost_so_far
                        or new_cost < cost_so_far[next_bytes]
                    ):
                        cost_so_far[next_bytes] = new_cost
                        time[next_bytes] = curr_time + 1
                        priority = new_cost + heuristic(next, goal.pos)
                        frontier.put((priority, next_bytes))
                        came_from[next_bytes] = current_bytes
            return None

        else:
            # Error if there isn't a constraint
            if constraint is None or existing_path is None:
                raise ValueError("Specify constraint if using an existing path")
            # Start the timestep before the constraint and append things appropriately
            starting_time = constraint.time

            path_pre_constriant = existing_path[:starting_time, :]
            starting_point: Point = existing_path[starting_time, :-1]

            start_byte = starting_point.tobytes()
            came_from[start_byte] = None
            cost_so_far[start_byte] = existing_path[starting_time, -1]
            time[start_byte] = starting_time

            frontier: PriorityQueue = PriorityQueue()
            frontier.put((existing_path[starting_time, -1], start_byte))
            goal = goals[0]

            while not frontier.empty():
                _, current_bytes = frontier.get()
                current_point = np.frombuffer(current_bytes)
                curr_time = time[current_bytes]
                if not np.array((current_point - goal.pos), dtype=bool).any():
                    path = reconstruct_path(starting_point, current_point)
                    if path is not None:
                        return (np.vstack((path_pre_constriant, path)), path[-1])
                    break

                new_neighbors = get_neighbors_cells(
                    curr_time + 1, current_point, constraint
                )

                for next in new_neighbors:
                    new_cost = cost_so_far[current_bytes] + float(
                        np.linalg.norm((next, current_point))
                    )
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        next_bytes = next.tobytes()
                        cost_so_far[next_bytes] = new_cost
                        time[next_bytes] = curr_time + 1
                        priority = new_cost + heuristic(next, goal.pos)
                        frontier.put((priority, next_bytes))
                        came_from[next_bytes] = current_bytes
            return None

    @staticmethod
    def validate_solution(omap: OMap, solution: Solution) -> Conflict | None:
        """Returns Conflicts within an AgentPaths."""
        agent_paths_cells = {}
        for agent, path in solution.items():
            agent_paths_cells[agent] = path[0][:, :-1]  # Cut off cost

        agents = solution.keys()
        curr_timestep = 0
        max_len = len(max(agent_paths_cells.values(), key=len))
        while curr_timestep < max_len:
            time_coordinates: Dict[Agent, Point] = {}
            for agent in agents:
                path = agent_paths_cells[agent]
                # Use the last agent position if it is done moving
                if len(path) < curr_timestep + 1:
                    pt = path[-1]
                else:
                    pt = path[curr_timestep]
                existing_values = list(time_coordinates.values())
                print(f"pt: {pt}, {existing_values}")
                if pt in existing_values:
                    other_agent = list(time_coordinates.keys())[
                        existing_values.index(pt)
                    ]
                    return Conflict({agent, other_agent}, pt, curr_timestep)
                else:
                    time_coordinates[agent] = pt
        # No conflicts
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

        starting_pos: Dict[Agent, Point] = {}
        if override_starting_pos is not None:
            starting_pos = override_starting_pos
        else:
            for agent in goals.keys():
                starting_pos[agent] = agent.pos

        initial_constraint: FrozenSet[Constraint] = frozenset()
        initial_solution: Solution = {}
        for agent, goal_list in goals.items():
            single_a_result = CBS.single_agent_astar(
                omap, starting_pos[agent], goal_list
            )
            if single_a_result is None:
                raise RuntimeError("Unable to find astar path")
            initial_solution[agent] = single_a_result

        initial_cost = _get_cost(initial_solution)
        initial_node = CBSNode(initial_constraint, initial_solution, initial_cost)
        # explored_node_set: List[CBSNode] = list()  # avoid duplicates
        explore_list: List[CBSNode] = [initial_node]  # "OPEN" in paper

        while explore_list != []:
            # Sort the open list
            cur_node = explore_list.pop(-1)  # Pop last element (lowest cost)

            # TODO: keep track of where you've gone so you don't do it again
            # explored_node_set.append(cur_node)

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
                single_a_result = CBS.single_agent_astar(
                    omap, starting_pos[agent], goals[agent]
                )
                if single_a_result is None:
                    raise RuntimeError("Unable to find astar path")
                solution[agent] = single_a_result
                cost = _get_cost(solution)
                explore_list.append(CBSNode(constraint_set, solution, cost))

            explore_list.sort(key=lambda a: a.cost)
