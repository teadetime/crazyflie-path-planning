import numpy as np
from pyinstrument import Profiler

from .algorithms.gcs_lib.FreespacePolytopes import FreespacePolytopes
from .algorithms.gcs_lib.GraphOfConvexSets import GraphOfConvexSets

from itertools import combinations_with_replacement

def make_obs_for_joint_config_space(obs, n_agents):
    hyper_polytopes = []

    obs_points = np.hstack([obstacle for obstacle in obs])
    min_coords = np.amin(obs_points, axis=1)
    max_coords = np.amax(obs_points, axis=1)

    for obs_i in obs:
        if n_agents < 2:
            return obs

        # Build one hyperpolytope
        # For an obstacle with vertices [[x1, y1], [x2, y2], [x3, y3]]:
        # - If configuration space were 3d (there's an additional 1d joint) then the obstacle would hold 
        # for any z within range [z_min, z_max].
        # - Thus the obstacle points would be [[x1, y1, z_min], [x1, y1, z_max], [x2, y2, z_min], [x2, y2, z_max], ...]
        # - In 4d you would have yet another layer for each of these, etc etc.
        # Here we build these "hyper-obstacles" (like hypercube), given the original 2d obstacle
        V = obs_i
        for i in range(n_agents - 1):
            ones = np.ones((1, V.shape[1]))
            # Add new x
            V = np.hstack([np.vstack([V, max_coords[0] * ones]), np.vstack([V, min_coords[0] * ones])])

            # Add new y
            ones = np.ones((1, V.shape[1]))
            V = np.hstack([np.vstack([V, max_coords[1] * ones]), np.vstack([V, min_coords[1] * ones])])

        # Now make the equivalent hyperpolytope for the other agents
        # We can just shift where the position points are.
        for n in range(n_agents):
            hyper_polytopes.append(np.roll(V, n*2, axis=0))
        
    return hyper_polytopes

def main():
    profiler = Profiler()
    profiler.start()

    # Example scene
    obs_cube1 = np.array([
        [1, 0],
        [0, 1],
        [-1, 0],
        [0, -1],
    ]).T
    obs_cube2 = np.array([
        [3,4],
        [4,4],
        [4,5],
        [3,5],
    ]).T
    r_wall = np.array([
        [7.5, 7],
        [7, 7],
        [7, -7],
        [7.5, -7],
    ]).T
    t_wall = np.array([
        [7, 7.5],
        [-7, 7.5],
        [-7, 7],
        [7, 7],
    ]).T

    l_wall = np.diag([-1, 1]) @ r_wall
    b_wall = np.diag([1, -1]) @ t_wall

    obs = [obs_cube1, obs_cube2, r_wall, t_wall, l_wall, b_wall]

    obs_config_space = make_obs_for_joint_config_space(obs, 2)
    polys = FreespacePolytopes(obs_config_space)

    profiler.stop()
    profiler.print()