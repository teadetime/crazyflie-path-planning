from typing import List

import numpy as np
import cvxpy as cp

from scipy.spatial import HalfspaceIntersection

class FreespacePolytopes():
    """An implementation of the IRIS free-space convex partitioning algorithm
    as described by Diets et al, 2014. TODO: Also implement the intelligent automatic seeding
    algorithm as described by Diets et al, 2015.
    """

    # TODO: Consider storing the As, bs outputs?

    obstacles: List[np.ndarray]
    # TODO: Maybe make a single polytope an object with A, b, vertices?
    # And then we can just have a list of the polytopes.
    As: List[np.ndarray]
    bs: List[np.ndarray]
    vertices: List[np.ndarray]

    def __init__(self, obstacles: List[np.ndarray]):
        self.obstacles = obstacles

    def __iter__(self):
        return zip(self.As, self.bs, self.vertices)

    def _find_hyperplanes(self, C, d):
        # Step 1: Find separating hyperplanes which will allow further expansion of the ellipse:
        
        # For each obstacle, find the point on its boundary which is closest to the origin
        # by solving the quadratic program defined in Eqn.4 in Deits 2014
        # Note that x_tilde is in ball-space!
        n_obs = len(self.obstacles)
        A = np.zeros((n_obs, 2))
        b = np.zeros((n_obs, 1))
        
        # TODO: This can be optimized by checking at the end of each step if there are other obstacles separated by the 
        # the new plane, and then ignoring them in future steps. That will drastically reduce the number of bounding planes many of the polytopes.
        # TODO: This can be optimized by starting with the obstacle closest to the ellipse, working outward
        for j, obs_j in enumerate(self.obstacles):
            # Transform obstacle points from world frame into world-space into ball-space
            obs_j_ball_space = np.linalg.inv(C) @ (obs_j - d)
            
            # Find closest point in ball-space
            x_tilde = cp.Variable((2, 1))
            w = cp.Variable((obs_j.shape[1], 1), nonneg=True)
            obj = cp.Minimize(cp.sum_squares(x_tilde)) # Find point which is closest to origin
            constr = [obs_j_ball_space @ w == x_tilde, cp.sum(w) == 1] # Constrain x to inside of obstacle via convex combination of vertices
            prob = cp.Problem(obj, constr)
            prob.solve()

            # Transform the closest point in ball-space back to world space
            x_star_j = C @ x_tilde.value + d

            # Step 1b: Find a and b defining the plane tangent to ellipse which passes through x_star
            # Eqn 6 of Deits 2014
            C_inv = np.linalg.inv(C)
            A[j, :] = (2 * C_inv @ C_inv.T @ (x_star_j - d)).T
            b[j] = A[j,:] @ x_star_j
        return A, b

    def _maximize_ellipsoid(self, A, b):
        """Find the largest ellipsoid that can fit within the bounding planes defined by A, b.
        An implementation of the semidefinite program in Eq. 10 in Diets 2014.

        Args:
            A (np.ndarray): Bounding line definition matrix
            b (np.ndarray): Bounding line definition vector

        Returns:
            C: Ellipse definition matrix: Cx for x in unit circle transforms the circle into an ellipse
            d: Centerpoint of ellipse
        """
        # Find the largest ellipsoid that can fit within the given set of planes
        # Implementation of the semidefinite pr"ogram in Eq. 10 in Diets 2014.
        C = cp.Variable((2, 2), PSD=True)
        d = cp.Variable((2, 1))
        
        obj = cp.Maximize(cp.log_det(C))
        constr = []
        for i, a in enumerate(A):
            constr += [cp.norm(a @ C) + a @ d <= b[i]]
        
        prob = cp.Problem(obj, constr)
        prob.solve()
        
        return C.value, d.value

    def _find_poly(self, seed_pt: np.ndarray, stop_rate: float = 0.1, r_start=0.1):
        """An implementation of the IRIS free-space convex partitioning algorithm from Diets et al 2014.
        Given an initial seed point, find a large polytope in free-space by alternating between
        growing an ellipse from the seed point up bounding planes, and then redefining bounding planes
        based on the new ellipse shape.

        Args:
            startpoint (np.ndarray): Seed point in configuration space to start the algorithm. A ellipse (circle) of radius r_start is placed there
            stop_rate (float, optional): If growth of ellipse volume is less than this percent then algorithm will terminate. Defaults to 0.1.
        """
        n_dims = len(seed_pt)
        d_0 = np.atleast_2d(seed_pt.flatten()).T
        C_0 = r_start * np.eye(n_dims)
        
        last_C = C_0
        while True:
            A, b = self._find_hyperplanes(C_0, d_0)
            C, d = self._maximize_ellipsoid(A, b)
            
            f_log_det = lambda A: np.log(np.linalg.det(A))
            if (f_log_det(C) - f_log_det(last_C)) / f_log_det(last_C) <= stop_rate:
                break
            last_C = C
        
        return A, b, d
    
    def convex_freespace_decomp(self):
        """Decompose the overall obstacle space into individual convex regions.

        Returns:
            As: list of A matrices
            bs: list of b vectors
        """
        start_posns = np.array([
            [3,3],
            [-3,3],
            [-3,-3],
            [3,-3]
        ])

        # Note we save d here for finding the polytope vertices from planes via HalfspaceIntersect
        # HalfspaceIntersect requires a point within the polytope, which we use d for.
        self.As, self.bs, self.ds, self.vertices = [], [], [], []
        for start_posn in start_posns:
            A_i, b_i, d_i = self._find_poly(start_posn)

            # Find vertices
            hs = HalfspaceIntersection(np.hstack([A_i, -b_i]), d_i.flatten()).intersections

            # reorder the halfspace intersection points by their polar angle
            # This way we make sure we're always plotting the points going around the perimeter
            hs_center = np.mean(hs, axis=1)
            thetas = np.arctan2(hs[:, 1] - hs_center[1], hs[:, 0] - hs_center[0])
            vertices_i = hs[np.argsort(thetas)]

            self.As.append(A_i)
            self.bs.append(b_i)
            self.vertices.append(vertices_i)

