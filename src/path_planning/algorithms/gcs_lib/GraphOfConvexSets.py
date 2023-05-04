import copy

import numpy as np
import cvxpy as cp
from scipy.spatial import HalfspaceIntersection

from .FreespacePolytopes import Polytope, FreespacePolytopes

class GraphOfConvexSets:

    polys: FreespacePolytopes
    mat_edges: np.ndarray

    def __init__(self, polys: FreespacePolytopes):
        self.polys = polys
        self.mat_edges = self._get_mat_edges(polys)
            
    @staticmethod
    def _get_mat_edges(polys: FreespacePolytopes):
        # Build adjacency matrix
        n_polys = len(polys) #TODO: It's ugly to just get the length of one of the properties
        mat_graph_adj = np.zeros((n_polys, n_polys))
        for i, poly_i in enumerate(polys):
            # For each polygon, loop through all polygons to find others that overlap
            for j, poly_j in enumerate(polys):
                # Pass if on diagonal: nodes are not connected to self
                if j == i:
                    continue
                    
                # Check if any vertices of this polygon_j overlaps with polygon_i
                # Note we use 1e-5 here because sometimes an overlapping polytope's vertex is on the edge of the polytope it overlaps.
                verts_overlapping = np.max(poly_j.A @ poly_i.vertices.T - poly_j.b, axis=0) <= 1e-5
                overlapping = np.any(verts_overlapping) # If any vertex overlaps with the current polytope, then there is overlap.
                if overlapping:
                    # Set connection to 1 in adjacency matrix
                    mat_graph_adj[i, j] = 1
                    
        # Build the list of edges from the adjacency matrix
        mat_edges = []
        for i, row in enumerate(mat_graph_adj):
            for j, target_edge in enumerate(row):
                if target_edge > 0:
                    mat_edges.append(np.array([i, j]))
        mat_edges = np.array(mat_edges)
        return mat_edges

    ## Helper functions for building the optimization problem
    # Get the set of incoming and outgoing edges for a given vertex v
    @staticmethod
    def _get_io_edges(v, edges):
        """Get indices of edges that are inbound or outbound from a given vertex

        Args:
            v (int): Index of vertex of interest
            edges (np.ndarray): Edge matrix

        Raises:
            ValueError: Neither incoming nor outgoing edges are found for the specified node

        Returns:
            Tuple[np.ndarray, np.ndarray]: Returns two lists, respectively containing indices for inbound and outbound edges
        """
        edges_in = np.where(edges[:, 1] == v)[0] # Edges that end with our vertex are coming in
        edges_out = np.where(edges[:, 0] == v)[0] # Edges that begin with our vertex are going out
        
        if (not np.any(edges_in)) and (not np.any(edges_out)):
            raise ValueError("Neither incoming nor outgoing edges found for specified node")
        
        return edges_in, edges_out

    def solve(self, x_0: np.ndarray, x_goal: np.ndarray):
        """Solves the shortest path problem over a graph of convex sets given start and end points

        Args:
            x_0 (np.ndarray): Start point
            x_goal (np.ndarray): End point

        Returns:
            np.ndarray: The sequence of points x which form the optimal path
        """
        # Find polytopes (graph nodes) which contain the start and end points
        s_poly, t_poly = 0, 0
        for i, poly in enumerate(self.polys):
            if np.all(poly.A @ np.atleast_2d(x_0).T - poly.b <= 0) and (s_poly == 0):
                s_poly = i
            if np.all(poly.A @ np.atleast_2d(x_goal).T - poly.b <= 0) and (t_poly == 0):
                t_poly = i

        # Insert additional start and endpoint nodes into the graph
        # Note: If we directly impose constraints on position of points in the regions, this makes problems overdefined and unsolvable.
        # Start and end point constraints call for the creation of new graph nodes to avoid overconstraining the problem.
        # Currently, we just insert a node that's a copy of the node with the polytope the start/end points are in.
        n_polys = len(self.polys)
        s, t = n_polys, n_polys + 1
        start_edge, end_edge = [s, s_poly], [t_poly, t]
        # TODO: If we have a better underlying graph structure, then these can be formulated as just adding two nodes and edges. This is kinda hacky
        mat_edges_st = np.vstack([self.mat_edges, start_edge, end_edge])
        polys_st = copy.deepcopy(self.polys)
        polys_st.extend([polys_st[s_poly], polys_st[t_poly]])

        # Define decision variables
        num_edges = len(mat_edges_st)
        z = cp.Variable((2, num_edges))
        z_prime = cp.Variable((2, num_edges))
        # Unfortunately cp.perspective cannot take elements of a vector for its 's' argument
        # So we have to manually make a list of individual y variables that we feed to cp.perspective one by one.
        ys = []
        for i in range(num_edges):
            y = cp.Variable(1, boolean=True)
            ys.append(y)

        cost = 0
        constr = []

        ## Construct costs and constraints that are per-edge
        for e, edge in enumerate(mat_edges_st):
            u, v = edge
            # Sum of squared distace along each edge connecting the vertices in each polytope
            f = cp.norm(z[:, e] - z_prime[:, e])
            f_recession = 0 * f
            cost += cp.perspective(f, ys[e], f_recession) # Eqn 5.5a
            
            # Edge constraint: edge must belong to the perspective cone of the free polytopes
            constr += [polys_st[u].A @ z[:, e] - cp.vec(ys[e] * polys_st[u].b) <= 0]
            constr += [polys_st[u].A @ z_prime[:, e] - cp.vec(ys[e] * polys_st[u].b) <= 0] # Eqn 5.5d
            
        ## Construct costs and constraints that are per-vertex
        # for i, (A_v, b_v) in enumerate(zip(As_st, bs_st)):
        for i, poly in enumerate(polys_st):
            if (i == s) or (i == t):
                continue
            # Flow constraints
            edges_in, edges_out = self._get_io_edges(i, mat_edges_st)
            
            # Constraint 5.5c, the part for y_e, and the mutual-equivalence part of 5.5b
            y_sum_in, y_sum_out = 0, 0 # Manually sum y by looping :( sadly because ys is in silly(python) vector form
            for y in [ys[i] for i in edges_in]:
                y_sum_in += y
            for y in [ys[i] for i in edges_out]:
                y_sum_out += y
            constr += [y_sum_in == y_sum_out]
            
            constr += [cp.sum(z_prime[:, edges_in], axis=1) == cp.sum(z[:, edges_out], axis=1)] # the z-part of 5.5c

            ## Eliminate 2-cycles
            # - GCS Control paper (Motion Planning around Obstacles with Convex Optimization) Appendix A.1
            # - Constrain all nodes to only have one output
            # - Example implementation: https://github.com/RobotLocomotion/drake/blob/386ef0b4985ee636777324f4dab94e0141aca832/geometry/optimization/graph_of_convex_sets.cc#L798
            constr += [y_sum_out <= 1]
            
            # Now forbid all 2-cycles in/out of this vertex. 
            # - See Appendix A.1, or Line 819 in the Drake implementation.
            for e_out in edges_out:
                for e_in in edges_in:
                    if mat_edges_st[e_in, 0] == mat_edges_st[e_out, 1]:
                        # Cyclical edge pair detected. Add flow constraint to prevent a cyclical path.
                        flow_diff = y_sum_out - ys[e_in] - ys[e_out] 
                        constr += [y_sum_out - ys[e_in] - ys[e_out] >= 0]
                        
                        # Spatial flow constraint:
                        # - Mentioned in passing in Appendix A.1, or Line 824 in Drake implementation.
                        # - Not neccessary but gives additional tightness for the convex relaxation - makes solving faster.
                        # - Keep in mind {z, z_prime} in GCS paper is {y, z} in Drake impl.
                        v_spatial_flow = cp.sum(z_prime[:, edges_in], axis=1) - z_prime[:, e_out] - z[:, e_in]
                        constr += [poly.A @ v_spatial_flow - cp.vec(flow_diff * poly.b) <= 0]
                        
                        
        ## Start/end point constraints
        _, edges_out_of_s = self._get_io_edges(s, mat_edges_st)
        edges_into_t, _ = self._get_io_edges(t, mat_edges_st)

        # Constraint 5.5b
        y_sum_s, y_sum_t = 0, 0
        for y in [ys[i] for i in edges_out_of_s]:
            y_sum_s += y
        for y in [ys[i] for i in edges_into_t]:
            y_sum_t += y
        constr += [y_sum_s == y_sum_t, y_sum_s == 1]

        # Constrain the start and end points to be at their defined positions
        # Boundary condition 1f in the GCS Control Paper
        constr += [cp.sum(z[:, edges_out_of_s], axis=1) == x_0, cp.sum(z_prime[:, edges_into_t], axis=1) == x_goal]

        ## Solve the problem!!
        prob = cp.Problem(cp.Minimize(cost), constr)
        prob.solve(solver="SCIP")