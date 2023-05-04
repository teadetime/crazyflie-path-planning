import numpy as np
import cvxpy as cp
from scipy.spatial import HalfspaceIntersection

from .FreespacePolytopes import FreespacePolytopes

class GraphOfConvexSets:

    polys: FreespacePolytopes
    mat_edges: np.ndarray

    def __init__(self, polys: FreespacePolytopes):
        self.polys = polys
            
        # Build adjacency matrix
        n_polys = len(polys.As) #TODO: It's ugly to just get the length of one of the properties
        mat_graph_adj = np.zeros((n_polys, n_polys))
        for i, (A_i, b_i, vertices_i) in enumerate(polys):
            # For each polygon, loop through all polygons to find others that overlap
            for j, (A_j, b_j, vertices_j) in enumerate(polys):
                # Pass if on diagonal: nodes are not connected to self
                if j == i:
                    continue
                    
                # Check if any vertices of this polygon_j overlaps with polygon_i
                # Note we use 1e-5 here because sometimes an overlapping polytope's vertex is on the edge of the polytope it overlaps.
                verts_overlapping = np.max(A_j @ vertices_i.T - b_j, axis=0) <= 1e-5
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
        self.mat_edges = np.array(mat_edges)