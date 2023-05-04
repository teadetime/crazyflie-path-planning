import numpy as np

import plotly.graph_objects as go

from scipy.spatial import HalfspaceIntersection
from .FreespacePolytopes import FreespacePolytopes

def plot_obs(obs: np.ndarray, fig=go.Figure()) -> go.Figure:
    for i, obs_i in enumerate(obs):
        fig.add_trace(go.Scatter(
            x=np.concatenate([obs_i[0, :], [obs_i[0, 0]]]),
            y=np.concatenate([obs_i[1, :], [obs_i[1, 0]]]),
            # fill="toself",
            name="Obstacle",
            legendgroup="obstacle",
            showlegend=True if i == 0 else False,
            fill="toself",
            line=dict(color="black")
        ))
    return fig

def plot_ellipse(C: np.ndarray, d: np.ndarray, fig = go.Figure()) -> go.Figure():
    s = np.linspace(0, 2*np.pi, 20)
    unit_circle_pts = np.array([np.cos(s), np.sin(s)])
    
    ellipse_pts = C @ unit_circle_pts + d
    fig.add_trace(go.Scatter(
        x=ellipse_pts[0, :],
        y=ellipse_pts[1, :],
        fill="toself",
        line=dict(color="darkseagreen"),
        name="Ellipse"
    ))

    fig.update_layout(yaxis_scaleanchor="x", width=600, height=500)
    return fig

def plot_zones(polys: FreespacePolytopes, colors, fig=go.Figure()):
    for i, ((A, b, d), color) in enumerate(zip(polys, colors)):
        hs = HalfspaceIntersection(np.hstack([A, -b]), d.flatten()).intersections

        # reorder the halfspace intersection points by their polar angle
        # This way we make sure we're always plotting the points going around the perimeter
        hs_center = np.mean(hs, axis=1)
        thetas = np.arctan2(hs[:, 1] - hs_center[1], hs[:, 0] - hs_center[0])
        hs = hs[np.argsort(thetas)]

        fig.add_trace(go.Scatter(
            x=np.concatenate([hs[:, 0], [hs[0,0]]]),
            y=np.concatenate([hs[:, 1],[hs[0,1]]]),
            fill="toself",
            line=dict(color=color),
            name=f"Region {i}",
        ))
