"""Vizualization Class."""

import plotly.graph_objects as go

from path_planning.utils import Points
from .omap import OMap


@staticmethod
def create_omap_trace(omap: OMap, properties: dict | None = None) -> go.Scatter3d:
    """Create Occupancy Map Plotly trace."""
    obstacles = omap.obstacles_in_global()
    if properties is None:
        marker = dict(size=5, symbol="square", color="red", opacity=0.25)
        properties = dict(opacity=1, mode="markers", marker=marker, name="Obstacles")

    omap_plot = go.Scatter3d(
        arg=properties,
        x=obstacles[0],
        y=obstacles[1],
        z=obstacles[2],
    )
    return omap_plot


@staticmethod
def create_points_trace(
    points: Points,  # pyright: ignore[reportGeneralTypeIssues]
    properties: dict | None = None,
) -> go.Scatter3d:
    """Create trace for points."""
    if properties is None:
        text = [str(n) for n in range(len(points))]
        marker = dict(
            size=2,
            symbol="circle",
            color="black",
            opacity=0.25,
        )
        line = dict(color="black", width=1)

        properties = dict(
            marker=marker,
            mode="markers+text+lines",
            textposition="top center",
            showlegend=False,
            opacity=1,
            line=line,
            text=text,
        )

    points_trace = go.Scatter3d(
        arg=properties, x=points[:, 0], y=points[:, 1], z=points[:, 2]
    )
    return points_trace


@staticmethod
def create_plot_update(omap: OMap) -> dict:
    """Create update Dictionary to use with fig.update_layout()."""
    min, max = omap.min_max
    margin = 5
    scene = (
        dict(
            camera=dict(
                eye=dict(x=0.0, y=-0.01, z=1000)
            ),  # Weirdly tuned for aspect ration, should consider a param of the omap
            xaxis=dict(
                nticks=omap.map.shape[0],
                range=[min[0], max[0]],
            ),
            yaxis=dict(
                nticks=omap.map.shape[1],
                range=[min[1], max[1]],
            ),
            zaxis=dict(
                nticks=omap.map.shape[2],
                range=[min[2], max[2]],
            ),
            aspectratio=dict(
                x=omap.map.shape[0],
                y=omap.map.shape[1],
                z=omap.map.shape[2],
            ),
        ),
    )
    margin = (dict(r=margin, l=margin, b=margin, t=margin),)
    return dict({"scene": scene[0], "margin": margin[0]})


@staticmethod
def create_omap_trace_2d(omap: OMap, properties: dict | None = None) -> go.Scatter:
    """Create Occupancy Map Plotly trace."""
    obstacles = omap.obstacles_in_global()
    if properties is None:
        marker = dict(size=5, symbol="square", color="red", opacity=0.25)
        properties = dict(opacity=1, mode="markers", marker=marker, name="Obstacles")

    omap_plot = go.Scatter(
        arg=properties,
        x=obstacles[0],
        y=obstacles[1],
    )
    return omap_plot


@staticmethod
def create_points_trace_2d(
    points: Points,  # pyright: ignore[reportGeneralTypeIssues]
    properties: dict | None = None,
) -> go.Scatter:
    """Create trace for points."""
    if properties is None:
        text = [str(n) for n in range(len(points))]
        marker = dict(
            size=2,
            symbol="circle",
            color="black",
            opacity=0.25,
        )
        line = dict(color="black", width=1)

        properties = dict(
            marker=marker,
            mode="markers+text+lines",
            textposition="top center",
            showlegend=False,
            opacity=1,
            line=line,
            text=text,
        )

    points_trace = go.Scatter(arg=properties, x=points[:, 0], y=points[:, 1])
    return points_trace


@staticmethod
def create_plot_update_2d(figure: go.Figure, omap: OMap) -> None:
    """Create update Dictionary to use with fig.update_layout()."""
    min, max = omap.min_max
    figure.update_xaxes(range=[min[0][:-1], max[0][:-1]], constrain="domain")
    figure.update_yaxes(
        range=[min[1][:-1], max[1][:-1]],
        constrain="domain",
        scaleanchor="x",
        scaleratio=1,
    )
