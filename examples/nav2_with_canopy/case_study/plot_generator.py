import plotly.express as px
import pandas as pd

navigate_through_poses = pd.read_csv("./navigate_through_poses_coverage.csv")
fig = px.scatter(navigate_through_poses, x="Percent Covered", y="Coverage Type", color_discrete_sequence=["#440154", "#21918c"], orientation="h")
fig.update_traces(marker_size=50)
fig.update_layout(
    plot_bgcolor="white",
    font=dict(
        size=30,
    ),
)
fig.update_xaxes(
    mirror=True,
    ticks="outside",
    showline=True,
    linecolor="black",
    gridcolor="lightgrey",
    range=[0, 1],
)
fig.update_yaxes(
    mirror=True,
    ticks="outside",
    showline=True,
    linecolor="black",
    gridcolor="lightgrey",
)
fig.show()

navigate_to_pose = pd.read_csv("./navigate_to_pose_coverage.csv")
fig = px.box(navigate_to_pose, x="Percent Covered", y="Coverage Type", color_discrete_sequence=["#440154", "#21918c"])
fig.update_layout(
    plot_bgcolor="white",
    font=dict(
        size=30,
    ),
)
fig.update_xaxes(
    mirror=True,
    ticks="outside",
    showline=True,
    linecolor="black",
    gridcolor="lightgrey",
    range=[0, 1],
)
fig.update_yaxes(
    mirror=True,
    ticks="outside",
    showline=True,
    linecolor="black",
    gridcolor="lightgrey",
)
fig.show()

lcov = pd.read_csv("./lcov_coverage.csv")
fig = px.scatter(lcov, x="Percent Covered", y="Coverage Type", color="Nav2 Package", color_discrete_sequence=["#440154", "#21918c"])
fig.update_traces(marker_size=50)
fig.update_layout(
    plot_bgcolor="white",
    legend=dict(
        x=0,
        y=0,
    ),
    font=dict(
        size=30,
    ),
)
fig.update_xaxes(
    mirror=True,
    ticks="outside",
    showline=True,
    linecolor="black",
    gridcolor="lightgrey",
    range=[0, 1],
)
fig.update_yaxes(
    mirror=True,
    ticks="outside",
    showline=True,
    linecolor="black",
    gridcolor="lightgrey",
)
fig.show()

