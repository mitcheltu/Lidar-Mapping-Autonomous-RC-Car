"""Phone-only walkthrough preview: run the mapping pipeline on the live cloud,
using the phone's pose as the pretend car. Returns whatever it could compute
(grid without a goal is still useful) and never raises on thin data."""

from dataclasses import dataclass, field

from nav import frontier, mapping, planner

MIN_POINTS = 100


@dataclass
class PreviewResult:
    grid: object = None
    floor_y: float = None
    goal_cell: tuple = None
    path_world: list = field(default_factory=list)


def preview_plan(map_xyz, pose2d, robot_radius=0.14, cell_size=0.05):
    out = PreviewResult()
    if map_xyz.shape[0] < MIN_POINTS:
        return out
    pts = mapping.clean_cloud(map_xyz)
    if pts.shape[0] < MIN_POINTS:
        return out
    out.floor_y = mapping.estimate_floor_height(pts)
    try:
        grid = mapping.build_occupancy_grid(pts, out.floor_y, cell_size=cell_size)
    except ValueError:
        return out
    out.grid = mapping.inflate(grid, robot_radius)

    x, z, _ = pose2d
    car_cell = out.grid.world_to_cell(x, z)
    out.goal_cell = frontier.choose_goal(out.grid, car_cell)
    if out.goal_cell is None:
        return out
    start = frontier.nearest_passable(out.grid, car_cell)
    if start is None:
        return out
    path = planner.astar(out.grid.passable(), start, out.goal_cell)
    if path:
        out.path_world = [
            out.grid.cell_to_world(r, c)
            for r, c in planner.simplify_path(path, out.grid.passable())]
    return out
