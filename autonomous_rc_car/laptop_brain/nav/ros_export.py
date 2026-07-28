"""ROS-free helpers that turn nav state into data ready for ROS2 messages.

Kept ROS-free (plain numpy / Python) so it stays unit-testable on any machine;
the ROS2 nodes wrap these outputs in nav_msgs/OccupancyGrid and sensor_msgs/
PointCloud2 messages.

Frame convention: the nav grid lives in the gravity-aligned floor plane where
ARKit +Y is up and the floor is the world x-z plane (rows index z, cols index x).
The exported OccupancyGrid is therefore a top-down map whose ROS x = world x and
ROS y = world z, drawn flat at the floor height.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from nav.grid import FREE, OCCUPIED

# ROS occupancy cost values (nav_msgs/OccupancyGrid data is int8, 0..100, -1 unknown).
ROS_UNKNOWN = -1
ROS_FREE = 0
ROS_INFLATED = 99   # inside the robot-radius inflation buffer (not the obstacle itself)
ROS_OCCUPIED = 100


@dataclass
class OccupancyExport:
    data: list          # row-major int8, length width*height (ROS OccupancyGrid.data)
    width: int          # cols (ROS x = world x)
    height: int         # rows (ROS y = -world z)
    resolution: float   # meters per cell
    origin_x: float     # ROS x of the data[0] corner
    origin_y: float     # ROS y of the data[0] corner


def grid_to_occupancy(grid) -> OccupancyExport:
    """Map a nav OccupancyGrid to a Z-up ROS nav_msgs/OccupancyGrid payload.

    Value mapping: UNKNOWN->-1, FREE->0, inflation buffer->99, OCCUPIED->100.

    The nav grid indexes rows by world z, cols by world x. The ROS map is Z-up
    with ROS x = world x and ROS y = -world z (see nav.frames), and its
    ``data[gy*width + gx]`` starts at the min-corner and increases with +x,+y.
    Since +ROS-y = -world-z, the rows are flipped and origin_y negated so the
    published grid lands under the (also z-up) cloud and voxels.
    """
    cells = np.asarray(grid.cells)
    out = np.full(cells.shape, ROS_UNKNOWN, dtype=np.int8)
    out[cells == FREE] = ROS_FREE
    if grid.blocked is not None:
        out[np.asarray(grid.blocked) & (cells != OCCUPIED)] = ROS_INFLATED
    out[cells == OCCUPIED] = ROS_OCCUPIED

    rows, cols = cells.shape
    cs = float(grid.cell_size)
    return OccupancyExport(
        data=np.flipud(out).ravel(order="C").astype(np.int8).tolist(),
        width=int(cols),
        height=int(rows),
        resolution=cs,
        origin_x=float(grid.origin[0]),
        origin_y=-(float(grid.origin[1]) + rows * cs),
    )


def categorize_points(xyz, floor_y, floor_band=0.04, clearance=0.06,
                      robot_height=0.35):
    """Split a (cleaned) cloud into visualization layers by height band.

    Uses the same bands as ``mapping.build_occupancy_grid`` so the voxel view
    matches the occupancy grid:
      |y - floor| <= floor_band                -> 'ground' (drivable floor)
      floor+clearance < y < floor+robot_height -> 'obstacle' (body would hit it)

    Returns {'ground': [N,3] float32, 'obstacle': [M,3] float32}.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.size == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return {"ground": empty, "obstacle": empty.copy()}
    y = xyz[:, 1]
    ground = xyz[np.abs(y - floor_y) <= floor_band]
    obstacle = xyz[(y > floor_y + clearance) & (y < floor_y + robot_height)]
    return {"ground": np.ascontiguousarray(ground),
            "obstacle": np.ascontiguousarray(obstacle)}
