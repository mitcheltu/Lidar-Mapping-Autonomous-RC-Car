# LiDAR Navigation & Autonomy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the point-cloud viewer into a navigation system: the PC builds a cleaned 2D occupancy grid from the streamed LiDAR map, estimates the car's pose, picks the next frontier to explore, plans an A* path, and drives the car there via a reverse command channel (PC → phone → ESP32 BLE) — with scan windows instead of continuous LiDAR spam, outlier removal, and layered failsafes.

**Architecture:** A pure-Python `nav` package (unit-tested with pytest, no hardware needed) implements the classic modern robot stack — pose source (ARKit VIO) + ICP drift correction → occupancy grid + inflation costmap → frontier exploration → A* global planner → waypoint-follower local controller — and a thin integration layer wires it into the existing `pc_viewer.py`. The Swift app gains a bidirectional socket (receives DRIVE/MODE commands, relays to the ESP32 over BLE, gates LiDAR capture on scan mode) plus a tracking-loss safety stop. The ESP32 firmware is unchanged (its 0.5 s failsafe is one redundancy layer).

**Tech Stack:** Python 3 (numpy, scipy, open3d, pytest) on the PC; Swift/ARKit on iPhone; existing ESP32 Arduino firmware.

---

## Research summary: how modern LiDAR robots do this

This is what current production stacks (ROS 2 Nav2, slam_toolbox, Cartographer) and the research literature converge on, and how each maps to this project:

| Modern practice | What the pros use | What we build here |
|---|---|---|
| **Localization** | Wheel/visual/inertial odometry corrected by scan-to-map matching (slam_toolbox, Cartographer loop closure eliminates accumulated drift) | ARKit 6-DoF VIO pose (already streamed) + opportunistic multi-scale ICP scan-to-map correction after each 360° scan |
| **Map representation** | Probabilistic occupancy grid; Nav2 layered costmap (static + obstacle + inflation layers) | 2D occupancy grid (`unknown/free/occupied`) from height-banded 3D points, with true-distance (EDT) inflation by robot radius |
| **Outlier handling** | Voxel filtering + statistical outlier rejection before the costmap; confidence gating at the sensor | Voxel downsample → statistical outlier removal → radius outlier removal (Open3D), plus ARKit depth-confidence gating on the phone |
| **Global planning** | A*, Dijkstra, D* Lite on the inflated costmap | 8-connected A* with octile heuristic, no corner cutting, line-of-sight path simplification |
| **Local control** | Pure pursuit / DWA / TEB following the global path | Turn-then-drive waypoint follower with proportional heading correction (right-sized for a differential-drive toy car at ~0.3 m/s) |
| **Exploration** | Frontier-based exploration (Yamauchi) — drive to the free/unknown boundary | Frontier detection + clustering + BFS-reachability + nearest-frontier goal selection |
| **Sensing cadence** | Scan at waypoints / on demand, not saturating the pipeline | Scan windows: LiDAR points stream only during commanded 360° spin scans; pose always streams |
| **Redundancy** | Command watchdogs, sensor-health monitoring, multi-layer e-stop | 4 layers: ESP32 0.5 s command failsafe (exists) + PC stale-pose stop + phone ARKit tracking-loss stop + command heartbeat |

Sources:
- [LiDAR-based SLAM: state of the art and new frontiers (arXiv 2311.00276)](https://arxiv.org/pdf/2311.00276)
- [Autonomous Navigation of Indoor Mobile Robots Using 2D LiDAR (MDPI)](https://www.mdpi.com/2227-7390/11/6/1455)
- [Exploration-Based SLAM (e-SLAM) for Indoor Mobile Robot Using Lidar (MDPI)](https://www.mdpi.com/1424-8220/22/4/1689)
- [Regulated Pure Pursuit for Robot Path Tracking (arXiv 2305.20026)](https://arxiv.org/pdf/2305.20026)
- [LiDAR-Driven A* Path Planning with Dynamic Obstacle Avoidance (SAE 2026-01-0030)](https://saemobilus.sae.org/papers/lidar-driven-a-path-planning-dynamic-obstacle-avoidance-autonomous-navigation-ros-2026-01-0030)
- [Adaptive Cost-Map-based Path Planning (arXiv 2510.15336)](https://arxiv.org/html/2510.15336v1)
- [Map Indoor Area Using Lidar SLAM (MathWorks)](https://www.mathworks.com/help/nav/ug/map-indoor-area-using-lidar-slam-and-factor-graph.html)
- Plus the project's own `Navigation-Pipeline.md` (Open3D calls, Yamauchi frontier exploration, Nav2 costmap references).

---

## Conventions (read before any task)

**Coordinate system.** ARKit world frame, gravity-aligned: **+Y is up**, the floor is the **X–Z plane**. All 2D navigation happens in (x, z). The camera looks down its local −Z axis, so world heading is `theta = atan2(fwd.z, fwd.x)` where `fwd = -R[:,2]` of the camera pose. Grid **rows index z, cols index x**.

**Grid cell values.** `UNKNOWN = -1`, `FREE = 0`, `OCCUPIED = 1` (int8). A cell is **passable** iff `FREE` and not inside the inflated `blocked` mask.

**Wire protocol (additions).** Same framing as today (1 byte type + uint32 LE length + payload):
- PC → phone `0x44 'D'` drive: ASCII `L<left>R<right>` (−100…100); phone relays to ESP32 BLE.
- PC → phone `0x4D 'M'` mode: ASCII `SCAN` or `IDLE`; gates LiDAR point capture on the phone.
- Phone → PC `0x54 'T'` tracking: 1 byte, `0` = normal, `1` = limited, `2` = not available.

**Calibration constant.** `TURN_SIGN` in `nav/config.py`: `+1` means "left=+s, right=−s makes theta increase". Flipped once during the field test if the car turns the wrong way.

**Redundancy layers (the "make it redundant" requirement).**
1. ESP32 stops motors if no BLE command for 0.5 s (already in firmware) → PC must heartbeat drive commands every ~0.15 s.
2. PC sends `L0R0` immediately if the latest pose is older than 0.7 s or tracking status ≠ normal.
3. Phone calls `carController.stop()` itself the moment ARKit tracking degrades (works even if the PC link dies).
4. ICP scan-to-map correction snaps VIO drift back onto the map after every 360° scan.

**Test commands** run from the `viewer/` directory: `python -m pytest tests/ -v`.

---

## File structure

```
viewer/
  pc_viewer.py            # MODIFIED: reverse channel, nav tick, overlay, keys
  requirements.txt        # NEW
  pytest.ini              # NEW
  nav/
    __init__.py           # NEW (empty)
    config.py             # NEW: TURN_SIGN + speed/geometry tunables
    grid.py               # NEW: OccupancyGrid dataclass, cell<->world
    mapping.py            # NEW: clean_cloud, floor estimate, grid build, inflate
    localization.py       # NEW: pose matrix -> (x, z, theta), angle_diff
    frontier.py           # NEW: frontier mask, clusters, BFS, choose_goal
    planner.py            # NEW: A*, line-of-sight simplify
    controller.py         # NEW: WaypointFollower
    explorer.py           # NEW: SPIN -> PLAN -> DRIVE -> DONE state machine
    drift.py              # NEW: ICP scan-to-map pose correction
    protocol.py           # NEW: D/M frame builders
    runner.py             # NEW: NavRunner (heartbeat, failsafes, mode switching)
    overlay.py            # NEW: grid/path -> Open3D-ready arrays
  tests/
    test_grid.py, test_mapping.py, test_localization.py, test_frontier.py,
    test_planner.py, test_controller.py, test_explorer.py, test_drift.py,
    test_protocol.py, test_runner.py, test_overlay.py      # NEW
PointCloudScanner/
  PointCloudStreamer.swift  # MODIFIED: receive loop, onDrive/onScanMode, sendTracking
  ContentView.swift         # MODIFIED: CarController wiring, BLE status, scan gating
  ARDepthView.swift         # MODIFIED: pose always streams, points gated, tracking stop
firmware/
  esp32_car.ino             # UNCHANGED (failsafe verified in field test)
```

Each `nav/` file has one responsibility and is unit-testable without hardware, a socket, or a GUI. `pc_viewer.py` stays the only place that touches Open3D visualization and sockets.

---

### Task 1: Package scaffolding + test harness

**Files:**
- Create: `viewer/requirements.txt`
- Create: `viewer/pytest.ini`
- Create: `viewer/nav/__init__.py`
- Create: `viewer/tests/__init__.py`

- [ ] **Step 1: Create the files**

`viewer/requirements.txt`:
```
numpy
scipy
open3d
opencv-python
pytest
```

`viewer/pytest.ini`:
```ini
[pytest]
testpaths = tests
```

`viewer/nav/__init__.py` and `viewer/tests/__init__.py`: empty files.

- [ ] **Step 2: Install and verify the harness runs**

Run: `cd viewer; python -m pip install -r requirements.txt; python -m pytest tests/ -v`
Expected: `no tests ran` (exit code 5 is fine — harness works, no tests yet).

- [ ] **Step 3: Commit**

```bash
git add viewer/requirements.txt viewer/pytest.ini viewer/nav/__init__.py viewer/tests/__init__.py
git commit -m "chore: scaffold nav package and pytest harness"
```

---

### Task 2: OccupancyGrid data structure

**Files:**
- Create: `viewer/nav/grid.py`
- Test: `viewer/tests/test_grid.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_grid.py`:
```python
import numpy as np
import pytest

from nav.grid import OccupancyGrid, UNKNOWN, FREE, OCCUPIED


def make_grid():
    cells = np.full((10, 20), UNKNOWN, dtype=np.int8)
    return OccupancyGrid(cells=cells, origin=(-1.0, -2.0), cell_size=0.05)


def test_world_to_cell_maps_origin_corner_to_zero():
    g = make_grid()
    assert g.world_to_cell(-1.0, -2.0) == (0, 0)


def test_world_to_cell_rows_are_z_cols_are_x():
    g = make_grid()
    # x = -1.0 + 3 cells * 0.05, z = -2.0 + 7 cells * 0.05 (plus a hair inside)
    row, col = g.world_to_cell(-1.0 + 0.16, -2.0 + 0.36)
    assert (row, col) == (7, 3)


def test_cell_to_world_returns_cell_center_and_round_trips():
    g = make_grid()
    x, z = g.cell_to_world(7, 3)
    assert x == pytest.approx(-1.0 + 3.5 * 0.05)
    assert z == pytest.approx(-2.0 + 7.5 * 0.05)
    assert g.world_to_cell(x, z) == (7, 3)


def test_in_bounds():
    g = make_grid()
    assert g.in_bounds(0, 0) and g.in_bounds(9, 19)
    assert not g.in_bounds(-1, 0) and not g.in_bounds(10, 0) and not g.in_bounds(0, 20)


def test_passable_requires_inflation():
    g = make_grid()
    with pytest.raises(ValueError):
        g.passable()


def test_passable_is_free_and_not_blocked():
    g = make_grid()
    g.cells[2, 2] = FREE
    g.cells[2, 3] = FREE
    g.cells[2, 4] = OCCUPIED
    g.blocked = np.zeros_like(g.cells, dtype=bool)
    g.blocked[2, 3] = True   # inflated zone
    p = g.passable()
    assert p[2, 2] and not p[2, 3] and not p[2, 4]
    assert not p[0, 0]  # unknown is never passable
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_grid.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.grid'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/grid.py`:
```python
"""2D occupancy grid over the floor (x-z) plane. Rows index z, cols index x."""

from dataclasses import dataclass

import numpy as np

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(1)


@dataclass
class OccupancyGrid:
    cells: np.ndarray          # int8 [rows, cols] of UNKNOWN / FREE / OCCUPIED
    origin: tuple              # world (x, z) of the corner of cell [0, 0]
    cell_size: float           # meters per cell
    blocked: np.ndarray = None # bool [rows, cols]; obstacles inflated by robot radius

    @property
    def shape(self):
        return self.cells.shape

    def world_to_cell(self, x, z):
        col = int(np.floor((x - self.origin[0]) / self.cell_size))
        row = int(np.floor((z - self.origin[1]) / self.cell_size))
        return row, col

    def cell_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.cell_size
        z = self.origin[1] + (row + 0.5) * self.cell_size
        return x, z

    def in_bounds(self, row, col):
        return 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]

    def passable(self):
        """Bool mask of cells the robot may occupy: FREE and outside inflation."""
        if self.blocked is None:
            raise ValueError("grid not inflated yet -- call mapping.inflate() first")
        return (self.cells == FREE) & ~self.blocked
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_grid.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/grid.py viewer/tests/test_grid.py
git commit -m "feat: OccupancyGrid with world/cell mapping and passable mask"
```

---

### Task 3: Point cloud cleaning (downsample + outlier removal)

This is the "get rid of outlier artifacts" stage: voxel downsample for uniform density, then statistical + radius outlier removal so stray LiDAR specks never become phantom obstacles.

**Files:**
- Create: `viewer/nav/mapping.py`
- Test: `viewer/tests/test_mapping.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_mapping.py`:
```python
import numpy as np

from nav.mapping import clean_cloud


def dense_plane(n_side=60, spacing=0.02, y=0.0):
    """A dense flat floor patch: n_side x n_side points."""
    xs, zs = np.meshgrid(np.arange(n_side) * spacing, np.arange(n_side) * spacing)
    pts = np.stack([xs.ravel(), np.full(xs.size, y), zs.ravel()], axis=1)
    return pts.astype(np.float32)


def test_clean_cloud_removes_isolated_outliers():
    floor = dense_plane()
    outliers = np.array([[5.0, 3.0, 5.0], [-4.0, 2.5, -4.0], [0.5, 9.0, 0.5]],
                        dtype=np.float32)
    cleaned = clean_cloud(np.vstack([floor, outliers]), voxel_size=0.04)
    assert cleaned.shape[0] > 100                    # kept the floor
    assert cleaned[:, 1].max() < 1.0                 # dropped the flying points


def test_clean_cloud_downsamples_dense_regions():
    floor = dense_plane(spacing=0.005)               # much denser than voxel size
    cleaned = clean_cloud(floor, voxel_size=0.04)
    assert cleaned.shape[0] < floor.shape[0] * 0.25


def test_clean_cloud_passes_tiny_clouds_through():
    tiny = np.random.rand(10, 3).astype(np.float32)
    out = clean_cloud(tiny)
    assert out.shape == (10, 3)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_mapping.py -v`
Expected: FAIL with `ImportError: cannot import name 'clean_cloud'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/mapping.py`:
```python
"""Point cloud -> navigable occupancy grid pipeline (the 'costmap' builder)."""

import numpy as np
import open3d as o3d
from scipy import ndimage

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN


def clean_cloud(xyz, voxel_size=0.04, nb_neighbors=20, std_ratio=2.0,
                radius=0.10, min_neighbors=4):
    """Voxel-downsample then drop statistical + radius outliers.

    Returns an [N, 3] float32 array. Clouds under 50 points pass through
    untouched (not enough neighbors for the statistics to mean anything).
    """
    if xyz.shape[0] < 50:
        return xyz.astype(np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd = pcd.voxel_down_sample(voxel_size)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors,
                                            std_ratio=std_ratio)
    pcd, _ = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)
    return np.asarray(pcd.points, dtype=np.float32)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_mapping.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/mapping.py viewer/tests/test_mapping.py
git commit -m "feat: cloud cleaning -- voxel downsample + outlier removal"
```

---

### Task 4: Floor height estimation

**Files:**
- Modify: `viewer/nav/mapping.py` (append function)
- Test: `viewer/tests/test_mapping.py` (append tests)

- [ ] **Step 1: Write the failing test** (append to `viewer/tests/test_mapping.py`)

```python
from nav.mapping import estimate_floor_height


def room_with_table(floor_y=-1.4):
    """Floor plus a smaller 'table top' plane 0.7 m above it."""
    floor = dense_plane(n_side=60, y=floor_y)
    table = dense_plane(n_side=20, y=floor_y + 0.7)
    rng = np.random.default_rng(0)
    noise = np.stack([rng.uniform(0, 1.2, 40),
                      rng.uniform(floor_y, floor_y + 1.0, 40),
                      rng.uniform(0, 1.2, 40)], axis=1).astype(np.float32)
    return np.vstack([floor, table, noise])


def test_floor_is_lowest_dominant_plane_not_the_table():
    pts = room_with_table(floor_y=-1.4)
    assert abs(estimate_floor_height(pts) - (-1.4)) < 0.05


def test_floor_ignores_sparse_low_noise():
    pts = room_with_table(floor_y=0.0)
    low_specks = np.array([[0.1, -0.9, 0.1], [0.9, -0.8, 0.9]], dtype=np.float32)
    assert abs(estimate_floor_height(np.vstack([pts, low_specks]))) < 0.05
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_mapping.py -v`
Expected: new tests FAIL with `ImportError: cannot import name 'estimate_floor_height'`

- [ ] **Step 3: Write the implementation** (append to `viewer/nav/mapping.py`)

```python
def estimate_floor_height(xyz, bin_size=0.02, min_fraction=0.20):
    """Height (y) of the floor: the LOWEST strongly-populated horizontal slab.

    Histogram the y values; the floor is the lowest bin whose count is at
    least `min_fraction` of the largest bin, so sparse below-floor noise is
    skipped but a big table can't win just by having more points.
    """
    ys = xyz[:, 1]
    lo, hi = np.percentile(ys, [1, 99])
    edges = np.arange(lo, hi + bin_size, bin_size)
    if edges.size < 2:
        return float(np.median(ys))
    hist, edges = np.histogram(ys, bins=edges)
    threshold = hist.max() * min_fraction
    idx = int(np.argmax(hist >= threshold))   # first (lowest) qualifying bin
    return float(edges[idx] + bin_size / 2)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_mapping.py -v`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/mapping.py viewer/tests/test_mapping.py
git commit -m "feat: robust floor height estimation from y-histogram"
```

---

### Task 5: Occupancy grid build + inflation

The "simplify the LiDAR map" core: collapse the cleaned 3D cloud into a 2D `unknown/free/occupied` grid using height bands (floor band = free evidence, robot-height band = obstacle), then inflate obstacles by the robot radius using a true Euclidean distance transform (circular inflation, like Nav2's inflation layer).

**Files:**
- Modify: `viewer/nav/mapping.py` (append functions)
- Test: `viewer/tests/test_mapping.py` (append tests)

- [ ] **Step 1: Write the failing test** (append to `viewer/tests/test_mapping.py`)

```python
from nav.grid import FREE, OCCUPIED, UNKNOWN
from nav.mapping import build_occupancy_grid, inflate


def box_room(floor_y=0.0, size=2.0, spacing=0.025):
    """Floor with a 0.3 m box obstacle at the center, plus an overhang to ignore."""
    n = int(size / spacing)
    floor = dense_plane(n_side=n, spacing=spacing, y=floor_y)
    bx, bz = np.meshgrid(np.arange(0.9, 1.1, spacing), np.arange(0.9, 1.1, spacing))
    box = []
    for h in (0.10, 0.20, 0.30):                    # box within robot height
        box.append(np.stack([bx.ravel(), np.full(bx.size, floor_y + h),
                             bz.ravel()], axis=1))
    # a 'shelf' 1 m up over x=0.2..0.4 -- too high to matter, must be ignored
    sx, sz = np.meshgrid(np.arange(0.2, 0.4, spacing), np.arange(0.2, 0.4, spacing))
    shelf = np.stack([sx.ravel(), np.full(sx.size, floor_y + 1.0), sz.ravel()], axis=1)
    return np.vstack([floor] + box + [shelf]).astype(np.float32)


def test_grid_classifies_free_occupied_unknown():
    pts = box_room()
    g = build_occupancy_grid(pts, floor_y=0.0, cell_size=0.05)
    assert g.cells[g.world_to_cell(1.0, 1.0)] == OCCUPIED   # the box
    assert g.cells[g.world_to_cell(0.3, 1.5)] == FREE       # open floor
    assert g.cells[g.world_to_cell(0.3, 0.3)] == FREE       # under the high shelf
    assert g.cells[g.world_to_cell(-0.4, -0.4)] == UNKNOWN  # outside scanned area


def test_inflation_blocks_a_ring_around_obstacles():
    pts = box_room()
    g = inflate(build_occupancy_grid(pts, floor_y=0.0, cell_size=0.05),
                robot_radius=0.12)
    assert g.blocked[g.world_to_cell(1.0, 1.0)]             # obstacle itself
    assert g.blocked[g.world_to_cell(1.0, 0.82)]            # within 0.12 m of box
    assert not g.blocked[g.world_to_cell(1.0, 0.5)]         # well clear of it
    assert g.passable()[g.world_to_cell(1.0, 0.5)]


def test_grid_raises_on_empty_band():
    with pytest.raises(ValueError):
        build_occupancy_grid(np.zeros((0, 3), np.float32), floor_y=0.0)
```

Also add `import pytest` at the top of the file if not already present.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_mapping.py -v`
Expected: new tests FAIL with `ImportError`

- [ ] **Step 3: Write the implementation** (append to `viewer/nav/mapping.py`)

```python
def build_occupancy_grid(xyz, floor_y, cell_size=0.05, floor_band=0.04,
                         clearance=0.06, robot_height=0.35,
                         min_obstacle_points=2, padding=0.5):
    """Collapse a cleaned 3D cloud into a 2D occupancy grid on the x-z plane.

    Height bands (all relative to floor_y):
      |y - floor| <= floor_band                -> evidence of drivable floor
      clearance < y - floor < robot_height     -> obstacle the body would hit
      above robot_height / below floor - 0.15  -> ignored (overhangs / noise)
    """
    y = xyz[:, 1]
    keep = (y > floor_y - 0.15) & (y < floor_y + robot_height)
    pts = xyz[keep]
    if pts.shape[0] == 0:
        raise ValueError("no points in the navigation height band")

    x0 = float(pts[:, 0].min()) - padding
    z0 = float(pts[:, 2].min()) - padding
    cols = int(np.ceil((float(pts[:, 0].max()) + padding - x0) / cell_size))
    rows = int(np.ceil((float(pts[:, 2].max()) + padding - z0) / cell_size))

    col_idx = np.clip(((pts[:, 0] - x0) / cell_size).astype(int), 0, cols - 1)
    row_idx = np.clip(((pts[:, 2] - z0) / cell_size).astype(int), 0, rows - 1)
    flat = row_idx * cols + col_idx

    is_floor = np.abs(pts[:, 1] - floor_y) <= floor_band
    is_obstacle = ((pts[:, 1] > floor_y + clearance) &
                   (pts[:, 1] < floor_y + robot_height))
    floor_count = np.bincount(flat[is_floor], minlength=rows * cols).reshape(rows, cols)
    obst_count = np.bincount(flat[is_obstacle], minlength=rows * cols).reshape(rows, cols)

    cells = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    cells[floor_count > 0] = FREE
    cells[obst_count >= min_obstacle_points] = OCCUPIED
    return OccupancyGrid(cells=cells, origin=(x0, z0), cell_size=cell_size)


def inflate(grid, robot_radius=0.12):
    """Circular inflation: block every cell within robot_radius of an obstacle.

    Uses a Euclidean distance transform so inflation is a true disk, not the
    diamond that repeated binary dilation gives.
    """
    dist = ndimage.distance_transform_edt(grid.cells != OCCUPIED) * grid.cell_size
    grid.blocked = dist <= robot_radius
    return grid
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_mapping.py -v`
Expected: 8 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/mapping.py viewer/tests/test_mapping.py
git commit -m "feat: height-banded occupancy grid with EDT inflation"
```

---

### Task 6: Localization — pose matrix to 2D floor pose

**Files:**
- Create: `viewer/nav/localization.py`
- Test: `viewer/tests/test_localization.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_localization.py`:
```python
import math

import numpy as np
import pytest

from nav.localization import angle_diff, pose_from_streamed, pose_to_2d


def pose_matrix(x, y, z, yaw):
    """Camera-to-world matrix for a camera at (x,y,z) whose -Z (look direction)
    points at world heading `yaw` in the x-z plane (yaw = atan2 convention)."""
    fwd = np.array([math.cos(yaw), 0.0, math.sin(yaw)])
    up = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, up)          # camera +X
    M = np.eye(4)
    M[:3, 0] = right
    M[:3, 1] = up
    M[:3, 2] = -fwd                    # camera looks down its local -Z
    M[:3, 3] = [x, y, z]
    return M


def test_pose_to_2d_extracts_position_and_heading():
    M = pose_matrix(1.5, 0.3, -2.0, yaw=math.pi / 4)
    x, z, theta = pose_to_2d(M)
    assert (x, z) == pytest.approx((1.5, -2.0))
    assert theta == pytest.approx(math.pi / 4)


def test_pose_from_streamed_matches_viewer_convention():
    # pc_viewer treats the 16 floats as column-major: reshape(4,4).T
    M = pose_matrix(0.5, 0.0, 0.25, yaw=0.0)
    vals = tuple(M.T.ravel())          # column-major flatten
    x, z, theta = pose_to_2d(pose_from_streamed(vals))
    assert (x, z) == pytest.approx((0.5, 0.25))
    assert theta == pytest.approx(0.0)


def test_angle_diff_wraps_to_pi():
    assert angle_diff(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)
    assert angle_diff(0.1, -0.1) == pytest.approx(0.2)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_localization.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.localization'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/localization.py`:
```python
"""ARKit camera pose -> 2D floor pose (x, z, theta). +Y is up; floor is x-z."""

import math

import numpy as np


def pose_from_streamed(vals):
    """16 column-major floats (the 'O' message) -> 4x4 camera-to-world matrix."""
    return np.array(vals, dtype=np.float64).reshape(4, 4).T


def pose_to_2d(pose):
    """4x4 camera-to-world -> (x, z, theta). Camera looks down its local -Z,
    so world heading is the -Z column projected onto the floor plane."""
    M = np.asarray(pose, dtype=np.float64)
    fwd = -M[:3, 2]
    theta = math.atan2(fwd[2], fwd[0])
    return float(M[0, 3]), float(M[2, 3]), theta


def angle_diff(a, b):
    """Shortest signed angle a - b, wrapped to (-pi, pi]."""
    d = (a - b) % (2.0 * math.pi)
    if d > math.pi:
        d -= 2.0 * math.pi
    return d
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_localization.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/localization.py viewer/tests/test_localization.py
git commit -m "feat: pose matrix to 2D floor pose with heading"
```

---

### Task 7: Frontier detection + goal selection

Yamauchi frontier exploration: a frontier is a passable FREE cell adjacent to UNKNOWN. Cluster frontiers, keep reachable clusters big enough to matter, and pick the nearest one by BFS distance through free space (not straight-line, so a frontier behind a wall isn't "near").

**Files:**
- Create: `viewer/nav/frontier.py`
- Test: `viewer/tests/test_frontier.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_frontier.py`:
```python
import numpy as np

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.frontier import bfs_distances, choose_goal, frontier_mask, nearest_passable


def open_grid(rows=20, cols=20):
    g = OccupancyGrid(cells=np.full((rows, cols), FREE, dtype=np.int8),
                      origin=(0.0, 0.0), cell_size=0.05)
    g.blocked = np.zeros((rows, cols), dtype=bool)
    return g


def test_frontier_is_free_cells_touching_unknown():
    g = open_grid()
    g.cells[:, 10:] = UNKNOWN            # right half unexplored
    m = frontier_mask(g)
    assert m[5, 9]                        # free cell bordering unknown
    assert not m[5, 5]                    # interior free cell
    assert not m[5, 12]                   # unknown cell itself


def test_blocked_cells_are_not_frontiers():
    g = open_grid()
    g.cells[:, 10:] = UNKNOWN
    g.blocked[:, 9] = True                # inflation covers the border column
    assert not frontier_mask(g).any()


def test_nearest_passable_snaps_out_of_inflation():
    g = open_grid()
    g.blocked[4:7, 4:7] = True
    assert nearest_passable(g, (5, 5)) is not None
    assert not g.blocked[nearest_passable(g, (5, 5))]


def test_bfs_goes_around_walls():
    g = open_grid()
    g.cells[0:19, 10] = OCCUPIED          # wall with a gap at the bottom
    g.blocked[0:19, 10] = True
    dist = bfs_distances(g, (0, 0))
    assert np.isfinite(dist[0, 15])       # reachable around the gap
    assert dist[0, 15] > 20               # but much farther than straight-line


def test_choose_goal_prefers_reachable_frontier():
    g = open_grid()
    g.cells[:, 15:] = UNKNOWN             # frontier on the right
    g.cells[0:20, 12] = OCCUPIED          # fully sealed off by a wall
    g.blocked[0:20, 12] = True
    assert choose_goal(g, (10, 2)) is None  # only frontier is unreachable

    g2 = open_grid()
    g2.cells[:, 15:] = UNKNOWN
    goal = choose_goal(g2, (10, 2))
    assert goal is not None
    assert goal[1] == 14                  # a frontier-column cell
    assert frontier_mask(g2)[goal]


def test_choose_goal_ignores_tiny_clusters():
    g = open_grid()
    g.cells[10, 10] = UNKNOWN             # a speck of unknown (e.g. one missing
    # floor sample) rings itself with 8 frontier cells -- the default cluster
    # threshold must be big enough to ignore it
    assert choose_goal(g, (2, 2)) is None


def test_fully_explored_returns_none():
    g = open_grid()
    assert choose_goal(g, (5, 5)) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_frontier.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.frontier'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/frontier.py`:
```python
"""Frontier-based exploration (Yamauchi 1997): go to the free/unknown boundary."""

from collections import deque

import numpy as np
from scipy import ndimage

from nav.grid import FREE, UNKNOWN

_EIGHT = np.ones((3, 3), dtype=bool)


def frontier_mask(grid):
    """Passable FREE cells that touch at least one UNKNOWN cell (8-connected)."""
    unknown = grid.cells == UNKNOWN
    near_unknown = ndimage.binary_dilation(unknown, structure=_EIGHT)
    return grid.passable() & near_unknown


def nearest_passable(grid, cell, max_radius=12):
    """The passable cell closest to `cell` (itself if already passable).

    The car's own cell can sit inside the inflation ring right after a scan
    (walls near the start), so planning snaps to the nearest legal cell.
    """
    p = grid.passable()
    r0, c0 = cell
    best, best_d2 = None, None
    for radius in range(max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if max(abs(dr), abs(dc)) != radius:
                    continue
                r, c = r0 + dr, c0 + dc
                if grid.in_bounds(r, c) and p[r, c]:
                    d2 = dr * dr + dc * dc
                    if best is None or d2 < best_d2:
                        best, best_d2 = (r, c), d2
        if best is not None:
            return best
    return None


def bfs_distances(grid, start):
    """Steps from `start` to every passable cell (8-connected); inf if unreachable."""
    passable = grid.passable()
    dist = np.full(grid.shape, np.inf)
    if not grid.in_bounds(*start) or not passable[start]:
        return dist
    dist[start] = 0.0
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                n = (r + dr, c + dc)
                if grid.in_bounds(*n) and passable[n] and not np.isfinite(dist[n]):
                    dist[n] = dist[r, c] + 1
                    q.append(n)
    return dist


def choose_goal(grid, car_cell, min_cluster_size=12):
    """Nearest-frontier goal: the reachable frontier cell of the closest
    big-enough cluster. Returns a (row, col) cell, or None when exploration
    is complete (no reachable frontier remains).

    min_cluster_size=12 because even a single unexplored cell rings itself
    with 8 frontier cells; a real doorway/room-edge frontier at 5 cm cells
    is dozens of cells wide."""
    fmask = frontier_mask(grid)
    labels, n_clusters = ndimage.label(fmask, structure=_EIGHT.astype(int))
    if n_clusters == 0:
        return None
    start = nearest_passable(grid, car_cell)
    if start is None:
        return None
    dist = bfs_distances(grid, start)

    best_d, best_cell = np.inf, None
    for lab in range(1, n_clusters + 1):
        cells = np.argwhere(labels == lab)
        if len(cells) < min_cluster_size:
            continue
        ds = dist[cells[:, 0], cells[:, 1]]
        i = int(np.argmin(ds))
        if np.isfinite(ds[i]) and ds[i] < best_d:
            best_d = ds[i]
            best_cell = (int(cells[i, 0]), int(cells[i, 1]))
    return best_cell
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_frontier.py -v`
Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/frontier.py viewer/tests/test_frontier.py
git commit -m "feat: frontier detection, BFS reachability, nearest-frontier goal"
```

---

### Task 8: A* planner + line-of-sight path simplification

**Files:**
- Create: `viewer/nav/planner.py`
- Test: `viewer/tests/test_planner.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_planner.py`:
```python
import numpy as np

from nav.planner import astar, line_clear, simplify_path


def open_map(rows=20, cols=20):
    return np.ones((rows, cols), dtype=bool)


def test_astar_straight_line_in_open_space():
    path = astar(open_map(), (0, 0), (0, 10))
    assert path is not None
    assert path[0] == (0, 0) and path[-1] == (0, 10)
    assert len(path) == 11


def test_astar_routes_around_a_wall():
    p = open_map()
    p[0:15, 10] = False                       # wall with gap at rows 15..19
    path = astar(p, (5, 5), (5, 15))
    assert path is not None
    assert all(p[cell] for cell in path)      # never enters a blocked cell
    assert max(r for r, _ in path) >= 15      # went around via the gap


def test_astar_no_corner_cutting():
    p = open_map(3, 3)
    p[0, 1] = p[1, 0] = False                 # diagonal squeeze at (0,0)->(1,1)
    path = astar(p, (0, 0), (2, 2))
    assert path is None                       # the only exit is a blocked squeeze


def test_astar_unreachable_returns_none():
    p = open_map()
    p[:, 10] = False                          # full wall
    assert astar(p, (5, 5), (5, 15)) is None


def test_line_clear_sees_blockers():
    p = open_map()
    assert line_clear(p, (0, 0), (10, 10))
    p[5, 5] = False
    assert not line_clear(p, (0, 0), (10, 10))


def test_simplify_collapses_collinear_and_keeps_corners():
    p = open_map()
    p[0:15, 10] = False
    path = astar(p, (5, 5), (5, 15))
    simple = simplify_path(path, p)
    assert simple[0] == path[0] and simple[-1] == path[-1]
    assert len(simple) <= 5                   # a few corners, not 30 cells
    for a, b in zip(simple, simple[1:]):
        assert line_clear(p, a, b)            # every leg is collision-free
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_planner.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.planner'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/planner.py`:
```python
"""A* on the inflated occupancy grid + line-of-sight waypoint simplification."""

import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)


def octile(a, b):
    """Admissible 8-connected heuristic."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def astar(passable, start, goal):
    """8-connected A* over a bool passability mask.

    Diagonal moves cost sqrt(2) and are forbidden when either adjacent
    cardinal cell is blocked (no corner cutting -- the robot has a body).
    Returns a list of (row, col) from start to goal, or None.
    """
    rows, cols = passable.shape
    if not passable[start] or not passable[goal]:
        return None
    g = {start: 0.0}
    came = {}
    pq = [(octile(start, goal), start)]
    closed = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        r, c = cur
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols) or not passable[nr, nc]:
                    continue
                if dr != 0 and dc != 0 and not (passable[r, nc] and passable[nr, c]):
                    continue
                step = SQRT2 if dr != 0 and dc != 0 else 1.0
                ng = g[cur] + step
                nxt = (nr, nc)
                if ng < g.get(nxt, math.inf):
                    g[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(pq, (ng + octile(nxt, goal), nxt))
    return None


def line_clear(passable, a, b):
    """True if the straight segment a->b stays on passable cells (dense sampling
    at half-cell resolution, so it cannot skip over a blocked cell)."""
    steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    for t in np.linspace(0.0, 1.0, steps + 1):
        r = int(round(a[0] + (b[0] - a[0]) * t))
        c = int(round(a[1] + (b[1] - a[1]) * t))
        if not passable[r, c]:
            return False
    return True


def simplify_path(path, passable):
    """Greedy shortcutting: from each kept waypoint jump to the farthest path
    cell still in line of sight. Turns a cell-by-cell path into a few legs."""
    if not path:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_clear(passable, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_planner.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/planner.py viewer/tests/test_planner.py
git commit -m "feat: A* planner with no corner cutting + LOS path simplification"
```

---

### Task 9: Config + waypoint-follower controller

The local controller: turn in place until roughly facing the next waypoint, then drive with proportional heading correction (a right-sized "regulated pure pursuit" for a differential-drive toy). `TURN_SIGN` lives in config because the real rotation direction depends on wiring and is calibrated in the field test.

**Files:**
- Create: `viewer/nav/config.py`
- Create: `viewer/nav/controller.py`
- Test: `viewer/tests/test_controller.py`

- [ ] **Step 1: Create the config**

`viewer/nav/config.py`:
```python
"""Runtime tunables. TURN_SIGN is set during field calibration (Task 19):
+1 means the command (left=+s, right=-s) rotates the car so theta INCREASES."""

TURN_SIGN = 1

ROBOT_RADIUS = 0.14      # meters, chassis half-diagonal + margin
DRIVE_SPEED = 45         # -100..100 motor units, indoor-safe cruise
TURN_SPEED = 40          # in-place rotation speed
SPIN_SPEED = 35          # slow 360-degree scan spin (slow = better LiDAR)
ARRIVE_DIST = 0.10       # meters to consider a waypoint reached
TURN_THRESHOLD = 0.44    # rad (~25 deg): above this, stop and turn in place
```

- [ ] **Step 2: Write the failing test**

`viewer/tests/test_controller.py`:
```python
import math

from nav.controller import WaypointFollower


def test_done_when_no_waypoints():
    f = WaypointFollower(waypoints=[])
    assert f.done
    assert f.update(0.0, 0.0, 0.0) == (0, 0)


def test_turns_in_place_when_facing_wrong_way():
    # waypoint straight "ahead" in +x; car faces +z (90 deg off)
    f = WaypointFollower(waypoints=[(1.0, 0.0)])
    left, right = f.update(0.0, 0.0, math.pi / 2)
    assert left == -right and left != 0          # pure rotation


def test_drives_forward_when_aligned():
    f = WaypointFollower(waypoints=[(1.0, 0.0)])
    left, right = f.update(0.0, 0.0, 0.0)        # facing +x, waypoint at +x
    assert left > 0 and right > 0
    assert abs(left - right) <= 6                # near-straight


def test_correction_steers_toward_small_heading_error():
    f = WaypointFollower(waypoints=[(1.0, 0.0)])
    l_straight, r_straight = f.update(0.0, 0.0, 0.0)
    l_err, r_err = f.update(0.0, 0.0, 0.15)      # slightly rotated +theta
    assert (l_err - r_err) != (l_straight - r_straight)  # correction applied


def test_advances_through_waypoints_and_finishes():
    f = WaypointFollower(waypoints=[(0.5, 0.0), (0.5, 0.5)], arrive_dist=0.1)
    f.update(0.45, 0.0, 0.0)                     # within arrive_dist of wp0
    assert f.current_waypoint == (0.5, 0.5)
    assert f.update(0.5, 0.45, math.pi / 2) == (0, 0)  # reached the last one
    assert f.done
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_controller.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.controller'`

- [ ] **Step 4: Write the implementation**

`viewer/nav/controller.py`:
```python
"""Waypoint follower: turn-then-drive local controller for differential drive."""

import math
from dataclasses import dataclass, field

from nav import config
from nav.localization import angle_diff


@dataclass
class WaypointFollower:
    waypoints: list                    # [(x, z), ...] world coordinates
    arrive_dist: float = config.ARRIVE_DIST
    turn_threshold: float = config.TURN_THRESHOLD
    drive_speed: int = config.DRIVE_SPEED
    turn_speed: int = config.TURN_SPEED
    _index: int = field(default=0, repr=False)

    @property
    def done(self):
        return self._index >= len(self.waypoints)

    @property
    def current_waypoint(self):
        return None if self.done else self.waypoints[self._index]

    def update(self, x, z, theta):
        """Current pose -> (left, right) motor command. Call at >= 5 Hz."""
        while not self.done:
            wx, wz = self.waypoints[self._index]
            if math.hypot(wx - x, wz - z) < self.arrive_dist:
                self._index += 1
            else:
                break
        if self.done:
            return 0, 0

        wx, wz = self.waypoints[self._index]
        bearing = math.atan2(wz - z, wx - x)
        err = angle_diff(bearing, theta)

        if abs(err) > self.turn_threshold:
            s = self.turn_speed if err > 0 else -self.turn_speed
            return config.TURN_SIGN * s, -config.TURN_SIGN * s

        correction = config.TURN_SIGN * int(max(-15, min(15, err * 40)))
        return self.drive_speed - correction, self.drive_speed + correction
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_controller.py -v`
Expected: 5 passed

- [ ] **Step 6: Commit**

```bash
git add viewer/nav/config.py viewer/nav/controller.py viewer/tests/test_controller.py
git commit -m "feat: waypoint follower controller with calibratable turn sign"
```

---

### Task 10: Explorer state machine

The brain: `SPIN` (360° in-place scan — the only time LiDAR points stream, which is the "no auto-spamming" requirement) → `PLAN` (clean cloud → grid → frontier goal → A* path) → `DRIVE` (follow waypoints) → back to `SPIN` at the new vantage point → `DONE` when no reachable frontier remains.

**Files:**
- Create: `viewer/nav/explorer.py`
- Test: `viewer/tests/test_explorer.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_explorer.py`:
```python
import math

import numpy as np

from nav.explorer import Explorer


def plane(x0, x1, z0, z1, y, spacing=0.03):
    xs, zs = np.meshgrid(np.arange(x0, x1, spacing), np.arange(z0, z1, spacing))
    return np.stack([xs.ravel(), np.full(xs.size, y), zs.ravel()],
                    axis=1).astype(np.float32)


def wall(x0, x1, z0, z1):
    return np.vstack([plane(x0, x1, z0, z1, y=h) for h in (0.10, 0.20, 0.30)])


def walled_room(explored_x=2.0):
    """2x2 m room enclosed by walls; floor only observed for x < explored_x."""
    return np.vstack([
        plane(0.0, explored_x, 0.0, 2.0, y=0.0),      # observed floor
        wall(-0.06, 0.0, -0.06, 2.06),                # left wall
        wall(2.0, 2.06, -0.06, 2.06),                 # right wall
        wall(0.0, 2.0, -0.06, 0.0),                   # near wall
        wall(0.0, 2.0, 2.0, 2.06),                    # far wall
    ])


def spin(explorer, pts, x=0.5, z=1.0):
    """Feed a simulated in-place rotation until the 360-degree scan completes."""
    for i in range(14):
        theta = ((i * 0.55) + math.pi) % (2 * math.pi) - math.pi
        explorer.update((x, z, theta), pts)


def test_starts_idle_and_stopped():
    e = Explorer()
    assert e.state == Explorer.IDLE
    assert e.update((0.0, 0.0, 0.0), np.zeros((0, 3), np.float32)) == (0, 0)


def test_spin_emits_pure_rotation_and_wants_scan():
    e = Explorer()
    e.start()
    left, right = e.update((0.5, 1.0, 0.0), walled_room())
    assert e.wants_scan
    assert left == -right and left != 0


def test_full_spin_then_plans_a_drive_to_the_frontier():
    e = Explorer()
    pts = walled_room(explored_x=1.2)
    e.start()
    spin(e, pts)
    e.update((0.5, 1.0, 0.0), pts)                # runs the planner
    assert e.state == Explorer.DRIVE
    assert not e.wants_scan
    assert e.goal_cell is not None and len(e.path_world) >= 1
    gx, _ = e.grid.cell_to_world(*e.goal_cell)
    assert gx > 1.0                               # goal is on the unexplored side


def test_fully_explored_room_finishes():
    e = Explorer()
    pts = walled_room(explored_x=2.0)
    e.start()
    spin(e, pts)
    e.update((0.5, 1.0, 0.0), pts)
    assert e.state == Explorer.DONE
    assert e.update((0.5, 1.0, 0.0), pts) == (0, 0)


def test_arriving_at_goal_triggers_a_rescan():
    e = Explorer()
    pts = walled_room(explored_x=1.2)
    e.start()
    spin(e, pts)
    e.update((0.5, 1.0, 0.0), pts)
    assert e.state == Explorer.DRIVE
    for wx, wz in list(e.path_world):             # teleport through each waypoint
        e.update((wx, wz, 0.0), pts)
    assert e.state == Explorer.SPIN               # scans again at the new spot
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_explorer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.explorer'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/explorer.py`:
```python
"""Exploration state machine: SPIN (scan) -> PLAN -> DRIVE -> rescan -> DONE."""

import math

from nav import config, frontier, mapping, planner
from nav.controller import WaypointFollower
from nav.localization import angle_diff

SPIN_OVERSHOOT = 1.05          # rotate slightly past 360 deg so the scan closes
MIN_PLAN_POINTS = 100          # below this the map is too thin to plan on


class Explorer:
    IDLE, SPIN, PLAN, DRIVE, DONE = "IDLE", "SPIN", "PLAN", "DRIVE", "DONE"

    def __init__(self, robot_radius=config.ROBOT_RADIUS, cell_size=0.05):
        self.robot_radius = robot_radius
        self.cell_size = cell_size
        self.state = self.IDLE
        # Debug / overlay state, refreshed on every PLAN:
        self.grid = None
        self.floor_y = None
        self.goal_cell = None
        self.path_world = []
        self.follower = None
        self._spin_accum = 0.0
        self._last_theta = None

    def start(self):
        """(Re)enter the SPIN state to take a fresh 360-degree scan."""
        self.state = self.SPIN
        self._spin_accum = 0.0
        self._last_theta = None

    def stop(self):
        self.state = self.IDLE

    @property
    def wants_scan(self):
        """True only while spinning -- LiDAR points should stream only then."""
        return self.state == self.SPIN

    def update(self, pose2d, map_xyz):
        """One control step. pose2d = (x, z, theta); map_xyz = accumulated cloud.
        Returns the (left, right) motor command for right now."""
        x, z, theta = pose2d

        if self.state == self.SPIN:
            if self._last_theta is not None:
                self._spin_accum += abs(angle_diff(theta, self._last_theta))
            self._last_theta = theta
            if self._spin_accum >= 2.0 * math.pi * SPIN_OVERSHOOT:
                self.state = self.PLAN
                return 0, 0
            s = config.SPIN_SPEED
            return config.TURN_SIGN * s, -config.TURN_SIGN * s

        if self.state == self.PLAN:
            self._plan(x, z, map_xyz)
            if self.state != self.DRIVE:
                return 0, 0

        if self.state == self.DRIVE:
            left, right = self.follower.update(x, z, theta)
            if self.follower.done:
                self.start()           # arrived: rescan from the new vantage point
                return 0, 0
            return left, right

        return 0, 0                    # IDLE / DONE

    def _plan(self, x, z, map_xyz):
        pts = mapping.clean_cloud(map_xyz)
        if pts.shape[0] < MIN_PLAN_POINTS:
            self.state = self.DONE
            return
        self.floor_y = mapping.estimate_floor_height(pts)
        grid = mapping.build_occupancy_grid(pts, self.floor_y,
                                            cell_size=self.cell_size)
        self.grid = mapping.inflate(grid, self.robot_radius)

        car_cell = self.grid.world_to_cell(x, z)
        self.goal_cell = frontier.choose_goal(self.grid, car_cell)
        if self.goal_cell is None:
            self.state = self.DONE     # no reachable frontier: room fully mapped
            return

        start = frontier.nearest_passable(self.grid, car_cell)
        path = planner.astar(self.grid.passable(), start, self.goal_cell)
        if path is None:               # defensive; choose_goal proved reachability
            self.state = self.DONE
            return
        waypoints = [self.grid.cell_to_world(r, c)
                     for r, c in planner.simplify_path(path, self.grid.passable())]
        self.path_world = waypoints
        self.follower = WaypointFollower(waypoints=waypoints)
        self.state = self.DRIVE
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_explorer.py -v`
Expected: 5 passed

- [ ] **Step 5: Run the whole suite (regression check)**

Run: `cd viewer; python -m pytest tests/ -v`
Expected: all tests pass

- [ ] **Step 6: Commit**

```bash
git add viewer/nav/explorer.py viewer/tests/test_explorer.py
git commit -m "feat: SPIN/PLAN/DRIVE exploration state machine with scan gating"
```

---

### Task 11: ICP drift correction (scan-to-map)

ARKit VIO drifts over long runs; the standard fix (used by every modern SLAM stack as "scan matching") is aligning the most recent scan against the accumulated map and snapping the pose back on. Multi-scale ICP (coarse→fine) so it doesn't fall into a local minimum.

**Files:**
- Create: `viewer/nav/drift.py`
- Test: `viewer/tests/test_drift.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_drift.py`:
```python
import numpy as np

from nav.drift import estimate_correction


def structured_room():
    """Floor + two perpendicular walls + a box: enough structure that x and z
    translation are both observable (a bare floor can slide along itself)."""
    rng = np.random.default_rng(1)
    floor = np.stack([rng.uniform(0, 2, 4000), np.zeros(4000),
                      rng.uniform(0, 2, 4000)], axis=1)
    wall_x = np.stack([np.zeros(1500), rng.uniform(0, 0.5, 1500),
                       rng.uniform(0, 2, 1500)], axis=1)
    wall_z = np.stack([rng.uniform(0, 2, 1500), rng.uniform(0, 0.5, 1500),
                       np.zeros(1500)], axis=1)
    box = np.stack([rng.uniform(1.2, 1.5, 800), rng.uniform(0, 0.3, 800),
                    rng.uniform(0.7, 1.0, 800)], axis=1)
    return np.vstack([floor, wall_x, wall_z, box]).astype(np.float32)


def test_recovers_a_small_translation_drift():
    room = structured_room()
    drift = np.array([0.05, 0.0, 0.03], dtype=np.float32)
    T = estimate_correction(room + drift, room)
    assert T is not None
    assert np.allclose(T[:3, 3], -drift, atol=0.02)   # correction undoes the drift


def test_rejects_too_few_points():
    room = structured_room()
    assert estimate_correction(room[:50], room) is None
    assert estimate_correction(room, room[:50]) is None


def test_rejects_wild_corrections():
    room = structured_room()
    # 2 m of "drift" is beyond anything plausible between two spins
    assert estimate_correction(room + np.float32([2.0, 0.0, 0.0]), room) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_drift.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.drift'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/drift.py`:
```python
"""Scan-to-map ICP: estimate the rigid correction for accumulated VIO drift."""

import numpy as np
import open3d as o3d


def estimate_correction(scan_xyz, map_xyz, voxel=0.05, max_dist=0.25,
                        min_fitness=0.3, max_translation=0.5):
    """Align `scan_xyz` (latest 360-degree scan, in the drifted frame) onto
    `map_xyz` (accumulated map). Returns a 4x4 matrix that pre-multiplies
    raw poses to correct them, or None when no trustworthy alignment exists
    (too few points, poor overlap, or an implausibly large jump)."""
    if scan_xyz.shape[0] < 200 or map_xyz.shape[0] < 200:
        return None
    src = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(scan_xyz.astype(np.float64)))
    tgt = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(map_xyz.astype(np.float64)))

    T = np.eye(4)
    fitness = 0.0
    for scale, dist in ((4, max_dist), (2, max_dist / 2), (1, max_dist / 4)):
        s = src.voxel_down_sample(voxel * scale)
        t = tgt.voxel_down_sample(voxel * scale)
        reg = o3d.pipelines.registration.registration_icp(
            s, t, dist, T,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        T = np.array(reg.transformation)
        fitness = reg.fitness

    if fitness < min_fitness or np.linalg.norm(T[:3, 3]) > max_translation:
        return None
    return T
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_drift.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/drift.py viewer/tests/test_drift.py
git commit -m "feat: multi-scale ICP scan-to-map drift correction"
```

---

### Task 12: Command protocol frames (PC → phone)

**Files:**
- Create: `viewer/nav/protocol.py`
- Test: `viewer/tests/test_protocol.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_protocol.py`:
```python
import struct

from nav.protocol import MSG_DRIVE, MSG_MODE, MSG_TRACKING, drive_frame, frame, mode_frame


def unpack(data):
    mtype = data[0]
    length = struct.unpack("<I", data[1:5])[0]
    payload = data[5:]
    assert len(payload) == length
    return mtype, payload


def test_frame_layout_matches_existing_wire_format():
    mtype, payload = unpack(frame(0x41, b"hello"))
    assert mtype == 0x41 and payload == b"hello"


def test_drive_frame_encodes_speeds():
    assert unpack(drive_frame(60, -40)) == (MSG_DRIVE, b"L60R-40")


def test_drive_frame_clamps_to_motor_range():
    assert unpack(drive_frame(250, -999)) == (MSG_DRIVE, b"L100R-100")


def test_mode_frames():
    assert unpack(mode_frame(True)) == (MSG_MODE, b"SCAN")
    assert unpack(mode_frame(False)) == (MSG_MODE, b"IDLE")


def test_message_type_bytes_are_ascii_letters():
    assert (MSG_DRIVE, MSG_MODE, MSG_TRACKING) == (0x44, 0x4D, 0x54)  # D, M, T
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_protocol.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.protocol'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/protocol.py`:
```python
"""PC -> phone command frames. Same framing as the phone -> PC stream:
1 byte type + uint32 little-endian payload length + payload."""

import struct

MSG_DRIVE = 0x44      # 'D' PC -> phone: ASCII "L<left>R<right>", -100..100
MSG_MODE = 0x4D       # 'M' PC -> phone: ASCII "SCAN" or "IDLE"
MSG_TRACKING = 0x54   # 'T' phone -> PC: 1 byte; 0=normal 1=limited 2=lost


def frame(mtype, payload):
    return bytes([mtype]) + struct.pack("<I", len(payload)) + payload


def drive_frame(left, right):
    l = max(-100, min(100, int(left)))
    r = max(-100, min(100, int(right)))
    return frame(MSG_DRIVE, f"L{l}R{r}".encode("ascii"))


def mode_frame(scan):
    return frame(MSG_MODE, b"SCAN" if scan else b"IDLE")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_protocol.py -v`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/protocol.py viewer/tests/test_protocol.py
git commit -m "feat: PC-to-phone drive/mode command frames"
```

---

### Task 13: NavRunner — heartbeat, failsafes, scan-mode switching

The glue between `pc_viewer.py` and the nav stack. Owns the redundancy behaviors: command heartbeat every 0.15 s (so the ESP32's 0.5 s failsafe stays fed while driving, and trips if the PC dies), immediate stop on stale pose or tracking loss, and SCAN/IDLE mode transitions.

**Files:**
- Create: `viewer/nav/runner.py`
- Test: `viewer/tests/test_runner.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_runner.py`:
```python
import numpy as np

from nav.protocol import MSG_DRIVE, MSG_MODE
from nav.runner import NavRunner

IDENTITY_POSE = tuple(np.eye(4, dtype=np.float32).ravel())
EMPTY = np.zeros((0, 3), np.float32)


class SendSpy:
    def __init__(self):
        self.frames = []

    def __call__(self, data):
        self.frames.append((data[0], bytes(data[5:])))

    def of_type(self, mtype):
        return [p for t, p in self.frames if t == mtype]


def test_inactive_runner_sends_nothing():
    spy = SendSpy()
    NavRunner(spy).tick(0.0, IDENTITY_POSE, 0.0, EMPTY, True)
    assert spy.frames == []


def test_start_enables_scan_mode_and_spins():
    spy = SendSpy()
    r = NavRunner(spy)
    r.start()
    r.tick(10.0, IDENTITY_POSE, 10.0, EMPTY, True)
    assert spy.of_type(MSG_MODE) == [b"SCAN"]
    drives = spy.of_type(MSG_DRIVE)
    assert len(drives) == 1 and drives[0] != b"L0R0"   # spinning, not idle


def test_heartbeat_repeats_commands_within_esp32_failsafe():
    spy = SendSpy()
    r = NavRunner(spy)
    r.start()
    r.tick(10.0, IDENTITY_POSE, 10.0, EMPTY, True)
    r.tick(10.05, IDENTITY_POSE, 10.05, EMPTY, True)   # too soon: no resend
    assert len(spy.of_type(MSG_DRIVE)) == 1
    r.tick(10.16, IDENTITY_POSE, 10.16, EMPTY, True)   # past 0.15 s: resend
    assert len(spy.of_type(MSG_DRIVE)) == 2


def test_stale_pose_stops_the_car():
    spy = SendSpy()
    r = NavRunner(spy)
    r.start()
    r.tick(10.0, IDENTITY_POSE, 10.0, EMPTY, True)
    r.tick(11.0, IDENTITY_POSE, 10.0, EMPTY, True)     # pose is 1.0 s old
    assert spy.of_type(MSG_DRIVE)[-1] == b"L0R0"


def test_tracking_loss_stops_the_car():
    spy = SendSpy()
    r = NavRunner(spy)
    r.start()
    r.tick(10.0, IDENTITY_POSE, 10.0, EMPTY, True)
    r.tick(10.2, IDENTITY_POSE, 10.2, EMPTY, False)    # ARKit tracking degraded
    assert spy.of_type(MSG_DRIVE)[-1] == b"L0R0"


def test_stop_sends_zero_and_returns_to_manual_scan():
    spy = SendSpy()
    r = NavRunner(spy)
    r.start()
    r.tick(10.0, IDENTITY_POSE, 10.0, EMPTY, True)
    r.stop()
    assert spy.of_type(MSG_DRIVE)[-1] == b"L0R0"
    assert spy.of_type(MSG_MODE)[-1] == b"SCAN"        # manual scanning restored
    n = len(spy.frames)
    r.tick(11.0, IDENTITY_POSE, 11.0, EMPTY, True)     # inactive: no more frames
    assert len(spy.frames) == n
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_runner.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.runner'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/runner.py`:
```python
"""Glue between pc_viewer and the nav stack: pacing, heartbeat, failsafes."""

from nav.explorer import Explorer
from nav.localization import pose_from_streamed, pose_to_2d
from nav.protocol import drive_frame, mode_frame

HEARTBEAT_S = 0.15    # ESP32 stops after 0.5 s of silence; refresh well inside
NAV_PERIOD_S = 0.2    # explorer/controller update rate (5 Hz)
POSE_STALE_S = 0.7    # no fresh pose for this long -> emergency stop


class NavRunner:
    def __init__(self, send, explorer=None):
        self.send = send                      # callable(bytes) -> None
        self.explorer = explorer or Explorer()
        self.active = False
        self._cmd = (0, 0)
        self._last_heartbeat = -1e9
        self._last_nav = -1e9
        self._scan_mode = None

    def start(self):
        self.active = True
        self._last_nav = -1e9
        self.explorer.start()

    def stop(self):
        """Halt autonomy: stop the car and restore always-on manual scanning."""
        self.active = False
        self.explorer.stop()
        self._cmd = (0, 0)
        self.send(drive_frame(0, 0))
        self._set_scan(True)

    def tick(self, now, pose_vals, pose_time, map_xyz, tracking_ok):
        """Call every viewer loop iteration (fast); internally rate-limited."""
        if not self.active:
            return
        if pose_vals is None or (now - pose_time) > POSE_STALE_S or not tracking_ok:
            self._cmd = (0, 0)                # failsafe: hold still until healthy
        elif now - self._last_nav >= NAV_PERIOD_S:
            self._last_nav = now
            pose2d = pose_to_2d(pose_from_streamed(pose_vals))
            self._cmd = self.explorer.update(pose2d, map_xyz)
            self._set_scan(self.explorer.wants_scan)
        self._heartbeat(now)

    def _heartbeat(self, now):
        if now - self._last_heartbeat >= HEARTBEAT_S:
            self._last_heartbeat = now
            self.send(drive_frame(*self._cmd))

    def _set_scan(self, scan):
        if scan != self._scan_mode:
            self._scan_mode = scan
            self.send(mode_frame(scan))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_runner.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/runner.py viewer/tests/test_runner.py
git commit -m "feat: NavRunner with heartbeat, stale-pose and tracking failsafes"
```

---

### Task 14: Wire drift correction into the explorer + runner

The explorer records which slice of the accumulated cloud each 360° spin produced; after the spin, the runner ICP-aligns that slice against the older map and pre-multiplies all subsequent raw poses with the correction. (The stored cloud itself stays raw — this corrects the *pose used for navigation*, which is the standard scan-matching localization fix. Full map re-alignment is out of scope for v1.)

**Files:**
- Modify: `viewer/nav/explorer.py`
- Modify: `viewer/nav/runner.py`
- Test: `viewer/tests/test_explorer.py`, `viewer/tests/test_runner.py` (append tests)

- [ ] **Step 1: Write the failing explorer test** (append to `viewer/tests/test_explorer.py`)

```python
def test_records_the_point_range_captured_during_a_spin():
    e = Explorer()
    full = walled_room(explored_x=1.2)
    e.start()
    for i in range(14):
        theta = ((i * 0.55) + math.pi) % (2 * math.pi) - math.pi
        # first update sees a partial map; the rest see the growing full map
        e.update((0.5, 1.0, theta), full[:1000] if i == 0 else full)
    assert e.last_spin_range == (1000, len(full))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_explorer.py -v`
Expected: new test FAILS with `AttributeError: ... no attribute 'last_spin_range'`

- [ ] **Step 3: Add spin-range tracking to the explorer**

In `viewer/nav/explorer.py`, add to `__init__` (after `self._last_theta = None`):
```python
        self._spin_start_count = None
        self.last_spin_range = None    # (start_idx, end_idx) into the map cloud
```

In `start()`, add:
```python
        self._spin_start_count = None
```

Replace the SPIN branch of `update()` with:
```python
        if self.state == self.SPIN:
            if self._spin_start_count is None:
                self._spin_start_count = int(map_xyz.shape[0])
            if self._last_theta is not None:
                self._spin_accum += abs(angle_diff(theta, self._last_theta))
            self._last_theta = theta
            if self._spin_accum >= 2.0 * math.pi * SPIN_OVERSHOOT:
                self.last_spin_range = (self._spin_start_count,
                                        int(map_xyz.shape[0]))
                self.state = self.PLAN
                return 0, 0
            s = config.SPIN_SPEED
            return config.TURN_SIGN * s, -config.TURN_SIGN * s
```

Run: `cd viewer; python -m pytest tests/test_explorer.py -v` — Expected: all pass.

- [ ] **Step 4: Write the failing runner test** (append to `viewer/tests/test_runner.py`)

```python
import pytest


class FakeExplorer:
    """Duck-typed explorer that records the poses it is driven with."""
    def __init__(self):
        self.wants_scan = False
        self.last_spin_range = None
        self.poses = []
        self.grid = None
        self.floor_y = None
        self.path_world = []

    def start(self):
        pass

    def stop(self):
        pass

    def update(self, pose2d, map_xyz):
        self.poses.append(pose2d)
        return 0, 0


def test_icp_correction_shifts_subsequent_poses(monkeypatch):
    shift = np.eye(4)
    shift[0, 3] = -0.05                     # pretend ICP found 5 cm of x drift
    monkeypatch.setattr("nav.runner.estimate_correction",
                        lambda scan, mp, **kw: shift)
    spy = SendSpy()
    fake = FakeExplorer()
    r = NavRunner(spy, explorer=fake)
    r.start()
    pts = np.random.default_rng(0).random((2000, 3)).astype(np.float32)

    r.tick(10.0, IDENTITY_POSE, 10.0, pts, True)
    assert fake.poses[-1][0] == pytest.approx(0.0)     # no correction yet

    fake.last_spin_range = (500, 2000)                 # a spin just completed
    r.tick(10.3, IDENTITY_POSE, 10.3, pts, True)       # correction estimated here
    r.tick(10.6, IDENTITY_POSE, 10.6, pts, True)
    assert fake.poses[-1][0] == pytest.approx(-0.05)   # pose now corrected


def test_correction_runs_once_per_spin(monkeypatch):
    calls = []
    monkeypatch.setattr("nav.runner.estimate_correction",
                        lambda scan, mp, **kw: calls.append(1) or None)
    spy = SendSpy()
    fake = FakeExplorer()
    r = NavRunner(spy, explorer=fake)
    r.start()
    pts = np.zeros((2000, 3), np.float32)
    fake.last_spin_range = (500, 2000)
    r.tick(10.0, IDENTITY_POSE, 10.0, pts, True)
    r.tick(10.3, IDENTITY_POSE, 10.3, pts, True)
    assert len(calls) == 1                             # same range, not re-run
```

- [ ] **Step 5: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_runner.py -v`
Expected: new tests FAIL (`AttributeError` on `nav.runner.estimate_correction`)

- [ ] **Step 6: Add drift correction to the runner**

In `viewer/nav/runner.py`, add imports at the top:
```python
import numpy as np

from nav.drift import estimate_correction
```

Add to `__init__`:
```python
        self.pose_correction = np.eye(4)   # raw ARKit frame -> map frame
        self._corrected_range = None
```

In `tick()`, replace the healthy-pose branch body with:
```python
        elif now - self._last_nav >= NAV_PERIOD_S:
            self._last_nav = now
            pose = self.pose_correction @ pose_from_streamed(pose_vals)
            self._cmd = self.explorer.update(pose_to_2d(pose), map_xyz)
            self._set_scan(self.explorer.wants_scan)
            self._maybe_correct_drift(map_xyz)
```

Add the method:
```python
    def _maybe_correct_drift(self, map_xyz):
        """After each completed spin, ICP-align the spin's points against the
        older map; on success, replace the pose correction (the spin points
        carry the full accumulated drift, so this is absolute, not chained)."""
        rng = getattr(self.explorer, "last_spin_range", None)
        if rng is None or rng == self._corrected_range:
            return
        self._corrected_range = rng
        start, end = rng
        if start < 500 or end - start < 200 or map_xyz.shape[0] < end:
            return                        # not enough prior map or scan overlap
        T = estimate_correction(np.asarray(map_xyz[start:end]),
                                np.asarray(map_xyz[:start]))
        if T is not None:
            self.pose_correction = T
```

- [ ] **Step 7: Run the whole suite**

Run: `cd viewer; python -m pytest tests/ -v`
Expected: all tests pass

- [ ] **Step 8: Commit**

```bash
git add viewer/nav/explorer.py viewer/nav/runner.py viewer/tests/test_explorer.py viewer/tests/test_runner.py
git commit -m "feat: apply ICP drift correction to navigation poses after each spin"
```

---

### Task 15: Viewer overlay geometry

**Files:**
- Create: `viewer/nav/overlay.py`
- Test: `viewer/tests/test_overlay.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_overlay.py`:
```python
import numpy as np

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.overlay import grid_overlay, path_overlay


def small_grid():
    cells = np.full((4, 4), UNKNOWN, dtype=np.int8)
    cells[1, 1] = FREE
    cells[1, 2] = OCCUPIED
    g = OccupancyGrid(cells=cells, origin=(0.0, 0.0), cell_size=0.1)
    g.blocked = np.zeros((4, 4), dtype=bool)
    g.blocked[1, 2] = True
    return g


def test_grid_overlay_draws_only_observed_cells_above_floor():
    pts, colors = grid_overlay(small_grid(), floor_y=-1.0)
    assert pts.shape == (2, 3) and colors.shape == (2, 3)
    assert np.allclose(pts[:, 1], -0.99)             # 1 cm above the floor
    assert not np.array_equal(colors[0], colors[1])  # free vs occupied differ


def test_grid_overlay_empty_grid():
    g = small_grid()
    g.cells[:] = UNKNOWN
    pts, colors = grid_overlay(g, floor_y=0.0)
    assert pts.shape == (0, 3) and colors.shape == (0, 3)


def test_path_overlay_builds_line_segments():
    pts, lines = path_overlay([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], floor_y=0.0)
    assert pts.shape == (3, 3)
    assert lines.tolist() == [[0, 1], [1, 2]]


def test_path_overlay_empty():
    pts, lines = path_overlay([], floor_y=0.0)
    assert pts.shape == (0, 3) and lines.shape == (0, 2)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_overlay.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.overlay'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/overlay.py`:
```python
"""Turn nav state into flat colored geometry for the Open3D viewer window."""

import numpy as np

from nav.grid import OCCUPIED, UNKNOWN


def grid_overlay(grid, floor_y):
    """(points Nx3, colors Nx3) for every observed cell, drawn 1 cm above the
    floor: green = passable, amber = inflation buffer, red = obstacle."""
    rr, cc = np.nonzero(grid.cells != UNKNOWN)
    if rr.size == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    xs = grid.origin[0] + (cc + 0.5) * grid.cell_size
    zs = grid.origin[1] + (rr + 0.5) * grid.cell_size
    pts = np.stack([xs, np.full(rr.size, floor_y + 0.01), zs], axis=1)

    colors = np.zeros((rr.size, 3))
    occ = grid.cells[rr, cc] == OCCUPIED
    blk = grid.blocked[rr, cc] & ~occ
    colors[~occ & ~blk] = (0.15, 0.55, 0.20)
    colors[blk] = (0.75, 0.55, 0.10)
    colors[occ] = (0.90, 0.15, 0.15)
    return pts, colors


def path_overlay(path_world, floor_y):
    """(points Nx3, lines Mx2) for an Open3D LineSet of the planned path,
    drawn 3 cm above the floor so it reads over the grid overlay."""
    if not path_world:
        return np.zeros((0, 3)), np.zeros((0, 2), dtype=np.int32)
    pts = np.array([[x, floor_y + 0.03, z] for x, z in path_world])
    lines = np.array([[i, i + 1] for i in range(len(pts) - 1)], dtype=np.int32)
    return pts, lines
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_overlay.py -v`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add viewer/nav/overlay.py viewer/tests/test_overlay.py
git commit -m "feat: occupancy grid and path overlay geometry for the viewer"
```

---

### Task 15b: Walkthrough preview — map the room with just the phone (no car)

**This is Milestone A's payoff and needs no car, no ESP32, and no Swift changes.** Walk around holding the phone ("pretend to drive"), press `G` in the viewer, and every ~2 s the accumulated cloud is cleaned, collapsed to the occupancy grid, and drawn over the point cloud — green drivable floor, amber inflation buffer, red obstacles — plus the frontier goal and A* path a car *would* take next, using the phone's own pose as the pretend car. This validates the whole mapping/planning stack in your real room before any hardware exists.

**Files:**
- Create: `viewer/nav/preview.py`
- Test: `viewer/tests/test_preview.py`
- Modify: `viewer/pc_viewer.py`

- [ ] **Step 1: Write the failing test**

`viewer/tests/test_preview.py`:
```python
import numpy as np

from nav.preview import preview_plan


def plane(x0, x1, z0, z1, y, spacing=0.03):
    xs, zs = np.meshgrid(np.arange(x0, x1, spacing), np.arange(z0, z1, spacing))
    return np.stack([xs.ravel(), np.full(xs.size, y), zs.ravel()],
                    axis=1).astype(np.float32)


def wall(x0, x1, z0, z1):
    return np.vstack([plane(x0, x1, z0, z1, y=h) for h in (0.10, 0.20, 0.30)])


def partly_scanned_room():
    """2x2 m walled room; floor observed only for x < 1.2 (rest unexplored)."""
    return np.vstack([
        plane(0.0, 1.2, 0.0, 2.0, y=0.0),
        wall(-0.06, 0.0, -0.06, 2.06),
        wall(2.0, 2.06, -0.06, 2.06),
        wall(0.0, 2.0, -0.06, 0.0),
        wall(0.0, 2.0, 2.0, 2.06),
    ])


def test_preview_produces_grid_goal_and_path():
    res = preview_plan(partly_scanned_room(), (0.5, 1.0, 0.0))
    assert res.grid is not None and res.floor_y is not None
    assert res.goal_cell is not None
    assert len(res.path_world) >= 1
    gx, _ = res.grid.cell_to_world(*res.goal_cell)
    assert gx > 1.0                       # the goal points at the unexplored side


def test_preview_survives_thin_data():
    res = preview_plan(np.zeros((0, 3), np.float32), (0.0, 0.0, 0.0))
    assert res.grid is None and res.path_world == []


def test_preview_with_no_frontier_still_returns_the_grid():
    full = np.vstack([
        plane(0.0, 2.0, 0.0, 2.0, y=0.0),
        wall(-0.06, 0.0, -0.06, 2.06),
        wall(2.0, 2.06, -0.06, 2.06),
        wall(0.0, 2.0, -0.06, 0.0),
        wall(0.0, 2.0, 2.0, 2.06),
    ])
    res = preview_plan(full, (0.5, 1.0, 0.0))
    assert res.grid is not None           # map still drawn
    assert res.goal_cell is None          # nothing left to explore
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd viewer; python -m pytest tests/test_preview.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'nav.preview'`

- [ ] **Step 3: Write the implementation**

`viewer/nav/preview.py`:
```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd viewer; python -m pytest tests/test_preview.py -v`
Expected: 3 passed

- [ ] **Step 5: Hook it into pc_viewer.py**

a) Imports, after `import cv2`:

```python
from nav.localization import pose_from_streamed, pose_to_2d
from nav.overlay import grid_overlay, path_overlay
from nav.preview import preview_plan
```

b) In `main()`, after the `threading.Thread(...).start()` line:

```python
    nav_grid_pcd = o3d.geometry.PointCloud()
    nav_path_lines = o3d.geometry.LineSet()
    nav_grid_added = False
    nav_path_added = False
    preview_on = False
    last_preview = 0.0
```

c) Also in `main()`, next to `save_ply()`, add the shared overlay updater (Task 16's explorer reuses it):

```python
    def update_nav_overlay(grid, floor_y, path_world):
        nonlocal nav_grid_added, nav_path_added
        if grid is None:
            return
        gpts, gcol = grid_overlay(grid, floor_y)
        nav_grid_pcd.points = o3d.utility.Vector3dVector(gpts)
        nav_grid_pcd.colors = o3d.utility.Vector3dVector(gcol)
        if not nav_grid_added:
            vis.add_geometry(nav_grid_pcd, reset_bounding_box=False)
            nav_grid_added = True
        else:
            vis.update_geometry(nav_grid_pcd)
        ppts, plines = path_overlay(path_world, floor_y)
        if len(ppts) >= 2:
            nav_path_lines.points = o3d.utility.Vector3dVector(ppts)
            nav_path_lines.lines = o3d.utility.Vector2iVector(plines)
            nav_path_lines.paint_uniform_color([0.2, 0.5, 1.0])
            if not nav_path_added:
                vis.add_geometry(nav_path_lines, reset_bounding_box=False)
                nav_path_added = True
            else:
                vis.update_geometry(nav_path_lines)
```

d) In the while loop, after the moving-marker block, before `if not vis.poll_events()`:

```python
            # --- walkthrough map preview (toggle with 'G') ---
            if preview_on and pose_vals is not None and time.time() - last_preview > 2.0:
                last_preview = time.time()
                res = preview_plan(store_xyz, pose_to_2d(pose_from_streamed(pose_vals)))
                update_nav_overlay(res.grid, res.floor_y, res.path_world)
```

e) Key handling — add before the ESC branch:

```python
            elif key == ord('g'):
                preview_on = not preview_on
                print(f"[viewer] map preview {'ON' if preview_on else 'OFF'}")
```

f) Update the module docstring's key list to:
`Keys (focus the camera window): S = save PLY, G = toggle map preview, ESC = quit (also saves).`

- [ ] **Step 6: Walk-around verification (phone only)**

Run: `cd viewer; python -m pytest tests/ -v` → all pass. Then `python pc_viewer.py`, connect the phone app as usual, and walk the phone around the room at roughly car height, pointed slightly down. Press `G`. Expected within a couple of seconds: a flat colored grid appears on the floor of the 3D view (green = drivable, amber = too close to obstacles, red = obstacles), refreshing every ~2 s as you scan more, with a blue line from your position to the nearest unexplored frontier. Stray outlier specks in the raw cloud should NOT appear as red cells (they're filtered by `clean_cloud`).

- [ ] **Step 7: Commit**

```bash
git add viewer/nav/preview.py viewer/tests/test_preview.py viewer/pc_viewer.py
git commit -m "feat: phone-only walkthrough map preview (G key)"
```

---

### Task 16: pc_viewer.py integration (reverse channel + nav tick + overlay + keys)

No unit tests here — this file is sockets + GUI; all logic it calls is already tested. Verification is a hardware-free smoke test at the end.

**Files:**
- Modify: `viewer/pc_viewer.py`

- [ ] **Step 1: Add import** — with the nav imports added in Task 15b:

```python
from nav.runner import NavRunner
```

- [ ] **Step 2: Add shared state** — after the `_running = [True]` line:

```python
_conn_holder = [None]        # live phone connection, for PC -> phone commands
_latest_pose_time = [0.0]    # wall-clock time of the last 'O' pose message
_tracking_ok = [True]        # last ARKit tracking status reported by the phone
```

- [ ] **Step 3: Track the connection in `receiver()`** — after `conn.settimeout(None)` add `_conn_holder[0] = conn`, and make the `finally` block:

```python
        finally:
            _conn_holder[0] = None
            conn.close()
            print("[viewer] iPhone disconnected -- waiting for reconnect")
```

- [ ] **Step 4: Record pose time and handle tracking messages** — replace the `'O'` branch and add a `'T'` branch after the `'I'` branch:

```python
                elif mtype == 0x4F:  # 'O' pose
                    vals = struct.unpack("<16f", payload)
                    with _lock:
                        _latest_pose[0] = vals
                        _latest_pose_time[0] = time.time()
                        _path.append(np.array([vals[12], vals[13], vals[14]], dtype=np.float64))

                elif mtype == 0x49:  # 'I' jpeg
                    with _lock:
                        _latest_jpeg[0] = payload

                elif mtype == 0x54:  # 'T' ARKit tracking status (0 = normal)
                    with _lock:
                        _tracking_ok[0] = (len(payload) >= 1 and payload[0] == 0)
```

- [ ] **Step 5: Add the send helper** — a new top-level function after `receiver()`:

```python
def send_to_phone(data):
    """Best-effort command frame to the phone; silently dropped when the phone
    is disconnected (every safety layer assumes commands can vanish)."""
    conn = _conn_holder[0]
    if conn is None:
        return
    try:
        conn.sendall(data)
    except OSError:
        pass
```

- [ ] **Step 6: Create the runner in `main()`** — next to the preview state from Task 15b:

```python
    nav_runner = NavRunner(send_to_phone)
    last_overlay_grid = None
```

(The overlay geometries, `preview_on`, and `update_nav_overlay` already exist from Task 15b.)

- [ ] **Step 7: Extend the lock snapshot** at the top of the while loop:

```python
            with _lock:
                batch = _incoming_points[:]
                _incoming_points.clear()
                path_copy = list(_path)
                pose_vals = _latest_pose[0]
                jpeg = _latest_jpeg[0]
                pose_time = _latest_pose_time[0]
                tracking_ok = _tracking_ok[0]
```

- [ ] **Step 8: Add the nav tick + overlay refresh** — after the walkthrough-preview block from Task 15b:

```python
            # --- autonomous navigation tick (rate-limited internally) ---
            nav_runner.tick(time.time(), pose_vals, pose_time, store_xyz, tracking_ok)

            # --- occupancy grid / path overlay (refreshed after each PLAN) ---
            ex = nav_runner.explorer
            if ex.grid is not None and ex.grid is not last_overlay_grid:
                last_overlay_grid = ex.grid
                update_nav_overlay(ex.grid, ex.floor_y, ex.path_world)
```

- [ ] **Step 9: Add the keyboard controls** — replace the key-handling block (keeping the `g` branch from Task 15b; exploration turns the preview off since the explorer owns the overlay while active):

```python
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save_ply()
            elif key == ord('g'):
                preview_on = not preview_on
                print(f"[viewer] map preview {'ON' if preview_on else 'OFF'}")
            elif key == ord('e'):
                preview_on = False
                print("[viewer] exploration STARTED ('x' to stop)")
                nav_runner.start()
            elif key == ord('x'):
                print("[viewer] exploration STOPPED")
                nav_runner.stop()
            elif key == 27:  # ESC
                break
```

- [ ] **Step 10: Stop autonomy on exit** — first line of the `finally:` block in `main()`:

```python
        nav_runner.stop()
```

Also update the module docstring's key list to:
`Keys (focus the camera window): S = save PLY, G = map preview, E = start exploration, X = stop exploration, ESC = quit (also saves).`

- [ ] **Step 11: Smoke test without hardware**

Run: `cd viewer; python -m pytest tests/ -v` — Expected: all pass.
Run: `cd viewer; python pc_viewer.py` — Expected: window opens, `listening on port 9000` printed. Focus the camera window, press `e` (prints exploration STARTED), `x` (STOPPED), then ESC. No traceback at any point (with no phone connected, commands are silently dropped by `send_to_phone`).

- [ ] **Step 12: Commit**

```bash
git add viewer/pc_viewer.py
git commit -m "feat: viewer reverse command channel, nav tick, overlay, E/X keys"
```

---

### Task 17: Swift — bidirectional streamer (receive DRIVE/MODE, send tracking)

Swift changes have no test runner in this repo; each Swift task's verification is "builds clean in Xcode and behaves in the smoke test." Keep the diffs exactly as written.

**Files:**
- Modify: `PointCloudScanner/PointCloudStreamer.swift`

- [ ] **Step 1: Add command callbacks and start the receive loop**

Add these properties to `PointCloudStreamer` (after `@Published var status`):

```swift
    /// Commands arriving from the PC (set by ContentView before streaming).
    var onDrive: ((Int, Int) -> Void)?
    var onScanMode: ((Bool) -> Void)?
```

In `start(host:port:)`, replace the `stateUpdateHandler` assignment so the receive loop starts once the socket is ready:

```swift
        conn.stateUpdateHandler = { [weak self] state in
            if case .ready = state { self?.receiveLoop() }
            DispatchQueue.main.async {
                switch state {
                case .ready:
                    self?.isStreaming = true;  self?.status = "connected"
                case .waiting(let e):
                    self?.isStreaming = false; self?.status = "waiting: \(e.localizedDescription)"
                case .failed(let e):
                    self?.isStreaming = false; self?.status = "failed: \(e.localizedDescription)"
                case .cancelled:
                    self?.isStreaming = false; self?.status = "not connected"
                default:
                    break
                }
            }
        }
```

- [ ] **Step 2: Add the receive loop and message handler**

Add a new section at the bottom of the class:

```swift
    // MARK: - Receiving (PC -> phone commands)

    // Frames use the same layout as outgoing ones:
    // type(1 byte) + payloadLength(uint32 LE) + payload.
    private func receiveLoop() {
        guard let conn = connection else { return }
        conn.receive(minimumIncompleteLength: 5, maximumLength: 5) { [weak self] header, _, isDone, error in
            guard let self = self, let header = header, header.count == 5,
                  error == nil else { return }
            let type = header[0]
            let length = UInt32(header[1]) | (UInt32(header[2]) << 8)
                       | (UInt32(header[3]) << 16) | (UInt32(header[4]) << 24)
            if length == 0 {
                self.handleMessage(type, Data())
                if !isDone { self.receiveLoop() }
                return
            }
            conn.receive(minimumIncompleteLength: Int(length), maximumLength: Int(length)) { payload, _, isDone2, error2 in
                guard let payload = payload, payload.count == Int(length),
                      error2 == nil else { return }
                self.handleMessage(type, payload)
                if !isDone2 { self.receiveLoop() }
            }
        }
    }

    private func handleMessage(_ type: UInt8, _ payload: Data) {
        switch type {
        case 0x44: // 'D' drive: ASCII "L<left>R<right>"
            guard let s = String(data: payload, encoding: .ascii),
                  s.hasPrefix("L"),
                  let ri = s.firstIndex(of: "R"),
                  let left = Int(s[s.index(after: s.startIndex)..<ri]),
                  let right = Int(s[s.index(after: ri)...]) else { return }
            DispatchQueue.main.async { self.onDrive?(left, right) }
        case 0x4D: // 'M' mode: ASCII "SCAN" or "IDLE"
            let scan = String(data: payload, encoding: .ascii) == "SCAN"
            DispatchQueue.main.async { self.onScanMode?(scan) }
        default:
            break
        }
    }
```

- [ ] **Step 3: Add the tracking-status message**

Add to the `// MARK: - Messages` section:

```swift
    /// ARKit tracking health: 0 = normal, 1 = limited, 2 = not available.
    func sendTracking(_ status: UInt8) {
        guard isStreaming else { return }
        send(type: 0x54, payload: Data([status])) // 'T'
    }
```

- [ ] **Step 4: Verify it builds and still streams**

Build the app in Xcode (⌘B): no errors. Run it against `pc_viewer.py` as before: points, pose, and video still arrive (nothing regresses while the new receive path idles).

- [ ] **Step 5: Commit**

```bash
git add PointCloudScanner/PointCloudStreamer.swift
git commit -m "feat: bidirectional streamer -- receive drive/mode, send tracking status"
```

---

### Task 18: Swift — scan gating, BLE relay, tracking-loss safety stop

Three behaviors: (1) **pose always streams, LiDAR points only in SCAN mode** — this is the "stop auto-spamming LiDAR" requirement; (2) PC drive commands relay to the ESP32 through the existing `CarController`; (3) the phone stops the car *itself* when ARKit tracking degrades (redundancy layer 3 — works even if the PC link is dead).

**Files:**
- Modify: `PointCloudScanner/ARDepthView.swift`
- Modify: `PointCloudScanner/ContentView.swift`

- [ ] **Step 1: Gate point capture (not pose) on `isScanning`**

In `ARDepthView.swift`:

a) The struct gains a tracking-loss callback. Replace the property list of `ARDepthView` with:

```swift
    let accumulator: PointCloudAccumulator
    let streamer: PointCloudStreamer
    @Binding var isScanning: Bool
    let onTrackingLost: () -> Void
    let onPointCountUpdate: (Int) -> Void
```

and pass it through `makeCoordinator`:

```swift
    func makeCoordinator() -> Coordinator {
        Coordinator(accumulator: accumulator, streamer: streamer,
                    onTrackingLost: onTrackingLost,
                    onPointCountUpdate: onPointCountUpdate)
    }
```

b) In `Coordinator`, add the stored property and init parameter:

```swift
        let onTrackingLost: () -> Void
```

```swift
        init(accumulator: PointCloudAccumulator,
             streamer: PointCloudStreamer,
             onTrackingLost: @escaping () -> Void,
             onPointCountUpdate: @escaping (Int) -> Void) {
            self.accumulator = accumulator
            self.streamer = streamer
            self.onTrackingLost = onTrackingLost
            self.onPointCountUpdate = onPointCountUpdate
        }
```

c) In `session(_:didUpdate:)`, remove the early `guard isScanning else { return }` and instead capture the flag (pose/video must flow even when not scanning):

```swift
        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            frameCounter += 1
            guard frameCounter % frameStride == 0 else { return }
            guard !isProcessing else { return }
            guard let depth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return }
            let capturePoints = isScanning
```

and pass it into `process` (add `capturePoints: capturePoints` to the call and `capturePoints: Bool` to the method signature).

d) In `process(...)`, wrap only the unprojection double-loop:

```swift
            if capturePoints {
                for row in stride(from: 0, to: depthHeight, by: pixelStride) {
                    ...existing loop body unchanged...
                }
            }
```

Everything after the loop (streaming pose/video, throttled geometry rebuild) stays outside the `if`. (`accumulator.drainNew()` is empty when nothing was captured, so no stray point messages go out.)

- [ ] **Step 2: Report tracking state and stop the car on degradation**

Add to `Coordinator`:

```swift
        func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
            let status: UInt8
            switch camera.trackingState {
            case .normal:       status = 0
            case .limited:      status = 1
            case .notAvailable: status = 2
            }
            streamer.sendTracking(status)
            if status != 0 {
                DispatchQueue.main.async { self.onTrackingLost() }
            }
        }
```

- [ ] **Step 3: Wire the CarController in ContentView**

In `ContentView.swift`:

a) Add the controller next to the other state objects:

```swift
    @StateObject private var car = CarController()
```

b) Update the `ARDepthView` call site:

```swift
            ARDepthView(accumulator: model.accumulator,
                        streamer: streamer,
                        isScanning: $isScanning,
                        onTrackingLost: { car.stop() }) { count in
                pointCount = count
            }
```

c) Wire the PC command callbacks. Add to the outer `ZStack`'s modifiers (next to `.sheet`):

```swift
        .onAppear {
            streamer.onDrive = { [weak car] left, right in
                car?.drive(left: left, right: right)
            }
            streamer.onScanMode = { scan in isScanning = scan }
        }
```

d) Show BLE state in `connectionBar` — add after the existing stream `Circle()`:

```swift
                Circle()
                    .fill(car.isConnected ? Color.blue : Color.gray)
                    .frame(width: 10, height: 10)
                Text("BLE").font(.caption2).foregroundStyle(.secondary)
```

- [ ] **Step 4: Add the Bluetooth permission**

In the Xcode target's Info tab add `NSBluetoothAlwaysUsageDescription` = `Connects to the robot car over Bluetooth to send drive commands.` (Instantiating `CarController` starts CoreBluetooth; without this key the app crashes at launch.)

- [ ] **Step 5: Verify on device**

Build & run on the iPhone. Expected: app launches (no permission crash), streams to `pc_viewer.py` as before, BLE dot turns blue when the ESP32 is powered. Pressing `e` in the viewer prints STARTED and (with the ESP32 on blocks) the wheels counter-rotate — full loop proof.

- [ ] **Step 6: Commit**

```bash
git add PointCloudScanner/ARDepthView.swift PointCloudScanner/ContentView.swift
git commit -m "feat: scan-mode gating, BLE command relay, tracking-loss safety stop"
```

---

### Task 19: Field test & calibration (manual checklist)

No code changes except possibly flipping `TURN_SIGN`. Run in order; each step gates the next.

- [ ] **Step 1: Full regression** — `cd viewer; python -m pytest tests/ -v` → all pass.

- [ ] **Step 2: Bench test (wheels off the ground).** Firmware unchanged — flash `firmware/esp32_car.ino` only if not already on the board. Start `pc_viewer.py`, connect the app, confirm BLE dot blue. Press `e`: phone should enter SCAN mode and the wheels counter-rotate (SPIN). Press `x`: wheels stop.

- [ ] **Step 3: ESP32 failsafe (redundancy layer 1).** While wheels spin, close the viewer window. Expected: wheels stop within ~0.5 s (no more heartbeats → firmware failsafe).

- [ ] **Step 4: Tracking failsafe (redundancy layer 3).** Press `e` again, then cover the iPhone cameras with your hand. Expected: wheels stop immediately (phone-side `car.stop()`), and they stay stopped while covered (PC-side `tracking_ok` stop). Uncover: spin resumes within a second or two.

- [ ] **Step 5: TURN_SIGN calibration (on the floor now).** Place the car in the room, press `e`. During SPIN, watch the pose marker in the viewer — the spin direction doesn't matter (progress is measured as absolute rotation). After the spin, the grid (green/amber/red) and blue path appear. Watch the first DRIVE: if the car repeatedly rotates **away** from the blue path or oscillates left-right without settling, edit `viewer/nav/config.py` and set `TURN_SIGN = -1`, restart the viewer, and repeat. Commit the calibrated value:

```bash
git add viewer/nav/config.py
git commit -m "chore: calibrate TURN_SIGN for this chassis"
```

- [ ] **Step 6: First exploration run.** Clear a room area, good lighting, press `e`, and let it run scan → drive → scan cycles. Success criteria: it visits at least 2 frontiers, the grid grows to cover the area, and it eventually prints no new goals (DONE) or you stop it with `x`.

- [ ] **Step 7: Tune.** Adjust in `viewer/nav/config.py` as observed:
  - Car clips furniture → raise `ROBOT_RADIUS` (re-inflates next PLAN).
  - Overshoots waypoints / lurches → lower `DRIVE_SPEED`, raise `ARRIVE_DIST` to 0.15.
  - Stops short of walls calling them frontiers → normal; it rescans there.
  - On thick rugs the floor band may misclassify → widen `floor_band` to 0.06 in the `build_occupancy_grid` call inside `explorer._plan`.

**Troubleshooting:**

| Symptom | Likely cause | Fix |
|---|---|---|
| Car never leaves SPIN | Pose not streaming / stale | Check viewer prints pose marker moving; check Wi-Fi |
| PLAN → DONE instantly | Whole floor area walled/inflated | Lower `ROBOT_RADIUS`; check floor estimate vs a table being picked |
| Car drives into a wall | Obstacle below `clearance` band (very low object) | Lower `clearance` to 0.03 in `_plan`'s grid call |
| Pose visibly wrong after long run, ICP not helping | Poor overlap between spin and map | Spin slower (`SPIN_SPEED` down); ensure spins happen in feature-rich spots |
| Endless rescan loop at one spot | Frontier behind glass/mirror | Press `x`, reposition the car, restart with `e` |

---

## Execution notes — two milestones

**Milestone A: phone-only mapping (no car needed).** Tasks 1–8, 15, 15b. Everything except the final walk-around verification runs on this Windows machine with no hardware at all; the verification needs only the iPhone with the *existing, unmodified* app. Deliverable: walk the room holding the phone, press `G`, and watch the cleaned occupancy grid, frontier goal, and planned path render live over the point cloud. This proves the map simplification, outlier removal, and pathfinding on real data before any car exists.

**Milestone B: the car drives itself.** Tasks 9–14, 16 (still PC-only, fully unit-tested), then 17–18 (need Xcode/Mac + iPhone) and 19 (needs the assembled car).

- Task order within a milestone matters (later tasks import earlier modules); Milestone A can be executed before Tasks 9–14 exist.
- The ESP32 firmware is deliberately untouched: its command grammar (`L..R..`) and 0.5 s failsafe are already exactly what the stack needs.




