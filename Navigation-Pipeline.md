# Point Cloud → Navigable Map: Design & Research

How to turn the streaming LiDAR point cloud into something the robot can drive
through, and how to pick where it scans next. Tailored to your setup: **iPhone
(ARKit) produces the cloud + pose → streams to the PC → Python/Open3D does the
processing**. Each stage below names the concrete method, the library call, and
sane starting parameters for indoor iPhone-LiDAR data.

## The pipeline at a glance

```
raw points  ─►  1. downsample  ─►  2. remove outliers  ─►  3. split floor vs
(streamed)        (voxel grid)       (statistical/radius)     obstacles (RANSAC/height)
                                                                     │
   6. pick next scan target  ◄─  5. locate car in grid  ◄─  4. build 2D occupancy
      (frontier + path)              (from ARKit pose)          grid + inflate
                                                                     │
                                            ─►  drive to goal, 360° scan, repeat
```

Stages 1–4 run on the **accumulated map** (your `store_xyz` in `pc_viewer.py`),
re-run every second or two — not on every incoming packet.

---

## 0. Where the car is (localization) — you mostly already have it

This is the part people usually struggle with, and your architecture hands it to
you: **ARKit's visual-inertial odometry gives the phone's full 6-DoF pose every
frame** (position in meters + orientation), in a **gravity-aligned** map frame.
The phone is bolted to the car, so the **car's pose = the phone's pose** (times a
fixed mounting offset you measure once). You're already streaming this (the `O`
pose messages), so "where is the car" is answered continuously.

For navigation you only need the **2D floor pose**: project the phone position
onto the floor plane and take the heading (yaw) about the up axis → `(x, y, θ)`.

**When you need more than raw ARKit pose — drift correction.** VIO slowly drifts
over long runs, and can jump if tracking is briefly lost (blank wall, fast spin).
The standard fix is **scan-to-map ICP registration**: align the current local
scan to the accumulated map and snap the pose back onto it. In Open3D that's
`registration_icp` (use **multi-scale ICP** — coarse-to-fine — so it doesn't fall
into a local minimum on sparse data). Run it opportunistically (e.g., after each
360° scan), not every frame. This is also how you **relocalize** after a tracking
loss. For a first version, raw ARKit pose is enough; add ICP when you see drift.

---

## 1. Downscale the points (voxel downsampling)

**Why:** uniform density, far less compute, and it removes the redundant clumping
where many samples hit the same surface.

**Method:** voxel-grid downsampling — overlay a 3D grid and keep one point per
occupied cell.

```python
pcd = pcd.voxel_down_sample(voxel_size=0.03)   # 3 cm cells
```

**Params:** `voxel_size` 0.03–0.05 m for navigation. (You already voxel-accumulate
at 1.5 cm on the phone; for the *nav* map, coarser is faster and plenty.) Bigger
voxel = fewer points = faster planning, less detail.

---

## 2. Filter outliers

LiDAR produces stray "flying" points (edge noise, reflective surfaces, a person
walking through). Remove them so they don't become phantom obstacles.

**Statistical outlier removal** — drops points whose mean distance to their `k`
neighbors is far above the cloud average:

```python
pcd, keep = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
```

**Radius outlier removal** — drops points with too few neighbors within a radius
(kills isolated specks):

```python
pcd, keep = pcd.remove_radius_outlier(nb_points=12, radius=0.05)
```

**Params:** `nb_neighbors=20, std_ratio=2.0` is a good default (lower `std_ratio`
= more aggressive). Do this **after** downsampling (cheaper, and neighbor counts
are more uniform). Statistical removal alone is usually enough; add radius removal
if specks persist.

---

## 3. Split floor from obstacles

A ground robot cares about three things per location: **floor** (drivable),
**obstacle at its own height** (blocked), and **stuff too high to matter**
(ignore). Two complementary ways to separate them:

**a) RANSAC plane fit** — find the dominant plane (the floor):

```python
plane, inliers = pcd.segment_plane(distance_threshold=0.02,
                                   ransac_n=3, num_iterations=1000)
# plane = (a,b,c,d): ax+by+cz+d = 0; inliers index the floor points
```

**b) Height-band thresholding (simpler here)** — because ARKit's map is
**gravity-aligned**, "up" is a known axis, so you can just slice by height:

- points within ~0.02 m of floor level → **floor / free**
- points from `floor+clearance` up to `robot_height` (e.g., 0.05–0.35 m) →
  **obstacle** (this is what the robot body would hit)
- points above `robot_height` → **ignore** (overhangs it drives under)
- absence of floor where floor is expected → **drop-off / unknown** (treat as
  blocked, for safety)

Use the plane fit to establish the exact floor height/tilt, then the height-band
test to classify everything else. This directional 3D reasoning is exactly why
the phone beats a flat 2D LiDAR: it sees low toys *and* table-edge overhangs.

---

## 4. Build the 2D occupancy grid (the navigable map)

Collapse the classified 3D points into a top-down grid — the representation the
planner actually uses. This is the standard **occupancy grid → costmap** approach
(same idea as ROS Nav2's costmap, projected to 2D).

- Lay a 2D grid over the floor (horizontal axes), **cell size ≈ 0.05 m**.
- For each cell, look at the column of points above it:
  - never observed → **unknown**
  - floor seen, no obstacle points in the height band → **free**
  - obstacle points present in the band (above a small `mark_threshold` count) →
    **occupied**
- **Inflate** the occupied cells by the robot's radius so the planner can treat
  the robot as a single point. Cheapest implementation: `scipy.ndimage` binary
  dilation of the obstacle mask by `ceil(robot_radius / cell_size)` cells, or a
  distance transform for a graded "keep-away" cost.

```python
# obstacle_mask: bool grid of occupied cells
from scipy.ndimage import binary_dilation, distance_transform_edt
r_cells = int(np.ceil(robot_radius / cell_size))     # e.g. 0.12 m / 0.05 = 3
blocked = binary_dilation(obstacle_mask, iterations=r_cells)
```

Result: a grid of `free / blocked / unknown`. That grid **is** "something the
robot can navigate through."

---

## 5. Locate the car in the grid

Trivial once you have pose + grid origin: `col = (x - origin_x)/cell_size`,
`row = (y - origin_y)/cell_size`. Keep the heading `θ` for choosing turn
direction. (If you added ICP drift-correction in Stage 0, use the corrected pose
here.)

---

## 6. Pick the next place to scan (frontier-based exploration)

This is the classic, proven strategy (**Yamauchi, 1997**): drive to the boundary
between known-free and unknown space, scan, and repeat until no boundary remains.

**Detect frontiers.** A **frontier cell** = a **free** cell touching at least one
**unknown** cell. Find them, then group adjacent frontier cells into clusters:

```python
frontier = free_mask & neighbor_is_unknown(free_mask, unknown_mask)
from skimage.measure import label
clusters = label(frontier)          # connected frontier groups
# centroid + size of each cluster
```

**Filter clusters:** keep only those that are (a) **reachable** — connected to the
car through free space (flood-fill/BFS from the car cell over free cells), and
(b) **big enough** to be worth visiting (drop 1–2 cell specks).

**Choose the goal.** Two common cost functions:

- **Nearest frontier** (Yamauchi) — simplest, minimizes travel: pick the reachable
  cluster whose centroid is closest to the car (distance along free space).
- **Best info-gain / next-best-view** — pick `argmax(cluster_size / distance)` (or
  weight expected newly-revealed area against travel cost) for more efficient
  coverage.

Start with nearest-frontier; upgrade to info-gain later.

**Plan the path** to the chosen cell with **A\*** or Dijkstra on the inflated free
grid (grid graph, 8-connected). The output is a list of waypoints.

**Execute + rescan.** Drive the waypoints closed-loop against ARKit pose (turn to
face the next waypoint, drive while the path stays clear, using the BLE `L..R..`
commands), and on arrival do the **360° in-place spin scan** to fill in that area.
Then re-run stages 1–6. When no reachable frontiers remain, the room is mapped.

---

## Where it runs & the command path

Prototype all of this on the **PC in Python** (that's where Open3D lives and where
you already receive the stream). To actually drive, the plan's commands need to
reach the ESP32. Cleanest with what you've built:

- **PC → phone → ESP32:** add a small reverse channel (PC sends the chosen
  `L..R..` command back over a socket to the phone; the phone relays it via the
  existing `CarController` BLE). Reuses everything.
- **PC → ESP32 directly:** the PC speaks BLE to the ESP32 itself (Python `bleak`
  library), bypassing the phone for driving. The phone then only does
  perception/pose.
- **All on the phone:** eventually port the grid+frontier logic to Swift so the
  robot is self-contained (no PC needed to drive). The PC stays a nice big-screen
  monitor/recorder.

A heavier but batteries-included alternative is the full **ROS 2 + Nav2** stack
(`nav2_costmap_2d` for the grid/inflation, `slam_toolbox`, plus a frontier
exploration package). Powerful, but a big dependency jump — only worth it if you
want the whole professional navigation stack.

---

## Parameter cheat-sheet (indoor iPhone LiDAR)

| Stage | Call | Start value |
|---|---|---|
| Downsample | `voxel_down_sample` | `voxel_size = 0.03–0.05` m |
| Outliers (statistical) | `remove_statistical_outlier` | `nb_neighbors=20, std_ratio=2.0` |
| Outliers (radius) | `remove_radius_outlier` | `nb_points=12, radius=0.05` |
| Floor fit | `segment_plane` | `distance_threshold=0.02, ransac_n=3, iters=1000` |
| Obstacle height band | height threshold | `0.05 m … 0.35 m` above floor |
| Grid resolution | occupancy grid | `cell_size = 0.05` m |
| Robot inflation | dilation radius | `robot_radius ≈ 0.12` m |
| Frontier cluster | min size | `≥ 5` cells |
| Drift fix | `registration_icp` (multi-scale) | run after each 360° scan |

---

## How it plugs into `pc_viewer.py`

You already accumulate the full map in `store_xyz / store_rgb`. Add a `nav`
module that, every ~1–2 s, takes that array and runs: downsample → outlier removal
→ floor/obstacle split → occupancy grid + inflation → frontier detection → goal +
A\* path. Then draw the grid, frontier clusters, chosen goal, and path in the same
Open3D window (as a flat colored overlay at floor height), and — when you're ready
to drive — emit the waypoint commands over the reverse channel.

I can build that `nav` module next: a self-contained Python file that takes a
point-cloud array and returns `(occupancy_grid, car_cell, next_goal, path)`, with
an Open3D overlay so you can watch the grid and the chosen frontier update live.

---

## Sources

- Open3D — voxel downsampling, statistical/radius outlier removal, RANSAC plane segmentation: https://www.open3d.org/docs/latest/tutorial/geometry/pointcloud_outlier_removal.html and https://www.open3d.org/docs/release/tutorial/t_geometry/pointcloud.html
- Open3D — ICP / multi-scale ICP registration (scan-to-map, drift correction): https://www.open3d.org/docs/0.15.1/tutorial/t_pipelines/t_icp_registration.html
- Yamauchi frontier-based exploration (overview + implementation): https://arxiv.org/pdf/1806.03581
- Next-best-view exploration planning: https://arxiv.org/pdf/2109.09323
- ROS 2 Nav2 costmap_2d — occupancy grid, voxel layer, obstacle inflation: https://docs.nav2.org/configuration/packages/configuring-costmaps.html and https://index.ros.org/p/nav2_costmap_2d/
- Nav2 mapping & localization overview: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
