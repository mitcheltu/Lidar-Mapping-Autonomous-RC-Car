# Technical Specification & System Design

## 1. System Architecture & Flow Summary

```text
+------------------+   ARFrame Data    +-------------------+   Binary WS Buffer  +--------------------+
|   iPhone LiDAR   |  ==============>  |  iOS Swift Client |  =================> | Laptop Workstation |
| (dToF + ARKit)   |  (Depth + Pose)   | (Pack Binary TCP) |   (30 Hz / Wi-Fi)   | (PyQt6 + PyVista)  |
+------------------+                   +-------------------+                     +--------------------+
                                                                                           |
                                                                                           v
+-------------------------------------------------------------------------------------------------+
| Laptop Processing Pipeline                                                                      |
|                                                                                                 |
|   1. Data Ingestion: Unpack 6-DOF Pose Matrix T_ARKit & Depth Matrix                          |
|   2. Point Cloud Unprojection: P_cam = K^-1 * [u, v, d]^T                                       |
|   3. Rig Rigid Transform: P_world = T_ARKit * T_extrinsic * P_cam                             |
|   4. KISS-ICP SLAM: Map Registration, Drift Detection, & Pose Correction Matrix T_corr        |
|   5. Voxel Engine (OpenVDB / STVL Raycasting): Categorize Space (Occupied, Free, Ground)      |
|   6. 3D GUI Rendering Engine: PyQt6 + PyVista (Interactive Checkbox Toggles per Geometry Layer) |
+-------------------------------------------------------------------------------------------------+
```

## 2. Sensor Processing & Coordinate Transformation Formulas

### 2.1 Depth Unprojection (Intrinsic Projection Matrix)

Given camera intrinsics $(f_x, f_y, c_x, c_y)$ from ARFrame.camera.intrinsics and pixel coordinates $(u, v)$ with depth value $d = D(u, v)$ in meters:

$$
P_{cam} =
\begin{bmatrix}
X_{cam} \\
Y_{cam} \\
Z_{cam}
\end{bmatrix}
=
\begin{bmatrix}
\frac{(u - c_x) \cdot d}{f_x} \\
\frac{(v - c_y) \cdot d}{f_y} \\
 d
\end{bmatrix}
$$

### 2.2 Extrinsic Rigid Transformation

Transform camera-frame 3D coordinates $P_{cam}$ to global world-frame coordinates $P_{world}$ using ARKit 6-DOF pose $(T_{ARKit})$ and a static extrinsic translation/rotation offset from the robot base $(T_{extrinsic})$:

$$
P_{world} = T_{ARKit} \cdot T_{extrinsic} \cdot \begin{bmatrix} P_{cam} \\ 1 \end{bmatrix}
$$

Where $T_{ARKit}, T_{extrinsic} \in \mathbb{SE}(3)$ are $4 \times 4$ homogeneous transformation matrices.

## 3. Drift Verification & Closed-Loop Correction (KISS-ICP)

To compensate for Visual-Inertial Odometry (VIO) drift without physical wheel encoders, real-time point-to-point / point-to-plane ICP registration is applied on incoming point clouds against a persistent global point cloud map $M_{t-1}$.

### 3.1 Point Cloud Alignment Optimization

Find the relative spatial correction matrix $T_{corr} \in \mathbb{SE}(3)$ minimizing point-to-plane residual errors:

$$
\arg\min_{T_{corr}} \sum_i \left( \left( T_{corr} \cdot P_{live,i} - M_{map,i} \right) \cdot \mathbf{n}_i \right)^2
$$

Where $\mathbf{n}_i$ is the surface normal vector at map landmark point $M_{map,i}$.

### 3.2 Correction Integration & Execution Loop

- Pose calculation: $T_{corrected} = T_{corr} \cdot T_{ARKit}$
- Drift threshold verification: compute pose delta
  $$\Delta p = \| T_{corrected}(1:3,4) - T_{ARKit}(1:3,4) \|$$
- If $\Delta p > \delta_{drift}$ (for example $0.05\,m$ or $3^\circ$): overwrite the active trajectory state with $T_{corrected}$ and recalculate closed-loop differential motor commands via a Pure Pursuit controller.

## 4. Voxel Categorization & Mathematical Filtering

Voxels are discretized volumetric elements bounded by resolution $R$ (for example $R = 0.03\,m$). Continuous coordinates map to discrete index space $(V_x, V_y, V_z)$:

$$
V_x = \left\lfloor \frac{X_{world}}{R} \right\rfloor, \quad
V_y = \left\lfloor \frac{Y_{world}}{R} \right\rfloor, \quad
V_z = \left\lfloor \frac{Z_{world}}{R} \right\rfloor
$$

### 4.1 Categorization Rules

| Voxel Class | Mathematical / Geometric Condition | Functional Use |
| :--- | :--- | :--- |
| Occupied Voxels | $L(V) > L_{threshold}$ via 3D raycasting hit points ($Z_{ground\_max} < Z_{world} < Z_{robot\_height}$) | Obstacle avoidance and collision mapping |
| Free Voxels | $L(V) < L_{free\_threshold}$ along the ray path between the sensor origin $O_{cam}$ and the target hit point $P_{world}$ | Clear flight / drive space verification |
| Ground Voxels | $\|\mathbf{n}_z\| > \cos(\theta_{max\_slope})$ and $Z_{robot\_ground\_min} \le Z_{world} \le Z_{robot\_ground\_max}$ | Pathfinding traversability surface |

### 4.2 Log-Odds Updating Equation

$$
L(V_t) = L(V_{t-1}) + \begin{cases}
L_{occ} & \text{if hit by point-cloud ray end} \\
-L_{free} & \text{if traversed by ray}
\end{cases}
$$

## 5. Technology Stack & Software Libraries

- iOS client: ARKit for VIO tracking and dToF depth data, Metal Performance Shaders for GPU point unprojection, and Network.framework or Starscream for low-latency WebSockets.
- Point cloud SLAM and verification: kiss-icp / open3d.pipelines.registration for ICP registration and pose correction.
- Voxel engine: OpenVDB, OctoMap, or STVL (the OpenVDB-based spatio-temporal voxel layer) with automated voxel decay.
- Pathfinding and navigation: networkx for 3D grid graph A* or nav2_smac_planner / nav2_costmap_2d in the ROS 2 Navigation stack.
- Desktop UI and 3D visualization: PyQt6 for the desktop UI wrapper with sidebar checkboxes, combined with PyVista / PyVistaQt (VTK-backed 3D visualization with independent layer actor visibilities).

## 6. Python Reference Implementation (Laptop Node)

This script manages incoming WebSocket stream data, processes point-cloud voxels, runs real-time ICP drift correction, and displays a 3D interface with toggleable visualization layers.

```python
import sys
import json
import asyncio
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QCheckBox, QHBoxLayout
import open3d as o3d
import kiss_icp


class VoxelPipeline3D:
    def __init__(self, voxel_size=0.03):
        self.voxel_size = voxel_size
        self.global_map = o3d.geometry.PointCloud()

    def process_frame(self, raw_points, arkit_pose):
        """
        Unprojects depth, applies KISS-ICP pose correction, and categorizes voxels.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(raw_points)

        # 1. Transform points using ARKit pose
        pcd.transform(arkit_pose)

        # 2. KISS-ICP Drift Check & Verification
        # Align live frame with the aggregated global map.
        if len(self.global_map.points) > 1000:
            reg_icp = o3d.pipelines.registration.registration_icp(
                pcd,
                self.global_map,
                max_distance_threshold=0.05,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            )
            corrected_pose = reg_icp.transformation @ arkit_pose
            pcd.transform(reg_icp.transformation)
        else:
            corrected_pose = arkit_pose

        self.global_map += pcd

        # 3. Discretize into voxels and categorize
        points = np.asarray(pcd.points)
        voxel_indices = np.floor(points / self.voxel_size).astype(int)

        # Ground classification filter based on Z height and normals
        normals = np.asarray(pcd.normals) if pcd.has_normals() else np.zeros_like(points)
        ground_mask = (points[:, 2] < 0.05)

        ground_voxels = points[ground_mask]
        occupied_voxels = points[~ground_mask]

        return points, occupied_voxels, ground_voxels, corrected_pose


class AutonomousCarGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Autonomous RC Car - 3D Voxel Stream & Pathfinding")
        self.setGeometry(100, 100, 1280, 720)

        main_widget = QWidget()
        layout = QHBoxLayout(main_widget)
        self.setCentralWidget(main_widget)

        sidebar = QVBoxLayout()
        self.chk_pcd = QCheckBox("Show Point Cloud")
        self.chk_occ = QCheckBox("Show Occupied Voxels")
        self.chk_free = QCheckBox("Show Free Voxels")
        self.chk_ground = QCheckBox("Show Ground (Pathfinding) Voxels")

        for chk in [self.chk_pcd, self.chk_occ, self.chk_free, self.chk_ground]:
            chk.setChecked(True)
            chk.stateChanged.connect(self.update_layer_visibilities)
            sidebar.addWidget(chk)

        sidebar.addStretch()
        layout.addLayout(sidebar, stretch=1)

        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor, stretch=4)

        self.actors = {
            'point_cloud': None,
            'occupied_voxels': None,
            'free_voxels': None,
            'ground_voxels': None,
        }

    def update_layer_visibilities(self):
        """Dynamically controls layer actor visibilities based on PyQt checkboxes."""
        if self.actors['point_cloud']:
            self.actors['point_cloud'].SetVisibility(self.chk_pcd.isChecked())
        if self.actors['occupied_voxels']:
            self.actors['occupied_voxels'].SetVisibility(self.chk_occ.isChecked())
        if self.actors['free_voxels']:
            self.actors['free_voxels'].SetVisibility(self.chk_free.isChecked())
        if self.actors['ground_voxels']:
            self.actors['ground_voxels'].SetVisibility(self.chk_ground.isChecked())

        self.plotter.render()

    def render_frame_data(self, pcd_pts, occ_pts, free_pts, ground_pts):
        """Render geometries to the 3D scene."""
        if pcd_pts.size > 0:
            mesh_pcd = pv.PolyData(pcd_pts)
            if self.actors['point_cloud'] is None:
                self.actors['point_cloud'] = self.plotter.add_mesh(mesh_pcd, color='white', point_size=2)
            else:
                self.actors['point_cloud'].mapper.dataset = mesh_pcd

        if occ_pts.size > 0:
            mesh_occ = pv.PolyData(occ_pts).glyph(geom=pv.Cube(size=0.03))
            if self.actors['occupied_voxels'] is None:
                self.actors['occupied_voxels'] = self.plotter.add_mesh(mesh_occ, color='red', opacity=0.8)
            else:
                self.actors['occupied_voxels'].mapper.dataset = mesh_occ

        if free_pts.size > 0:
            mesh_free = pv.PolyData(free_pts).glyph(geom=pv.Cube(size=0.03))
            if self.actors['free_voxels'] is None:
                self.actors['free_voxels'] = self.plotter.add_mesh(mesh_free, color='blue', opacity=0.1)
            else:
                self.actors['free_voxels'].mapper.dataset = mesh_free

        if ground_pts.size > 0:
            mesh_ground = pv.PolyData(ground_pts).glyph(geom=pv.Cube(size=0.03))
            if self.actors['ground_voxels'] is None:
                self.actors['ground_voxels'] = self.plotter.add_mesh(mesh_ground, color='green', opacity=0.5)
            else:
                self.actors['ground_voxels'].mapper.dataset = mesh_ground

        self.update_layer_visibilities()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AutonomousCarGUI()
    window.show()
    sys.exit(app.exec())
```
