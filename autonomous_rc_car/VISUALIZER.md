# Custom Visualizer (Rerun) — Plan & Usage

A tailored 3D viewer as an alternative to RViz2, built as a thin ROS→Rerun bridge
(`rerun_viz_node`). It subscribes to the same topics the graph already publishes and
streams them to the [Rerun](https://rerun.io) viewer, where every layer is its own
entity you can toggle, and the camera/interaction come for free.

## Why Rerun (vs Open3D GUI / PyVista+Qt)
- **Least code, most robust:** the viewer is a separate process, so there's no
  rclpy-vs-GUI threading to get wrong — the node just `rr.log()`s in its callbacks.
- **Toggling built in:** Rerun's entity tree gives a visibility checkbox per layer.
- **Runs where you want:** spawn the viewer in WSLg, or run the `rerun` viewer
  **natively on Windows** and have the node `connect` to it (better perf, no WSLg).
- Open3D `O3DVisualizer` and PyVista+PyQt6 remain valid alternatives (self-contained,
  real in-window widgets) if you later want the control buttons embedded in the 3D
  window — documented in chat. This plan implements the Rerun path first.

## Features (entity tree layout)
Each is a separate, independently-toggleable entity under `world/`:

| Entity | Source topic | Rerun archetype | Look |
|---|---|---|---|
| `world/cloud` | `/points` | `Points3D` | raw LiDAR cloud, light grey |
| `world/voxels/ground` | `/voxels/ground` | `Boxes3D` | green voxel cubes (floor) |
| `world/voxels/obstacle` | `/voxels/obstacle` | `Boxes3D` | red voxel cubes (obstacles) |
| `world/map` | `/map` | `Points3D` | flat 2D grid: green free / amber inflation / red occupied |
| `world/path` | `/cmd_path` | `LineStrips3D` | blue planned path |
| `world/phone` | `/pose` | `Points3D` + `Transform3D` | yellow point + RGB pose axes |

The world is logged **Z-up** to match the ROS `map` frame. Toggle any layer with its
checkbox in the Rerun **Blueprint/entity** panel; expand `world/voxels` to toggle
ground vs obstacle independently.

## Install (once, in the ROS2 Python env)
```bash
pip install rerun-sdk
```

## Run

**Normally you don't run this node yourself** — `autonomous_rc_car/run.sh` starts it
with the rest of the graph:
```bash
cd autonomous_rc_car && ./run.sh
```
It defaults to connecting to a **Rerun viewer running natively on Windows**
(recommended for performance): start `rerun` on Windows first — from a
`pip install rerun-sdk` there, or the Rerun desktop app — and `run.sh` auto-detects
the host address. If nothing answers on port 9876 it falls back to a WSLg viewer.

`./run.sh --spawn` forces the WSLg viewer; `./run.sh --connect <host>:9876` sets the
address explicitly; `./run.sh --no-viz` leaves the visualizer out.

Standalone, against a graph that is already running:
```bash
# spawn the viewer via WSLg:
ros2 run autonomous_rc_car_ros rerun_viz_node

# or point it at a viewer on Windows:
ros2 run autonomous_rc_car_ros rerun_viz_node --ros-args -p connect_addr:=<windows-ip>:9876
```

## Control
Visualization only — driving control stays in the existing console
(`motion_enable_node`: p=plan, g=go, h/SPACE=hold). This keeps the viewer read-only
and the safety gate in one place. (An Open3D/PyVista build could embed the buttons in
the 3D window instead; not needed here.)

## Notes / caveats
- Rerun's Python API evolves; this targets a recent `rerun-sdk`. If your version
  differs, the only likely tweaks are `Transform3D`/`Boxes3D` keyword names — the node
  is small and easy to adjust.
- It's additive: RViz2 still works. Run whichever you prefer against the same graph.
