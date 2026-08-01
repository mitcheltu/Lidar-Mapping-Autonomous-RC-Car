# Unified Launcher — Design

**Date:** 2026-08-01
**Status:** approved, implementing

## Problem

Bringing up the stack takes three terminals and six commands: `colcon build`,
`source install/setup.bash`, `ros2 launch ... bringup.launch.py`, a second
terminal for `rerun_viz_node`, a third for `motion_enable_node`. The launch file
covers only five of the seven nodes — the viewer and the control console are not
in it.

## Constraint that shapes the design

`motion_enable_node` calls `tty.setraw(sys.stdin)`
(`autonomous_rc_car_ros/motion_enable_node.py:56`). A `ros2 launch` `Node` action
gives its child a pipe, not a TTY, so the console **cannot** be a launch entry —
it would fail in `termios.tcgetattr`. The console must run in the foreground of a
real terminal.

## Approach

A shell wrapper owns the terminal; `ros2 launch` owns everything else.

```
run.sh
 ├─ source /opt/ros/humble/setup.bash + ros2_ws/install/setup.bash
 ├─ colcon build            (only with --build, or if install/ is missing)
 ├─ resolve the Rerun address (see below)
 ├─ ros2 launch ... bringup.launch.py  ── background, own process group
 │     bridge / icp_slam / voxel_mapper / frontier_planner / motion_controller
 │     + rerun_viz_node                (viz:=true)
 └─ ros2 run ... motion_enable_node    ── FOREGROUND, inherits the real TTY
       on exit: trap kills the launch process group
```

One command, one terminal. Quitting the console (`q`, or Ctrl-C) tears down the
whole graph.

### Rerun address resolution

Default is **connect to a Rerun viewer running natively on Windows** — markedly
faster than rendering through WSLg.

1. `--connect <host:port>` wins if given.
2. `--spawn` forces the in-WSL WSLg viewer (`connect_addr` empty).
3. Otherwise auto-detect the Windows host:
   - `wslinfo --networking-mode` reports `mirrored` → `127.0.0.1`
   - otherwise NAT → default gateway from `ip route show default`
   - fall back to the `nameserver` line in `/etc/resolv.conf`
4. Probe `host:9876` with a 1 s TCP connect. **Unreachable → warn and fall back
   to the WSLg spawn**, so a forgotten viewer degrades instead of hanging.

### Launch file changes

`bringup.launch.py` gains `rerun_viz_node` and four arguments, keeping it usable
standalone:

| Arg | Default | Effect |
|---|---|---|
| `viz` | `true` | include `rerun_viz_node` |
| `connect_addr` | `''` | empty = spawn viewer in WSLg |
| `continuous` | `false` | `frontier_planner_node` replans on every `/map` |
| `start_enabled` | `false` | `motion_controller_node` armed at boot |

`continuous` and `start_enabled` are declared as `bool` in their nodes, so they
are passed through `ParameterValue(..., value_type=bool)` — a raw
`LaunchConfiguration` would arrive as a string and be rejected.

The existing docstring calling four of the nodes "stubs" is stale and gets fixed.

### run.sh flags

`--build`, `--no-viz`, `--no-console`, `--connect <addr>`, `--spawn`,
`--continuous`, `--go`, `-h/--help`.

`--no-console` runs the graph only and waits on the launch, for when the console
is being run elsewhere.

Safety default is unchanged: HOLD, on-demand planning. `--go` is opt-in.

## Error handling

- Not on Linux/WSL (`termios`, `/opt/ros`) → fail with a clear message.
- `install/setup.bash` missing → build automatically rather than erroring.
- `colcon build` failure → abort before launching anything.
- Rerun viewer unreachable → warn, fall back to WSLg spawn.
- Any exit path (`q`, Ctrl-C, error) → `trap` kills the launch process group, so
  no orphaned nodes hold port 9000 against the next run.

## Verification

The launcher orchestrates processes, so it is verified by running it in WSL2, not
by unit tests: `bash -n` syntax check, `--help`, `--no-console` bringup with
`ros2 node list` showing all six, then a full run with the console and a `q` exit
leaving no surviving `ros2` processes.

## Out of scope

- The Windows-side `wt.exe` wrapper (declined).
- `--symlink-install` for colcon — the existing install is a file copy
  (obs 805); mixing modes in one build tree is a separate change.
- `car_driver_node` (`/drive` → ESP32 websocket) — still blocked on the ESP32
  command format.
