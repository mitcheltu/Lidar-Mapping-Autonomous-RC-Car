#!/usr/bin/env python3
"""
pc_viewer.py -- live viewer + recorder for the PointCloudScanner iPhone app.

Runs a TCP server on this computer; the iPhone connects over Wi-Fi and streams:
  'P' points : uint32 count, then per point  float32 x,y,z + uint8 r,g,b   (15 bytes/pt)
  'O' pose   : 16 float32, the camera 4x4 transform (column-major)
  'I' image  : JPEG bytes of a downscaled camera frame
Each message is framed as: 1 byte type + uint32 little-endian payload length + payload.

Shows:
  - a live, growing 3D point cloud (Open3D window)
  - the robot's path (red) and a MOVING MARKER at the phone's current pose
    (RGB coordinate axes = position + which way it's facing)
  - the robot's camera feed (OpenCV window)

Continuously RECORDS every point to disk so the phone never has to keep them all:
  - captures/<session>/points_log.f32   (all points, appended live, crash-safe)
  - captures/<session>/map.ply          (autosaved every 15 s, on 'S', and on exit)

------------------------------------------------------------------
SETUP (Windows or macOS) -- one time:
    python -m pip install open3d numpy opencv-python
RUN:
    python pc_viewer.py
Then type THIS computer's LAN IP into the app and tap "Stream".
    Windows : ipconfig            (IPv4 Address, e.g. 192.168.1.20)
    macOS   : ipconfig getifaddr en0
Both devices on the SAME Wi-Fi. On Windows, allow Python through the firewall.

Keys (focus the camera window): S = save PLY, G = toggle map preview, ESC = quit (also saves).
Reload a saved session later:  np.fromfile("points_log.f32", "<f4").reshape(-1, 6)
                               columns = x, y, z, r, g, b   (rgb 0..1)
------------------------------------------------------------------
"""

import os
import socket
import struct
import threading
import time
from datetime import datetime

import numpy as np
import open3d as o3d
import cv2

from nav.localization import pose_from_streamed, pose_to_2d
from nav.overlay import grid_overlay, path_overlay
from nav.preview import preview_plan

HOST = "0.0.0.0"
PORT = 9000

AUTOSAVE_SECONDS = 15.0       # how often to write map.ply
RENDER_MAX = 500_000          # above this, the *displayed* cloud is downsampled
RENDER_REBUILD_SEC = 1.0      # how often to rebuild the displayed cloud when large
MARKER_SIZE = 0.30            # size of the phone-pose axes, meters

# If the camera feed looks sideways, set to cv2.ROTATE_90_CLOCKWISE, etc. None = off.
IMAGE_ROTATION = None

_lock = threading.Lock()
_incoming_points = []         # list of (xyz float32 [N,3], rgb float32 [N,3])
_path = []                    # camera world positions
_latest_pose = [None]         # last 16-float pose payload
_latest_jpeg = [None]
_running = [True]


def recvall(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def receiver():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[viewer] listening on port {PORT} -- enter this computer's IP in the app")

    while _running[0]:
        srv.settimeout(1.0)
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        print(f"[viewer] iPhone connected from {addr[0]}")
        conn.settimeout(None)
        try:
            while _running[0]:
                header = recvall(conn, 5)
                if header is None:
                    break
                mtype = header[0]
                length = struct.unpack("<I", header[1:5])[0]
                payload = recvall(conn, length) if length else b""
                if payload is None:
                    break

                if mtype == 0x50:  # 'P' points
                    count = struct.unpack("<I", payload[:4])[0]
                    body = np.frombuffer(payload[4:], dtype=np.uint8)
                    if body.size >= count * 15:
                        body = body[:count * 15].reshape(count, 15)
                        xyz = np.ascontiguousarray(body[:, :12]).view(np.float32).reshape(count, 3)
                        rgb = body[:, 12:15].astype(np.float32) / 255.0
                        with _lock:
                            _incoming_points.append((xyz.copy(), rgb))

                elif mtype == 0x4F:  # 'O' pose
                    vals = struct.unpack("<16f", payload)
                    with _lock:
                        _latest_pose[0] = vals
                        _path.append(np.array([vals[12], vals[13], vals[14]], dtype=np.float64))

                elif mtype == 0x49:  # 'I' jpeg
                    with _lock:
                        _latest_jpeg[0] = payload
        except (ConnectionResetError, OSError):
            pass
        finally:
            conn.close()
            print("[viewer] iPhone disconnected -- waiting for reconnect")
    srv.close()


def pose_matrix(vals):
    """16 column-major floats -> 4x4 camera-to-world matrix."""
    return np.array(vals, dtype=np.float64).reshape(4, 4).T


def main():
    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures", session)
    os.makedirs(outdir, exist_ok=True)
    log_path = os.path.join(outdir, "points_log.f32")
    ply_path = os.path.join(outdir, "map.ply")
    log_file = open(log_path, "ab")
    print(f"[viewer] recording to {outdir}")

    threading.Thread(target=receiver, daemon=True).start()

    nav_grid_pcd = o3d.geometry.PointCloud()
    nav_path_lines = o3d.geometry.LineSet()
    nav_grid_added = False
    nav_path_added = False
    preview_on = False
    last_preview = 0.0

    # Full-resolution store (kept for disk saves; never downsampled).
    store_xyz = np.zeros((0, 3), np.float32)
    store_rgb = np.zeros((0, 3), np.float32)

    pcd = o3d.geometry.PointCloud()
    traj = o3d.geometry.PointCloud()
    marker = o3d.geometry.TriangleMesh.create_coordinate_frame(size=MARKER_SIZE)
    marker_base = np.asarray(marker.vertices).copy()

    vis = o3d.visualization.Visualizer()
    vis.create_window("Live Map -- PointCloudScanner", width=1100, height=800)
    opt = vis.get_render_option()
    opt.point_size = 2.5
    opt.background_color = np.array([0.05, 0.05, 0.07])

    pcd_added = traj_added = marker_added = False
    last_render_build = 0.0
    last_autosave = time.time()

    cv2.namedWindow("Robot camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot camera", 480, 360)

    def save_ply():
        if store_xyz.shape[0] == 0:
            return
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(store_xyz.astype(np.float64))
        cloud.colors = o3d.utility.Vector3dVector(store_rgb.astype(np.float64))
        o3d.io.write_point_cloud(ply_path, cloud)
        print(f"[viewer] saved {store_xyz.shape[0]} points -> {ply_path}")

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

    try:
        while True:
            with _lock:
                batch = _incoming_points[:]
                _incoming_points.clear()
                path_copy = list(_path)
                pose_vals = _latest_pose[0]
                jpeg = _latest_jpeg[0]

            # --- ingest + store new points ---
            if batch:
                add_xyz = np.vstack([b[0] for b in batch]).astype(np.float32)
                add_rgb = np.vstack([b[1] for b in batch]).astype(np.float32)
                store_xyz = np.vstack([store_xyz, add_xyz])
                store_rgb = np.vstack([store_rgb, add_rgb])
                # append to disk continuously (x,y,z,r,g,b float32)
                np.hstack([add_xyz, add_rgb]).astype("<f4").tofile(log_file)
                log_file.flush()

            # --- rebuild the displayed cloud (throttled if large) ---
            now = time.time()
            if batch and (store_xyz.shape[0] <= RENDER_MAX or now - last_render_build > RENDER_REBUILD_SEC):
                last_render_build = now
                if store_xyz.shape[0] > RENDER_MAX:
                    tmp = o3d.geometry.PointCloud()
                    tmp.points = o3d.utility.Vector3dVector(store_xyz.astype(np.float64))
                    tmp.colors = o3d.utility.Vector3dVector(store_rgb.astype(np.float64))
                    tmp = tmp.voxel_down_sample(voxel_size=0.02)
                    pcd.points, pcd.colors = tmp.points, tmp.colors
                else:
                    pcd.points = o3d.utility.Vector3dVector(store_xyz.astype(np.float64))
                    pcd.colors = o3d.utility.Vector3dVector(store_rgb.astype(np.float64))
                if not pcd_added:
                    vis.add_geometry(pcd)
                    vis.reset_view_point(True)
                    pcd_added = True
                else:
                    vis.update_geometry(pcd)

            # --- path trail ---
            if path_copy:
                tp = np.array(path_copy, dtype=np.float64)
                traj.points = o3d.utility.Vector3dVector(tp)
                traj.colors = o3d.utility.Vector3dVector(np.tile([1.0, 0.25, 0.1], (len(tp), 1)))
                if not traj_added:
                    vis.add_geometry(traj)
                    traj_added = True
                else:
                    vis.update_geometry(traj)

            # --- moving phone-pose marker (position + orientation) ---
            if pose_vals is not None:
                M = pose_matrix(pose_vals)
                v = (M[:3, :3] @ marker_base.T).T + M[:3, 3]
                marker.vertices = o3d.utility.Vector3dVector(v)
                if not marker_added:
                    vis.add_geometry(marker)
                    marker_added = True
                else:
                    vis.update_geometry(marker)

            # --- walkthrough map preview (toggle with 'G') ---
            if preview_on and pose_vals is not None and time.time() - last_preview > 2.0:
                last_preview = time.time()
                res = preview_plan(store_xyz, pose_to_2d(pose_from_streamed(pose_vals)))
                update_nav_overlay(res.grid, res.floor_y, res.path_world)

            if not vis.poll_events():
                break
            vis.update_renderer()

            # --- camera feed ---
            if jpeg is not None:
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    if IMAGE_ROTATION is not None:
                        img = cv2.rotate(img, IMAGE_ROTATION)
                    cv2.imshow("Robot camera", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save_ply()
            elif key == ord('g'):
                preview_on = not preview_on
                print(f"[viewer] map preview {'ON' if preview_on else 'OFF'}")
            elif key == 27:  # ESC
                break

            # --- periodic autosave ---
            if now - last_autosave > AUTOSAVE_SECONDS:
                last_autosave = now
                save_ply()
    finally:
        _running[0] = False
        save_ply()
        log_file.close()
        vis.destroy_window()
        cv2.destroyAllWindows()
        print(f"[viewer] done. Full map in {outdir}")


if __name__ == "__main__":
    main()
