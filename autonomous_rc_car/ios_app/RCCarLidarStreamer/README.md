# RCCarLidarStreamer (iPhone 12 Pro Max LiDAR)

A SwiftUI + ARKit app that builds a colored 3D point cloud from your phone's
LiDAR as you wave it around, shows it live over the camera feed, and exports a
`.PLY` file you can open in MeshLab, CloudCompare, or Blender.

It also ships with `CarController.swift`, a BLE bridge to the ESP32 robot (see
`../../esp32_firmware/src/esp32_car.ino`), so this one project covers both perception and,
later, driving.

## Requirements

- A LiDAR-equipped iPhone/iPad. The **iPhone 12 Pro Max** qualifies.
- Xcode 15 or newer, iOS 16+ deployment target.
- A **physical device** — ARKit scene depth does not run in the Simulator.

## Create the Xcode project (5 minutes)

1. Xcode → **File ▸ New ▸ Project… ▸ App**. Product name `RCCarLidarStreamer`,
   Interface **SwiftUI**, Language **Swift**.
2. Delete the auto-generated `ContentView.swift` and `*App.swift`, then drag
   all 7 source files into the project (check "Copy items if needed"):
   - `PointCloudScannerApp.swift`  *(the `@main` entry point)*
   - `ContentView.swift`
   - `ARDepthView.swift`
   - `PointCloudStreamer.swift`
   - `PointCloudAccumulator.swift`
   - `PLYExporter.swift`
   - `CarController.swift`
3. Add these keys to **Info** (Target ▸ Info tab):
   - `NSCameraUsageDescription` → "Used to scan your surroundings with LiDAR."
   - `NSLocalNetworkUsageDescription` → "Used to stream the scan to the computer."
     **(required — without it iOS blocks the Wi-Fi stream to the laptop.)**
   - `NSBluetoothAlwaysUsageDescription` → "Used to connect to the robot." *(only
     needed once you start using `CarController`.)*
4. Select your iPhone as the run target, set your Team under **Signing &
   Capabilities**, and press ▶.

## Using it

- **Move slowly** and keep surfaces 0.3–5 m away (LiDAR range). Sweep the phone
  across the room; points accumulate in real time.
- **Pause** freezes accumulation; **Clear** empties the cloud; **Export PLY**
  writes a file and opens the share sheet (AirDrop to a Mac, or Save to Files).

## How it works

`ARDepthView` runs an `ARWorldTrackingConfiguration` with `.sceneDepth` /
`.smoothedSceneDepth`. For every few frames it locks the depth map, unprojects
each depth pixel into world space using the camera intrinsics and pose, samples
color from the captured image, and adds the point to a **voxel grid**
(`PointCloudAccumulator`) that keeps one averaged point per ~1.5 cm cell. The
grid is rendered as SceneKit points a few times per second.

## Tuning

In `ARDepthView.Coordinator`:

- `voxelSize` (in `PointCloudAccumulator`) — smaller = denser/heavier.
- `pixelStride` — raise to 3–4 for speed, lower to 1 for density.
- `maxDepth` — LiDAR gets noisy past ~5 m.
- `minConfidence` — set to `2` to keep only high-confidence points.

## Live streaming to your computer

The app can stream the scan to a computer over Wi-Fi so you can watch the map
build in real time (and see the robot's camera) on a big screen — with **no
400k-point limit on the computer side**.

**On the computer** (Windows or Mac, same Wi-Fi network):

```
python -m pip install open3d numpy opencv-python
python autonomous_rc_car/laptop_brain/pc_viewer.py
```

Find the computer's LAN IP — Windows `ipconfig`, Mac `ipconfig getifaddr en0` —
e.g. `192.168.1.20`. On Windows, allow Python through the firewall (Private).

**In the app:** type that IP into the field at the top, tap **Stream**. The dot
turns green when connected. You'll get:

- a live 3D point cloud that keeps growing as the robot drives (Open3D window),
- the robot's path drawn in red,
- the robot's camera feed in a separate window.

What gets sent: only *new* points each frame (light on bandwidth), the camera
pose ~10×/sec, and a downscaled JPEG ~5×/sec. The phone still shows its own
(capped) preview; the computer holds the full map. If the camera window looks
sideways, set `IMAGE_ROTATION` near the top of `pc_viewer.py`.

Wire protocol (if you want to write your own receiver): TCP, each message is
`type(1 byte) + length(uint32 LE) + payload`, where type `P`=points
(`count u32`, then per point `x,y,z float32 + r,g,b u8`), `O`=pose (16 float32,
column-major 4×4), `I`=JPEG bytes.
