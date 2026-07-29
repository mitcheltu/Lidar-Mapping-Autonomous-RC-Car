# ROS2 Setup — WSL2 + Ubuntu 22.04 + ROS2 Humble

The laptop_brain runs its ROS2 graph in **WSL2** (ROS2 is painful natively on
Windows). The iPhone app and ESP32 are unchanged; only the laptop's ROS2 host
lives in WSL2. Decided 2026-07-27 (`PROJECT_STATUS.md` §3/§7.2).

## 1. Install WSL2 + Ubuntu 22.04

In an **admin PowerShell** (Windows 11):

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if prompted, set your Ubuntu username/password. ROS2 Humble targets
Ubuntu 22.04 (Jammy) — use that image, not 24.04.

## 2. Install ROS2 Humble (inside Ubuntu)

Follow the official Humble deb install (locale → add ROS2 apt repo → install):

```bash
sudo apt update && sudo apt install -y locales curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions \
  ros-humble-sensor-msgs-py ros-humble-nav-msgs ros-humble-geometry-msgs
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 3. Get the repo + the `nav` library into the ROS2 env

The Windows repo is reachable from WSL2 under `/mnt/c/...`. You can work in place
or clone into the Linux filesystem (faster). Then install the `nav` library so the
ROS2 nodes can `import nav`:

```bash
# Ubuntu 22.04 ships setuptools 59 (pre-PEP 660) -> `pip install -e` fails.
# Upgrade first so editable installs work:
pip3 install --upgrade pip setuptools wheel

# open3d/scipy/numpy on Python 3.10 (Ubuntu 22.04 default)
cd "/mnt/c/Users/mitch/OneDrive/Documents/Claude/Projects/Remote Car/autonomous_rc_car"
pip3 install -e ./laptop_brain          # installs the rc-car-nav library (nav.*)
# if editable still errors: pip3 install --no-build-isolation -e ./laptop_brain
pip3 install rerun-sdk                   # optional: custom Rerun viewer (see VISUALIZER.md)
```

> If `open3d` is heavy/slow to import in the mapping node, that's expected; it is a
> real dependency of `nav.mapping`.

## 4. Build + run the ROS2 package

```bash
cd autonomous_rc_car/ros2_ws
colcon build --packages-select autonomous_rc_car_ros
source install/setup.bash
ros2 launch autonomous_rc_car_ros bringup.launch.py
```

Currently only `bridge_node` is implemented; the other four (`voxel_mapper_node`,
`frontier_planner_node`, `motion_controller_node`, `icp_slam_node`) are stubs that
log a TODO. Fill them in per `TODO.md` Phase 1/2/3.

## 5. ⚠️ WSL2 networking — let the phone reach `bridge_node`

`bridge_node` runs a TCP server on **port 9000**. By default **WSL2 is NAT'd** —
it has its own IP (e.g. `172.x.x.x`), so a phone on your Wi-Fi LAN **cannot** reach
it directly. Two fixes:

**Option A — Windows 11 mirrored networking (simplest).** In
`C:\Users\<you>\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
Then `wsl --shutdown` and reopen. WSL2 now shares the Windows host's LAN IP, so
the phone streams to the laptop's normal `192.168.x.x` and it reaches `bridge_node`.

**Option B — port proxy (older Windows).** Forward host:9000 → WSL2:9000 in an
admin PowerShell (re-run after each WSL restart; the WSL IP changes):
```powershell
$wsl = (wsl hostname -I).Trim().Split()[0]
netsh interface portproxy add v4tov4 listenport=9000 listenaddress=0.0.0.0 connectport=9000 connectaddress=$wsl
New-NetFirewallRule -DisplayName "WSL ROS2 9000" -Direction Inbound -LocalPort 9000 -Protocol TCP -Action Allow
```
In the iPhone app, enter the **Windows** LAN IP (from `ipconfig`), not the WSL IP.

## 6. Sanity checks

```bash
ros2 topic list                       # /points /pose /image after the phone connects
ros2 topic echo /pose --once          # confirm pose frames arrive
ros2 topic hz /points                 # rate while the phone streams during a scan
```
