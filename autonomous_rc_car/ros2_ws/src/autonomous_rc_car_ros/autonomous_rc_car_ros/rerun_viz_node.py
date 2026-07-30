"""rerun_viz_node: stream the ROS topics to a Rerun 3D viewer.

A tailored alternative to RViz2 (see autonomous_rc_car/VISUALIZER.md). Subscribes to
the graph's topics and logs each as its own toggleable Rerun entity: the point cloud,
ground/obstacle voxel cubes, the 2D occupancy map, the planned path, and the phone
pose. The Rerun viewer (separate process) provides toggling + camera, so this node is
just rr.log() calls in callbacks -- no GUI threading.

Install once:  pip install rerun-sdk
Run:           ros2 run autonomous_rc_car_ros rerun_viz_node
View on Windows: run `rerun` there, then  ... -p connect_addr:=<windows-ip>:9876
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray

try:
    import rerun as rr
except ImportError:
    rr = None

GROUND = (40, 190, 60)
OBSTACLE = (230, 40, 40)


class RerunVizNode(Node):
    def __init__(self):
        super().__init__('rerun_viz_node')
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(PointCloud2, '/points', self._on_cloud, 10)
        self.create_subscription(
            MarkerArray, '/voxels/ground',
            lambda m: self._on_voxels(m, 'world/voxels/ground', GROUND), 1)
        self.create_subscription(
            MarkerArray, '/voxels/obstacle',
            lambda m: self._on_voxels(m, 'world/voxels/obstacle', OBSTACLE), 1)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(Path, '/cmd_path', self._on_path, latched)
        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)

        self.get_logger().info('rerun_viz_node up: streaming ROS topics to Rerun')

    def _on_cloud(self, msg: PointCloud2):
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        if xyz.shape[0]:
            rr.log('world/cloud', rr.Points3D(xyz, radii=0.006, colors=(200, 200, 200)))

    def _on_voxels(self, msg: MarkerArray, path, color):
        centers = np.zeros((0, 3), dtype=np.float32)
        half = 0.015
        if msg.markers and msg.markers[0].points:
            m = msg.markers[0]
            centers = np.array([[p.x, p.y, p.z] for p in m.points], dtype=np.float32)
            half = (m.scale.x / 2.0) if m.scale.x else half
        if centers.shape[0]:
            rr.log(path, rr.Boxes3D(centers=centers,
                                    half_sizes=np.full_like(centers, half),
                                    colors=color))
        else:
            rr.log(path, rr.Clear(recursive=False))

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        data = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        ys, xs = np.nonzero(data != -1)
        if xs.size == 0:
            rr.log('world/map', rr.Clear(recursive=False))
            return
        wx = info.origin.position.x + (xs + 0.5) * info.resolution
        wy = info.origin.position.y + (ys + 0.5) * info.resolution
        wz = np.full(xs.shape, info.origin.position.z, dtype=np.float32)
        pts = np.stack([wx, wy, wz], axis=1).astype(np.float32)
        vals = data[ys, xs]
        colors = np.zeros((xs.size, 3), dtype=np.uint8)
        colors[vals == 0] = (40, 120, 50)      # free
        colors[vals == 99] = (200, 160, 30)    # inflation buffer
        colors[vals == 100] = (200, 40, 40)    # occupied
        rr.log('world/map', rr.Points3D(pts, radii=info.resolution * 0.4, colors=colors))

    def _on_path(self, msg: Path):
        pts = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                        for p in msg.poses], dtype=np.float32)
        if pts.shape[0] >= 2:
            rr.log('world/path', rr.LineStrips3D([pts], colors=(60, 130, 255), radii=0.012))
        else:
            rr.log('world/path', rr.Clear(recursive=False))

    def _on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        rr.log('world/phone', rr.Points3D([[p.x, p.y, p.z]], radii=0.05, colors=(255, 220, 0)))
        rr.log('world/phone/axes', rr.Transform3D(
            translation=[p.x, p.y, p.z],
            rotation=rr.Quaternion(xyzw=[o.x, o.y, o.z, o.w]),
            axis_length=0.3))


def _connect_viewer(addr):
    """Connect to an already-running Rerun viewer, tolerant of API changes across
    rerun-sdk versions (connect_grpc in newer, connect_tcp / connect in older)."""
    if hasattr(rr, 'connect_grpc'):
        url = addr if addr.startswith('rerun+') else f'rerun+http://{addr}/proxy'
        rr.connect_grpc(url)
    elif hasattr(rr, 'connect_tcp'):
        rr.connect_tcp(addr)
    else:
        rr.connect(addr)


def main(args=None):
    if rr is None:
        print('rerun-sdk not installed. Run:  pip install rerun-sdk')
        return
    rclpy.init(args=args)
    node = RerunVizNode()
    connect = node.declare_parameter('connect_addr', '').get_parameter_value().string_value
    rr.init('rc_car_viz')
    if connect:
        _connect_viewer(connect)          # viewer running elsewhere (e.g. Windows)
    else:
        rr.spawn()                        # open the viewer locally (WSLg)
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
