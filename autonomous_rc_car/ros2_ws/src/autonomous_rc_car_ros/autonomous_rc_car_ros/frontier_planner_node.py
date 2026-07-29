"""frontier_planner_node: /map + /pose -> /cmd_path.

On each occupancy-grid update, rebuild a nav ``OccupancyGrid`` from the ROS
message, locate the car cell from the latest pose, pick the nearest reachable
frontier (``nav.frontier.choose_goal``), plan an A* path over the passable mask
(``nav.planner.astar``), simplify it to a few line-of-sight waypoints
(``nav.planner.simplify_path``), and publish it as a Z-up nav_msgs/Path on
``/cmd_path``. An empty path means no reachable frontier (exploration complete).

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Empty, Header

from nav.frames import points_arkit_to_ros, points_ros_to_arkit
from nav.frontier import choose_goal, nearest_passable
from nav.planner import astar, simplify_path
from nav.ros_export import occupancy_to_grid


class FrontierPlannerNode(Node):
    """Plans ON DEMAND: stores the latest /map and /pose, and computes one path
    each time /plan_trigger fires. /cmd_path is latched so RViz shows it steadily."""

    def __init__(self):
        super().__init__('frontier_planner_node')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('continuous', False)   # True = replan on every /map
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._continuous = self.get_parameter('continuous').get_parameter_value().bool_value

        self._pose = None    # latest ROS pose position (x, y, z)
        self._map = None     # latest OccupancyGrid message

        # Latched so a plan computed once stays visible to RViz / the controller.
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, '/cmd_path', latched)

        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(Empty, '/plan_trigger', lambda _: self._plan(), 10)

        self.get_logger().info(
            'frontier_planner_node up: publish to /plan_trigger to compute a path '
            f'({"continuous" if self._continuous else "on-demand"} mode)')

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self._pose = (p.x, p.y, p.z)

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg
        if self._continuous:
            self._plan()

    def _plan(self):
        if self._map is None:
            self.get_logger().warn('no /map yet -- cannot plan')
            return
        if self._pose is None:
            self.get_logger().warn('no /pose yet -- cannot locate the car. Is the phone streaming?')
            return
        info = self._map.info
        msg = self._map
        grid = occupancy_to_grid(info.width, info.height, info.resolution,
                                 info.origin.position.x, info.origin.position.y, msg.data)
        floor_y = info.origin.position.z

        # ROS pose -> ARKit world; nav grid indexes (world x, world z).
        world = points_ros_to_arkit(np.array([self._pose], dtype=np.float32))[0]
        car_cell = grid.world_to_cell(float(world[0]), float(world[2]))

        goal = choose_goal(grid, car_cell)
        if goal is None:
            self._publish_path([], floor_y)
            return
        start = nearest_passable(grid, car_cell)
        if start is None:
            return
        cells = astar(grid.passable(), start, goal)
        if not cells:
            return
        cells = simplify_path(cells, grid.passable())
        world_pts = [grid.cell_to_world(r, c) for (r, c) in cells]  # [(x, z), ...]
        self._publish_path(world_pts, floor_y)

    def _publish_path(self, world_pts, floor_y):
        path = Path()
        path.header = self._header()
        for wx, wz in world_pts:
            ros = points_arkit_to_ros(np.array([[wx, floor_y, wz]], dtype=np.float32))[0]
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(ros[0])
            ps.pose.position.y = float(ros[1])
            ps.pose.position.z = float(ros[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)
        if world_pts:
            self.get_logger().info(f'planned path: {len(world_pts)} waypoints')
        else:
            self.get_logger().info('no reachable frontier -- exploration may be complete')

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self._frame_id
        return h


def main(args=None):
    rclpy.init(args=args)
    node = FrontierPlannerNode()
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
