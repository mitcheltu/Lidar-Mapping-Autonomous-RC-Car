"""Frontier planner node (STUB).

Intended behavior:
    Subscribes:
        /map       (nav_msgs/OccupancyGrid) -- inflated occupancy grid
    Publishes:
        /cmd_path  (nav_msgs/Path)          -- planned path to the goal

Wraps ``nav.frontier.choose_goal`` to pick the next exploration goal cell,
then ``nav.planner.astar`` over the grid's passable mask and
``nav.planner.simplify_path`` to produce a compact waypoint path, converted
into nav_msgs/Path in world coordinates.

NOTE: ROS2 Humble / build + run in WSL2. Not yet implemented.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path

from nav import frontier, planner  # noqa: F401  (used once implemented)


class FrontierPlannerNode(Node):
    """Chooses a frontier goal and plans an A* path to it (stub)."""

    def __init__(self):
        super().__init__('frontier_planner_node')
        self.get_logger().warn('TODO: frontier_planner_node not yet implemented')

        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 1
        )
        self._path_pub = self.create_publisher(Path, '/cmd_path', 1)

    def _on_map(self, msg):
        # TODO: choose_goal -> astar -> simplify_path -> publish /cmd_path.
        pass


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
