from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node_path = Path(__file__).resolve().parents[1] / "nodes" / "bridge_node.py"
    return LaunchDescription([
        Node(
            package="autonomous_rc_car",
            executable="python",
            name="bridge_node",
            arguments=[str(node_path)],
        )
    ])
