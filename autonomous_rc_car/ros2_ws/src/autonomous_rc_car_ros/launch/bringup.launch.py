"""Bringup launch: bridge_node (real) + the four stub nodes.

Starts the whole autonomous_rc_car_ros graph. Only ``bridge_node`` is a real
implementation today; the other four are stubs that warn and idle. Comment out
any of the stub Node() entries below to run a leaner graph.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_rc_car_ros',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
        ),
        # --- Stub nodes (included by default; safe to comment out) ---
        Node(
            package='autonomous_rc_car_ros',
            executable='voxel_mapper_node',
            name='voxel_mapper_node',
            output='screen',
        ),
        Node(
            package='autonomous_rc_car_ros',
            executable='frontier_planner_node',
            name='frontier_planner_node',
            output='screen',
        ),
        Node(
            package='autonomous_rc_car_ros',
            executable='motion_controller_node',
            name='motion_controller_node',
            output='screen',
        ),
        Node(
            package='autonomous_rc_car_ros',
            executable='icp_slam_node',
            name='icp_slam_node',
            output='screen',
        ),
    ])
