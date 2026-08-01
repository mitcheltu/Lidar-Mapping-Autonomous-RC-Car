"""Bringup launch: the full autonomous_rc_car_ros graph + the Rerun visualizer.

Starts all six background nodes. The seventh, ``motion_enable_node``, is NOT
here on purpose: it puts the terminal in raw mode (``tty.setraw``) to read
keystrokes, and a launched node gets a pipe rather than a TTY. Run it in its own
terminal, or use ``autonomous_rc_car/run.sh``, which starts this launch file and
then hands the console the real terminal.

    ros2 launch autonomous_rc_car_ros bringup.launch.py
    ros2 launch autonomous_rc_car_ros bringup.launch.py connect_addr:=172.20.1.1:9876
    ros2 launch autonomous_rc_car_ros bringup.launch.py viz:=false

Arguments:
    viz            include rerun_viz_node                     (default true)
    connect_addr   Rerun viewer to connect to; empty = spawn
                   the viewer locally via WSLg                (default '')
    continuous     frontier_planner replans on every /map
                   instead of only on /plan_trigger           (default false)
    start_enabled  motion_controller armed at boot rather
                   than starting in HOLD                      (default false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    connect_addr = LaunchConfiguration('connect_addr')
    continuous = LaunchConfiguration('continuous')
    start_enabled = LaunchConfiguration('start_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'viz', default_value='true',
            description='Run rerun_viz_node alongside the graph.'),
        DeclareLaunchArgument(
            'connect_addr', default_value='',
            description='host:port of a running Rerun viewer. '
                        'Empty spawns the viewer locally (WSLg).'),
        DeclareLaunchArgument(
            'continuous', default_value='false',
            description='Replan on every /map instead of on demand (p key).'),
        DeclareLaunchArgument(
            'start_enabled', default_value='false',
            description='Arm driving at boot. Default is HOLD -- leave false '
                        'unless you mean it.'),

        Node(
            package='autonomous_rc_car_ros',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
        ),
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
            parameters=[{
                'continuous': ParameterValue(continuous, value_type=bool),
            }],
        ),
        Node(
            package='autonomous_rc_car_ros',
            executable='motion_controller_node',
            name='motion_controller_node',
            output='screen',
            parameters=[{
                'start_enabled': ParameterValue(start_enabled, value_type=bool),
            }],
        ),
        Node(
            package='autonomous_rc_car_ros',
            executable='icp_slam_node',
            name='icp_slam_node',
            output='screen',
        ),
        Node(
            package='autonomous_rc_car_ros',
            executable='rerun_viz_node',
            name='rerun_viz_node',
            output='screen',
            condition=IfCondition(viz),
            parameters=[{
                'connect_addr': ParameterValue(connect_addr, value_type=str),
            }],
        ),
    ])
