from glob import glob
import os

from setuptools import setup

package_name = 'autonomous_rc_car_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitcheltu',
    maintainer_email='tumk@uci.edu',
    description='ROS2 Humble bringup for the autonomous RC car '
                '(iPhone stream bridge, mapping, planning, control, SLAM).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = autonomous_rc_car_ros.bridge_node:main',
            'voxel_mapper_node = autonomous_rc_car_ros.voxel_mapper_node:main',
            'frontier_planner_node = '
            'autonomous_rc_car_ros.frontier_planner_node:main',
            'motion_controller_node = '
            'autonomous_rc_car_ros.motion_controller_node:main',
            'icp_slam_node = autonomous_rc_car_ros.icp_slam_node:main',
            'motion_enable_node = autonomous_rc_car_ros.motion_enable_node:main',
            'rerun_viz_node = autonomous_rc_car_ros.rerun_viz_node:main',
        ],
    },
)
