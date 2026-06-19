import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'patrol_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'patrol_node      = patrol_robot.patrol_node:main',
            'checker_node     = patrol_robot.checker_node:main',
            'visualizer_node  = patrol_robot.visualizer_node:main',
            'initial_pose_node = patrol_robot.initial_pose_node:main',
        ],
    },
)
