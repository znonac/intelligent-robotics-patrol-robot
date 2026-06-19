import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'),   glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user_znonac',
    maintainer_email='znonac@ewha.ac.kr',
    description='Gazebo Classic 강의실 시뮬레이션 (ROS2 Humble)',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
