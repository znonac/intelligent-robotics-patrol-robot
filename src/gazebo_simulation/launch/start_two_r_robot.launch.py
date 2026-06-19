"""
Two-R 로봇 Gazebo Harmonic 시뮬레이션 런치 파일 (ROS2 Jazzy)

빈 Gazebo 월드에 Two-R 로봇을 스폰하고 joint_state_publisher_gui로 관절 제어.
※ Two-R URDF는 시각화(visual)만 정의되어 있으므로 물리 시뮬레이션은 지원되지 않음.

실행:
  ros2 launch gazebo_simulation start_two_r_robot.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_r_robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([
        # Gazebo Harmonic 서버 (빈 월드)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-r -s -v2 empty.sdf',
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # Gazebo GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-g -v2',
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Two-R 로봇 스폰 (/robot_description 토픽에서 읽음)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'two_r_robot',
                '-topic', '/robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.0',
            ],
            output='screen',
        ),

        # 관절 GUI 슬라이더
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    ])
