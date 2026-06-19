"""
강의실 Gazebo Harmonic 시뮬레이션 런치 파일 (ROS2 Jazzy)

실행 순서:
  1. 이 파일로 Gazebo 강의실 + TurtleBot3 Waffle 실행
     export TURTLEBOT3_MODEL=waffle
     ros2 launch gazebo_simulation classroom.launch.py

  2. Nav2 실행
     ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true \
       map:=$(ros2 pkg prefix gazebo_simulation)/share/gazebo_simulation/maps/classroom.yaml

  3. 출석 관리 시스템 실행
     ros2 launch patrol_robot patrol_system.launch.py

  4. 순찰 시작
     ros2 service call /start_patrol std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'classroom.world')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    tb3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([
        # TurtleBot3 모델 리소스 경로 추가
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'models')
        ),

        # Gazebo Harmonic 서버 (헤드리스 — 센서·물리 시뮬레이션만, GUI 없음)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-s -r -v2 {world_path}',   # -s: GUI 없는 서버 전용 모드
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # robot_state_publisher (TurtleBot3 Waffle URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # TurtleBot3 Waffle 스폰 + ros_gz_bridge + 카메라 브리지
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '-4.5',
                'y_pose': '0.0',
            }.items()
        ),
    ])
