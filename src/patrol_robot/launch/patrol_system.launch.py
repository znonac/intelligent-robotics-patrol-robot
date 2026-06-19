"""
출석 관리 시스템 런치 파일

실행 순서:
  1. Gazebo 강의실 실행:
     export TURTLEBOT3_MODEL=waffle
     ros2 launch gazebo_simulation classroom.launch.py
  2. Nav2 실행:
     ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true \\
       map:=$(ros2 pkg prefix gazebo_simulation)/share/gazebo_simulation/maps/classroom.yaml
  3. 이 파일 실행:
     ros2 launch patrol_robot patrol_system.launch.py
  4. 출석 등록 노드는 별도 터미널에서 실행 (입력 안정성을 위해 분리):
     ros2 run attendance_robot attendance_node
  5. 순찰 시작:
     ros2 service call /start_patrol std_srvs/srv/Trigger
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('patrol_robot'),
        'rviz', 'patrol_system.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('max_rounds',             default_value='3'),
        DeclareLaunchArgument('patrol_interval_sec',    default_value='300.0'),
        DeclareLaunchArgument('detection_window_sec',   default_value='3.0'),
        DeclareLaunchArgument('use_rviz',               default_value='true'),
        DeclareLaunchArgument('initial_pose_delay_sec', default_value='5.0'),

        # AMCL 초기 위치 자동 발행 (Nav2가 준비될 때까지 5초 대기)
        Node(
            package='patrol_robot',
            executable='initial_pose_node',
            name='initial_pose_node',
            output='screen',
            parameters=[{
                'x': -4.5,
                'y': 0.0,
                'delay_sec': LaunchConfiguration('initial_pose_delay_sec'),
                'use_sim_time': True,
            }],
        ),

        # 자율 순찰 노드 (Nav2 + 판정 + CSV)
        Node(
            package='patrol_robot',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[{
                'max_rounds': LaunchConfiguration('max_rounds'),
                'patrol_interval_sec': LaunchConfiguration('patrol_interval_sec'),
                'use_sim_time': True,
            }],
        ),

        # YOLOv8 착석 감지 노드
        Node(
            package='patrol_robot',
            executable='checker_node',
            name='checker_node',
            output='screen',
            parameters=[{
                'detection_window_sec': LaunchConfiguration('detection_window_sec'),
                'use_sim_time': True,
            }],
        ),

        # 출석 상태 시각화 노드 (RViz2 MarkerArray)
        Node(
            package='patrol_robot',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # RViz2 (지도 + 경로 + 출석 마커 통합 뷰)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])