#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  박스 스폰 launch — 실행 중인 TurtleBot3 turtlebot3_world에 박스 배치
#  사용:
#    ros2 launch nd1_capstone spawn_boxes.launch.py
#  ※ Classic Gazebo는 프로세스당 월드 1개(월드명 구분 불필요) — gazebo_ros의
#    spawn_entity.py 스크립트가 /spawn_entity 서비스로 스폰.
# ════════════════════════════════════════════════════════════════
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("nd1_capstone")
    sdf = os.path.join(pkg, "models", "box1.sdf")

    # 기본은 A구역(-2.0, 1.2)에 box1 1개. 상 난이도(다중)면 box_B/C 주석 해제.
    spawn_box1 = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-entity", "box1", "-file", sdf,
                   "-x", "-2.0", "-y", "1.2", "-z", "0.1"])

    # spawn_box_B = Node(
    #     package="gazebo_ros", executable="spawn_entity.py", output="screen",
    #     arguments=["-entity", "box_B", "-file", sdf,
    #                "-x", "-2.7", "-y", "-1.8", "-z", "0.1"])

    return LaunchDescription([
        spawn_box1,
        # spawn_box_B,
    ])
