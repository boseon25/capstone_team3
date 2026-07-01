#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 — 4노드 일괄 기동 launch (+ 선택적 SLAM/Nav2 포함)
#
#  사용 (시뮬 단독, B/C/Gazebo 불필요):
#     ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
#
#  사용 (실연동 통합 — Gazebo는 터미널1에서 먼저 기동):
#     # 터미널1: ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
#     # 터미널2(이 런처가 SLAM+Nav2+4노드를 한 번에):
#     ros2 launch nd1_capstone bringup.launch.py \
#         sim_mode:=false slam:=true nav2:=true
#
#  사전 맵을 쓰는 경우(SLAM 대신 AMCL 로컬라이제이션):
#     ros2 launch nd1_capstone bringup.launch.py \
#         sim_mode:=false localization:=true map:=/경로/warehouse.yaml nav2:=true
#     → RViz "2D Pose Estimate"로 초기 위치를 찍어야 map→odom 발행됨.
#
#  ⚠️ 핵심: global_costmap이 base_link→map 변환을 얻으려면 map→odom 공급원이
#     반드시 떠 있어야 함. 그 공급원이 SLAM(slam:=true) 또는 AMCL(localization:=true).
#     둘 다 없으면 'Invalid frame ID "map"' 타임아웃이 무한 반복됨.
# ════════════════════════════════════════════════════════════════
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("sim_mode")
    use_slam = LaunchConfiguration("slam")
    use_loc = LaunchConfiguration("localization")
    use_nav2 = LaunchConfiguration("nav2")
    map_yaml = LaunchConfiguration("map")

    params = [{"sim_mode": sim}]
    tb4_nav = FindPackageShare("turtlebot4_navigation")

    # ── 인자 선언 ────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("sim_mode", default_value="true",
                              description="true=B/C 시뮬, false=실연동"),
        DeclareLaunchArgument("slam", default_value="false",
                              description="true=SLAM 기동(map→odom 공급, 사전맵 불필요)"),
        DeclareLaunchArgument("localization", default_value="false",
                              description="true=AMCL 로컬라이제이션(사전맵 필요, map 인자 함께)"),
        DeclareLaunchArgument("nav2", default_value="false",
                              description="true=Nav2 기동(navigate_to_pose 액션 서버)"),
        DeclareLaunchArgument("map", default_value="",
                              description="localization:=true 일 때 사용할 맵 yaml 경로"),
        DeclareLaunchArgument("world_name", default_value="warehouse",
                              description="Node C 텔레포트 대상 Ignition 월드 이름"),
        DeclareLaunchArgument("box_model", default_value="box1",
                              description="Node C 텔레포트 대상 박스 모델 이름"),
        DeclareLaunchArgument("box_sdf_path", default_value="",
                              description="배치(place) 재생성용 SDF 절대경로(비우면 리소스경로 box1.sdf)"),
        DeclareLaunchArgument("auto_redock", default_value="true",
                              description="미션 완료 후 자동 재도킹(멀티미션이면 false 권장)"),
    ]

    # ── map→odom 공급원 (둘 중 택1) + Nav2 (조건부 포함) ──────────
    includes = [
        # SLAM: 사전 맵 없이 map→odom 발행 (시뮬 시연에 가장 간단)
        # use_sim_time=true 필수 — 안 넘기면 기본값 false로 돌아서 controller_server가
        # Gazebo 시뮬 시각과 실제 시각을 혼동해 "Transform too old" 후 즉시 성공 처리해버림.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [tb4_nav, "launch", "slam.launch.py"])),
            condition=IfCondition(use_slam),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
        # Localization: 사전 맵 + AMCL (RViz 2D Pose Estimate로 초기화 필요)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [tb4_nav, "launch", "localization.launch.py"])),
            condition=IfCondition(use_loc),
            launch_arguments={"map": map_yaml, "use_sim_time": "true"}.items(),
        ),
        # Nav2: 경로계획/제어/코스트맵 (navigate_to_pose 액션 서버)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [tb4_nav, "launch", "nav2.launch.py"])),
            condition=IfCondition(use_nav2),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ]

    # ── 캡스톤 4노드 (항상 기동) ─────────────────────────────────
    nodes = [
        Node(package="nd1_capstone", executable="node_a_llm", name="node_a_llm",
             output="screen"),
        Node(package="nd1_capstone", executable="node_b_nav", name="node_b_nav",
             output="screen", parameters=params),
        Node(package="nd1_capstone", executable="node_c_grasp", name="node_c_grasp",
             output="screen", parameters=[{
                 "sim_mode": sim,
                 "world_name": LaunchConfiguration("world_name"),
                 "box_model": LaunchConfiguration("box_model"),
                 "box_sdf_path": LaunchConfiguration("box_sdf_path"),
             }]),
        Node(package="nd1_capstone", executable="coordinator_fsm", name="coordinator_fsm",
             output="screen", parameters=[{
                 "sim_mode": sim,
                 "auto_redock": LaunchConfiguration("auto_redock"),
             }]),
    ]

    return LaunchDescription(args + includes + nodes)
