#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 — 4노드 일괄 기동 launch (+ 선택적 SLAM/Nav2 포함)
#
#  사용 (시뮬 단독, B/C/Gazebo 불필요):
#     ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
#
#  사용 (실연동 통합 — Gazebo는 터미널1에서 먼저 기동):
#     # 터미널1: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#     # 터미널2(이 런처가 SLAM+Nav2+4노드를 한 번에):
#     ros2 launch nd1_capstone bringup.launch.py \
#         sim_mode:=false slam:=true nav2:=true
#
#  사전 맵을 쓰는 경우(SLAM 대신 AMCL 로컬라이제이션):
#     ros2 launch nd1_capstone bringup.launch.py \
#         sim_mode:=false localization:=true map:=/경로/map.yaml nav2:=true
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration("sim_mode")
    use_slam = LaunchConfiguration("slam")
    use_loc = LaunchConfiguration("localization")
    use_nav2 = LaunchConfiguration("nav2")
    use_planner = LaunchConfiguration("use_planner")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    map_yaml = LaunchConfiguration("map")

    params = [{"sim_mode": sim}]

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
        DeclareLaunchArgument("box_model", default_value="box1",
                              description="Node C 텔레포트 대상 박스 모델 이름"),
        DeclareLaunchArgument("box_sdf_path", default_value="",
                              description="배치(place) 재생성용 SDF 절대경로(비우면 리소스경로 box1.sdf)"),
        DeclareLaunchArgument("auto_redock", default_value="true",
                              description="미션 완료 후 자동 재도킹(멀티미션이면 false 권장)"),
        DeclareLaunchArgument("use_planner", default_value="false",
                              description="true=LLM 멀티미션 플래너(상 난이도) 기동 — /goal_command 입력"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="true=RViz2 기동(DISPLAY 환경변수 필요)"),
        DeclareLaunchArgument("use_gazebo", default_value="false",
                              description="true=TurtleBot3 Gazebo Classic 월드 기동(sim_mode:=false와 함께 사용)"),
        DeclareLaunchArgument("tb3_world", default_value="turtlebot3_world",
                              description="use_gazebo:=true 시 기동할 월드 이름"),
    ]

    # turtlebot3_gazebo 경로 — FindPackageShare를 PathJoinSubstitution 안에 쓰면
    # PythonLaunchDescriptionSource가 경로를 '' 로 해석하는 버그가 있으므로
    # ros2 pkg prefix 결과를 빌드 시점에 Python 상수로 고정.
    # PathJoinSubstitution([FindPackageShare(...)]) 조합이 다중 launch include 시
    # 빈 문자열로 해석되는 ROS2 Humble launch 버그 → Python 상수로 경로 고정.
    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd
        _tb3_gz_world  = _gpsd("turtlebot3_gazebo")      + "/launch/turtlebot3_world.launch.py"
        _tb3_nav2      = _gpsd("turtlebot3_navigation2") + "/launch/navigation2.launch.py"
        _nav2_slam     = _gpsd("nav2_bringup")            + "/launch/slam_launch.py"
        _nav2_loc      = _gpsd("nav2_bringup")            + "/launch/localization_launch.py"
        _nav2_rviz_cfg    = _gpsd("nav2_bringup") + "/rviz/nav2_default_view.rviz"
        _nav2_params_yaml = _gpsd("nav2_bringup") + "/params/nav2_params.yaml"
    except Exception:
        _base = "/opt/ros/humble/share"
        _tb3_gz_world     = f"{_base}/turtlebot3_gazebo/launch/turtlebot3_world.launch.py"
        _tb3_nav2         = f"{_base}/turtlebot3_navigation2/launch/navigation2.launch.py"
        _nav2_slam        = f"{_base}/nav2_bringup/launch/slam_launch.py"
        _nav2_loc         = f"{_base}/nav2_bringup/launch/localization_launch.py"
        _nav2_rviz_cfg    = f"{_base}/nav2_bringup/rviz/nav2_default_view.rviz"
        _nav2_params_yaml = f"{_base}/nav2_bringup/params/nav2_params.yaml"

    # ── map→odom 공급원 (둘 중 택1) + Nav2 + Gazebo (조건부 포함) ──
    includes = [
        # Gazebo Classic + TurtleBot3 스폰 (use_gazebo:=true 시)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_tb3_gz_world),
            condition=IfCondition(use_gazebo),
        ),
        # SLAM: 사전 맵 없이 map→odom 발행
        # params_file 명시 필수: gzserver.launch.py가 params_file=''을 먼저 선언해
        # launch_configurations를 공유하는 Humble launch 시스템에서 빈 경로가 유지됨.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_nav2_slam),
            condition=IfCondition(use_slam),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": _nav2_params_yaml,
            }.items(),
        ),
        # Localization: 사전 맵 + AMCL (RViz 2D Pose Estimate로 초기화 필요)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_nav2_loc),
            condition=IfCondition(use_loc),
            launch_arguments={"map": map_yaml, "use_sim_time": "true"}.items(),
        ),
        # Nav2: TurtleBot3 전용 bringup (params 자동 포함, map 미전달 → SLAM 모드 호환)
        # use_composition 명시 필수: gzserver가 context에 lowercase 'true' 계열 값을 오염시켜
        # navigation_launch.py의 PythonExpression(['not ', use_composition]) 평가 시
        # 'not true' → NameError 발생. 'True'(대문자)로 고정해 Python eval이 올바르게 작동하게 함.
        # slam/use_composition 명시 필수:
        # - bringup_launch.py에 PythonExpression(['not ', slam]) 있는데,
        #   외부 context의 slam='true'(소문자)가 유입되면 eval('not true') → NameError.
        #   nav2 bringup 안에서 SLAM을 별도 기동하지 않도록 slam='False'로 고정.
        # - use_composition='True'(대문자): navigation_launch.py PythonExpression이
        #   eval('not true') 실패하는 것을 방지.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_tb3_nav2),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": "true",
                "slam": "False",
                "use_composition": "True",
            }.items(),
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
                 "box_model": LaunchConfiguration("box_model"),
                 "box_sdf_path": LaunchConfiguration("box_sdf_path"),
             }]),
        Node(package="nd1_capstone", executable="coordinator_fsm", name="coordinator_fsm",
             output="screen", parameters=[{
                 "sim_mode": sim,
                 "auto_redock": LaunchConfiguration("auto_redock"),
             }]),
        # ── (상 난이도) LLM 멀티미션 플래너 — use_planner:=true 시만 기동 ──
        Node(package="nd1_capstone", executable="llm_planner", name="llm_planner",
             output="screen", condition=IfCondition(use_planner)),
        # ── RViz2 — use_rviz:=true 이고 nav2:=false 일 때만 기동 ──────
        # turtlebot3_navigation2/navigation2.launch.py가 nav2:=true 시 자체 RViz를
        # 항상 띄우므로, nav2 사용 시엔 우리 RViz를 끄고 중복을 방지함.
        Node(package="rviz2", executable="rviz2", name="rviz2",
             output="screen",
             condition=IfCondition(PythonExpression(
                 ["'true' if '", use_rviz, "' == 'true' and '", use_nav2, "' == 'false' else 'false'"])),
             arguments=["-d", _nav2_rviz_cfg]),
    ]

    return LaunchDescription(args + includes + nodes)
