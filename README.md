# capstone_team3 — 병원 백신 배달 로봇 (TurtleBot3 + Nav2 + LLM)

`capstone_team3`(원본: TurtleBot4 + Gazebo Ignition + Docker) 캡스톤 키트를
**TurtleBot3 + Gazebo classic + 네이티브(Docker 미사용) 환경**으로 이식하고,
**AWS RoboMaker Hospital World** 위에서 자연어 명령으로 병실에 백신을 배달하는
시나리오로 재구성한 프로젝트.

![데모 스크린샷](demo/thumbnail.jpg)

▶ [데모 영상 (demo/demo.webm)](demo/demo.webm)

## 시나리오
"301호 환자에게 A형 독감 백신 가져다줘" 같은 자연어 명령을 LLM(Groq)이 해석해
로봇이 약국(pharmacy)에서 해당 백신을 픽업(텔레포트)한 뒤 지정된 병실로 이동해
배달(텔레포트)한다. 백신은 총 4종(A/B/AB/O형), 병실은 4개 + 약국 1개.

## 아키텍처 (토픽 계약 — capstone_team3 원본과 동일, 변경 금지)
```
/llm_command ─▶ Node A(LLM+폴백) ─/mission─▶ Coordinator(FSM)
                                              │  /nav_request   ─▶ Node B(Nav2) ─/nav_result─┐
                                              │  /grasp_request ─▶ Node C(텔레포트) ─/grasp_result┘
                                              └─ /robot_status (상태 로그)
```
- SLAM 대신 **map_server + AMCL**로 고정 지도 위에서 위치추정 (map: `maps/hospital_map.yaml`)
- Node C의 "파지/배치"는 실제 로봇팔 없이 **Gazebo `/spawn_entity`, `/delete_entity`로 텔레포트**해 시뮬레이션

## 폴더 구조
```
capstone_team3/
├─ README.md
├─ .env.example                       # GROQ_API_KEY (비워둠 — .env로 복사해 채울 것)
├─ config/
│  ├─ rooms.yaml                      # 약국 + 병실 4개 좌표(map 프레임)
│  ├─ vaccines.yaml                   # 백신 4종 → box 엔티티명
│  └─ burger_hospital.yaml            # Nav2/AMCL 파라미터(hospital world용으로 튜닝)
├─ maps/hospital_map.{pgm,yaml}       # 완성된 병원 지도
├─ scripts/smoke_test.py
└─ src/
   ├─ nd1_capstone_tb3/
   │  ├─ launch/
   │  │  ├─ hospital_world.launch.py    # 병원 월드 + TurtleBot3 스폰
   │  │  ├─ map_localization.launch.py  # map_server + AMCL + Nav2 + RViz
   │  │  ├─ spawn_boxes.launch.py       # 백신 4종 스폰(약국 근처)
   │  │  └─ bringup.launch.py           # 캡스톤 4노드(A/B/C/Coordinator)
   │  ├─ models/vaccine_{a,b,ab,o}.sdf  # 백신 소품(색상별)
   │  └─ nd1_capstone_tb3/ (node_a_llm·node_b_nav·node_c_grasp·coordinator_fsm)
   ├─ nd1_m7_ik/                       # IK 라이브러리(node_c_grasp가 사용)
   ├─ aws-robomaker-hospital-world/    # 병원 Gazebo 월드 — git 추적 제외, 별도 clone 필요
   └─ m-explore-ros2/                  # explore_lite — git 추적 제외, 별도 clone 필요
```

> `aws-robomaker-hospital-world`, `m-explore-ros2`는 각각 자체 `.git`을 가진 외부
> 저장소라 `.gitignore`로 제외했다. 이 컴퓨터가 아닌 곳에서 새로 clone했다면 아래를 먼저 실행:
> ```bash
> git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-hospital-world.git src/aws-robomaker-hospital-world
> git clone https://github.com/robo-friends/m-explore-ros2.git src/m-explore-ros2
> touch src/m-explore-ros2/map_merge/COLCON_IGNORE   # map_merge는 미사용
> ```

## 0. 최초 1회 — 빌드
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash   # turtlebot3_gazebo 등 오버레이 대상 먼저 소싱
cd ~/capstone_team3
PATH=/usr/bin:$PATH colcon build --symlink-install
```
> `aws-robomaker-hospital-world`는 cmake 빌드 시점에 가구 3D 모델 40여 종을
> Fuel에서 받는다 — 인터넷 필요, 수 분 걸릴 수 있음.

새 터미널을 열 때마다 아래 3줄을 항상 먼저 실행한다:
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
cd ~/capstone_team3 && source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 1. 실행 순서 (터미널 5개)

**터미널 1 — 병원 월드 + 로봇 스폰**
```bash
ros2 launch nd1_capstone_tb3 hospital_world.launch.py
```
(gzserver가 시작할 때 온라인 모델 DB 조회로 1~2분 걸릴 수 있음. 로봇 스폰 로그까지 기다릴 것)

**터미널 2 — map_server + AMCL + Nav2 + RViz**
```bash
ros2 launch nd1_capstone_tb3 map_localization.launch.py
```

**터미널 3 — 초기 위치 설정** (로봇 스폰 좌표 3.75, 2.75와 반드시 일치시킬 것)
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {pose: {position: {x: 3.75, y: 2.75, z: 0.0}, orientation: {w: 1.0}},
         covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}
}"
```
RViz에 지도 위 로봇 파티클이 한 점으로 수렴하면 성공.

**터미널 4 — 백신 4종 스폰** (최초 1회, 약국 근처에 배치)
```bash
ros2 launch nd1_capstone_tb3 spawn_boxes.launch.py
```

**터미널 5 — 캡스톤 4노드(LLM/Nav2/텔레포트/FSM) 기동**
```bash
ros2 launch nd1_capstone_tb3 bringup.launch.py sim_mode:=false
```
시작 로그에 `Node A 시작 — LLM=ON`이 뜨면 GROQ_API_KEY 인식 성공(안 뜨면 폴백 파서로만 동작).

## 2. 명령 내리기
```bash
ros2 topic pub --once /llm_command std_msgs/msg/String "{data: '301호 환자에게 A형 독감 백신 가져다줘'}"
```
가능한 조합: 301호·A형, 302호·B형, 303호·AB형, 304호·O형.
`ros2 topic echo /robot_status`로 약국 이동→픽업→병실 이동→배치까지 진행 상황을 볼 수 있다.

> `--once`는 가끔 최초 1회 발행이 구독자 매칭 타이밍에 유실될 수 있어
> `ros2 topic pub -r 2 -t 3 /llm_command ...` (2Hz로 3초간 반복 발행)을 권장.

## 3. GROQ API 키
```bash
cp .env.example .env   # .env에 실제 GROQ_API_KEY 입력 (.env는 .gitignore 처리되어 git에 안 올라감)
```
키가 없거나 유효하지 않으면 `node_a_llm.py`가 규칙 기반 폴백 파서로 자동 전환되어
동작 자체는 계속된다(로그에 `⚠️ LLM 파싱 실패` 표시).

## 4. 알려진 이슈 / 트러블슈팅
- **gzclient가 "Camera px != 0" assertion으로 죽는 경우**: `gazebo_ros`의
  `gzclient.launch.py`가 붙이는 `libgazebo_ros_eol_gui.so`(EOL 배너) 플러그인과
  충돌하는 경우가 있다. `hospital_world.launch.py`는 이미 플러그인 없는 순정
  `gzclient`를 쓰도록 수정되어 있음.
- **`/spawn_entity`, `/delete_entity` 응답이 간헐적으로 타임아웃**: classic
  Gazebo factory 플러그인이 부하 상황에서 응답을 늦게 보내는 경우가 있다.
  `node_c_grasp.py`는 `spin_until_future_complete`(중첩 스핀, 콜백 안에서
  재진입 스핀 시 응답 유실 가능) 대신 `add_done_callback` 비동기 패턴으로
  수정되어 있음 — 이 패턴을 유지할 것.
- **VSCode(Snap) 환경변수(GTK_PATH 등) leak으로 gzclient/rviz2/터미널 앱이
  `libpthread.so.0` 심볼 에러로 죽는 경우**: launch 전에
  `unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR`
  로 정리 후 실행.
- **AMCL 초기 위치를 안 잡으면 Nav2가 계속 "map 프레임 없음" 경고**: 위 1번
  터미널3의 `/initialpose` 발행을 반드시 먼저 할 것.

## 5. 원본(capstone_team3, TurtleBot4) 대비 변경점
- TurtleBot4/Ignition → **TurtleBot3/Gazebo classic** (`gazebo_ros spawn_entity.py`, `/spawn_entity`·`/delete_entity`)
- 기본 월드 → **AWS RoboMaker `hospital.world`**
- SLAM 실시간 매핑 대신 **완성된 고정 지도 + map_server/AMCL**
- 좌표 2개(A/B) pick&place → **병실 4개 + 약국 1개, 백신 4종 매핑**으로 확장
- 도킹(dock/undock) 관련 인자·상태 제거 (TB3에는 Create3 도킹 스테이션 없음)
- Docker/noVNC 미사용 — 호스트 네이티브 ROS2 Humble + Gazebo 11(classic)
