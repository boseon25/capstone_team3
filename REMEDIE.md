# ND1 캡스톤 — 전체 작업 정리 (REMEDIE)

> 작성일: 2026-07-01  
> 작업 범위: 하/중/상 난이도 전체 동작까지 수행한 모든 변경사항 기록

---

## 1. 환경 구성

### 호스트 (로컬 Linux 데스크탑)
| 항목 | 값 |
|------|-----|
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 5070 (VRAM 12GB) |
| DISPLAY | `:1` |
| Docker | docker-compose v2 |

### 도커 컨테이너
| 항목 | 값 |
|------|-----|
| 이름 | `nd1_capstone_dev` |
| 이미지 | `nd1capstone/ros2-gazebo-vnc:humble-1.3-tb3` |
| ROS2 | Humble |
| Gazebo | Classic 11 |
| 로봇 | TurtleBot3 burger |
| ROS2 워크스페이스 | `/home/ubuntu/ros2_ws` |
| 소스 마운트 | `./workspace` → `/home/ubuntu/ros2_ws/src` |

---

## 2. 파일 현황 (최종)

### 프로젝트 루트
```
capstone_team3/
├── docker-compose.yml          ← 메인 실행 설정 (수정됨)
├── Dockerfile                  ← 컨테이너 빌드
├── .env                        ← GROQ_API_KEY 등 환경변수
├── .gitignore
├── REMEDIE.md                  ← 이 문서
├── docs/                       ← 캡스톤 제출 문서 (docx)
├── foxglove/                   ← Foxglove 레이아웃
├── m7_ik/                      ← IK 라이브러리 (nd1_m7_ik)
└── workspace/nd1_capstone/     ← ROS2 패키지
```

### workspace/nd1_capstone/
```
├── launch/
│   ├── bringup.launch.py       ← 전체 스택 런치 (수정됨)
│   └── spawn_boxes.launch.py   ← Gazebo 박스 스폰
├── models/
│   └── box1.sdf                ← 박스 모델
├── nd1_capstone/
│   ├── node_a_llm.py           ← 자연어 파싱 노드
│   ├── node_b_nav.py           ← Nav2 이동 노드
│   ├── node_c_grasp.py         ← 파지/배치 노드
│   ├── coordinator_fsm.py      ← FSM 조정 노드
│   ├── linear_orchestrator.py  ← 선형 오케스트레이터 (수정됨)
│   └── llm_planner.py          ← LLM 멀티미션 플래너 (수정됨)
├── scripts/
│   ├── term1_launch.sh         ← 전체 스택 기동 스크립트
│   ├── term2_test.sh           ← 테스트 명령어 안내
│   ├── smoke_test.py           ← 하/중 난이도 자동 테스트
│   └── smoke_test_planner.py   ← 상 난이도 자동 테스트
└── setup.py                    ← 패키지 설정 (수정됨)
```

---

## 3. 변경 내역 상세

### `docker-compose.yml` (수정)
- `build.network: host` 추가
- 볼륨에 `/tmp/.X11-unix:/tmp/.X11-unix` 추가 (호스트 X11 소켓 공유)
- `LIBGL_ALWAYS_SOFTWARE=1` → `LIBGL_ALWAYS_SOFTWARE=0` (GPU 렌더링 활성화)
- `GALLIUM_DRIVER=llvmpipe` → `GALLIUM_DRIVER=` 제거
- `NVIDIA_VISIBLE_DEVICES=all`, `NVIDIA_DRIVER_CAPABILITIES=all` 추가
- `deploy.resources.reservations.devices` nvidia GPU 추가
- `restart: unless-stopped` 추가 (데몬 재시작 시 컨테이너 자동 복구)
- `DISPLAY=:1` 설정

### `bringup.launch.py` (수정 — 버그 3개 수정 + 기능 추가)

**버그 1: PathJoinSubstitution 빈 문자열 버그**
- 기존: `PathJoinSubstitution([FindPackageShare(...)])` → 다중 include 시 빈 문자열 반환
- 수정: `get_package_share_directory()` 결과를 Python 상수로 고정

**버그 2: SLAM params_file 오염 (FileNotFoundError)**
- 기존: `launch_arguments={"use_sim_time": "true"}.items()`
- 수정: `"params_file": _nav2_params_yaml` 명시 추가
- 이유: gzserver.launch.py가 `params_file=''`을 launch context에 먼저 주입

**버그 3: Nav2 NameError (`eval('not true')`)**
- 기존: Nav2 include에 slam/use_composition 미전달
- 수정: `slam='False'`, `use_composition='True'` 명시
- 이유: gzserver가 lowercase `'true'`를 context에 오염 → PythonExpression eval 실패

**추가된 기능:**
- `use_gazebo`: true 시 Gazebo Classic + TurtleBot3 스폰
- `use_rviz`: true 시 RViz2 기동 (nav2=true면 중복 방지로 비활성)
- `use_planner`: true 시 llm_planner 노드 기동
- `tb3_world`: 기동할 월드 이름

### `llm_planner.py` (수정 — TODO 4개 구현)

| 항목 | 원본 | 변경 후 |
|------|------|---------|
| `PLANNER_PROMPT` | JSON 배열 요청 | `{"missions":[...]}` 객체 요청으로 변경 |
| `_plan()` | `return None` | groq 호출 + missions/tasks/steps 키 파싱 |
| `_plan_fallback()` | `return []` | 정규식 구역 추출 → pick_and_place 배열 생성 |
| `_dispatch()` | `pass` | 큐 pop → /mission 발행, active=True |

### `linear_orchestrator.py` (수정 — TODO 2개 구현)

| 항목 | 원본 | 변경 후 |
|------|------|---------|
| `_on_mission()` | `self.steps = []` | JSON 파싱 → navigate/pick_and_place 단계 리스트 구성 |
| `_next()` | `pass` | 순차 실행 + sim/실연동 분기 |
| `_busy` 플래그 | 없음 | 추가 (중복 진입 방지) |
| 결과 토픽 구독 | 항상 구독 | `if not self.sim_mode:` 조건부 구독 |

### `setup.py` (수정)
```python
# 추가:
(os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
```

### 신규 생성 파일
| 파일 | 설명 |
|------|------|
| `scripts/term1_launch.sh` | 전체 스택 기동 스크립트 (gazebo-11 setup.sh 소싱 포함) |
| `scripts/term2_test.sh` | 테스트 명령어 안내 |
| `scripts/smoke_test_planner.py` | 상 난이도 자동 스모크 테스트 |

### 컨테이너 내부 직접 패치 (재생성 시 재패치 필요)
```bash
# 컨테이너 안에서:
sed -i "s/'-z', '0.01'/'-z', '0.5'/" \
  /opt/ros/humble/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
# 이유: z=0.01이면 로봇이 지면 아래로 떨어지는 물리 버그 발생
```

---

## 4. 아키텍처

```
[하] /mission ──────────────────────────────────→ coordinator_fsm
[중] /llm_command → node_a_llm → /mission ───────→ coordinator_fsm
[상] /goal_command → llm_planner → /mission(순차)→ coordinator_fsm

coordinator_fsm → /nav_request   → node_b_nav   → Nav2 navigate_to_pose
coordinator_fsm → /grasp_request → node_c_grasp → Gazebo SpawnEntity/DeleteEntity
모든 노드       → /robot_status  (상태 모니터링)
```

### 구역 좌표 (고정)
| 구역 | x | y |
|------|-----|------|
| A | -2.0 | 1.2 |
| B | -1.5 | -1.0 |
| C | -1.0 | -2.2 |

### coordinator_fsm 상태 흐름
```
IDLE → PLANNING → UNDOCKING → NAVIGATING → GRASPING → TRANSPORTING → PLACING → DONE
```

---

## 5. 실행 방법

### Step 0. X11 권한 (호스트, 1회)
```bash
xhost +local:docker
```

### Step 1. 컨테이너 시작
```bash
cd /home/jj/capstone_team3
docker compose up -d
```

### Step 2. 전체 스택 기동 (터미널 1)
```bash
docker exec -it nd1_capstone_dev bash
source /opt/ros/humble/setup.bash && \
source /usr/share/gazebo-11/setup.sh && \
source /home/ubuntu/ros2_ws/install/setup.bash && \
export TURTLEBOT3_MODEL=burger && export DISPLAY=:1 && \
ros2 launch nd1_capstone bringup.launch.py \
  sim_mode:=false use_gazebo:=true slam:=true nav2:=true \
  use_planner:=true use_rviz:=true auto_redock:=false
```

### Step 3. 박스 스폰 (터미널 2, Gazebo 뜬 후)
```bash
docker exec -it nd1_capstone_dev bash
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && \
ros2 launch nd1_capstone spawn_boxes.launch.py
```

### Step 4. 상태 모니터링 (터미널 3)
```bash
docker exec -it nd1_capstone_dev bash
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && \
ros2 topic echo /robot_status
```

---

## 6. 난이도별 테스트

> 모든 명령어 실행 전 source 필수:
> ```bash
> source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash
> ```

### 하 ★☆☆ — /mission 직접 발행
```bash
ros2 topic pub --once /mission std_msgs/String \
'{data: "{\"action\":\"pick_and_place\",\"pick_x\":-2.0,\"pick_y\":1.2,\"place_x\":-1.5,\"place_y\":-1.0,\"yaw\":0.0,\"object\":\"박스\"}"}'
```
흐름: `/mission` → coordinator_fsm → A구역 이동 → grasp → B구역 이동 → place → DONE

### 중 ★★☆ — /llm_command 자연어
```bash
ros2 topic pub --once /llm_command std_msgs/String \
'{data: "A구역 박스를 B구역으로 옮겨줘"}'
```
흐름: `/llm_command` → node_a_llm(파싱) → `/mission` → coordinator_fsm → DONE

### 상 ★★★ — /goal_command 복합 자연어
```bash
ros2 topic pub --once /goal_command std_msgs/String \
'{data: "A구역과 B구역을 C구역으로 옮겨줘"}'
```
흐름: `/goal_command` → llm_planner(미션 2건 분해) → 미션1 DONE → 미션2 DONE → 모든 미션 완료 ✅

### 자동 스모크 테스트
```bash
# 하/중
python3 ~/ros2_ws/src/nd1_capstone/scripts/smoke_test.py "A구역 박스를 B구역으로 옮겨줘" 120

# 상
python3 ~/ros2_ws/src/nd1_capstone/scripts/smoke_test_planner.py "A구역과 B구역을 C구역으로" 2 180
```

---

## 7. 주의사항

### GROQ API 키 없을 때
- node_a_llm: 키워드 폴백 자동 전환 ("A구역", "옮겨줘" 등 한국어 인식)
- llm_planner: 정규식 폴백 자동 전환
- `/llm_command`에 JSON 직접 전달 금지 → 폴백 파서가 파싱 못해 STOP 반환

### colcon 재빌드 필요 시
```bash
cd /home/ubuntu/ros2_ws
colcon build --symlink-install --packages-select nd1_capstone
source install/setup.bash
```

### 성공 확인
```
[FSM:DONE] 미션 완료 → DONE ✅
[PLAN] 모든 미션 완료 ✅   ← 상 난이도만
```
