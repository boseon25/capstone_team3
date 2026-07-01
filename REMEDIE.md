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
| XAUTHORITY | `/run/user/1000/gdm/Xauthority` → `/tmp/.docker.xauth` 복사 |
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

## 2. 파일 변경 내역 (하나도 빠짐없이)

### 2-1. 수정된 파일

#### `docker-compose.yml`
**변경 전 → 변경 후 요약:**
- `build.network: host` 추가
- 볼륨에 `/tmp/.X11-unix:/tmp/.X11-unix` 추가 (호스트 X11 소켓 공유)
- 환경변수: `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe` → `LIBGL_ALWAYS_SOFTWARE=0`, `GALLIUM_DRIVER=` (GPU 렌더링 활성화)
- `NVIDIA_VISIBLE_DEVICES=all`, `NVIDIA_DRIVER_CAPABILITIES=all` 추가
- `deploy.resources.reservations.devices` (nvidia GPU) 추가
- `restart: unless-stopped` 추가 (Docker 데몬 재시작 시 컨테이너 자동 복구)
- `DISPLAY=:1` (호스트 디스플레이 번호 맞춤)

#### `workspace/nd1_capstone/launch/bringup.launch.py`
**주요 버그 수정 3개 + 기능 추가:**

1. **PathJoinSubstitution 버그 수정**
   - 기존: `PathJoinSubstitution([FindPackageShare(...), "launch", "slam_launch.py"])` → 다중 include 시 빈 문자열로 해석되는 ROS2 Humble 버그
   - 수정: `get_package_share_directory()` 결과를 Python 상수로 고정

2. **SLAM params_file 명시 (FileNotFoundError 수정)**
   - 기존: `launch_arguments={"use_sim_time": "true"}.items()`
   - 수정: `launch_arguments={"use_sim_time": "true", "params_file": _nav2_params_yaml}.items()`
   - 이유: gzserver.launch.py가 `params_file=''`을 먼저 선언해 launch context 오염

3. **Nav2 NameError 수정 (`eval('not true')` → 수정)**
   - 기존: Nav2 include에 slam/use_composition 미전달
   - 수정: `slam='False'`, `use_composition='True'` 명시 전달
   - 이유: gzserver가 lowercase `'true'`를 context에 주입 → `PythonExpression(['not ', slam])` eval 시 NameError

4. **추가된 launch 인자:**
   - `use_gazebo` (true=Gazebo 기동)
   - `use_rviz` (true=RViz2 기동)
   - `use_planner` (true=LLM 멀티미션 플래너 기동)
   - `tb3_world` (기동할 월드 이름)

5. **RViz 중복 방지:**
   - tb3_navigation2가 nav2=true 시 자체 RViz를 띄우므로, `nav2=false`일 때만 우리 RViz 기동

6. **llm_planner 노드 조건부 추가:**
   ```python
   Node(package="nd1_capstone", executable="llm_planner", condition=IfCondition(use_planner))
   ```

#### `workspace/nd1_capstone/nd1_capstone/llm_planner.py`
**TODO 3개 구현:**

1. **PLANNER_PROMPT 수정**: LLM이 배열이 아닌 `{"missions":[...]}` 객체를 반환하도록 프롬프트 변경 (JSON object 모드 강제)

2. **`_plan()` 구현**: groq LLM 호출 → `missions` 키 파싱 (list/dict 양쪽 처리)
   ```python
   missions = data.get("missions", data.get("tasks", data.get("steps", [])))
   ```

3. **`_plan_fallback()` 구현**: 정규식으로 구역 추출 → 마지막 구역을 목적지로, 나머지를 출발지로 pick_and_place 생성

4. **`_dispatch()` 구현**: 큐에서 1건 pop → /mission 발행, active=True

#### `workspace/nd1_capstone/nd1_capstone/linear_orchestrator.py`
**TODO 2개 구현:**

1. **`_on_mission()` 구현**: JSON 파싱 → action 타입별 단계 리스트 구성
   - `navigate` → [nav to place]
   - `pick_and_place` → [nav to pick, grasp, nav to place, place]

2. **`_next()` 구현**: 단계 순차 실행 (sim_mode: 타이머, 실연동: 결과 토픽 구독)

3. **`_busy` 플래그 추가**: 중복 진입 방지

4. **sim_mode 분기 개선**: `if not self.sim_mode:` 조건으로 결과 토픽 구독

#### `workspace/nd1_capstone/setup.py`
**변경:**
```python
# 추가된 줄:
(os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
```
- scripts 디렉토리의 Python 파일들을 패키지에 포함시킴

---

### 2-2. 새로 생성된 파일

#### `supervisord.conf` (호스트 루트)
- VNC + noVNC 프로세스 관리용 supervisor 설정
- `[program:vnc]`: ubuntu 유저로 VNC 서버 기동
- `[program:novnc]`: websockify로 포트 80에 noVNC 서빙 # 리눅스 데스크탑에서 vnc 접속이 튕겨서 한겁니다.

#### `workspace/nd1_capstone/scripts/term1_launch.sh`
- 전체 스택 기동 스크립트 (컨테이너 내부에서 실행)
- gazebo-11 setup.sh 소싱 포함 (GAZEBO_MODEL_PATH 설정 필수)
- `sim_mode:=false use_gazebo:=true slam:=true nav2:=true use_planner:=true use_rviz:=true auto_redock:=false`

#### `workspace/nd1_capstone/scripts/term2_test.sh`
- 테스트 명령어 안내 스크립트

#### `workspace/nd1_capstone/scripts/smoke_test_planner.py`
- 상 난이도 자동 스모크 테스트
- `/goal_command` 주입 → DONE N건 수신 시 PASS

#### `workspace/nd1_capstone/scripts/gui_start.sh`
- GUI 시작 헬퍼 스크립트


---

### 2-4. 삭제된 파일
- 없음

---

## 3. 아키텍처

```
[하] /mission ─────────────────────────────────→ coordinator_fsm
[중] /llm_command → node_a_llm → /mission ──────→ coordinator_fsm
[상] /goal_command → llm_planner → /mission(순차)→ coordinator_fsm

coordinator_fsm → /nav_request → node_b_nav → Nav2 navigate_to_pose
coordinator_fsm → /grasp_request → node_c_grasp → Gazebo SpawnEntity/DeleteEntity
모든 노드 → /robot_status (상태 모니터링)
```

### 구역 좌표 (고정, 변경 금지)
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

## 4. 실행 명령어 전체

### Step 0. X11 권한 (호스트에서 1회)
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
export TURTLEBOT3_MODEL=burger && \
export DISPLAY=:1 && \
ros2 launch nd1_capstone bringup.launch.py \
  sim_mode:=false \
  use_gazebo:=true \
  slam:=true \
  nav2:=true \
  use_planner:=true \
  use_rviz:=true \
  auto_redock:=false
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

## 5. 난이도별 테스트 명령어

### 하 난이도 ★☆☆ — /mission 직접 발행
node_a를 건너뛰고 coordinator_fsm에 직접 JSON 명령 전달
```bash
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && \
ros2 topic pub --once /mission std_msgs/String \
'{data: "{\"action\":\"pick_and_place\",\"pick_x\":-2.0,\"pick_y\":1.2,\"place_x\":-1.5,\"place_y\":-1.0,\"yaw\":0.0,\"object\":\"박스\"}"}'
```

**동작 흐름:**
```
/mission → coordinator_fsm
  UNDOCKING(성공) → A구역 이동 → grasp → B구역 이동 → place → DONE
```

### 중 난이도 ★★☆ — /llm_command 자연어
node_a_llm이 자연어를 파싱해 /mission으로 변환
```bash
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && \
ros2 topic pub --once /llm_command std_msgs/String \
'{data: "A구역 박스를 B구역으로 옮겨줘"}'
```

**동작 흐름:**
```
/llm_command → node_a_llm (groq LLM 또는 키워드 폴백 파싱)
  → /mission → coordinator_fsm → 동일 흐름
```

### 상 난이도 ★★★ — /goal_command 복합 자연어
llm_planner가 복합 목표를 미션 N개로 분해해 순차 투입
```bash
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && \
ros2 topic pub --once /goal_command std_msgs/String \
'{data: "A구역과 B구역을 C구역으로 옮겨줘"}'
```

**동작 흐름:**
```
/goal_command → llm_planner
  계획: 미션1(A→C), 미션2(B→C)
  미션1 /mission 발행 → coordinator_fsm → DONE
  DONE 감지 → 미션2 /mission 발행 → coordinator_fsm → DONE
  모든 미션 완료 ✅
```

---

## 6. 각 Python 파일 원본 대비 변경 상세

### `node_a_llm.py` — 이전 커밋에서 완성, 금번 무변경
- `_parse_with_llm()`: groq API 호출 구현 완료
- `_parse_fallback()`: 키워드 기반 파서 구현 완료
- ZONES: A(-2.0,1.2) B(-1.5,-1.0) C(-1.0,-2.2) (TB3 월드 좌표로 마이그레이션 완료)

### `node_b_nav.py` — 이전 커밋에서 완성, 금번 무변경
- `_on_nav()`: Nav2 navigate_to_pose 액션 클라이언트 구현 완료
- `_on_dock()`: `sim_mode or not self._dock_ready`이면 True 즉시 반환 (TB3 도킹 스테이션 없음)
- `_init_dock()`: irobot_create_msgs 미가용 시 자동 폴백 (dock_ready=False)

### `node_c_grasp.py` — 이전 커밋에서 완성, 금번 무변경
- `_solve_ik()`: nd1_m7_ik 호출, 실패 시 sim_mode면 [0,0,0] 반환
- `_teleport()`: gazebo_msgs SpawnEntity/DeleteEntity 비동기 서비스 클라이언트 구현

### `coordinator_fsm.py` — 이전 커밋에서 완성, 금번 무변경
- `_tick()`: PLANNING 분기 구현 (navigate/pick_and_place/stop)
- `_advance()`: 전이표 구현 (UNDOCKING→NAVIGATING→GRASPING→TRANSPORTING→PLACING→DONE)
- `_retry_current()`: 현재 단계 재시도 구현

### `llm_planner.py` — 금번 TODO 3개 구현
| 메서드 | 원본 | 변경 후 |
|--------|------|---------|
| `PLANNER_PROMPT` | JSON 배열 요청 | `{"missions":[...]}` 객체 요청으로 변경 |
| `_plan()` | `return None` (TODO) | groq 호출 + missions 키 파싱 구현 |
| `_plan_fallback()` | `return []` (TODO) | 구역 추출 → pick_and_place 배열 생성 |
| `_dispatch()` | `pass` (TODO) | 큐 pop → /mission 발행 구현 |

### `linear_orchestrator.py` — 금번 TODO 2개 구현
| 메서드 | 원본 | 변경 후 |
|--------|------|---------|
| `_on_mission()` | `self.steps = []` (TODO) | JSON 파싱 → 단계 리스트 구성 |
| `_next()` | `pass` (TODO) | 순차 실행 + sim/실연동 분기 |
| `__init__` | `_busy` 없음 | `_busy` 플래그 추가 |
| 구독 | 항상 nav_result/grasp_result 구독 | `if not self.sim_mode:` 조건부 구독 |

### `setup.py` — scripts 등록 추가
```python
# 추가:
(os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
```

### `bringup.launch.py` — 대규모 수정
| 항목 | 원본 | 변경 후 |
|------|------|---------|
| 경로 처리 | `PathJoinSubstitution([FindPackageShare(...)])` | `get_package_share_directory()` Python 상수 |
| SLAM 인자 | `use_sim_time` 만 | `params_file` 명시 추가 |
| Nav2 인자 | `use_sim_time` 만 | `slam='False'`, `use_composition='True'` 추가 |
| Gazebo 기동 | 없음 | `use_gazebo:=true` 조건부 추가 |
| RViz | 없음 | `use_rviz:=true` + nav2=false 조건부 추가 |
| llm_planner | 없음 | `use_planner:=true` 조건부 추가 |
| import | `PathJoinSubstitution`, `FindPackageShare` | `PythonExpression` (RViz 조건용) |

---

## 7. 알려진 주의사항


### GROQ API 키 없을 때
- node_a_llm: 키워드 폴백 파서로 자동 전환 ("A구역", "B구역" 등 한국어 키워드 인식)
- llm_planner: 정규식 폴백으로 자동 전환
- `/llm_command`에 JSON 직접 전달은 폴백이 파싱 못해 STOP 반환 → 반드시 자연어 사용

### /mission 직접 발행 시 필수 필드
```json
{
  "action": "pick_and_place",
  "pick_x": -2.0, "pick_y": 1.2,
  "place_x": -1.5, "place_y": -1.0,
  "yaw": 0.0,
  "object": "박스"
}
```

---

## 8. 성공 확인 기준

```
[PLAN] 계획 생성: 미션 N건
[PLAN] 미션 투입: pick_and_place
[FSM:NAVIGATING] 이동 요청 → (x, y)
[B] 이동 결과: 성공
[FSM:GRASPING] grasp 요청
[C] 결과: 성공
[FSM:DONE] 미션 완료 → DONE ✅
[PLAN] 모든 미션 완료 ✅     ← 상 난이도만
```
