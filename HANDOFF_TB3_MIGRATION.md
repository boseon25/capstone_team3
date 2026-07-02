# 인수인계 — TurtleBot4→TurtleBot3 마이그레이션 (2026-07-01 진행분)

## 왜 이 작업을 하고 있나
WSL2 + Intel Arc GPU 환경에서 TurtleBot4/Ignition-Fortress 실연동(`sim_mode:=false`)을 검증하다가
두 가지 환경 블로커를 만남 (코드 버그 아님):
1. GPU(d3d12) 렌더링 — Gazebo의 GPU LIDAR가 360도 전부 `range_min`(0.164m)만 반환하는 깨진 값을 냄
   → Nav2가 "사방이 벽"으로 오판해서 모든 내비게이션 목표가 즉시 실패.
2. 소프트웨어 렌더(llvmpipe) 대체 — RTF가 ~0.037까지 떨어져서 `controller_manager` 자체가
   기동을 못 함.

→ 사용자가 TurtleBot3 + Gazebo Classic(11)로 전체 마이그레이션 결정. 계획 파일:
`/home/jj/.claude/plans/unified-greeting-kernighan.md` (Plan 모드에서 승인된 원본 계획).

## 지금까지 확인된 핵심 사실 (검증 완료)
- **GO 판정 확정**: TurtleBot3 + Classic Gazebo + `turtlebot3_world`는 **소프트웨어 렌더링만으로도**
  RTF ≈ 0.99~1.00, LIDAR 스캔도 정상(방향별로 다른 값, `.inf`/유한값 혼재). GPU 가속 전혀 불필요.
  → Ignition에서 겪은 두 블로커가 모두 해소됨. 이 부분은 리눅스 데스크탑에서도 그대로 재현될 것으로 예상.
- `turtlebot3_world`는 **벽이 없는 열린 월드**다. 중앙에 3x3 기둥 장식물(터틀봇 로고, ros_symbol)만
  있고 그 외엔 사방이 뚫려 있음. SLAM 맵의 "경계"는 실제 벽이 아니라 로봇이 LIDAR로 아직 못 본
  영역(회색/미탐사)과의 경계선일 뿐.
- **중요한 함정**: Nav2는 로봇이 SLAM으로 아직 관측하지 못한(미탐사=회색) 좌표로는 애초에 경로
  계산을 거부한다 ("off the global costmap" / "GridBased: failed to create plan"). 좌표가 지도
  파일의 정적 경계 안에 있어 보여도, 실제 occupancy 값이 -1(unknown)이면 실패한다.
  → 존(zone) 좌표는 반드시 로봇이 실제로 스캔한 free(0) 셀 안에 있어야 한다. `/map` 토픽을
  `TRANSIENT_LOCAL` QoS로 직접 구독해서 특정 좌표의 occupancy 값을 코드로 확인하는 방법이
  가장 확실함 (아래 "좌표 재확인 스크립트" 참고).

## 코드 변경 완료분 (Phase 0~4, 커밋 안 됨 — 아직 working tree)
`git diff --stat` 기준 9개 파일 수정됨:
- `Dockerfile`: Ignition/Fortress 설치 제거, `ros-humble-gazebo-ros-pkgs` +
  `ros-humble-turtlebot3-{gazebo,msgs,navigation2}` 추가 (전부 apt 바이너리로 정상 설치 확인됨,
  소스 빌드 폴백 불필요). `ENV TURTLEBOT3_MODEL=burger` 추가.
- `docker-compose.yml`: 이미지 태그를 `humble-1.3-tb3`로 변경.
- `workspace/nd1_capstone/package.xml`: `ros_gz_sim`/`irobot_create_msgs` 제거,
  `gazebo_msgs` 추가.
- `node_b_nav.py`: `_on_dock`에 `if self.sim_mode or not self._dock_ready:` 한 줄 추가 —
  TB3엔 실제 도킹 스테이션이 없으므로 `irobot_create_msgs` 미가용 시 무조건 즉시 성공 처리.
  (FSM `coordinator_fsm.py`는 손대지 않음 — 완전히 opaque하게 동작해서 변경 불필요했음.)
- `node_c_grasp.py`: `_teleport`을 `ign service` subprocess 방식에서
  `gazebo_msgs/srv/{SpawnEntity,DeleteEntity}` ROS 서비스 클라이언트로 완전 재작성.
  **주의**: 처음엔 `rclpy.spin_until_future_complete()`를 콜백 안에서 중첩 호출했다가
  타임아웃 버그가 남 (이미 `rclpy.spin(node)`로 도는 콜백 안에서 또 spin하면 executor 충돌).
  → `future.add_done_callback(...)` 비동기 패턴으로 수정해서 해결(Node B의 액션 콜백과 동일한
  스타일). `world_name` 파라미터는 제거(Classic Gazebo는 프로세스당 월드 1개라 불필요).
- `node_a_llm.py`, `llm_planner.py`: 표준 구역 좌표 `ZONES` 갱신 (아래 참고, 계속 바뀌는 중).
- `launch/spawn_boxes.launch.py`: `ros_gz_sim create` → `gazebo_ros spawn_entity.py`로 교체,
  `world` 인자 제거.
- `launch/bringup.launch.py`: SLAM/localization/nav2 include를 `turtlebot4_navigation` →
  `nav2_bringup`(`slam_launch.py`/`localization_launch.py`/`navigation_launch.py`)로 교체,
  `world_name` 인자 제거.

## 구역 좌표 — 아직 확정 안 됨 (여기서 이어가야 함)
`turtlebot3_world` 스폰 위치는 `(-2.0, -0.5)`. 지금까지 시도한 좌표들과 결과:
- 1차 `A(0.6,0.6) B(-0.6,-0.6) C(0.6,-0.6)`: 기둥 격자 내부라 DWB 로컬플래너가 좁은 통로에서
  "No valid trajectories / Hits Obstacle" 반복 실패.
- 2차 `A(-2.2,1.5) B(2.2,-1.5) C(2.2,1.5)`: NAVIGATING·GRASPING(텔레포트)까지 성공! 하지만
  B가 로봇이 한 번도 안 가본 먼 지역이라 TRANSPORTING에서 "off the global costmap"로 실패.
- 3차 `A(-2.0,1.2) B(-2.7,-1.8) C(-1.0,-2.2)`: A/GRASPING까지 다시 성공. B가 여전히 unknown
  셀이라 실패 (`ros2 topic echo /map` 직접 조회로 확인: `(-2.7,-1.8) → -1`).
- **마지막 시도(중단된 지점)**: B를 확인된 free 셀인 `(-1.5, -1.0)`로 교체하고 `colcon build`까지
  마친 상태. **아직 재검증 못 함** — box1 재스폰 → 4노드 재기동 → smoke_test.py 재실행이 다음 할 일.

### 좌표 재확인 스크립트 (그대로 재사용 가능)
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class M(Node):
    def __init__(self):
        super().__init__('mapcheck')
        self.msg = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
    def cb(self, msg): self.msg = msg

rclpy.init(); n = M()
import time
t0=time.time()
while n.msg is None and time.time()-t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
m = n.msg
w,h,res = m.info.width, m.info.height, m.info.resolution
ox,oy = m.info.origin.position.x, m.info.origin.position.y
def val(x,y):
    mx,my = int((x-ox)/res), int((y-oy)/res)
    return m.data[my*w+mx] if 0<=mx<w and 0<=my<h else 'OOB'
# 0=free, -1=unknown(미탐사), 100=occupied
for (x,y) in [(-2.0,1.2),(-1.5,-1.0),(-1.0,-2.2)]:
    print((x,y), '->', val(x,y))
rclpy.shutdown()
```
(`docker exec -u ubuntu nd1_capstone_dev bash -lc "source /opt/ros/humble/setup.bash && python3 <<'EOF' ... EOF"` 형태로 실행)

## 아직 안 한 것 (Phase 5, 6 마무리)
1. **Phase 6 완주**: 위 스크립트로 A/B/C 셀이 전부 `0`(free)인지 재확인 → box1 재스폰 →
   `bringup.launch.py sim_mode:=false` 재기동 → `smoke_test.py`로 `DONE`까지 도달 확인.
   (참고: `sim_mode:=true` 회귀 스모크 테스트는 이미 PASS 확인 완료.)
2. **Phase 5 — 문서 6개 재작성**: 아직 손 안 댐.
   `실연동_검증_절차.md`, `기본월드_박스세팅.md`, `README_capstone.md`, `STRUCTURE.md`,
   `학생_시작가이드.md`, `WSLg_렌더_가이드.md` — 전부 TurtleBot4/Ignition 명령어·용어가 남아있음.
   상세 교체 내용은 `/home/jj/.claude/plans/unified-greeting-kernighan.md`의 "Phase 5" 섹션 참고.

## 리눅스 데스크탑으로 옮길 때 참고
- 네이티브 리눅스 + (아마도) 실제 GPU라면 RTF/LIDAR 문제 자체가 아예 없을 가능성이 높음 —
  Phase 1 GO/NO-GO 재검증은 가볍게 한 번만 확인하면 됨 (`/scan`이 방향별로 다른 값 내는지,
  RTF 1.0 근처인지).
- **좌표-탐사 문제는 환경과 무관하게 동일하게 재현됨** (온라인 SLAM의 근본 특성) — 로봇이
  실제로 지나간 적 없는 좌표는 아무리 지도 범위 안이라도 Nav2가 거부한다는 점 유의.
  네이티브 환경에서도 위 스크립트로 좌표를 검증하고 진행할 것.
- 컨테이너/이미지 이름: `nd1_capstone_dev` / `nd1capstone/ros2-gazebo-vnc:humble-1.3-tb3`.
  `docker compose up --build`로 재기동하면 됨 (`docker-compose.yml` 이미 TB3용으로 수정됨).
- `.env` 파일(GROQ_API_KEY 등)은 `.gitignore`에 있어 git에 안 잡힘 — 새 환경에서 다시 만들어야 함
  (`env.example` 참고, 이번 세션에서 쓴 키: `api_key.txt` 파일에 남아있음).
