# ND1 캡스톤 키트 — 폴더 구조 (학생 배포본 / 반골격)

```
capstone/
├─ Dockerfile                      # ROS2 Humble + Gazebo Fortress + noVNC (SW 렌더 기본)
├─ docker-compose.yml              # 기본 baseline (noVNC·소프트웨어 렌더·학생 배포)
├─ docker-compose.wslg.yml         # (개인개발 opt-in) WSLg 렌더 → GPU(d3d12) 가속
├─ .env.example                    # → .env 복사, GROQ_API_KEY 입력
├─ README_capstone.md              # 아키텍처 + 기동 + 스모크테스트
├─ STRUCTURE.md                    # (이 파일)
├─ 학생_시작가이드.md             # ★ 먼저 읽기 — TODO 맵·개발순서·자가검증
├─ 실연동_검증_절차.md             # v2: sim_mode→SLAM/Nav2→실연동 + 트리아지 T1~T8,T3'
├─ 기본월드_박스세팅.md            # TurtleBot4 기본 월드 재활용 + box1 스폰/테스트
├─ WSLg_렌더_가이드.md             # (개인개발) noVNC→WSLg GPU 가속 렌더 절차
│
├─ m7_ik/                          # ★ M7 IK 패키지 (배치 완료)
│   ├─ setup.py  pyproject.toml  README.md
│   ├─ nd1_m7_ik/ (__init__·arm·jacobian·ik).py
│   └─ tests/smoke.py
│
├─ scripts/smoke_test.py           # 자동 스모크 테스트 (명령→DONE, 종료코드 0/1)
│
├─ workspace/nd1_capstone/         # 컨테이너 ros2_ws/src 로 마운트
│   ├─ package.xml  setup.py  setup.cfg
│   ├─ launch/
│   │   ├─ bringup.launch.py       # 4노드 + 선택적 SLAM/localization/Nav2 + 박스 인자
│   │   └─ spawn_boxes.launch.py   # 기본 월드에 box1 스폰 (world:= 인자)
│   ├─ models/box1.sdf             # 파지 대상 박스 (Fortress SDF 1.8)
│   └─ nd1_capstone/
│      ├─ node_a_llm.py            # Node A — LLM 해석 + 폴백          [LLM] (TODO)
│      ├─ node_b_nav.py            # Node B — Nav2 래퍼                [ROS2] (TODO)
│      ├─ node_c_grasp.py          # Node C — IK 파지/배치+텔레포트     [기구학] (TODO)
│      ├─ coordinator_fsm.py       # 중 ★★☆ — FSM 조정(권장)          [통합] (TODO)
│      ├─ linear_orchestrator.py   # 하 ★☆☆ 선형 (TODO)
│      └─ llm_planner.py           # 상 ★★★ 플래너 (TODO)
│
├─ foxglove/  (nd1_capstone_layout.json · Foxglove_시각화_가이드.md)
├─ docs/  (기획서양식 · 제출체크리스트 · 난이도가이드)
```

## 토픽 계약 (난이도 공통)
- `/llm_command` → Node A → `/mission` (RobotCommand JSON)
- 조정노드 → `/nav_request` → Node B → `/nav_result`
- 조정노드 → `/grasp_request` → Node C → `/grasp_result`
- (상) `/goal_command` → llm_planner → `/mission` (1건씩)
- 모니터링: `/robot_status`

## 빠른 시작 (sim_mode — 시뮬 불필요)
```bash
cp .env.example .env            # m7_ik 배치 완료 상태
docker compose up --build       # noVNC http://localhost:8080
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
python3 src/.../scripts/smoke_test.py
```

## 실연동 (sim_mode:=false) — 기본 월드 재활용 + SLAM
```bash
# 터미널1: 시뮬
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
# 터미널2: SLAM+Nav2+4노드 통합 + 박스 파라미터(launch 인자로 깔끔하게)
BOX_SDF=$(ros2 pkg prefix nd1_capstone)/share/nd1_capstone/models/box1.sdf
ros2 launch nd1_capstone bringup.launch.py sim_mode:=false slam:=true nav2:=true \
  world_name:=<월드명> box_model:=box1 box_sdf_path:=$BOX_SDF
# 터미널3: 박스 스폰 + 스모크
ros2 launch nd1_capstone spawn_boxes.launch.py world:=<월드명>
python3 src/.../scripts/smoke_test.py "A구역 박스를 B구역으로 옮겨줘" 60
```
상세·트리아지: `실연동_검증_절차.md`(SLAM/map), `기본월드_박스세팅.md`(박스/월드명).

## 검증 상태 (사실)
- m7_ik: 임포트 OK, numerical_ik→3개 float([-0.03,0.95,0.302]), 스모크 ALL PASSED (격리 venv)
- 6개 노드 + 2개 launch + setup + m7_ik: py_compile 통과 / box1.sdf XML 유효
- 3DOF IK: y=0 발산, y≥0.20 수렴 / 폴백·FSM·플래너 로직 단독 검증
- 문서 4종 docx validate + 시각 QA
- ⚠️ ROS2 통신·Gazebo 스폰/텔레포트/Nav2·SLAM 실행 검증은 컨테이너에서만 가능

## 주의 (기준)
- 평가 루브릭/합격선 수치는 모두 **예시** — 기관·차수·팀 수준에 맞게 조정.
- 파지 = 텔레포트(방식 A) → 물리 팔 불필요. 물리방식(B)은 별도 URDF+SDF 선언.
- groq 모델(openai/gpt-oss-20b)은 변동 가능 → `GROQ_MODEL`로만 관리.
- 월드명/박스명은 실제 시뮬과 일치해야 동작 (`ign topic -l`로 확인).
- 실연동 시 SLAM(또는 localization)을 반드시 함께 기동해야 map 프레임 생성 (T3').
