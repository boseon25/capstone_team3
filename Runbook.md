# 🛠️ ND1 Capstone Troubleshooting Runbook

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/) [![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange.svg)](https://classic.gazebosim.org/) [![Docker](https://img.shields.io/badge/Docker-Supported-2496ED.svg)](https://www.docker.com/)

> **최종 갱신일:** 2026-07-03  
> **문서 목적:** 시뮬레이션 및 실연동 간 발생한 장애 증상(Symptom) 기반 원인 파악 및 조치 가이드

---

## 📑 목차
1. [Phase 1. 인프라 및 호스트 환경 오류](#phase-1-인프라-및-호스트-환경-오류)
2. [Phase 2. 가상 물리 엔진 구동 및 매핑 오류](#phase-2-가상-물리-엔진-구동-및-매핑-오류)
3. [Phase 3. ROS2 프레임워크 및 파이프라인 에러](#phase-3-ros2-프레임워크-및-파이프라인-에러)
4. [Phase 4. 노드 통합 및 시나리오 런타임 제약](#phase-4-노드-통합-및-시나리오-런타임-제약)
5. [Phase 5. 시나리오 구현 맵 선정과 SLAM 맵 생성](#Phase-5-시나리오-구현-맵-선정과-slam-맵-생성)
6. [📌 부록: 물리 환경 확정 구역 좌표](#-부록-물리-환경-확정-구역-좌표)
7. [📅 작업 로그 (Daily Log)](#-작업-로그-daily-log)

---

## Phase 1. 인프라 및 호스트 환경 오류

### 1-1. LLM 노드 가동 및 명령 파싱 불가
* **발생 원인:** 시스템 환경 변수 내 API 키 누락.
* **조치 방안:** `cp .env.example .env` 명령어를 통해 환경 변수 파일을 생성하고 직접 설정 값을 주입.

### 1-2. 시각화 툴(Gazebo/RViz) 출력 에러
* **발생 원인:** 도커 내부 애플리케이션이 호스트 화면에 창을 띄울 디스플레이 권한을 확보하지 못함.
* **조치 방안:** 호스트 부팅 후 도커 실행 전에 터미널에서 `xhost +local:docker` 명령어를 1회 필수 실행.

### 1-3. 시뮬레이션 연산 지연 및 자율주행 정지 (시스템 뻗음)
* **발생 원인:** 고해상도 메쉬 파일이 다수 포함된 `hospital.world` 맵 구동 시, 연산을 CPU 기반의 소프트웨어 렌더링에 의존하여 부하 발생.
* **조치 방안:** GPU 가속 활용을 위해 데스크탑 환경으로 이관. `docker-compose.yml`에서 `LIBGL_ALWAYS_SOFTWARE=0` 속성으로 가속을 켜고, NVIDIA 그래픽 디바이스 맵핑 및 X11 소켓 공유를 추가하여 지연 없는 그래픽 화면 확보.

---

## Phase 2. 가상 물리 엔진 구동 및 매핑 오류

### 2-1. 로봇 스폰 직후 지면 침하 및 뒤집힘
* **발생 원인:** `spawn_turtlebot3.launch.py` 내 기본 스폰 Z축 고도가 0.01로 잡혀 있어 병원 맵 바닥 메쉬와 물리적 충돌 발생.
* **조치 방안:** 컨테이너 기동 후, 가제보 실행 전에 다음 `sed` 명령어를 실행하여 고도를 0.5로 강제 패치. 약간의 공중에서 안전하게 안착시킴으로써 정상적인 오도메트리 확보 가능.
```bash
sed -i "s/'-z', '0.01'/'-z', '0.5'/" /opt/ros/humble/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
```
* **주의 사항:** 해당 패치는 컨테이너를 새로 빌드하거나 재생성하면 초기화되므로 반드시 런치 전 재실행해야 함.

### 2-2. 대규모 맵 자동 탐색 시 로봇 전복 및 SLAM 맵 유실 *(07.03 update)
* **발생 상황:** AWS Hospital World 맵에서 야간 장시간 무인 `turtlebot3_drive` 노드를 이용해 자동 탐색(Auto-Explore)을 지시했으나 맵 생성 실패.
* **발생 원인:**
  1. 병원 맵과 같이 문턱, 좁은 통로가 많은 환경에서는 자동 탐색 알고리즘이 쉽게 한계에 부딪힘.
  2. 로봇이 메쉬 가장자리와 반복 충돌하며 물리량이 누적되어 전복(뒤집힘)되거나 비정상적인 회전 무한 루프에 빠짐.
  3. 로봇 전복 시 LiDAR 센서의 Z축이 틀어지며 허공을 스캔하게 되고, 누적되던 오도메트리(Odometry)와 맵 데이터가 완전히 붕괴됨.
* **조치 방안 (사전 제작 맵 활용 우회):**
  1. 직접 맵을 생성하는 SLAM 방식을 과감히 폐기하고, 오픈소스로 검증된 2D 맵 파일을 차용함.
  2. 맵 파일 확보: GitHub 오픈소스 저장소(예: `Docencia-fmrico/plansys2-hospital-l4ros2`)에서 `hospital_map.yaml` 및 `.pgm` 맵 파일 쌍 다운로드.
  3. 프로젝트 내부 `workspace/nd1_capstone/maps/` 경로에 파일 배치.
  4. 기존 SLAM 기동 인자 대신, AMCL(Localization) 기반 위치 추정 인자로 런치 파일을 변경하여 스택 기동.
```bash
ros2 launch nd1_capstone bringup.launch.py \
  sim_mode:= false \
  localization:= true \
  map:=/home/ubuntu/ros2_ws/src/nd1_capstone/maps/hospital_map.yaml \
  nav2:= true
```

---

## Phase 3. ROS2 프레임워크 및 파이프라인 에러

### 3-1. 런타임 하위 파일 로드 실패 (경로 증발)
* **발생 원인:** ROS2 Humble 환경 고유의 버그로, 다중 include 수행 시 `PathJoinSubstitution`을 사용하면 런타임에 경로가 빈 문자열로 처리됨.
* **조치 방안:** 파이썬 환경 레벨에서 `get_package_share_directory()` 메서드를 이용해 절대 경로를 추출, 상수로 고정하여 사용.

### 3-2. `FileNotFoundError` 및 파이썬 `NameError`
* **발생 원인:** 외부 노드가 `params_file`을 빈 값으로 오염시키거나, 불리언(Boolean) 타입의 문자열이 덮어씌워지며 파이썬 내 `eval()` 함수 처리 중 에러 발생.
* **조치 방안:** SLAM 및 Nav2 실행 시 인자로 파일 경로와 함께 매개변수(`slam='False'`, `use_composition='True'`)를 명시적으로 선언.

### 3-3. 로봇 이동 경로 탐색 실패 및 Costmap 차단
* **발생 원인:** Nav2 단독 기동으로 인해 기준 좌표인 `map` 프레임 공급처가 부재함. (`Invalid frame ID "map"` 에러 발생)
* **조치 방안:** 스택 기동 시 SLAM (또는 AMCL 위치추정) 노드를 필수적으로 동반 기동하여 `map → odom → base_link` TF 변환 체인을 구성.

---

## Phase 4. 노드 통합 및 시나리오 런타임 제약

### 4-1. LLM 텍스트 파싱 깨짐 현상 (Node A)
* **발생 원인:** 프롬프트 결괏값이 순수 배열(`[...]`) 형태로 반환될 경우 다른 자연어 텍스트와 섞이면 파서가 붕괴됨.
* **조치 방안:** 반환 포맷을 반드시 `{"missions": [...]}` 객체 형태로 제한.
* **대체 방안(Fallback):** 네트워크 단절이나 API 키 누락 상황 발생 시 시스템이 뻗지 않고, 한국어 키워드 및 정규식 기반의 경량 폴백 파서로 자동 전환되도록 안전장치 구현.

### 4-2. 3DOF 기구학(IK) 수식 발산 방지 (Node C)
* **발생 원인:** 파지(Grasp) 연산 중 순수 x축 직선상인 Y=0 좌표가 요청되면 DLS IK 수식이 발산하여 특이점(Singularity) 에러 발생.
* **조치 방안:** 소스 코드 내에서 y-offset이 무조건 `0.20` 이상으로 유지되도록 자동 클램프(Clamp)를 적용. 6~8회 반복 연산 내 안정적인 수치 수렴 보장.

### 4-3. 미션 중복 투입 시 상태 기계(FSM) 엉킴 (Coordinator)
* **발생 원인:** 고난도 복합 명령 수행 중(예: 물건을 집고 이동 중) 새로운 미션 토픽이 치고 들어오면 상태 기계가 꼬임.
* **조치 방안:** `_busy` 상태 플래그를 도입하여 `IDLE → NAVIGATING → GRASPING → TRANSPORTING → PLACING → DONE` 하나의 온전한 사이클이 완결될 때까지 외부 개입을 하드웨어적으로 차단.

---

## Phase 5. 시나리오 구현 맵 선정과 SLAM 맵 생성

### 5-1. 테스트 맵 서칭 및 선정
* **후보군 검토:** AWS Hospital World, Simple Colored Warehouse, Jetty World
* **최종 선정:** 복잡도와 시나리오 가정을 고려하여 `aws-robomaker-hospital-world` 채택

* ### 5-2. 자동 탐색(Auto-Explore) 연동 테스트
* TurtleBot3 Waffle_pi 모델을 Gazebo와 RViz에 연동하여 맵 자동 탐색 시도
* **실행 명령어:**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_gazebo turtlebot3_drive
```
* **이슈 식별 1 (맵 스케일 문제):** 병원 맵의 규모가 기존 환경 대비 거대하여, RViz 격자(Grid) 크기 설정과 터틀봇 LiDAR 센서 인지 범위를 대폭 상향 조정함. 환경 로드(맵+로봇 스폰)까지는 완벽히 성공.
* **이슈 식별 2 (로봇 제자리 회전 현상):** 자동 탐색 명령 하달 시, 가상 환경 내 로봇이 전진하지 못하고 y축 기준으로 제자리에서 회전하는 이상 행동(Recovery Behavior)발생. RViz상 이동 지시도 정상 수행 실패.
* **조치 결과:** 내비게이션 복구 동작 무한 루프 문제로 판단하여, 자율 탐색 프로세스를 중단하고 키보드를 이용한 수동(Teleop) 조작으로 전환하여 맵 탐색을 계속 진행함.

---

## 📌 부록: 물리 환경 확정 구역 좌표
파지 및 이동 타겟 시 참조하는 맵(Gazebo) 기준 절대 좌표입니다.

| 구역 (Zone) | X 좌표 | Y 좌표 |
| :---: | :---: | :---: |
| **A** | -2.0 | 1.2 |
| **B** | -1.5 | -1.0 |
| **C** | -1.0 | -2.2 |

---

## 📅 작업 로그 (Daily Log)

### 2026-07-02
1. **테스트 맵 서칭 및 선정**
    * 후보군 검토: AWS Hospital World, Simple Colored Warehouse, Jetty World
    * 최종 선정: 복잡도와 실용성을 고려하여 `aws-robomaker-hospital-world` 채택
2. **자동 탐색 (Auto-Explore) 연동 및 한계 식별**
    * TurtleBot3 Waffle_pi 모델과 `turtlebot3_drive` 노드를 이용해 자동 맵핑 시도.
    * 맵 스케일 문제로 RViz 격자 크기 및 LiDAR 범위 대폭 상향 조정.

### 2026-07-03
1. **Auto-Explore을 통한 SLAM 생성 중 에러 발생**
    * **이슈:** 장시간 탐색 과정에서 로봇이 장애물과 충돌 후 제자리 회전 무한 루프 및 물리적 전복 현상 발생. 기존 맵 데이터(Odometry) 완전 붕괴.
    * **결과:** SLAM 기반의 맵 자체 생성을 중단하고, 사전 제작된 2D 맵 파일(`.yaml`, `.pgm`)을 Nav2(AMCL)에 덮어씌워 사용하는 우회 방식으로 아키텍처 노선 변경.
