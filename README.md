# 🤖 ND1 Capstone Project: Physical AI Simulation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/) [![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange.svg)](https://classic.gazebosim.org/) [![Docker](https://img.shields.io/badge/Docker-Supported-2496ED.svg)](https://www.docker.com/)

본 프로젝트는 LLM을 활용한 자율주행 및 작업 스케줄링 시뮬레이션입니다. 자연어 명령을 기반으로 복합적인 목표를 분석하고, TurtleBot3 로봇이 Gazebo 병원 맵 환경에서 네비게이션 및 Pick & Place 작업을 자율적으로 수행하도록 4개의 노드(LLM, Nav2, IK Grasp, Coordinator)가 유기적으로 통합되어 있습니다.

## 📑 목차
1. [Phase 1: 인프라 구축 및 환경 설정](#-phase-1-인프라-구축-및-환경-설정)
2. [Phase 2: 가상 물리 환경 패치](#-phase-2-가상-물리-환경-패치)
3. [Phase 3: ROS2 파이프라인 및 실행 가이드](#-phase-3-ros2-파이프라인-및-실행-가이드)
4. [Phase 4: 아키텍처 및 런타임 제약사항](#-phase-4-아키텍처-및-런타임-제약사항)

---

## 🛠️ Phase 1: 인프라 구축 및 환경 설정
> **의존성:** 컨테이너 기동 전 호스트 환경(GPU, X11, API 키)이 완벽히 구성되어야 합니다.

### 1. 호스트 요구 사항
* **OS**: Ubuntu 22.04
* **GPU**: NVIDIA GeForce RTX 5070 (VRAM 12GB 이상 권장)
* **디스플레이**: `DISPLAY=:1`
* **의존성**: Docker (`docker-compose v2`)

### 2. 초기 셋업 명령어
```bash
# 1. 환경 변수 및 API 키 셋업 (누락 시 LLM 폴백 파서로 작동)
cp .env.example .env
# .env 파일에 GROQ_API_KEY 입력 필수

# 2. 호스트 X11 디스플레이 권한 허용 (1회 필수)
xhost +local:docker

# 3. 도커 컨테이너 빌드 및 백그라운드 기동
docker compose up --build -d
