# 🤖 ND1 Capstone Project: Physical AI Simulation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/) [![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange.svg)](https://classic.gazebosim.org/) [![Docker](https://img.shields.io/badge/Docker-Supported-2496ED.svg)](https://www.docker.com/)

본 프로젝트는 LLM을 활용한 자율주행 및 작업 스케줄링 시뮬레이션입니다. 자연어 명령을 기반으로 복합적인 목표를 분석하고, TurtleBot3 로봇이 Gazebo 병원 맵 환경에서 네비게이션 및 Pick & Place 작업을 자율적으로 수행하도록 구성되어 있습니다.

## 📑 목차
1. [시스템 아키텍처](#1-시스템-아키텍처)
2. [시작하기 (Quick Start)](#2-시작하기-quick-start)
3. [🚨 주요 트러블슈팅 및 버그 픽스 (Troubleshooting)](#3--주요-트러블슈팅-및-버그-픽스-troubleshooting)
4. [안전장치 및 제약 조건](#4-안전장치-및-제약-조건)

---

## 1. 시스템 아키텍처

로봇의 자율주행, 파지(Grasp), 복합 미션 스케줄링을 위해 4개의 핵심 노드가 유기적으로 통신합니다.

```text
/llm_command ─▶ Node A(LLM+폴백) ─/mission─▶ Coordinator(FSM)
                                              │  /nav_request   ─▶ Node B(Nav2)
                                              │  /grasp_request ─▶ Node C(IK)
                                              └─ /robot_status (상태 모니터링)
