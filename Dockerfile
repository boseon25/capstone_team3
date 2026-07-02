# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 통합 실습 환경
#  베이스: ROS2 Humble + noVNC (브라우저 데스크탑) — Windows 학생용
#  기준: Ubuntu 22.04 / ROS2 Humble / Gazebo Classic 11 / Python 3.10
#  (2026-07 TurtleBot4/Ignition-Fortress → TurtleBot3/Gazebo-Classic 마이그레이션:
#   WSL2+GPU(d3d12) 환경에서 Ignition GPU LIDAR가 깨진 값을 반환하는 렌더링 버그를
#   회피하기 위해 Classic Gazebo11 기반 TurtleBot3 스택으로 전환. 상세: WSLg_렌더_가이드.md)
# ════════════════════════════════════════════════════════════════
FROM tiryoh/ros2-desktop-vnc:humble

USER root
SHELL ["/bin/bash", "-c"]

# ── 1~2. (구) Gazebo Classic 제거 + Fortress(Ignition) 설치 — 더 이상 사용 안 함 ──
#    베이스 이미지가 기본 제공하는 Gazebo Classic 11을 그대로 사용.

# ── 3. Nav2 + TurtleBot3 (Gazebo Classic 기반) ────────────────────
#    turtlebot3-*, gazebo-ros-pkgs가 apt에 없으면 turtlebot3_simulations
#    (humble-devel 소스)를 colcon build 하는 폴백이 필요할 수 있음.
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-navigation2

ENV TURTLEBOT3_MODEL=burger

# ── 4. Foxglove + 분석 라이브러리 (M13 연계) ──────────────────────
RUN apt-get install -y ros-humble-foxglove-bridge python3-pip && \
    pip3 install --no-cache-dir matplotlib numpy scipy rosbags

# ── 5. Node A(LLM) 의존성 + 테스트 도구 ───────────────────────────
RUN pip3 install --no-cache-dir groq pydantic python-dotenv pytest

# ── 6. M7 IK 코어 (Node C가 nd1_m7_ik 임포트) ─────────────────────
COPY m7_ik /opt/nd1/m7_ik
RUN pip3 install --no-cache-dir /opt/nd1/m7_ik && \
    python3 -c "from nd1_m7_ik import RobotArm3DOF, jacobian_analytical_3dof, manipulability; print('nd1_m7_ik OK')"

# ── 7. 사용자 / 워크스페이스 ──────────────────────────────────────
RUN id -u ubuntu &>/dev/null || \
    (useradd -m -s /bin/bash ubuntu && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers)
RUN mkdir -p /home/ubuntu/ros2_ws/src && chown -R ubuntu:ubuntu /home/ubuntu

# ── 8. ★ Windows/WSL2 안정 기동 — 소프트웨어 렌더링 기본값 ─────────
#    WSL 가상 GPU의 OpenGL 미구현으로 인한 Ogre GL3Plus 크래시를 원천 차단.
#    (가속이 필요한 네이티브 Linux 학생은 compose override로 이 값을 끄면 됨)
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV QT_X11_NO_MITSHM=1

# Node A 기본 LLM 모델 (2026-06-17 llama-3.x 계열 deprecated → gpt-oss 권장)
ENV GROQ_MODEL=openai/gpt-oss-20b

# ⚠️ USER ubuntu 라인 없음 — 베이스 시작 스크립트(supervisord)가 root여야 VNC 정상 구동
WORKDIR /home/ubuntu/ros2_ws
