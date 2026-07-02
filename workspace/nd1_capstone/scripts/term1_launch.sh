#!/bin/bash
export DISPLAY=:1
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || source /usr/share/gazebo-11/setup.sh 2>/dev/null
source /home/ubuntu/ros2_ws/install/setup.bash

echo "=== Gazebo + SLAM + Nav2 + 4노드 + RViz + Planner ==="
ros2 launch nd1_capstone bringup.launch.py \
  sim_mode:=false \
  use_gazebo:=true \
  slam:=true \
  nav2:=true \
  use_planner:=true \
  use_rviz:=true \
  auto_redock:=false

read -p "Press Enter to close..."
