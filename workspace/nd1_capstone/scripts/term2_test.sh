#!/bin/bash
export DISPLAY=:1
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== [2] Capstone 테스트 터미널 ==="
echo ""
echo "Gazebo 기동 대기 중... (약 30~60초)"
echo ""
echo "  [하/중] 단일 미션:"
echo "  python3 ~/ros2_ws/src/nd1_capstone/scripts/smoke_test.py"
echo ""
echo "  [상] 멀티미션 플래너:"
echo "  python3 ~/ros2_ws/src/nd1_capstone/scripts/smoke_test_planner.py 'A구역과 B구역을 C구역으로' 2 120"
echo ""
echo "  [수동]:"
echo "  ros2 topic pub --once /goal_command std_msgs/String '{data: \"A구역 박스를 B구역으로\"}'"
echo ""
exec bash
