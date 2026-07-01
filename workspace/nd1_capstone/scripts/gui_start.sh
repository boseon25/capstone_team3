#!/bin/bash
export DISPLAY=:1
pkill -f 'nd1_capstone|rviz2|gzserver|gzclient|slam_toolbox' 2>/dev/null
sleep 1

xterm -geometry 130x38+0+0 -bg '#0d1117' -fg '#39d353' -fa 'Monospace' -fs 10 \
  -title '[1] Gazebo+SLAM+Nav2+4노드+RViz' \
  -e bash /home/ubuntu/term1_launch.sh &

sleep 2

xterm -geometry 90x22+0+700 -bg '#0d1117' -fg '#ffa657' -fa 'Monospace' -fs 10 \
  -title '[2] 테스트 터미널' \
  -e bash /home/ubuntu/term2_test.sh &

echo "GUI 기동 완료"
