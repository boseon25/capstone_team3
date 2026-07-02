#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  [상 난이도] LLM 멀티미션 플래너 스모크 테스트
#  /goal_command 주입 → llm_planner가 미션을 큐에 넣고 순차 실행
#  → 모든 DONE 수신 or 타임아웃/FAILED
#
#  사전조건:
#    bringup.launch.py sim_mode:=true use_planner:=true 실행 중
#
#  사용:
#    python3 smoke_test_planner.py                        # 기본: A→B 1건
#    python3 smoke_test_planner.py "A구역과 B구역을 C구역으로" 2 60
#    인자: <목표문자열> <예상미션수> <타임아웃초>
#  종료코드: PASS=0, FAIL=1
# ════════════════════════════════════════════════════════════════
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SmokePlanner(Node):
    def __init__(self, goal: str, expected_missions: int, timeout: float):
        super().__init__("smoke_test_planner")
        self.goal = goal
        self.expected = expected_missions
        self.timeout = timeout
        self.done_count = 0
        self.failed = False
        self._sent = False
        self.start = time.time()
        self.last = ""
        self.create_subscription(String, "/robot_status", self._on_status, 10)
        self.pub = self.create_publisher(String, "/goal_command", 10)
        self.create_timer(0.5, self._tick)

    def _tick(self):
        elapsed = time.time() - self.start
        if not self._sent and elapsed > 2.0:
            self.pub.publish(String(data=self.goal))
            self.get_logger().info(f"목표 발행: {self.goal} (예상 미션 {self.expected}건)")
            self._sent = True
        if elapsed > self.timeout:
            self.get_logger().error(f"타임아웃 — DONE {self.done_count}/{self.expected}")
            self.failed = True

    def _on_status(self, msg: String):
        s = msg.data
        if s != self.last:
            print(" ", s)
        self.last = s
        if "[FSM:" in s and "DONE" in s:
            self.done_count += 1
            self.get_logger().info(f"DONE {self.done_count}/{self.expected}")
        if "[FSM:" in s and "FAILED" in s:
            self.get_logger().error("FSM FAILED")
            self.failed = True

    @property
    def passed(self):
        return self.done_count >= self.expected and not self.failed


def main():
    goal = sys.argv[1] if len(sys.argv) > 1 else "A구역 박스를 B구역으로 옮겨줘"
    expected = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0

    rclpy.init()
    node = SmokePlanner(goal, expected, timeout)
    while rclpy.ok() and not node.passed and not node.failed:
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = node.passed
    print(f"\nSMOKE TEST (planner): {'PASS' if ok else 'FAIL'} "
          f"— DONE {node.done_count}/{expected}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
