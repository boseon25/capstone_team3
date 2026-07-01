#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  자동 스모크 테스트  (ND1 캡스톤)
#  /llm_command 주입 → /robot_status 가 DONE 도달하면 PASS, 타임아웃/FAILED면 FAIL
#  ※ 반드시 컨테이너에서, bringup.launch.py 실행 중인 상태로 별도 터미널에서 실행.
#
#  사용:
#    python3 smoke_test.py                                  # 기본 명령, 30s
#    python3 smoke_test.py "C구역으로 가" 20                # 명령/타임아웃 지정
#  종료코드: PASS=0, FAIL=1  (CI/체크리스트 자동화용)
# ════════════════════════════════════════════════════════════════
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SmokeTest(Node):
    def __init__(self, cmd: str, timeout: float):
        super().__init__("smoke_test")
        self.cmd = cmd
        self.timeout = timeout
        self.last = ""
        self.passed = False
        self.failed = False
        self._sent = False
        self.start = time.time()
        self.create_subscription(String, "/robot_status", self._on_status, 10)
        self.pub = self.create_publisher(String, "/llm_command", 10)
        self.create_timer(0.5, self._tick)

    def _tick(self):
        elapsed = time.time() - self.start
        if not self._sent and elapsed > 1.5:   # 구독/발행 연결 안정화 후 주입
            self.pub.publish(String(data=self.cmd))
            self.get_logger().info(f"명령 발행: {self.cmd}")
            self._sent = True
        if "DONE" in self.last:
            self.passed = True
        elif "FAILED" in self.last:
            self.failed = True
        elif elapsed > self.timeout:
            self.get_logger().error("타임아웃")
            self.failed = True

    def _on_status(self, msg: String):
        if msg.data != self.last:
            print(" ", msg.data)
        self.last = msg.data


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "A구역 박스를 B구역으로 옮겨줘"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

    rclpy.init()
    node = SmokeTest(cmd, timeout)
    while rclpy.ok() and not node.passed and not node.failed:
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = node.passed and not node.failed
    print("\nSMOKE TEST:", "PASS" if ok else "FAIL")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
