#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  [난이도 하 ★☆☆] 선형 오케스트레이터 [학생 구현]
#  Coordinator(FSM)의 단순화 버전 — 상태머신/재시도 없이 순차 실행만.
#  토픽 계약 동일 → Node A/B/C 그대로 사용, coordinator_fsm 과 교체 가능.
#
#  제공: pub/sub, sim 단발 처리 골격, main
#  구현(TODO): ① _on_mission (단계 리스트 구성) ② _next (단계 실행)
#  ※ 하 난이도는 undock/dock 미포함 → 실연동 시 수동 undock.
# ════════════════════════════════════════════════════════════════
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class LinearOrchestrator(Node):
    def __init__(self):
        super().__init__("linear_orchestrator")
        self.declare_parameter("sim_mode", True)
        self.sim_mode = self.get_parameter("sim_mode").value
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.pub_nav = self.create_publisher(String, "/nav_request", 10)
        self.pub_grasp = self.create_publisher(String, "/grasp_request", 10)
        self.create_subscription(String, "/mission", self._on_mission, 10)
        # 실연동(sim_mode=False)에서만 결과 토픽 구독 — sim에선 타이머로 자체 진행
        if not self.sim_mode:
            self.create_subscription(Bool, "/nav_result", lambda m: self._next(m.data), 10)
            self.create_subscription(Bool, "/grasp_result", lambda m: self._next(m.data), 10)
        self.steps = []     # [("nav"|"grasp", op, x, y), ...]
        self.idx = -1
        self._busy = False   # 현재 단계 진행 중 — 중복 진입 방지
        self._status(f"선형 오케스트레이터 시작 (sim_mode={self.sim_mode})")

    # ── ① TODO: 미션 → 단계 리스트 ───────────────────────────────
    def _on_mission(self, msg: String):
        """RobotCommand JSON → self.steps 구성 후 _next(True) 로 시작.
        힌트:
          c = json.loads(msg.data); a = c.get("action")
          - navigate      → [("nav","",place_x,place_y)]
          - pick_and_place→ [("nav","",pick..),("grasp","grasp",pick..),
                             ("nav","",place..),("grasp","place",place..)]
          - 그 외 → steps=[] 후 return
          마지막에 self.idx=-1; self._next(True)
        """
        try:
            c = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._status(f"⚠️ 미션 JSON 파싱 실패: {e}")
            return
        a = c.get("action")
        px, py = float(c.get("pick_x", 0.0)), float(c.get("pick_y", 0.0))
        lx, ly = float(c.get("place_x", 0.0)), float(c.get("place_y", 0.0))
        if a == "navigate":
            self.steps = [("nav", "", lx, ly)]
        elif a == "pick_and_place":
            self.steps = [
                ("nav", "", px, py),
                ("grasp", "grasp", px, py),
                ("nav", "", lx, ly),
                ("grasp", "place", lx, ly),
            ]
        else:
            self._status(f"알 수 없는 action: {a}")
            self.steps = []
            return
        self.idx = -1
        self._busy = False
        self._status(f"미션 접수: {a}, {len(self.steps)}단계")
        self._next(True)

    # ── ② TODO: 다음 단계 실행 ────────────────────────────────────
    def _next(self, prev_ok: bool):
        """직전 결과가 ok면 다음 단계 발행. (하 난이도: 실패 시 재시도 없이 중단)
        힌트:
          - not prev_ok → 중단 로그 후 return
          - self.idx += 1; 끝이면 'DONE' 로그
          - kind,op,x,y = self.steps[self.idx]
          - sim_mode → self.create_timer(1.5, lambda:self._next(True))
          - nav  → self.pub_nav.publish(String(data=json.dumps({"x":x,"y":y,"yaw":0.0})))
          - grasp→ self.pub_grasp.publish(String(data=json.dumps({"op":op,"x":x,"y":y})))
        """
        if self._busy:
            return  # 이전 단계 타이머/응답 대기 중 — 중복 호출 무시
        if not prev_ok:
            self._status("단계 실패 → 미션 중단")
            return
        self.idx += 1
        if self.idx >= len(self.steps):
            if self.idx == len(self.steps):  # 첫 완료 시에만 출력
                self._status("미션 완료 → DONE ✅")
            return
        kind, op, x, y = self.steps[self.idx]
        self._busy = True
        self._status(f"단계 {self.idx + 1}/{len(self.steps)}: {kind} op={op or '-'} ({x:.2f},{y:.2f})")
        if self.sim_mode:
            def _on_timer():
                t.cancel()
                self._busy = False
                self._next(True)
            t = self.create_timer(1.5, _on_timer)
            return
        self._busy = False  # 실연동: 결과 토픽 구독으로 _next 재진입
        if kind == "nav":
            self.pub_nav.publish(String(data=json.dumps({"x": x, "y": y, "yaw": 0.0})))
        else:
            self.pub_grasp.publish(String(data=json.dumps({"op": op, "x": x, "y": y})))

    def _status(self, t):
        self.get_logger().info(t)
        self.pub_status.publish(String(data=f"[LIN] {t}"))


def main(args=None):
    rclpy.init(args=args)
    node = LinearOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
