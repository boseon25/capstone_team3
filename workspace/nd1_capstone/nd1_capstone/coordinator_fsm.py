#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Coordinator — FSM 조정 노드 [학생 구현]
#  역할: /mission 수신 → 상태머신으로 이동/파지/도킹을 순서대로 지시
#
#  제공(인프라): 상태 enum, pub/sub, 단계진입 헬퍼(_enter_*), 결과 콜백, sim/재시도 골격
#  구현(TODO):  ① _tick 의 PLANNING 분기  ② _advance 전이표  ③ _retry_current
#
#  토픽 계약(고정):
#    In  /mission /nav_result /grasp_result /dock_result
#    Out /nav_request /grasp_request /dock_request (모두 String JSON) / /robot_status
#  목표 상태흐름(권장): PLANNING→UNDOCKING→NAVIGATING→GRASPING→TRANSPORTING→PLACING→DOCKING→DONE
# ════════════════════════════════════════════════════════════════
import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class State(Enum):
    IDLE = auto(); PLANNING = auto(); UNDOCKING = auto(); NAVIGATING = auto()
    GRASPING = auto(); TRANSPORTING = auto(); PLACING = auto(); DOCKING = auto()
    DONE = auto(); FAILED = auto()


class CoordinatorFSM(Node):
    def __init__(self):
        super().__init__("coordinator_fsm")
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("auto_redock", True)
        self.sim_mode = self.get_parameter("sim_mode").value
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.auto_redock = bool(self.get_parameter("auto_redock").value)

        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.pub_nav = self.create_publisher(String, "/nav_request", 10)
        self.pub_grasp = self.create_publisher(String, "/grasp_request", 10)
        self.pub_dock = self.create_publisher(String, "/dock_request", 10)
        self.create_subscription(String, "/mission", self._on_mission, 10)
        self.create_subscription(Bool, "/nav_result", self._on_nav_result, 10)
        self.create_subscription(Bool, "/grasp_result", self._on_grasp_result, 10)
        self.create_subscription(Bool, "/dock_result", self._on_dock_result, 10)

        self.state = State.IDLE
        self.cmd = None
        self.retries = 0
        self._busy = False
        self._done = False
        self._ok = False
        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self._tick)
        self._status(f"Coordinator 시작 (sim_mode={self.sim_mode}, auto_redock={self.auto_redock})")

    def _on_mission(self, msg: String):
        try:
            self.cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._status(f"⚠️ 미션 JSON 파싱 실패: {e}"); return
        if self.state not in (State.IDLE, State.DONE, State.FAILED):
            self._status("⚠️ 진행 중 — 새 미션 무시"); return
        self._reset_flags(); self.retries = 0
        self.state = State.PLANNING
        self._status(f"미션 접수: {self.cmd.get('action')}")

    # ── ① TODO: 틱 — PLANNING 분기 + busy 대기 ────────────────────
    def _tick(self):
        s = self.state
        if s in (State.IDLE, State.DONE, State.FAILED):
            return
        if s == State.PLANNING:
            action = self.cmd.get("action")
            if action == "stop":
                self.state = State.DONE
            elif action in ("navigate", "pick_and_place"):
                self._enter_dock("undock", State.UNDOCKING)
            else:
                self._status(f"⚠️ 알 수 없는 action: {action}")
                self.state = State.FAILED
            return
        # 단계 진행 중이면 결과 올 때까지 대기 → 결과 오면 _advance 호출 (구현됨)
        if self._busy and not self._done:
            return
        if self._busy and self._done:
            self._busy = False
            self._advance(self._ok)

    # ── ② TODO: 전이표 (성공 시 다음 단계, 실패 시 재시도/FAILED) ──
    def _advance(self, success: bool):
        if not success:
            if self.retries < self.max_retries:
                self.retries += 1
                self._status(f"단계 실패 → 재시도 {self.retries}/{self.max_retries} ({self.state.name})")
                self._retry_current(); return
            self._status(f"재시도 한도 초과 → FAILED ({self.state.name})")
            self.state = State.FAILED; return
        self.retries = 0
        action = self.cmd.get("action")
        s = self.state
        if s == State.UNDOCKING:
            if action == "navigate":
                self._enter_nav(self.cmd.get("place_x", 0.0), self.cmd.get("place_y", 0.0), State.NAVIGATING)
            else:
                self._enter_nav(self.cmd.get("pick_x", 0.0), self.cmd.get("pick_y", 0.0), State.NAVIGATING)
        elif s == State.NAVIGATING:
            if action == "navigate":
                self._finish_mission()
            else:
                self._enter_grasp("grasp", self.cmd.get("pick_x", 0.0), self.cmd.get("pick_y", 0.0), State.GRASPING)
        elif s == State.GRASPING:
            self._enter_nav(self.cmd.get("place_x", 0.0), self.cmd.get("place_y", 0.0), State.TRANSPORTING)
        elif s == State.TRANSPORTING:
            self._enter_grasp("place", self.cmd.get("place_x", 0.0), self.cmd.get("place_y", 0.0), State.PLACING)
        elif s == State.PLACING:
            self._finish_mission()
        elif s == State.DOCKING:
            self.state = State.DONE
            self._status("미션 완료 → DONE ✅")

    def _finish_mission(self):
        if self.auto_redock:
            self._status("미션 완료 → 재도킹")
            self._enter_dock("dock", State.DOCKING)
        else:
            self.state = State.DONE; self._status("미션 완료 → DONE ✅ (재도킹 생략)")

    # ── ③ TODO: 현재 단계 재시도 (해당 _enter_* 재호출) ───────────
    def _retry_current(self):
        action = self.cmd.get("action")
        s = self.state
        if s == State.UNDOCKING:
            self._enter_dock("undock", State.UNDOCKING)
        elif s == State.NAVIGATING:
            if action == "navigate":
                self._enter_nav(self.cmd.get("place_x", 0.0), self.cmd.get("place_y", 0.0), State.NAVIGATING)
            else:
                self._enter_nav(self.cmd.get("pick_x", 0.0), self.cmd.get("pick_y", 0.0), State.NAVIGATING)
        elif s == State.GRASPING:
            self._enter_grasp("grasp", self.cmd.get("pick_x", 0.0), self.cmd.get("pick_y", 0.0), State.GRASPING)
        elif s == State.TRANSPORTING:
            self._enter_nav(self.cmd.get("place_x", 0.0), self.cmd.get("place_y", 0.0), State.TRANSPORTING)
        elif s == State.PLACING:
            self._enter_grasp("place", self.cmd.get("place_x", 0.0), self.cmd.get("place_y", 0.0), State.PLACING)
        elif s == State.DOCKING:
            self._enter_dock("dock", State.DOCKING)

    # ── 제공: 단계 진입 헬퍼 (sim_mode면 성공 시뮬, 아니면 요청 발행) ──
    def _enter_dock(self, op, next_state):
        self.state = next_state; self._reset_flags(); self._busy = True
        self._status(f"{op} 요청 [{next_state.name}]")
        if self.sim_mode:
            self._simulate(True, 1.0); return
        self.pub_dock.publish(String(data=json.dumps({"op": op})))

    def _enter_nav(self, x, y, next_state):
        self.state = next_state; self._reset_flags(); self._busy = True
        self._status(f"이동 요청 → ({x:.2f}, {y:.2f}) [{next_state.name}]")
        if self.sim_mode:
            self._simulate(True, 2.0); return
        self.pub_nav.publish(String(data=json.dumps(
            {"x": float(x), "y": float(y), "yaw": float(self.cmd.get('yaw', 0.0))})))

    def _enter_grasp(self, op, x, y, next_state):
        self.state = next_state; self._reset_flags(); self._busy = True
        self._status(f"{op} 요청 → ({x:.2f}, {y:.2f}) [{next_state.name}]")
        if self.sim_mode:
            self._simulate(True, 1.5); return
        self.pub_grasp.publish(String(data=json.dumps(
            {"op": op, "x": float(x), "y": float(y)})))

    # ── 제공: 결과 콜백 ───────────────────────────────────────────
    def _on_dock_result(self, msg: Bool):
        if self._busy and self.state in (State.UNDOCKING, State.DOCKING):
            self._finish(bool(msg.data))

    def _on_nav_result(self, msg: Bool):
        if self._busy and self.state in (State.NAVIGATING, State.TRANSPORTING):
            self._finish(bool(msg.data))

    def _on_grasp_result(self, msg: Bool):
        if self._busy and self.state in (State.GRASPING, State.PLACING):
            self._finish(bool(msg.data))

    def _simulate(self, success, secs):
        t = self.create_timer(secs, lambda: (t.cancel(), self._finish(success)))

    def _finish(self, ok): self._ok = ok; self._done = True
    def _reset_flags(self): self._busy = False; self._done = False; self._ok = False

    def _status(self, text):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[FSM:{self.state.name}] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
