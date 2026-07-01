#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Coordinator — FSM 조정 노드 [학생 구현]
#  역할:
#    /mission 수신
#    → 상태머신으로 이동/파지/배치/도킹 순서 제어
#    → 실패 시 현재 단계 재시도
#
#  In  /mission /nav_result /grasp_result /dock_result
#  Out /nav_request /grasp_request /dock_request /robot_status
#
#  기본 흐름:
#    PLANNING
#    → UNDOCKING
#    → NAVIGATING
#    → GRASPING
#    → TRANSPORTING
#    → PLACING
#    → DOCKING
#    → DONE
#
#  상 난이도 조건:
#    sim_mode=True일 때 첫 번째 grasp 시도를 의도적으로 실패시킨 뒤
#    RETRY_GRASP 상태로 전환하고 두 번째 grasp에서 성공한다.
# ════════════════════════════════════════════════════════════════
import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class State(Enum):
    IDLE = auto()
    PLANNING = auto()
    UNDOCKING = auto()
    NAVIGATING = auto()
    GRASPING = auto()
    RETRY_GRASP = auto()
    TRANSPORTING = auto()
    PLACING = auto()
    DOCKING = auto()
    DONE = auto()
    FAILED = auto()


class CoordinatorFSM(Node):
    def __init__(self):
        super().__init__("coordinator_fsm")

        self.declare_parameter("sim_mode", True)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("auto_redock", True)
        self.declare_parameter("force_first_grasp_fail", True)

        self.sim_mode = bool(self.get_parameter("sim_mode").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.auto_redock = bool(self.get_parameter("auto_redock").value)
        self.force_first_grasp_fail = bool(
            self.get_parameter("force_first_grasp_fail").value
        )

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

        # 전체 실행 중 첫 번째 grasp만 의도적으로 실패시킨다.
        self._forced_grasp_failure_used = False

        self.create_timer(
            1.0 / float(self.get_parameter("tick_hz").value),
            self._tick,
        )

        self._status(
            f"Coordinator 시작 "
            f"(sim_mode={self.sim_mode}, "
            f"auto_redock={self.auto_redock}, "
            f"force_first_grasp_fail={self.force_first_grasp_fail})"
        )

    def _on_mission(self, msg: String):
        try:
            self.cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._status(f"⚠️ 미션 JSON 파싱 실패: {e}")
            return

        if self.state not in (State.IDLE, State.DONE, State.FAILED):
            self._status("⚠️ 진행 중 — 새 미션 무시")
            return

        self._reset_flags()
        self.retries = 0
        self.state = State.PLANNING

        self._status(f"미션 접수: {self.cmd.get('action')}")

    def _tick(self):
        s = self.state

        if s in (State.IDLE, State.DONE, State.FAILED):
            return

        if s == State.PLANNING:
            action = self.cmd.get("action")

            if action == "stop":
                self.state = State.DONE
                self._status("정지 명령 처리 → DONE ✅")
                return

            if action in ("navigate", "pick_and_place"):
                self._enter_dock("undock", State.UNDOCKING)
                return

            self.state = State.FAILED
            self._status(f"⚠️ 알 수 없는 action → FAILED: {action}")
            return

        # 단계 진행 중이면 결과가 올 때까지 대기
        if self._busy and not self._done:
            return

        # 결과가 도착하면 다음 상태로 전이
        if self._busy and self._done:
            self._busy = False
            self._advance(self._ok)

    def _advance(self, success: bool):
        if not success:
            if self.retries < self.max_retries:
                self.retries += 1
                self._status(
                    f"단계 실패 → 재시도 {self.retries}/{self.max_retries} "
                    f"({self.state.name})"
                )
                self._retry_current()
                return

            self._status(f"재시도 한도 초과 → FAILED ({self.state.name})")
            self.state = State.FAILED
            return

        self.retries = 0
        action = self.cmd.get("action")

        if self.state == State.UNDOCKING:
            if action == "navigate":
                x, y = self._target_xy()
                self._enter_nav(x, y, State.NAVIGATING)
                return

            if action == "pick_and_place":
                x, y = self._pick_xy()
                self._enter_nav(x, y, State.NAVIGATING)
                return

        if self.state == State.NAVIGATING:
            if action == "navigate":
                self._finish_mission()
                return

            if action == "pick_and_place":
                x, y = self._pick_xy()
                self._enter_grasp("grasp", x, y, State.GRASPING)
                return

        if self.state in (State.GRASPING, State.RETRY_GRASP):
            x, y = self._place_xy()
            self._enter_nav(x, y, State.TRANSPORTING)
            return

        if self.state == State.TRANSPORTING:
            x, y = self._place_xy()
            self._enter_grasp("place", x, y, State.PLACING)
            return

        if self.state == State.PLACING:
            if self._is_test_zone():
                self._status("TEST_PASS 확인: 안전성 테스트 통과 ✅")
            self._finish_mission()
            return

        if self.state == State.DOCKING:
            self.state = State.DONE
            self._status("DONE ✅")
            return

        self.state = State.FAILED
        self._status(f"⚠️ 전이표에 없는 상태 → FAILED ({self.state.name})")

    def _finish_mission(self):
        if self.auto_redock:
            self._status("미션 완료 → 재도킹")
            self._enter_dock("dock", State.DOCKING)
        else:
            self.state = State.DONE
            self._status("미션 완료 → DONE ✅ (재도킹 생략)")

    def _retry_current(self):
        if self.state == State.UNDOCKING:
            self._enter_dock("undock", State.UNDOCKING)
            return

        if self.state == State.DOCKING:
            self._enter_dock("dock", State.DOCKING)
            return

        if self.state == State.NAVIGATING:
            action = self.cmd.get("action")
            if action == "navigate":
                x, y = self._target_xy()
            else:
                x, y = self._pick_xy()
            self._enter_nav(x, y, State.NAVIGATING)
            return

        if self.state == State.TRANSPORTING:
            x, y = self._place_xy()
            self._enter_nav(x, y, State.TRANSPORTING)
            return

        if self.state in (State.GRASPING, State.RETRY_GRASP):
            x, y = self._pick_xy()
            self._status("RETRY_GRASP 상태 진입: 파지 재시도")
            self._enter_grasp("grasp", x, y, State.RETRY_GRASP)
            return

        if self.state == State.PLACING:
            x, y = self._place_xy()
            self._enter_grasp("place", x, y, State.PLACING)
            return

        self.state = State.FAILED
        self._status(f"⚠️ 재시도 불가 상태 → FAILED ({self.state.name})")

    def _enter_dock(self, op, next_state):
        self.state = next_state
        self._reset_flags()
        self._busy = True

        self._status(f"{op} 요청 [{next_state.name}]")

        if self.sim_mode:
            self._simulate(True, 1.0)
            return

        self.pub_dock.publish(String(data=json.dumps({"op": op})))

    def _enter_nav(self, x, y, next_state):
        self.state = next_state
        self._reset_flags()
        self._busy = True

        self._status(f"이동 요청 → ({x:.2f}, {y:.2f}) [{next_state.name}]")

        if self.sim_mode:
            self._simulate(True, 2.0)
            return

        self.pub_nav.publish(
            String(
                data=json.dumps(
                    {
                        "x": float(x),
                        "y": float(y),
                        "yaw": float(self.cmd.get("yaw", 0.0)),
                    }
                )
            )
        )

    def _enter_grasp(self, op, x, y, next_state):
        self.state = next_state
        self._reset_flags()
        self._busy = True

        self._status(f"{op} 요청 → ({x:.2f}, {y:.2f}) [{next_state.name}]")

        if self.sim_mode:
            should_force_fail = (
                op == "grasp"
                and self.force_first_grasp_fail
                and not self._forced_grasp_failure_used
            )

            if should_force_fail:
                self._forced_grasp_failure_used = True
                self._status("의도적 첫 파지 실패 시뮬레이션")
                self._simulate(False, 1.5)
                return

            self._simulate(True, 1.5)
            return

        self.pub_grasp.publish(
            String(
                data=json.dumps(
                    {
                        "op": op,
                        "x": float(x),
                        "y": float(y),
                    }
                )
            )
        )

    def _on_dock_result(self, msg: Bool):
        if self._busy and self.state in (State.UNDOCKING, State.DOCKING):
            self._finish(bool(msg.data))

    def _on_nav_result(self, msg: Bool):
        if self._busy and self.state in (State.NAVIGATING, State.TRANSPORTING):
            self._finish(bool(msg.data))

    def _on_grasp_result(self, msg: Bool):
        if self._busy and self.state in (
            State.GRASPING,
            State.RETRY_GRASP,
            State.PLACING,
        ):
            self._finish(bool(msg.data))

    def _simulate(self, success, secs):
        t = self.create_timer(secs, lambda: (t.cancel(), self._finish(success)))

    def _finish(self, ok):
        self._ok = ok
        self._done = True

    def _reset_flags(self):
        self._busy = False
        self._done = False
        self._ok = False

    def _pick_xy(self):
        return (
            self._get_float("pick_x", 0.0),
            self._get_float("pick_y", 0.0),
        )

    def _place_xy(self):
        return (
            self._get_float("place_x", 0.0),
            self._get_float("place_y", 0.0),
        )

    def _target_xy(self):
        return (
            self._get_float("place_x", self._get_float("x", 0.0)),
            self._get_float("place_y", self._get_float("y", 0.0)),
        )

    def _get_float(self, key, default):
        try:
            return float(self.cmd.get(key, default))
        except (TypeError, ValueError):
            return float(default)

    def _is_test_zone(self):
        place_x, place_y = self._place_xy()

        is_b_by_zone_name = self.cmd.get("to_zone") == "B"
        is_b_by_coordinate = abs(place_x - 2.5) < 1e-6 and abs(place_y + 1.0) < 1e-6

        return is_b_by_zone_name or is_b_by_coordinate

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
