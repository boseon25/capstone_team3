#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Node B — 내비게이션 + 도킹 노드 [학생 구현]
#  역할: /nav_request → Nav2 NavigateToPose / /dock_request → Undock·Dock
#
#  제공(인프라): pub/sub, 액션 클라이언트 생성, dock_status 구독, PoseStamped 헬퍼, main
#  구현(TODO):  ① _on_nav (목표 전송)  ② _on_dock (도킹 상태 판단 + 액션 호출)
#
#  토픽/액션 계약(고정):
#    In  /nav_request {x,y,yaw} / /dock_request {op:"undock"|"dock"} / /dock_status
#    Out /nav_result(Bool) / /dock_result(Bool) / /robot_status
#    Action: navigate_to_pose, undock, dock
# ════════════════════════════════════════════════════════════════
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NodeBNav(Node):
    def __init__(self):
        super().__init__("node_b_nav")
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("server_timeout", 5.0)
        self.declare_parameter("undock_action", "undock")
        self.declare_parameter("dock_action", "dock")
        self.declare_parameter("dock_status_topic", "dock_status")
        self.sim_mode = self.get_parameter("sim_mode").value
        self.timeout = self.get_parameter("server_timeout").value

        self.pub_nav_res = self.create_publisher(Bool, "/nav_result", 10)
        self.pub_dock_res = self.create_publisher(Bool, "/dock_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/nav_request", self._on_nav, 10)
        self.create_subscription(String, "/dock_request", self._on_dock, 10)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._active = False
        self._is_docked = None
        self._dock_ready = self._init_dock()
        self._status(f"Node B 시작 (sim_mode={self.sim_mode}, dock={'ON' if self._dock_ready else 'OFF'})")

    # ── ① TODO: 이동 요청 처리 ────────────────────────────────────
    def _on_nav(self, msg: String):
        """/nav_request {x,y,yaw} → Nav2 목표 전송 → 결과를 _nav_result(bool) 로.
        힌트:
          - self._active 진행 중이면 무시
          - JSON 파싱 (실패 시 self._nav_result(False))
          - sim_mode: self.create_timer(2.0, lambda:(t.cancel(), self._nav_result(True)))
          - 실제: self._nav_client.wait_for_server(timeout_sec=self.timeout) 확인
                  goal = NavigateToPose.Goal(); goal.pose = self._pose(x,y,yaw)
                  self._nav_client.send_goal_async(goal).add_done_callback(self._nav_goal_cb)
        """
        if self._active:
            self._status("⚠️ 이동 진행 중 — 요청 무시")
            return
        try:
            d = json.loads(msg.data)
            x, y, yaw = float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0))
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self._status(f"⚠️ nav_request 파싱 실패: {e}")
            self._nav_result(False)
            return

        self._active = True
        if self.sim_mode:
            t = self.create_timer(2.0, lambda: (t.cancel(), self._nav_result(True)))
            return
        if not self._nav_client.wait_for_server(timeout_sec=self.timeout):
            self._status("⚠️ navigate_to_pose 서버 없음")
            self._nav_result(False)
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)
        self._nav_client.send_goal_async(goal).add_done_callback(self._nav_goal_cb)

    def _nav_goal_cb(self, future):
        h = future.result()
        if not h.accepted:
            self._status("⚠️ Nav2 goal 거부"); self._nav_result(False); return
        h.get_result_async().add_done_callback(lambda f: self._nav_result(f.result().status == 4))

    def _nav_result(self, ok):
        self._active = False
        self.pub_nav_res.publish(Bool(data=ok))
        self._status(f"이동 결과: {'성공' if ok else '실패'}")

    # ── ② TODO: 도킹/언도킹 처리 ──────────────────────────────────
    def _on_dock(self, msg: String):
        """/dock_request {op} 처리.
        힌트(요구사항):
          - sim_mode면 무조건 성공: self._dock_result(True)
          - op=="undock":
              * self._is_docked is False → 이미 도크 밖 → self._dock_result(True) (no-op)
              * self._is_docked is None  → 안전상 undock 시도
              * 그 외(True) → self._send_dock_action(self._undock_action, self._UndockGoal())
          - op=="dock": self._send_dock_action(self._dock_action_cli, self._DockGoal())
        """
        try:
            d = json.loads(msg.data)
            op = d.get("op", "undock")
        except json.JSONDecodeError as e:
            self._status(f"⚠️ dock_request 파싱 실패: {e}")
            self._dock_result(False)
            return

        if self.sim_mode:
            self._dock_result(True)
            return

        if op == "undock":
            if self._is_docked is False:
                self._dock_result(True)
            else:
                self._send_dock_action(self._undock_action, self._UndockGoal())
        elif op == "dock":
            self._send_dock_action(self._dock_action_cli, self._DockGoal())
        else:
            self._status(f"⚠️ 알 수 없는 dock op: {op}")
            self._dock_result(False)

    def _send_dock_action(self, client, goal):
        if not self._dock_ready or client is None:
            self._status("⚠️ dock 액션 불가(irobot_create_msgs 미가용)"); self._dock_result(False); return
        if not client.wait_for_server(timeout_sec=self.timeout):
            self._status("⚠️ dock 액션 서버 없음"); self._dock_result(False); return
        client.send_goal_async(goal).add_done_callback(self._dock_goal_cb)

    def _dock_goal_cb(self, future):
        h = future.result()
        if not h.accepted:
            self._status("⚠️ dock goal 거부"); self._dock_result(False); return
        h.get_result_async().add_done_callback(lambda f: self._dock_result(True))

    def _dock_result(self, ok):
        self.pub_dock_res.publish(Bool(data=ok))
        self._status(f"dock 결과: {'성공' if ok else '실패'}")

    def _on_dock_status(self, msg):
        self._is_docked = bool(getattr(msg, "is_docked", False))

    def _init_dock(self):
        try:
            from irobot_create_msgs.action import Undock, Dock
            from irobot_create_msgs.msg import DockStatus
            self._UndockGoal = Undock.Goal
            self._DockGoal = Dock.Goal
            self._undock_action = ActionClient(self, Undock, self.get_parameter("undock_action").value)
            self._dock_action_cli = ActionClient(self, Dock, self.get_parameter("dock_action").value)
            dock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(DockStatus, self.get_parameter("dock_status_topic").value,
                                     self._on_dock_status, dock_qos)
            return True
        except Exception as e:
            self.get_logger().warn(f"irobot_create_msgs 미가용(sim/미설치 가능): {e}")
            self._UndockGoal = self._DockGoal = lambda: None
            self._undock_action = self._dock_action_cli = None
            return False

    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x; p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _status(self, text):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[B] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeBNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
