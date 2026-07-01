#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Node C — 파지/배치 노드 (IK) [학생 구현]
#  역할: /grasp_request → nd1_m7_ik IK 계산 → 텔레포트 파지/배치 → /grasp_result
#
#  제공(인프라): pub/sub, 파라미터, 팔 초기화, main
#  구현(TODO):  ① _solve_ik (nd1_m7_ik 호출)  ② _teleport (ign service)
#
#  토픽 계약(고정):
#    In  /grasp_request {op:"grasp"|"place", x, y}
#    Out /grasp_result(Bool) / /robot_status
#  ★ 표준안 제약: 팔 로컬 파지 타깃 y-offset ≥ 0.20 (y=0 특이점 → IK 발산)
# ════════════════════════════════════════════════════════════════
import json
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

Y_OFFSET_MIN = 0.20  # 특이점 회피 최소 y (표준안) — 변경 금지


class NodeCGrasp(Node):
    def __init__(self):
        super().__init__("node_c_grasp")
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("arm_links", [0.20, 0.18, 0.12])
        self.declare_parameter("grasp_x", 0.35)
        self.declare_parameter("grasp_y", 0.25)
        self.declare_parameter("world_name", "warehouse")
        self.declare_parameter("box_model", "box1")
        self.declare_parameter("box_sdf_path", "")
        self.sim_mode = self.get_parameter("sim_mode").value
        self.links = list(self.get_parameter("arm_links").value)

        self.pub_result = self.create_publisher(Bool, "/grasp_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/grasp_request", self._on_request, 10)
        self._robot = self._init_arm()
        self._status(f"Node C 시작 (sim_mode={self.sim_mode}, IK={'ON' if self._robot else 'OFF'})")

    def _on_request(self, msg: String):
        try:
            d = json.loads(msg.data)
            op = d.get("op", "grasp")
            wx, wy = float(d["x"]), float(d["y"])
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self._status(f"⚠️ grasp_request 파싱 실패: {e}"); self._result(False); return

        tx = float(self.get_parameter("grasp_x").value)
        ty = float(self.get_parameter("grasp_y").value)
        # ★ 특이점 회피: y-offset 강제
        if abs(ty) < Y_OFFSET_MIN:
            self._status(f"⚠️ y={ty:.2f} < {Y_OFFSET_MIN} → 클램프")
            ty = Y_OFFSET_MIN

        q = self._solve_ik(tx, ty)
        if q is None:
            self._status("⚠️ IK 수렴 실패"); self._result(False); return
        self._status(f"{op} IK 해 q={[round(float(v), 3) for v in q]} (target=({tx:.2f},{ty:.2f}))")

        ok = self._teleport(op, wx, wy)
        self._result(ok)

    # ── ① TODO: IK 계산 (nd1_m7_ik) ──────────────────────────────
    def _solve_ik(self, x, y):
        """팔 로컬 타깃 (x,y)의 관절각을 구한다. 실패 시 None.
        힌트:
          - self._robot 가 None 이면: sim_mode면 [0,0,0] 반환, 아니면 None
          - from nd1_m7_ik import numerical_ik
          - return list(numerical_ik(self._robot, (x, y)))   # [θ1,θ2,θ3]
          - 예외는 try/except, sim_mode면 [0,0,0] 반환
        """
        if self._robot is None:
            return [0.0, 0.0, 0.0] if self.sim_mode else None
        try:
            from nd1_m7_ik import numerical_ik
            return list(numerical_ik(self._robot, (x, y)))
        except Exception as e:
            self.get_logger().warn(f"IK 계산 실패: {e}")
            return [0.0, 0.0, 0.0] if self.sim_mode else None

    def _init_arm(self):
        try:
            from nd1_m7_ik import RobotArm3DOF
            return RobotArm3DOF(links=self.links)
        except Exception as e:
            self.get_logger().warn(f"nd1_m7_ik 로드 실패(sim 전용 가능): {e}")
            return None

    # ── ② TODO: 텔레포트 파지/배치 (ign service) ──────────────────
    def _teleport(self, op: str, x: float, y: float) -> bool:
        """op=grasp → 박스 제거 / op=place → (x,y)에 박스 재생성. 성공 bool.
        힌트:
          - sim_mode면 곧장 return True (로그만)
          - world = self.get_parameter('world_name').value, box = box_model
          - grasp: ign service -s /world/<world>/remove
                   --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean
                   --req 'name: "<box>" type: MODEL'
          - place: ign service -s /world/<world>/create
                   --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean
                   --req 'sdf_filename: "<box_sdf_path 또는 box.sdf>"
                          pose: { position: { x: <x> y: <y> z: 0.1 } } name: "<box>"'
          - subprocess.run(cmd, capture_output=True, text=True, timeout=5)
          - 응답 stdout 에 'true' 포함이면 성공
          - TimeoutExpired/FileNotFoundError 예외 처리 → False
        """
        if self.sim_mode:
            self._status(f"[sim] {op} 텔레포트 가정 — 성공")
            return True

        world = self.get_parameter("world_name").value
        box = self.get_parameter("box_model").value

        if op == "grasp":
            cmd = [
                "ign", "service", "-s", f"/world/{world}/remove",
                "--reqtype", "ignition.msgs.Entity",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "3000",
                "--req", f'name: "{box}" type: MODEL',
            ]
        else:
            sdf_path = self.get_parameter("box_sdf_path").value or f"{box}.sdf"
            cmd = [
                "ign", "service", "-s", f"/world/{world}/create",
                "--reqtype", "ignition.msgs.EntityFactory",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "3000",
                "--req",
                f'sdf_filename: "{sdf_path}" '
                f'pose: {{ position: {{ x: {x} y: {y} z: 0.1 }} }} name: "{box}"',
            ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            ok = "true" in result.stdout.lower()
            if not ok:
                self._status(f"⚠️ ign service 응답 실패: {result.stdout.strip()} {result.stderr.strip()}")
            return ok
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            self._status(f"⚠️ ign service 호출 실패: {e}")
            return False

    def _result(self, ok: bool):
        self.pub_result.publish(Bool(data=ok))
        self._status(f"결과: {'성공' if ok else '실패'}")

    def _status(self, text):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[C] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeCGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
