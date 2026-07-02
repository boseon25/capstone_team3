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
#!/usr/bin/env python3
import json
import os
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

Y_OFFSET_MIN = 0.20


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
        self.sim_mode = self._as_bool(self.get_parameter("sim_mode").value)
        self.links = list(self.get_parameter("arm_links").value)

        self.pub_result = self.create_publisher(Bool, "/grasp_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/grasp_request", self._on_request, 10)
        self._robot = self._init_arm()
        self._status(f"Node C 시작 (sim_mode={self.sim_mode}, IK={'ON' if self._robot else 'OFF'})")

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("true", "1", "yes", "y")
        return bool(value)

    def _on_request(self, msg: String):
        try:
            d = json.loads(msg.data)
            op = d.get("op", "grasp")
            wx, wy = float(d["x"]), float(d["y"])
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self._status(f"⚠️ grasp_request 파싱 실패: {e}")
            self._result(False)
            return

        if op not in ("grasp", "place"):
            self._status(f"⚠️ 알 수 없는 grasp op: {op}")
            self._result(False)
            return

        tx = float(self.get_parameter("grasp_x").value)
        ty = float(self.get_parameter("grasp_y").value)

        if abs(ty) < Y_OFFSET_MIN:
            self._status(f"⚠️ y={ty:.2f} < {Y_OFFSET_MIN} → 클램프")
            ty = Y_OFFSET_MIN

        q = self._solve_ik(tx, ty)
        if q is None:
            self._status("⚠️ IK 수렴 실패")
            self._result(False)
            return

        self._status(f"{op} IK 해 q={[round(float(v), 3) for v in q]} (target=({tx:.2f},{ty:.2f}))")

        ok = self._teleport(op, wx, wy)
        self._result(ok)

    def _solve_ik(self, x, y):
        if self._robot is None:
            return [0.0, 0.0, 0.0] if self.sim_mode else None

        try:
            from nd1_m7_ik import numerical_ik
            return list(numerical_ik(self._robot, (x, y)))
        except Exception as e:
            self._status(f"⚠️ IK 계산 예외: {e}")
            return [0.0, 0.0, 0.0] if self.sim_mode else None

    def _init_arm(self):
        try:
            from nd1_m7_ik import RobotArm3DOF
            return RobotArm3DOF(links=self.links)
        except Exception as e:
            self.get_logger().warn(f"nd1_m7_ik 로드 실패(sim 전용 가능): {e}")
            return None

    def _teleport(self, op: str, x: float, y: float) -> bool:
        if self.sim_mode:
            self._status(f"[sim] {op} 텔레포트 가정 — 성공")
            return True

        world = self.get_parameter("world_name").value
        box = self.get_parameter("box_model").value

        try:
            if op == "grasp":
                cmd = [
                    "ign", "service",
                    "-s", f"/world/{world}/remove",
                    "--reqtype", "ignition.msgs.Entity",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "5000",
                    "--req", f'name: "{box}" type: MODEL',
                ]
            elif op == "place":
                sdf_path = self._resolve_box_sdf_path()
                req = (
                    f'sdf_filename: "{sdf_path}" '
                    f'pose: {{ position: {{ x: {x} y: {y} z: 0.1 }} }} '
                    f'name: "{box}"'
                )
                cmd = [
                    "ign", "service",
                    "-s", f"/world/{world}/create",
                    "--reqtype", "ignition.msgs.EntityFactory",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "5000",
                    "--req", req,
                ]
            else:
                self._status(f"⚠️ 알 수 없는 teleport op: {op}")
                return False

            self._status(f"텔레포트 실행: {op} ({box})")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
            output = (result.stdout + result.stderr).lower()
            ok = result.returncode == 0 and "true" in output

            if not ok:
                self._status(f"⚠️ 텔레포트 실패: {result.stderr.strip() or result.stdout.strip()}")

            return ok

        except subprocess.TimeoutExpired:
            self._status("⚠️ ign service 타임아웃")
            return False
        except FileNotFoundError:
            self._status("⚠️ ign 명령을 찾을 수 없음")
            return False
        except Exception as e:
            self._status(f"⚠️ 텔레포트 예외: {e}")
            return False

    def _resolve_box_sdf_path(self):
        sdf_path = str(self.get_parameter("box_sdf_path").value).strip()
        if sdf_path:
            return sdf_path

        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("nd1_capstone"), "models", "box1.sdf")
        except Exception:
            return "box1.sdf"

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()