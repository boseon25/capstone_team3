#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Node A — 자연어 명령 해석 [학생 구현]
#  역할: /llm_command(String) → groq LLM 파싱(+폴백) → /mission(String, JSON)
#
#  제공(인프라): 데이터 계약(RobotCommand), pub/sub, groq 클라이언트, main
#  구현(TODO):  ① _parse_with_llm  ② _parse_fallback
#
#  토픽 계약(고정):
#    In  /llm_command (std_msgs/String) — 사용자 자연어
#    Out /mission     (std_msgs/String) — RobotCommand JSON 1건
#  표준 좌표: A(1.5,0.5) B(2.5,-1.0) C(0.5,2.0)  ※ 변경 금지
# ════════════════════════════════════════════════════════════════
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
import json
import os
import re
from dataclasses import asdict, dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ZONES = {"A": (1.5, 0.5), "B": (2.5, -1.0), "C": (0.5, 2.0)}


class ActionType(str, Enum):
    PICK_AND_PLACE = "pick_and_place"
    NAVIGATE = "navigate"
    STOP = "stop"


@dataclass
class RobotCommand:
    action: ActionType
    object: str = ""
    pick_x: float = 0.0
    pick_y: float = 0.0
    place_x: float = 0.0
    place_y: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_dict(cls, data):
        action = data.get("action", "stop")
        try:
            action = ActionType(action)
        except ValueError:
            action = ActionType.STOP

        return cls(
            action=action,
            object=str(data.get("object", "")),
            pick_x=float(data.get("pick_x", 0.0)),
            pick_y=float(data.get("pick_y", 0.0)),
            place_x=float(data.get("place_x", 0.0)),
            place_y=float(data.get("place_y", 0.0)),
            yaw=float(data.get("yaw", 0.0)),
        )

    def to_json(self):
        data = asdict(self)
        data["action"] = self.action.value
        return json.dumps(data, ensure_ascii=False)


SYSTEM_PROMPT = """너는 로봇 명령 파서다. 한국어 명령을 RobotCommand JSON으로만 변환한다.
구역 좌표: A=(1.5,0.5), B=(2.5,-1.0), C=(0.5,2.0).
스키마: {"action":"pick_and_place|navigate|stop","object":"","pick_x":0,"pick_y":0,"place_x":0,"place_y":0,"yaw":0}
설명 없이 JSON 객체 하나만 출력."""


class NodeALLM(Node):
    def __init__(self):
        super().__init__("node_a_llm")
        self.pub = self.create_publisher(String, "/mission", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/llm_command", self._on_command, 10)

        self._llm = self._init_groq()
        self._model = os.environ.get("GROQ_MODEL", "openai/gpt-oss-20b")
        self._status(f"Node A 시작 — LLM={'ON' if self._llm else 'OFF(폴백)'}")

    def _on_command(self, msg: String):
        text = msg.data
        self._status(f"명령 수신: '{text}'")

        cmd = self._parse_with_llm(text) or self._parse_fallback(text)
        self.pub.publish(String(data=cmd.to_json()))

        self._status(
            f"미션 발행: {cmd.action.value} "
            f"pick=({cmd.pick_x},{cmd.pick_y}) "
            f"place=({cmd.place_x},{cmd.place_y})"
        )

    def _parse_with_llm(self, text: str):
        if self._llm is None:
            return None

        try:
            response = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
            )

            raw = response.choices[0].message.content
            data = json.loads(raw)
            return RobotCommand.from_dict(data)

        except Exception as e:
            self._status(f"LLM 파싱 실패 → 폴백 사용: {e}")
            return None

    def _parse_fallback(self, text: str) -> RobotCommand:
        if any(k in text for k in ["정지", "멈춰", "스톱", "stop", "STOP"]):
            return RobotCommand(action=ActionType.STOP)

        zones = self._extract_zones(text)
        move_keywords = ["옮", "이동", "놓", "배치", "운반", "가져", "보내"]

        if any(k in text for k in move_keywords) and len(zones) >= 2:
            pick = ZONES[zones[0]]
            place = ZONES[zones[1]]
            return RobotCommand(
                action=ActionType.PICK_AND_PLACE,
                object=self._extract_object(text),
                pick_x=pick[0],
                pick_y=pick[1],
                place_x=place[0],
                place_y=place[1],
                yaw=0.0,
            )

        if len(zones) >= 1:
            place = ZONES[zones[0]]
            return RobotCommand(
                action=ActionType.NAVIGATE,
                object="",
                place_x=place[0],
                place_y=place[1],
                yaw=0.0,
            )

        return RobotCommand(action=ActionType.STOP)

    @staticmethod
    def _extract_zones(text: str):
        compact = text.upper().replace(" ", "")
        found = []

        pattern = re.compile(r"(A|B|C)(?:구역|지역|존|에서|으로|로|에)?")
        for match in pattern.finditer(compact):
            zone = match.group(1)
            if zone in ZONES:
                found.append(zone)

        return found

    @staticmethod
    def _extract_object(text: str) -> str:
        match = re.search(r"([가-힣A-Za-z0-9_]+)\s*(?:을|를)", text)
        if match:
            return match.group(1)
        return "박스"

    def _init_groq(self):
        key = os.environ.get("GROQ_API_KEY", "").strip()
        if not key or key.startswith("your_"):
            return None

        try:
            from groq import Groq
            return Groq(api_key=key)
        except Exception:
            return None

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[A] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeALLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()