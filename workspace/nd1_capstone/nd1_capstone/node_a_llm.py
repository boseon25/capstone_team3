#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Node A — 자연어 명령 해석 [학생 구현]
#  역할: /llm_command(String) → groq LLM 파싱(+폴백) → /mission(String, JSON)
#
#  In  /llm_command (std_msgs/String) — 사용자 자연어
#  Out /mission     (std_msgs/String) — RobotCommand JSON 1건
#  표준 좌표: A(1.5,0.5) B(2.5,-1.0) C(0.5,2.0)  ※ 변경 금지
# ════════════════════════════════════════════════════════════════
import json
import os
import re
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydantic import BaseModel, Field

ZONES = {"A": (1.5, 0.5), "B": (2.5, -1.0), "C": (0.5, 2.0)}


class ActionType(str, Enum):
    PICK_AND_PLACE = "pick_and_place"
    NAVIGATE = "navigate"
    STOP = "stop"


class RobotCommand(BaseModel):
    """LLM/폴백이 생성하는 구조화 명령. 이 스키마를 그대로 /mission 으로 발행한다."""
    action: ActionType
    object: str = ""
    pick_x: float = 0.0
    pick_y: float = 0.0
    place_x: float = 0.0
    place_y: float = 0.0
    yaw: float = Field(default=0.0)


SYSTEM_PROMPT = """너는 로봇 명령 파서다. 한국어 명령을 RobotCommand JSON으로만 변환한다.
구역 좌표: A=(1.5,0.5), B=(2.5,-1.0), C=(0.5,2.0).
스키마: {"action":"pick_and_place|navigate|stop","object":"","pick_x":0,"pick_y":0,"place_x":0,"place_y":0,"yaw":0}
규칙:
- 물체를 A에서 B로 옮기라는 명령은 pick_and_place로 변환한다.
- 특정 구역으로 이동하라는 명령은 navigate로 변환한다.
- 정지, 멈춤, stop은 stop으로 변환한다.
- 백신 샘플, 샘플 컨테이너, box1은 모두 object="box1"로 변환한다.
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

        self.pub.publish(String(data=cmd.model_dump_json()))
        self._status(
            f"미션 발행: {cmd.action.value} "
            f"object={cmd.object} "
            f"pick=({cmd.pick_x},{cmd.pick_y}) "
            f"place=({cmd.place_x},{cmd.place_y})"
        )

    def _parse_with_llm(self, text: str):
        """Groq LLM으로 자연어를 RobotCommand로 변환한다. 실패하면 None을 반환해 폴백 파서가 처리한다."""
        if self._llm is None:
            return None

        try:
            res = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
            )
            content = res.choices[0].message.content
            data = json.loads(content)
            return RobotCommand(**data)
        except Exception as e:
            self.get_logger().warn(f"LLM 파싱 실패 → 폴백 사용: {e}")
            return None

    def _parse_fallback(self, text: str) -> RobotCommand:
        """규칙 기반 파서. Groq API가 없어도 데모가 돌아가도록 하는 안전망이다."""
        normalized = text.strip()
        lower = normalized.lower()

        if any(k in lower for k in ["stop", "정지", "멈춰", "멈추", "스톱", "중단"]):
            return RobotCommand(action=ActionType.STOP)

        zones = self._extract_zones(normalized)
        obj = self._extract_object(normalized)

        move_keywords = ["옮", "이동", "놓", "배치", "운반", "가져", "전달", "반출"]
        has_move_intent = any(k in normalized for k in move_keywords)

        if has_move_intent and len(zones) >= 2:
            pick_zone = zones[0]
            place_zone = zones[1]
            pick_x, pick_y = ZONES[pick_zone]
            place_x, place_y = ZONES[place_zone]

            return RobotCommand(
                action=ActionType.PICK_AND_PLACE,
                object=obj,
                pick_x=pick_x,
                pick_y=pick_y,
                place_x=place_x,
                place_y=place_y,
                yaw=0.0,
            )

        if len(zones) >= 1:
            target_zone = zones[0]
            place_x, place_y = ZONES[target_zone]

            return RobotCommand(
                action=ActionType.NAVIGATE,
                object=obj,
                place_x=place_x,
                place_y=place_y,
                yaw=0.0,
            )

        return RobotCommand(action=ActionType.STOP)

    @staticmethod
    def _extract_zones(text: str):
        """문장 안에서 A, B, C 구역이 등장한 순서대로 추출한다."""
        hits = []

        for zone in ZONES.keys():
            patterns = [
                rf"{zone}\s*구역",
                rf"구역\s*{zone}",
                rf"(?<![A-Za-z0-9]){zone}(?![A-Za-z0-9])",
            ]

            for pattern in patterns:
                for match in re.finditer(pattern, text, flags=re.IGNORECASE):
                    hits.append((match.start(), zone))

        hits.sort(key=lambda x: x[0])

        ordered_zones = []
        for _, zone in hits:
            if zone not in ordered_zones:
                ordered_zones.append(zone)

        return ordered_zones

    @staticmethod
    def _extract_object(text: str) -> str:
        """프로젝트 시나리오의 백신 샘플 컨테이너는 box1로 통일한다."""
        lower = text.lower()

        if "box1" in lower:
            return "box1"

        if any(k in text for k in ["백신", "샘플", "컨테이너", "박스", "box"]):
            return "box1"

        return "box1"

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
