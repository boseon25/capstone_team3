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
#  표준 좌표: A(-2.0,1.2) B(-1.5,-1.0) C(-1.0,-2.2)  ※ 변경 금지
#  (turtlebot3_world 로봇 스폰(-2.0,-0.5) 인근 서쪽 열린 공간 — 중앙 기둥 장식물·
#   SLAM 미탐사 지역 회피. 온라인 SLAM은 로봇이 가본 적 없는 먼 좌표로는
#   Nav2 goal이 "off the global costmap"로 즉시 거부되므로 스폰 근처로 제한)
# ════════════════════════════════════════════════════════════════
import json
import os
import re
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydantic import BaseModel, Field

ZONES = {"A": (-2.0, 1.2), "B": (-1.5, -1.0), "C": (-1.0, -2.2)}


class ActionType(str, Enum):
    PICK_AND_PLACE = "pick_and_place"
    NAVIGATE = "navigate"
    STOP = "stop"


class RobotCommand(BaseModel):
    """LLM/폴백이 생성하는 구조화 명령 (이 스키마를 그대로 /mission 으로 발행)."""
    action: ActionType
    object: str = ""
    pick_x: float = 0.0
    pick_y: float = 0.0
    place_x: float = 0.0
    place_y: float = 0.0
    yaw: float = Field(default=0.0)


SYSTEM_PROMPT = """너는 로봇 명령 파서다. 한국어 명령을 RobotCommand JSON으로만 변환한다.
구역 좌표: A=(-2.0,1.2), B=(-1.5,-1.0), C=(-1.0,-2.2).
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
        self.pub.publish(String(data=cmd.model_dump_json()))
        self._status(f"미션 발행: {cmd.action.value} pick=({cmd.pick_x},{cmd.pick_y}) "
                     f"place=({cmd.place_x},{cmd.place_y})")

    # ── ① TODO: groq LLM 파싱 ─────────────────────────────────────
    def _parse_with_llm(self, text: str):
        """groq로 자연어 → RobotCommand. 실패/미가용이면 None 반환(→ 폴백).
        힌트:
          - self._llm 이 None 이면 곧장 return None
          - self._llm.chat.completions.create(model=self._model, temperature=0,
                response_format={"type":"json_object"},
                messages=[{"role":"system","content":SYSTEM_PROMPT},
                          {"role":"user","content":text}])
          - 응답 JSON을 RobotCommand(**json.loads(...)) 로 검증해 반환
          - 예외는 try/except로 잡고 return None (폴백이 받도록)
        """
        if self._llm is None:
            return None
        try:
            resp = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
            )
            data = json.loads(resp.choices[0].message.content)
            return RobotCommand(**data)
        except Exception as e:
            self.get_logger().warn(f"groq 파싱 실패 → 폴백 사용: {e}")
            return None

    # ── ② TODO: 키워드 폴백 파서 (groq 없거나 실패 시) ────────────
    def _parse_fallback(self, text: str) -> RobotCommand:
        """규칙 기반 파서. groq 차단 상황에서도 데모가 돌아가게 하는 안전망.
        힌트:
          - '정지/멈춰/스톱' 포함 → action=STOP
          - 텍스트에서 'A구역','B 구역' 등으로 등장 구역 추출 (ZONES 키)
          - 이동 키워드('옮','이동','놓','배치','운반','가져') + 구역 2개↑
              → PICK_AND_PLACE, pick=ZONES[첫구역], place=ZONES[둘째구역]
          - 구역 1개만 → NAVIGATE, place=ZONES[그 구역]
          - 아무것도 아니면 → STOP (안전 기본값)
        """
        if any(k in text for k in ("정지", "멈춰", "스톱")):
            return RobotCommand(action=ActionType.STOP)

        zones = [z for z in re.findall(r"([A-Z])\s*구역", text) if z in ZONES]
        move_kw = ("옮", "이동", "놓", "배치", "운반", "가져")

        if any(k in text for k in move_kw) and len(zones) >= 2:
            pick_x, pick_y = ZONES[zones[0]]
            place_x, place_y = ZONES[zones[1]]
            return RobotCommand(
                action=ActionType.PICK_AND_PLACE,
                object=self._extract_object(text),
                pick_x=pick_x, pick_y=pick_y,
                place_x=place_x, place_y=place_y,
            )
        if len(zones) == 1:
            place_x, place_y = ZONES[zones[0]]
            return RobotCommand(action=ActionType.NAVIGATE, place_x=place_x, place_y=place_y)

        return RobotCommand(action=ActionType.STOP)

    @staticmethod
    def _extract_object(text: str) -> str:
        # (선택) "~를/을" 앞 단어를 object로. 미구현 시 "박스" 고정도 무방.
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
