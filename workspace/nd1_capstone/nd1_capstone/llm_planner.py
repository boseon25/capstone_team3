#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  [난이도 상 ★★★] LLM 멀티미션 플래너 [학생 구현]
#  역할:
#    /goal_command(String) 복합 자연어 목표 수신
#    → LLM 또는 규칙 기반 폴백으로 mission sequence 생성
#    → /mission(String, JSON)을 1건씩 발행
#    → /robot_status에서 FSM DONE 감지 시 다음 mission 발행
#
#  In  /goal_command(String), /robot_status(String)
#  Out /mission(String), /robot_status(String)
#
#  표준 좌표:
#    A = (1.5, 0.5)   백신 제조 구역
#    B = (2.5, -1.0)  안전성 테스트 구역
#    C = (0.5, 2.0)   비상 반출 구역
# ════════════════════════════════════════════════════════════════
import json
import os
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ZONES = {"A": (1.5, 0.5), "B": (2.5, -1.0), "C": (0.5, 2.0)}

ZONE_NAMES = {
    "A": "백신 제조 구역",
    "B": "안전성 테스트 구역",
    "C": "비상 반출 구역",
}

PLANNER_PROMPT = f"""너는 로봇 태스크 플래너다.
복합 한국어 목표를 mission 배열 JSON 객체로만 변환하라.

반드시 아래 형식으로만 출력한다.
{{
  "missions": [
    {{"action":"pick_and_place","object":"box1","pick_x":0,"pick_y":0,"place_x":0,"place_y":0,"yaw":0}}
  ]
}}

구역 좌표:
A={ZONES['A']} : 백신 제조 구역
B={ZONES['B']} : 안전성 테스트 구역
C={ZONES['C']} : 비상 반출 구역

규칙:
- 백신 샘플, 샘플 컨테이너, box1은 모두 object="box1"로 설정한다.
- A에서 B로 검증 후 C로 반출하라는 목표는 두 mission으로 나눈다.
  1) A → B
  2) B → C
- 설명 문장 없이 JSON 객체 하나만 출력한다.
"""


class LLMPlanner(Node):
    def __init__(self):
        super().__init__("llm_planner")
        self.pub_mission = self.create_publisher(String, "/mission", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/goal_command", self._on_goal, 10)
        self.create_subscription(String, "/robot_status", self._on_status, 10)

        self.queue = []
        self.active = False
        self.current_index = 0
        self.total_count = 0

        self._llm = self._init_groq()
        self._model = os.environ.get("GROQ_MODEL", "openai/gpt-oss-20b")
        self._status(f"플래너 시작 — LLM={'ON' if self._llm else 'OFF(폴백)'}")

    def _on_goal(self, msg: String):
        text = msg.data
        self._status(f"복합 목표 수신: {text}")

        missions = self._plan(text)
        if not missions:
            missions = self._plan_fallback(text)

        self.queue = missions
        self.active = False
        self.current_index = 0
        self.total_count = len(self.queue)

        if not self.queue:
            self._status("⚠️ 생성된 미션이 없음 — 계획 중단")
            return

        self._status(f"계획 생성: 미션 {self.total_count}건")
        self._dispatch()

    def _on_status(self, msg: String):
        s = msg.data

        # Planner 자신의 [PLAN] 로그는 무시하고, FSM 상태만 감지한다.
        if "[FSM:" not in s:
            return

        if "FAILED" in s and self.active:
            self._status("⚠️ FSM 미션 실패 — 남은 계획 중단")
            self.queue = []
            self.active = False
            return

        if "DONE" in s and self.active:
            self.active = False
            self._status(f"FSM 미션 완료 감지: {self.current_index}/{self.total_count}")
            self._dispatch()

    def _dispatch(self):
        """큐에서 mission 1건을 꺼내 /mission으로 발행한다."""
        if self.active:
            return

        if not self.queue:
            self._status("계획 전체 완료 ✅")
            return

        mission = self.queue.pop(0)
        self.current_index += 1

        self.pub_mission.publish(String(data=json.dumps(mission, ensure_ascii=False)))
        self.active = True

        self._status(
            f"미션 발행 {self.current_index}/{self.total_count}: "
            f"{mission.get('object', 'box1')} "
            f"({mission.get('pick_x'):.2f}, {mission.get('pick_y'):.2f})"
            f" → "
            f"({mission.get('place_x'):.2f}, {mission.get('place_y'):.2f})"
        )

    def _plan(self, text):
        """Groq LLM으로 복합 목표를 mission list로 변환한다. 실패하면 None을 반환한다."""
        if self._llm is None:
            return None

        try:
            res = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": PLANNER_PROMPT},
                    {"role": "user", "content": text},
                ],
            )

            content = res.choices[0].message.content
            data = json.loads(content)

            if isinstance(data, dict):
                missions = data.get("missions", [])
            elif isinstance(data, list):
                missions = data
            else:
                return None

            return self._normalize_missions(missions)

        except Exception as e:
            self.get_logger().warn(f"LLM 계획 실패 → 폴백 사용: {e}")
            return None

    def _plan_fallback(self, text):
        """규칙 기반 폴백 플래너. LLM API가 없어도 백신 시나리오가 동작하도록 한다."""
        zones = self._extract_zones(text)

        # 백신 시나리오 기본값:
        # A 제조 구역 → B 테스트 구역 → C 비상 반출 구역
        if self._is_vaccine_scenario(text):
            zones = ["A", "B", "C"]

        if len(zones) < 2:
            self._status("⚠️ 폴백 계획 실패: 최소 2개 구역이 필요함")
            return []

        missions = []

        # 순서대로 pairwise mission 생성
        # 예: A,B,C → A→B, B→C
        for src, dst in zip(zones[:-1], zones[1:]):
            pick_x, pick_y = ZONES[src]
            place_x, place_y = ZONES[dst]

            missions.append(
                {
                    "action": "pick_and_place",
                    "object": "box1",
                    "pick_x": float(pick_x),
                    "pick_y": float(pick_y),
                    "place_x": float(place_x),
                    "place_y": float(place_y),
                    "yaw": 0.0,
                    "from_zone": src,
                    "to_zone": dst,
                    "from_zone_name": ZONE_NAMES[src],
                    "to_zone_name": ZONE_NAMES[dst],
                }
            )

            if dst == "B" and self._is_vaccine_scenario(text):
                self._status("TEST_PASS 확인 단계 포함: B 안전성 테스트 구역 검증 통과로 가정")

        return missions

    def _normalize_missions(self, missions):
        """LLM 결과를 FSM이 받을 수 있는 안전한 mission dict 리스트로 정리한다."""
        normalized = []

        for mission in missions:
            if not isinstance(mission, dict):
                continue

            action = mission.get("action", "pick_and_place")
            if action != "pick_and_place":
                continue

            normalized.append(
                {
                    "action": "pick_and_place",
                    "object": mission.get("object", "box1") or "box1",
                    "pick_x": float(mission.get("pick_x", 0.0)),
                    "pick_y": float(mission.get("pick_y", 0.0)),
                    "place_x": float(mission.get("place_x", 0.0)),
                    "place_y": float(mission.get("place_y", 0.0)),
                    "yaw": float(mission.get("yaw", 0.0)),
                }
            )

        return normalized

    @staticmethod
    def _extract_zones(text):
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
    def _is_vaccine_scenario(text):
        keywords = [
            "백신",
            "샘플",
            "컨테이너",
            "좀비",
            "바이러스",
            "안전성",
            "테스트",
            "검증",
            "비상",
            "반출",
        ]
        return any(k in text for k in keywords)

    def _init_groq(self):
        key = os.environ.get("GROQ_API_KEY", "").strip()
        if not key or key.startswith("your_"):
            return None

        try:
            from groq import Groq
            return Groq(api_key=key)
        except Exception:
            return None

    def _status(self, t):
        self.get_logger().info(t)
        self.pub_status.publish(String(data=f"[PLAN] {t}"))


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanner()
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
