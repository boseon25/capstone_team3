#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  [난이도 상 ★★★] LLM 멀티미션 플래너 [학생 구현]
#  복합 목표 → groq로 미션 '시퀀스' 생성 → DONE 신호마다 다음 미션 투입.
#  Coordinator(FSM) 위에 얹는 계획 레이어.
#
#  제공: pub/sub, groq 초기화, 상태감지 골격, main
#  구현(TODO): ① _plan (LLM) ② _plan_fallback ③ _dispatch (큐 투입)
#
#  In  /goal_command(String) 복합 목표 / /robot_status(String) DONE 감지
#  Out /mission(String) 1건씩
# ════════════════════════════════════════════════════════════════
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ZONES = {"A": (-2.0, 1.2), "B": (-1.5, -1.0), "C": (-1.0, -2.2)}

PLANNER_PROMPT = f"""너는 로봇 태스크 플래너다. 복합 한국어 목표를 JSON 객체로만 변환하라.
설명 금지, 반드시 {{"missions":[...]}} 형태의 JSON 객체 하나만 출력.
구역 좌표: A={ZONES['A']}, B={ZONES['B']}, C={ZONES['C']}.
missions 배열 원소 스키마:
{{"action":"pick_and_place","object":"","pick_x":0,"pick_y":0,"place_x":0,"place_y":0,"yaw":0}}"""


class LLMPlanner(Node):
    def __init__(self):
        super().__init__("llm_planner")
        self.pub_mission = self.create_publisher(String, "/mission", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/goal_command", self._on_goal, 10)
        self.create_subscription(String, "/robot_status", self._on_status, 10)
        self.queue = []
        self.active = False
        self._llm = self._init_groq()
        self._model = os.environ.get("GROQ_MODEL", "openai/gpt-oss-20b")
        self._status(f"플래너 시작 — LLM={'ON' if self._llm else 'OFF(폴백)'}")

    def _on_goal(self, msg: String):
        self.queue = self._plan(msg.data) or self._plan_fallback(msg.data)
        self._status(f"계획 생성: 미션 {len(self.queue)}건")
        self.active = False
        self._dispatch()

    def _on_status(self, msg: String):
        s = msg.data
        if "[FSM:" not in s:
            return
        if "DONE" in s and self.active:
            self.active = False
            self._dispatch()
        elif "FAILED" in s and self.active:
            self._status("⚠️ 미션 실패 — 계획 중단"); self.queue = []; self.active = False

    # ── ③ TODO: 큐에서 1건 투입 ──────────────────────────────────
    def _dispatch(self):
        """active가 아니고 큐가 있으면 1건 pop 해서 /mission 발행, active=True.
        힌트: 큐 비면 '모두 완료' 로그. m=self.queue.pop(0);
              self.pub_mission.publish(String(data=json.dumps(m))); self.active=True
        """
        if self.active:
            return
        if not self.queue:
            self._status("모든 미션 완료 ✅")
            return
        m = self.queue.pop(0)
        self.pub_mission.publish(String(data=json.dumps(m)))
        self.active = True
        self._status(f"미션 투입: {m.get('action')} (남은 큐: {len(self.queue)}건)")

    # ── ① TODO: LLM 계획 (미션 배열) ─────────────────────────────
    def _plan(self, text):
        """groq로 복합 목표 → 미션 리스트. 실패/미가용 None.
        힌트: self._llm None이면 None. create(..., response_format={"type":"json_object"},
              system=PLANNER_PROMPT). 결과가 list면 그대로, dict면 .get('missions',[]).
        """
        if self._llm is None:
            return None
        try:
            resp = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": PLANNER_PROMPT},
                    {"role": "user", "content": text},
                ],
            )
            data = json.loads(resp.choices[0].message.content)
            if isinstance(data, list):
                return data
            missions = data.get("missions", data.get("tasks", data.get("steps", [])))
            return missions if isinstance(missions, list) else []
        except Exception as e:
            self.get_logger().warn(f"LLM 계획 실패 → 폴백: {e}")
            return None

    # ── ② TODO: 폴백 분해 ────────────────────────────────────────
    def _plan_fallback(self, text):
        """규칙 기반: 등장 구역들 → 마지막 구역을 목적지로, 나머지를 출발지로 pick_and_place 생성.
        힌트: srcs=등장구역, dst=srcs[-1]; for z in srcs(z!=dst): pick=ZONES[z],place=ZONES[dst]
        """
        import re
        srcs = [z for z in re.findall(r"([A-C])\s*구역", text) if z in ZONES]
        if len(srcs) < 2:
            return []
        dst = srcs[-1]
        missions = []
        for z in srcs:
            if z == dst:
                continue
            px, py = ZONES[z]
            lx, ly = ZONES[dst]
            missions.append({
                "action": "pick_and_place",
                "object": "박스",
                "pick_x": px, "pick_y": py,
                "place_x": lx, "place_y": ly,
                "yaw": 0.0,
            })
        return missions

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
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
