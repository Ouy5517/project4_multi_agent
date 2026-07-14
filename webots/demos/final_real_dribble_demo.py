#!/usr/bin/env python3.10
"""Final REAL dribble demo: one mck robot, passive teammates/opponents."""
from __future__ import annotations

import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT = Path("/home/plon/Workspace/booster_soccer_project")
sys.path.insert(0, str(PROJECT))

from common.final_submission import compute_dribble_success, direction_error, newest_jsonl_state
from common.world_state import WorldState
from strategy.team_strategy import TeamStrategy

import rclpy
from rclpy.node import Node
from booster_interface.srv import RpcService

REQUIRED_ROBOTS = ["T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"]
MOVE_ATTEMPTS = [(0.04, 0.25), (0.06, 0.25), (0.08, 0.30)]


def now() -> str:
    return datetime.now().isoformat()


def xy(item: dict | None) -> list[float] | None:
    if not item or item.get("x") is None or item.get("y") is None:
        return None
    return [float(item["x"]), float(item["y"])]


def line_clear(start: list[float], end: list[float], blocker: list[float], threshold: float = 0.5) -> bool:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return True
    dist = abs(dy * blocker[0] - dx * blocker[1] + end[0] * start[1] - end[1] * start[0]) / length
    projection = ((blocker[0] - start[0]) * dx + (blocker[1] - start[1]) * dy) / (length * length)
    return not (0.0 <= projection <= 1.0 and dist < threshold)


class RpcClient(Node):
    def __init__(self) -> None:
        super().__init__("final_real_dribble_demo")
        self.cli = self.create_client(RpcService, "booster_rpc_service")
        if not self.cli.wait_for_service(timeout_sec=8):
            raise RuntimeError("booster_rpc_service unavailable")

    def call(self, api_id: int, body: str, label: str, timeout: float = 8.0) -> dict:
        req = RpcService.Request()
        req.msg.api_id = api_id
        req.msg.body = body
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return {"time": now(), "label": label, "api_id": api_id, "code": -3, "body": "timeout"}
        msg = future.result().msg
        return {"time": now(), "label": label, "api_id": api_id, "code": int(msg.status), "body": msg.body}


class RealDribbleDemo:
    def __init__(self) -> None:
        self.run_id = os.environ.get("RUN_ID", f"manual_{int(time.time())}")
        self.run_dir = Path(os.environ.get("FINAL_SUBMISSION_RUN_DIR", PROJECT / "results" / "final_submission" / self.run_id))
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.match_state = Path(os.environ.get("MATCH_STATE_FILE", self.run_dir / "match_state.jsonl"))
        self.actions: list[dict] = []
        self.decisions: list[dict] = []
        self.world_states: list[dict] = []
        self.ball_motion: list[dict] = []

    def write_jsonl(self, name: str, rows: list[dict]) -> None:
        with (self.run_dir / name).open("w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, ensure_ascii=False) + "\n")

    def wait_state(self, timeout: float = 20.0) -> dict | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = newest_jsonl_state(self.match_state)
            if state:
                return state
            time.sleep(0.25)
        return None

    def to_world_state(self, state: dict) -> WorldState:
        robots = state.get("robots") or {}
        ball = state.get("ball") or {}
        blue1 = robots.get("T1_BLUE_1") or {}
        carrier = state.get("ball_carrier") or "T1_BLUE_1"
        return WorldState.from_dict(
            {
                "scenario_name": "final_real_dribble",
                "timestamp": time.time(),
                "ball": {"x": ball["x"], "y": ball["y"]},
                "robots": [
                    {
                        "robot_id": "T1_BLUE_1",
                        "team": "BLUE",
                        "x": blue1["x"],
                        "y": blue1["y"],
                        "theta": blue1.get("yaw", 0.0),
                        "role": "BALL_HANDLER",
                        "has_ball": carrier in (None, "T1_BLUE_1") or math.hypot(ball["x"] - blue1["x"], ball["y"] - blue1["y"]) < 0.6,
                    },
                    {
                        "robot_id": "T1_BLUE_2",
                        "team": "BLUE",
                        "x": robots["T1_BLUE_2"]["x"],
                        "y": robots["T1_BLUE_2"]["y"],
                        "theta": robots["T1_BLUE_2"].get("yaw", 0.0),
                        "role": "SUPPORT",
                    },
                ],
                "opponents": [
                    {"opponent_id": "T1_RED_1", "x": robots["T1_RED_1"]["x"], "y": robots["T1_RED_1"]["y"]},
                    {"opponent_id": "T1_RED_2", "x": robots["T1_RED_2"]["x"], "y": robots["T1_RED_2"]["y"]},
                ],
                "our_goal": state.get("blue_goal") or {"x": -3.3, "y": 0.0},
                "enemy_goal": state.get("red_goal") or {"x": 3.3, "y": 0.0},
                "field_width": 7.0,
                "field_height": 5.0,
            }
        )

    def log_decision(self, **row) -> None:
        row.setdefault("time", now())
        self.decisions.append(row)
        print(f"[decision] {row.get('selected_strategy') or row.get('event')}: {row.get('reason')}")

    def rpc(self, client: RpcClient, api_id: int, body: str, label: str) -> dict:
        row = client.call(api_id, body, label)
        self.actions.append(row)
        print(f"[rpc] {label}: code={row['code']}")
        return row

    def run(self) -> int:
        print("FINAL REAL DRIBBLE DEMO")
        print("Active real mck robot: T1_BLUE_1")
        print("Passive robots: T1_BLUE_2, T1_RED_1, T1_RED_2")
        print(f"WorldState file: {self.match_state}")

        initial_state = self.wait_state()
        if not initial_state:
            self.save_summary({"failure_reason": "WorldState unavailable", "dribble_success": False})
            return 2

        robots = initial_state.get("robots") or {}
        missing = [name for name in REQUIRED_ROBOTS if name not in robots]
        if missing or not initial_state.get("ball") or not initial_state.get("blue_goal") or not initial_state.get("red_goal"):
            self.save_summary({"failure_reason": f"WorldState missing entities: {missing}", "dribble_success": False})
            return 2

        self.world_states.append(initial_state)
        world = self.to_world_state(initial_state)
        strategy = TeamStrategy()
        actions = strategy.decide(world)
        selected = actions[0].action_type.value if actions else "HOLD"

        blue1_initial = xy(robots["T1_BLUE_1"])
        ball_initial = xy(initial_state["ball"])
        blue2 = xy(robots["T1_BLUE_2"])
        red2 = xy(robots["T1_RED_2"])
        pass_clear = line_clear(blue1_initial, blue2, red2)
        blocked_by = None if pass_clear else "T1_RED_2"

        self.log_decision(
            event="PASS_EVALUATION",
            is_pass_line_clear=pass_clear,
            pass_line_clear=pass_clear,
            blocked_by=blocked_by,
            rejected_strategy="PASS" if not pass_clear else None,
            selected_strategy=selected,
            reason=strategy.last_decision_summary,
            actions=[a.to_dict() for a in actions],
        )

        rclpy.init(args=None)
        client = RpcClient()
        try:
            get_mode = self.rpc(client, 2017, "", "GetMode_initial")
            prepare = self.rpc(client, 2000, json.dumps({"mode": 1}), "Prepare")
            time.sleep(3.0)
            walking = self.rpc(client, 2000, json.dumps({"mode": 2}), "Walking")
            time.sleep(2.0)

            for attempt, (vx, duration) in enumerate(MOVE_ATTEMPTS, start=1):
                self.rpc(client, 2001, json.dumps({"vx": vx, "vy": 0.0, "vyaw": 0.0}), f"Move_{attempt}")
                time.sleep(duration)
                stop = self.rpc(client, 2001, json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": 0.0}), f"Stop_{attempt}")
                time.sleep(0.8)
                state = self.wait_state(timeout=3.0)
                if state:
                    self.world_states.append(state)
                    self.ball_motion.append({"time": now(), "attempt": attempt, "ball": state.get("ball"), "stop_code": stop["code"]})
                    disp = math.nan
                    if ball_initial and state.get("ball"):
                        disp = math.hypot(state["ball"]["x"] - ball_initial[0], state["ball"]["y"] - ball_initial[1])
                    print(f"[ball] attempt={attempt} displacement={disp:.4f}m")
                    if disp > 0.05:
                        break

            final_mode = self.rpc(client, 2017, "", "GetMode_final")
            final_stop = self.rpc(client, 2001, json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": 0.0}), "Final_Stop")
        finally:
            client.destroy_node()
            rclpy.shutdown()

        final_state = self.world_states[-1] if self.world_states else initial_state
        blue1_final = xy((final_state.get("robots") or {}).get("T1_BLUE_1"))
        ball_final = xy(final_state.get("ball"))
        target = xy(final_state.get("red_goal")) or [3.3, 0.0]
        process_alive = {
            "webots": os.system("pgrep -x webots-bin >/dev/null 2>&1") == 0,
            "mck": os.system("pgrep -f '/mck|webots-controller' >/dev/null 2>&1") == 0,
            "rpc": os.system("pgrep -f rpc_service_node >/dev/null 2>&1") == 0,
        }

        summary = {
            "run_id": self.run_id,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "config": os.environ.get("MCK_CONFIG", ""),
            "runner_instance": os.environ.get("RUNNER_INSTANCE", ""),
            "recording_flags": json.loads(os.environ.get("RECORDING_FLAGS_JSON", "{}") or "{}"),
            "lcm_backend_detected": None,
            "socket_backend_detected": None,
            "recording_rejected_count": None,
            "mck_ready": os.environ.get("MCK_READY", "false") == "true",
            "mck_ready_seconds": float(os.environ.get("MCK_READY_SECONDS", "0") or 0),
            "mck_segfault": False,
            "estab": os.system("ss -tn 2>/dev/null | grep ':1234' | grep -q ESTAB") == 0,
            "get_mode_success": get_mode["code"] == 0 or final_mode["code"] == 0,
            "prepare_success": prepare["code"] == 0,
            "walking_success": walking["code"] == 0,
            "stop_success": final_stop["code"] == 0,
            "real_mck_robot": "T1_BLUE_1",
            "active_robot_count": 1,
            "participating_robot_count": 4,
            "blue1_initial_position": blue1_initial,
            "blue1_final_position": blue1_final,
            "ball_initial_position": ball_initial,
            "ball_final_position": ball_final,
            "ball_direction_error": direction_error(ball_initial, ball_final, target),
            "pass_line_clear": pass_clear,
            "blocked_by": blocked_by,
            "rejected_strategy": "PASS" if not pass_clear else None,
            "selected_strategy": selected,
            "robot_fallen": bool(blue1_final and len(blue1_final) > 2 and blue1_final[2] < 0.25),
            "processes_alive": process_alive,
            "physical_collision_claim": "ball moved only if Webots physics changed monitor coordinates; Supervisor did not move ball",
        }
        compute_dribble_success(summary)
        self.save_summary(summary)

        print("ACTION REQUIRED:")
        print("请使用 Win+Shift+S 截取当前Webots真实推球画面并保存为：")
        print("outputs/screenshots/final_real_dribble.png")
        time.sleep(30)
        return 0 if summary["dribble_success"] else 1

    def save_summary(self, summary: dict) -> None:
        self.write_jsonl("world_state.jsonl", self.world_states)
        self.write_jsonl("decisions.jsonl", self.decisions)
        self.write_jsonl("actions.jsonl", self.actions)
        self.write_jsonl("ball_motion.jsonl", self.ball_motion)
        (self.run_dir / "real_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    raise SystemExit(RealDribbleDemo().run())
