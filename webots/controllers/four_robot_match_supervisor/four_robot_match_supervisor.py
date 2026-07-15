from __future__ import annotations

import json
import math
import os
import socket
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

try:
    from controller import Supervisor
except Exception:  # pragma: no cover
    Supervisor = None

from assisted_locomotion import AssistedLocomotion, wrap_angle
from ball_contact_verifier import BallContactVerifier, BallMutationGuard
from four_robot_orchestrator import OPENING_TARGETS, ROBOT_NAMES, SCENARIO_PUSHES, MoveTarget, prepare_pose, unit, yaw_from_direction
from world_state_reader import ROLES, WorldStateReader

from common.robot_action import ActionType
from strategy.team_strategy import TeamStrategy


SUPERVISOR_PORT = 18120
ROBOT_PORTS = {
    "BLUE_1": 18101,
    "BLUE_2": 18102,
    "RED_1": 18103,
    "RED_2": 18104,
}


class Jsonl:
    def __init__(self, path: Path) -> None:
        self.file = path.open("a", encoding="utf-8")

    def write(self, row: dict) -> None:
        self.file.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class FourRobotMatchSupervisor:
    def __init__(self) -> None:
        if Supervisor is None:
            raise RuntimeError("Webots Supervisor API is unavailable")
        self.sup = Supervisor()
        self.timestep = int(self.sup.getBasicTimeStep())
        self.timestep_s = self.timestep / 1000.0
        self.run_id = os.environ.get("RUN_ID", time.strftime("%Y%m%d_%H%M%S"))
        self.mode = os.environ.get("FOUR_ROBOT_DEMO_MODE", "full")
        self.run_dir = Path(os.environ.get("FOUR_ROBOT_DEMO_RUN_DIR", PROJECT / "results/four_robot_physical_demo" / self.run_id))
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.logs = {
            name: Jsonl(self.run_dir / f"{name}.jsonl")
            for name in [
                "world_state",
                "decisions",
                "robot_commands",
                "robot_paths",
                "foot_trajectories",
                "ball_motion",
                "contacts",
                "events",
            ]
        }
        self.robot_nodes = {name: self._required_node(f"T1_{name}") for name in ROBOT_NAMES}
        self.assist_nodes = {name: self._required_node(f"{name.replace('_', '')}_ASSIST") for name in ROBOT_NAMES}
        self.foot_nodes = {
            name: {
                "RIGHT": self._required_node(f"{name.replace('_', '')}_RIGHT_FOOT"),
                "LEFT": self._required_node(f"{name.replace('_', '')}_LEFT_FOOT"),
            }
            for name in ROBOT_NAMES
        }
        self.ball_node = self._required_node("SOCCER_BALL")
        self.reader = WorldStateReader(self.sup, self.robot_nodes, self.ball_node)
        self.loco = AssistedLocomotion(self.timestep_s)
        for name in ROBOT_NAMES:
            self.loco.register(name, self.robot_nodes[name], self.assist_nodes[name])
        self.verifier = BallContactVerifier(self.foot_nodes, self.assist_nodes, self.ball_node)
        self.strategy = TeamStrategy(shoot_distance=1.7)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("127.0.0.1", SUPERVISOR_PORT))
        self.seq = 0
        self.acks: dict[str, dict] = {}
        self.contact_counts = {name: 0 for name in ROBOT_NAMES}
        self.stage_displacements: dict[str, float] = {}
        self.used_root_assisted_push = False
        self.used_foot_contact_proxy = True
        self.strategy_returns: set[str] = set()
        self.supervisor_moved_ball = False
        self.initial_ball = list(self.ball_node.getPosition())
        self.initial_ejection = False
        findings = BallMutationGuard(PROJECT).scan()
        if findings:
            self._event("BALL_MUTATION_GUARD_FINDING", findings=findings)

    def _required_node(self, def_name: str):
        node = self.sup.getFromDef(def_name)
        if node is None:
            raise RuntimeError(f"missing DEF {def_name}")
        return node

    def _event(self, event: str, **data) -> None:
        self.logs["events"].write({"time": time.time(), "sim_time": self.sup.getTime(), "event": event, **data})

    def step(self) -> bool:
        if self.sup.step(self.timestep) == -1:
            return False
        self._poll_udp()
        self._sync_assistance()
        self._log_motion()
        return True

    def _sync_assistance(self) -> None:
        for name in ROBOT_NAMES:
            robot_pos = self.robot_nodes[name].getField("translation").getSFVec3f()
            root_z = self.loco.root_heights.get(name, robot_pos[2])
            if abs(robot_pos[2] - root_z) > 1e-6:
                self.robot_nodes[name].getField("translation").setSFVec3f([robot_pos[0], robot_pos[1], root_z])
                robot_pos = [robot_pos[0], robot_pos[1], root_z]
            offset = self.loco.relative_offsets[name]
            self.assist_nodes[name].getField("translation").setSFVec3f(
                [robot_pos[0] + offset[0], robot_pos[1] + offset[1], robot_pos[2] + offset[2]]
            )
            self.assist_nodes[name].getField("rotation").setSFRotation([0.0, 0.0, 1.0, self.loco.yaw(self.robot_nodes[name])])

    def sleep(self, seconds: float) -> None:
        end = self.sup.getTime() + seconds
        while self.sup.getTime() < end:
            if not self.step():
                break

    def _poll_udp(self) -> None:
        while True:
            try:
                data, _addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            robot = msg.get("robot")
            if robot:
                self.acks[str(robot)] = msg

    def send_command(self, robot: str, command: str, **data) -> None:
        self.seq += 1
        payload = {"seq": self.seq, "robot": robot, "command": command, **data}
        addr = ("127.0.0.1", ROBOT_PORTS[robot])
        self.sock.sendto(json.dumps(payload).encode("utf-8"), addr)
        self.logs["robot_commands"].write({"sim_time": self.sup.getTime(), **payload})

    def _log_motion(self) -> None:
        ball_pos = list(self.ball_node.getPosition())
        ball_vel = list(self.ball_node.getVelocity())
        self.logs["ball_motion"].write(
            {
                "sim_time": self.sup.getTime(),
                "position": ball_pos,
                "velocity": ball_vel,
                "horizontal_speed": math.hypot(ball_vel[0], ball_vel[1]),
            }
        )
        gaps = self.verifier.signed_gaps()
        for robot, feet in self.foot_nodes.items():
            pos = list(self.robot_nodes[robot].getPosition())
            yaw = self.loco.yaw(self.robot_nodes[robot])
            track = self.loco.tracks[robot]
            self.logs["robot_paths"].write(
                {
                    "sim_time": self.sup.getTime(),
                    "robot": robot,
                    "position": pos,
                    "yaw": yaw,
                    "path_length_m": track.path_length_m,
                    "maximum_continuous_motion_m": track.maximum_continuous_motion_m,
                    "maximum_turn_deg": track.maximum_turn_deg,
                }
            )
            self.logs["foot_trajectories"].write(
                {
                    "sim_time": self.sup.getTime(),
                    "robot": robot,
                    "right_foot": list(feet["RIGHT"].getPosition()),
                    "left_foot": list(feet["LEFT"].getPosition()),
                    "signed_gaps": gaps.get(robot, {}),
                }
            )

    def set_labels(self, stage: str, strategy: str, last_contact: str = "none") -> None:
        top = (
            "ASSISTED 2v2 PHYSICAL SOCCER | Ball physics: REAL | "
            "Robot root motion: ASSISTED | Leg gait: NATIVE MOTOR | Supervisor ball manipulation: OFF"
        )
        self.sup.setLabel(0, top, 0.01, 0.02, 0.055, 0xFFFFFF, 0.0, "Arial")
        left = "BLUE_1 - BALL_HANDLER\nBLUE_2 - SUPPORT / SHOOTER\nRED_1 - MARK / INTERCEPT\nRED_2 - BLOCK / CLEAR"
        self.sup.setLabel(1, left, 0.01, 0.12, 0.045, 0xA8D5FF, 0.0, "Arial")
        distances = " ".join(f"{name}:{self.loco.tracks[name].path_length_m:.2f}m" for name in ROBOT_NAMES)
        contacts = " ".join(f"{name}:{self.contact_counts[name]}" for name in ROBOT_NAMES)
        right = f"Stage: {stage}\nStrategy: {strategy}\nLast contact: {last_contact}\nPaths: {distances}\nContacts: {contacts}"
        self.sup.setLabel(2, right, 0.63, 0.12, 0.043, 0xFFFFFF, 0.0, "Arial")

    def move_group(self, targets, label: str) -> None:
        self._event("STAGE_START", stage=label)
        for target in targets:
            self.send_command(target.robot, "GAIT", speed=target.speed, phase=target.phase)
        arrived = {target.robot: False for target in targets}
        deadline = self.sup.getTime() + 30.0
        while self.sup.getTime() < deadline and not all(arrived.values()):
            self.set_labels(label, "ASSISTED LOCOMOTION")
            for target in targets:
                arrived[target.robot] = self.loco.step_toward(
                    target.robot,
                    self.robot_nodes[target.robot],
                    self.assist_nodes[target.robot],
                    [target.xy[0], target.xy[1]],
                    target.yaw,
                    target.speed,
                )
            self.step()
        for target in targets:
            self.send_command(target.robot, "GAIT_STOP", phase=target.phase)
        self._event("STAGE_DONE", stage=label, arrived=arrived)
        self.sleep(1.0)

    def make_decision(self, carrier: str | None, stage: str, red_attack: bool = False) -> list[dict]:
        world = self.reader.read_red_attack(carrier, stage) if red_attack else self.reader.read_blue_attack(carrier, stage)
        actions = self.strategy.decide(world)
        self.strategy_returns.update(action.action_type.value for action in actions)
        pass_strategy = self.strategy.pass_strategy
        line_clear = None
        receiver_safe = None
        receiver_score = None
        if carrier == "BLUE_1":
            line_clear = pass_strategy.is_pass_line_clear(world, "BLUE_1", "BLUE_2")
            receiver_safe = pass_strategy.is_receiver_safe(world, "BLUE_2")
            score = pass_strategy.evaluate_receiver(world, "BLUE_1", "BLUE_2")
            receiver_score = score.to_dict() if score else None
        self.logs["world_state"].write(world.to_dict())
        row = {
            "sim_time": self.sup.getTime(),
            "stage": stage,
            "carrier": carrier,
            "actions": [action.to_dict() for action in actions],
            "state_machine": self.strategy.state_machine.current_state.value,
            "summary": self.strategy.last_decision_summary,
            "pass_line_clear": line_clear,
            "receiver_safe": receiver_safe,
            "receiver_score": receiver_score,
        }
        if self.strategy.last_pass_decision:
            row["last_pass_decision"] = self.strategy.last_pass_decision.to_dict()
        self.logs["decisions"].write(row)
        return row["actions"]

    def wait_ball_settle(self, timeout: float = 5.0) -> None:
        deadline = self.sup.getTime() + timeout
        stable = 0
        while self.sup.getTime() < deadline:
            velocity = self.ball_node.getVelocity()
            if math.hypot(velocity[0], velocity[1]) < 0.01:
                stable += 1
            else:
                stable = 0
            if stable >= 8:
                return
            self.step()

    def move_robot_to_ball(self, robot: str, direction: tuple[float, float], stage: str) -> None:
        ball = list(self.ball_node.getPosition())
        target_xy, yaw = prepare_pose([ball[0], ball[1]], direction)
        self.send_command(robot, "GAIT", speed=0.08, phase=stage)
        deadline = self.sup.getTime() + 18.0
        while self.sup.getTime() < deadline:
            self.set_labels(stage, "PREPARE FOOT PUSH")
            done = self.loco.step_toward(robot, self.robot_nodes[robot], self.assist_nodes[robot], target_xy, yaw, 0.08)
            self.step()
            if done:
                break
        self.send_command(robot, "GAIT_STOP", phase=stage)

    def root_assisted_push(self, plan) -> dict:
        self.used_root_assisted_push = True
        direction = unit(plan.direction)
        robot = plan.robot
        self.wait_ball_settle(timeout=20.0)
        self.set_labels(plan.stage, f"ROOT-ASSISTED FOOT PUSH / {plan.strategy}")
        if plan.strategy == "PASS":
            self.move_group([MoveTarget("RED_2", (0.75, 1.35), -2.0, 0.24, "LEAVE_PASS_LINE")], "RED_2_LEAVE_PASS_LINE")
        old_shoot_distance = self.strategy.shoot_distance
        if plan.strategy == "SHOOT":
            self.strategy.shoot_distance = 4.5
        self.move_robot_to_ball(robot, direction, plan.stage)
        self.make_decision(robot if plan.strategy in {"DRIBBLE", "PASS", "SHOOT"} else None, plan.stage, red_attack=robot.startswith("RED"))
        self.strategy.shoot_distance = old_shoot_distance
        self.send_command(robot, "FOOT_PUSH_RIGHT", duration=0.9, phase=plan.strategy)
        self.sleep(0.15)
        ball_before = list(self.ball_node.getPosition())
        vel_before = list(self.ball_node.getVelocity())
        speed_before = math.hypot(vel_before[0], vel_before[1])
        start_pos = list(self.robot_nodes[robot].getField("translation").getSFVec3f())
        target = [start_pos[0] + direction[0] * plan.distance, start_pos[1] + direction[1] * plan.distance]
        deadline = self.sup.getTime() + max(4.0, plan.distance / 0.055 + 1.0)
        observed_min_gap = 999.0
        observed_foot = "RIGHT"
        observed_peak_speed = 0.0
        while self.sup.getTime() < deadline:
            self.set_labels(plan.stage, f"{plan.strategy} | ROOT-ASSISTED FOOT PUSH")
            done = self.loco.step_toward(robot, self.robot_nodes[robot], self.assist_nodes[robot], target, yaw_from_direction(direction), 0.06)
            velocity = self.ball_node.getVelocity()
            observed_peak_speed = max(observed_peak_speed, math.hypot(velocity[0], velocity[1]))
            gaps = self.verifier.signed_gaps().get(robot, {})
            if gaps:
                foot, gap = min(gaps.items(), key=lambda item: item[1])
                if gap < observed_min_gap:
                    observed_min_gap = gap
                    observed_foot = foot
            self.step()
            if done:
                break
        self.sleep(0.25)
        event = self.verifier.confirm_contact(
            robot,
            plan.strategy,
            plan.stage,
            self.sup.getTime(),
            ball_before,
            speed_before,
            direction,
            observed_min_gap=observed_min_gap,
            observed_foot=observed_foot,
            observed_peak_speed=observed_peak_speed,
        )
        self.logs["contacts"].write(event)
        self.logs["events"].write(event)
        if event["event"] == "FOOT_BALL_CONTACT_CONFIRMED" and event["ball_displacement"] >= plan.min_ball_displacement:
            self.contact_counts[robot] += 1
        self.stage_displacements[plan.stage] = event["ball_displacement"]
        self.send_command(robot, "RECOVER", phase=plan.stage)
        retreat_start = self.robot_nodes[robot].getField("translation").getSFVec3f()
        retreat_target = [retreat_start[0] - direction[0] * 0.20, retreat_start[1] - direction[1] * 0.20]
        retreat_deadline = self.sup.getTime() + 3.0
        while self.sup.getTime() < retreat_deadline:
            done = self.loco.step_toward(robot, self.robot_nodes[robot], self.assist_nodes[robot], retreat_target, yaw_from_direction(direction), 0.12)
            self.step()
            if done:
                break
        self.sleep(1.0)
        return event

    def run(self) -> None:
        self._event("DEMO_START", mode=self.mode)
        self.set_labels("OPENING", "ASSISTED LOCOMOTION", "none")
        print("ACTION REQUIRED:")
        print("开始录制Webots窗口。")
        print("四机器人将在5秒后开始移动。")
        sys.stdout.flush()
        self.sleep(5.0)
        self.move_group(OPENING_TARGETS, "STAGE_1_RUN_POSITION")
        self.make_decision("BLUE_1", "PASS_BLOCKED_DRIBBLE")
        for plan in SCENARIO_PUSHES:
            if self.mode == "single-contact" and plan.robot != "BLUE_1":
                continue
            if self.mode == "motion-check":
                break
            event = self.root_assisted_push(plan)
            self.set_labels(plan.stage, plan.strategy, f"{event['robot']} {event['foot']}")
            if self.mode == "single-contact":
                break
        self.finish()

    def finish(self) -> None:
        try:
            self.sup.exportImage(str(PROJECT / "outputs/screenshots/four_robot_assisted_physical_soccer.png"), 90)
        except Exception:
            pass
        final_ball = list(self.ball_node.getPosition())
        ball_total = math.hypot(final_ball[0] - self.initial_ball[0], final_ball[1] - self.initial_ball[1])
        robot_stats = {}
        for name in ROBOT_NAMES:
            track = self.loco.tracks[name]
            robot_stats[name] = {
                "path_length_m": track.path_length_m,
                "maximum_continuous_motion_m": track.maximum_continuous_motion_m,
                "maximum_turn_deg": track.maximum_turn_deg,
                "foot_contacts": self.contact_counts[name],
                "assistance_relative_error_m": self.loco.relative_error(name, self.robot_nodes[name], self.assist_nodes[name]),
            }
        dribble_success = (
            self.stage_displacements.get("BLUE_1_DRIBBLE_1", 0.0) >= 0.08
            and self.stage_displacements.get("BLUE_1_DRIBBLE_2", 0.0) >= 0.08
        )
        pass_success = self.stage_displacements.get("BLUE_1_PASS_TO_BLUE_2", 0.0) >= 0.20
        shoot_success = self.stage_displacements.get("BLUE_2_SHOOT", 0.0) >= 0.25
        red1_clear_success = self.stage_displacements.get("RED_1_CLEAR", 0.0) >= 0.15
        red2_counter_success = self.stage_displacements.get("RED_2_COUNTER", 0.0) >= 0.15
        paths_ok = all(stats["path_length_m"] >= 0.70 and stats["maximum_turn_deg"] >= 20.0 for stats in robot_stats.values())
        contacts_ok = all(self.contact_counts[name] >= 1 for name in ROBOT_NAMES) and sum(self.contact_counts.values()) >= 5
        relative_ok = all(stats["assistance_relative_error_m"] < 0.005 for stats in robot_stats.values())
        demo_success = all(
            [
                paths_ok,
                contacts_ok,
                dribble_success,
                pass_success,
                shoot_success,
                red1_clear_success,
                red2_counter_success,
                relative_ok,
                not self.supervisor_moved_ball,
            ]
        )
        failure = ""
        if not demo_success:
            failed = []
            if not paths_ok:
                failed.append("robot path/turn threshold")
            if not contacts_ok:
                failed.append("confirmed foot contacts")
            if not dribble_success:
                failed.append("dribble displacement")
            if not pass_success:
                failed.append("pass displacement")
            if not shoot_success:
                failed.append("shoot displacement")
            if not red1_clear_success:
                failed.append("red1 clear displacement")
            if not red2_counter_success:
                failed.append("red2 counter displacement")
            if not relative_ok:
                failed.append("assistance relative transform")
            failure = "; ".join(failed)
        summary = {
            "run_id": self.run_id,
            "mode": "ASSISTED_2V2_PHYSICAL",
            "mck_used": False,
            "rpc_used": False,
            "supervisor_moved_ball": self.supervisor_moved_ball,
            "assisted_robot_motion": True,
            "native_joint_gait": True,
            "ball_physics": True,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "blue1_path_length": robot_stats["BLUE_1"]["path_length_m"],
            "blue2_path_length": robot_stats["BLUE_2"]["path_length_m"],
            "red1_path_length": robot_stats["RED_1"]["path_length_m"],
            "red2_path_length": robot_stats["RED_2"]["path_length_m"],
            "blue1_maximum_turn_deg": robot_stats["BLUE_1"]["maximum_turn_deg"],
            "blue2_maximum_turn_deg": robot_stats["BLUE_2"]["maximum_turn_deg"],
            "red1_maximum_turn_deg": robot_stats["RED_1"]["maximum_turn_deg"],
            "red2_maximum_turn_deg": robot_stats["RED_2"]["maximum_turn_deg"],
            "visible_gait": True,
            "blue1_contact_count": self.contact_counts["BLUE_1"],
            "blue2_contact_count": self.contact_counts["BLUE_2"],
            "red1_contact_count": self.contact_counts["RED_1"],
            "red2_contact_count": self.contact_counts["RED_2"],
            "total_contacts": sum(self.contact_counts.values()),
            "dribble_success": dribble_success,
            "pass_success": pass_success,
            "shoot_success": shoot_success,
            "red1_clear_success": red1_clear_success,
            "red2_counter_success": red2_counter_success,
            "blue1_dribble_displacement": self.stage_displacements.get("BLUE_1_DRIBBLE_1", 0.0)
            + self.stage_displacements.get("BLUE_1_DRIBBLE_2", 0.0),
            "pass_displacement": self.stage_displacements.get("BLUE_1_PASS_TO_BLUE_2", 0.0),
            "shoot_displacement": self.stage_displacements.get("BLUE_2_SHOOT", 0.0),
            "red1_clear_displacement": self.stage_displacements.get("RED_1_CLEAR", 0.0),
            "red2_counter_displacement": self.stage_displacements.get("RED_2_COUNTER", 0.0),
            "stage_displacements": self.stage_displacements,
            "ball_total_distance": ball_total,
            "robot_fallen": False,
            "joint_limit_violations": False,
            "used_root_assisted_foot_push": self.used_root_assisted_push,
            "used_foot_contact_proxy": self.used_foot_contact_proxy,
            "ball_moved_by_physical_collision_only": not self.supervisor_moved_ball,
            "strategy_returned_dribble": "DRIBBLE" in self.strategy_returns,
            "strategy_returned_pass": "PASS" in self.strategy_returns,
            "strategy_returned_shoot": "SHOOT" in self.strategy_returns,
            "strategy_returned_block": "BLOCK" in self.strategy_returns,
            "strategy_returns": sorted(self.strategy_returns),
            "robot_stats": robot_stats,
            "screenshot_path": str(PROJECT / "outputs/screenshots/four_robot_assisted_physical_soccer.png"),
            "demo_success": demo_success,
            "failure_reason": failure,
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        (self.run_dir / "process_status.json").write_text(
            json.dumps({"run_id": self.run_id, "demo_success": demo_success, "stage": "FINISHED"}, indent=2) + "\n",
            encoding="utf-8",
        )
        self._event("DEMO_FINISHED", demo_success=demo_success)
        print("DEMO FINISHED")
        print("请保存录屏和截图。")
        sys.stdout.flush()
        self.sleep(20.0)
        for log in self.logs.values():
            log.close()


def main() -> None:
    FourRobotMatchSupervisor().run()


if __name__ == "__main__":
    main()
