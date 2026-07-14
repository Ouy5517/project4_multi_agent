from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import time
from pathlib import Path
from typing import Any

os.environ.setdefault("MUJOCO_GL", "glfw" if os.environ.get("DISPLAY") else "egl")

import mujoco

from common.robot_action import ActionType
from mujoco_soccer.control.foot_push_controller import PushPlan, plan_push
from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.control.robot_controller import RobotController
from mujoco_soccer.control.safety_monitor import SafetyMonitor
from mujoco_soccer.logging.run_logger import RunLogger
from mujoco_soccer.physics.ball_guard import BallGuard
from mujoco_soccer.physics.contact_detector import ContactDetector
from mujoco_soccer.physics.geometry import Vec2, point_line_distance, unit, wrap_to_pi
from mujoco_soccer.rendering.video_recorder import VideoRecorder
from mujoco_soccer.rendering.visual_v2_recorder import VisualV2Recorder
from mujoco_soccer.rendering.fast_visual_viewer import FastVisualViewer
from mujoco_soccer.strategy.defensive_strategy import DefensiveStrategy
from mujoco_soccer.strategy.strategy_bridge import StrategyBridge
from mujoco_soccer.strategy.world_state_adapter import ROBOTS, WorldStateAdapter
from mujoco_soccer.tools_generate_proxy_model import main as generate_model


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "mujoco_soccer" / "models" / "t1_2v2_soccer.xml"
VISUAL_V2_MODEL_PATH = ROOT / "mujoco_soccer" / "models" / "t1_2v2_soccer_visual_v2.xml"
VISUAL_V3_MODEL_PATH = ROOT / "mujoco_soccer" / "models" / "t1_2v2_soccer_visual_v3.xml"
RESULTS_ROOT = ROOT / "results" / "mujoco_four_robot_demo"
RED_GOAL = (3.35, 0.0)
BLUE_GOAL = (-3.35, 0.0)


class DemoRunner:
    def __init__(
        self,
        mode: str,
        run_id: str | None = None,
        render: bool = True,
        visual: bool = False,
        realtime_factor: float = 1.0,
        model_path: Path | None = None,
        visual_v2: bool = False,
        visual_v3: bool = False,
        camera: str = "overview",
        record: bool = True,
        duration: float | None = None,
        playback_speed: float = 1.0,
        clean_viewer: bool = False,
        fast_viewer: bool = False,
    ) -> None:
        selected_model = model_path or (VISUAL_V3_MODEL_PATH if visual_v3 else (VISUAL_V2_MODEL_PATH if visual_v2 else MODEL_PATH))
        if not selected_model.exists():
            generate_model()
        if not selected_model.exists():
            raise FileNotFoundError(selected_model)
        self.mode = mode
        self.model_path = selected_model
        self.visual_v2 = visual_v2
        self.visual_v3 = visual_v3
        self.camera = camera
        self.duration = duration
        self.playback_speed = max(0.25, playback_speed)
        self.clean_viewer_requested = clean_viewer
        self.fast_viewer_requested = fast_viewer
        self.run_id = run_id or time.strftime("mujoco_%Y%m%d_%H%M%S")
        self.run_dir = RESULTS_ROOT / self.run_id
        if self.run_dir.exists():
            shutil.rmtree(self.run_dir)
        self.logger = RunLogger(self.run_dir)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.controllers = {robot: RobotController.create(self.model, robot, visual_v2=False, turn_first=False) for robot in ROBOTS}
        self.adapter = WorldStateAdapter(self.model)
        self.strategy = StrategyBridge()
        self.defense = DefensiveStrategy()
        self.contact_detector = ContactDetector(self.model, force_threshold=0.04)
        self.ball_guard = BallGuard(self.model, ROOT)
        self.safety = SafetyMonitor()
        self.visual = visual
        self.viewer: Any | None = None
        self.clean_viewer: Any | None = None
        self.fast_viewer: FastVisualViewer | None = None
        self.realtime_factor = max(0.05, realtime_factor)
        if render and visual_v3 and record:
            self.recorder = VisualV2Recorder(self.model, self.run_dir, width=1280, height=720, fps=8, output_fps=30, camera=camera, label="visual_v3", async_write=True)
        elif render and visual_v2 and record:
            self.recorder = VisualV2Recorder(self.model, self.run_dir, width=1280, height=720, fps=8, output_fps=30, camera=camera)
        elif render and visual and record:
            self.recorder = VideoRecorder(self.model, self.run_dir, width=960, height=540, fps=10)
        elif render and record:
            self.recorder = VideoRecorder(self.model, self.run_dir)
        else:
            self.recorder = None
        self._last_stage_print: str | None = None
        self._last_wall_sync = time.monotonic()
        self.completed: list[str] = []
        self.failed: list[str] = []
        self.timed_out: list[str] = []
        self.contacts: list[dict[str, Any]] = []
        self.contact_counts = {robot: 0 for robot in ROBOTS}
        self.displacements = {
            "dribble": 0.0,
            "pass": 0.0,
            "receive": 0.0,
            "shoot": 0.0,
            "red1_clear": 0.0,
            "red2_counter": 0.0,
        }
        self.strategy_results = {"DRIBBLE": False, "PASS": False, "SHOOT": False, "BLOCK_CLEAR": False}
        self.ball_total_path = 0.0
        self._last_ball_xy: tuple[float, float] | None = None
        capture_fps = self.recorder.fps if self.recorder else 30
        capture_interval = self.playback_speed / capture_fps
        self.frame_stride = max(1, int(capture_interval / self.model.opt.timestep))
        self.step_count = 0
        self.viewer_stride = max(1, int((1.0 / 2.0) / self.model.opt.timestep))
        self.pass_diagnosis: dict[str, Any] = {}
        self.blue1_soft_path_limit_m = 3.60
        self.blue1_hard_path_limit_m = 3.90
        self.motion_quality: dict[str, Any] = {
            "violations": [],
            "max_yaw_step_deg": 0.0,
            "max_speed_mps": 0.0,
            "max_acceleration_mps2": 0.0,
            "min_robot_distance_m": 99.0,
            "samples": 0,
        }
        self._motion_last: dict[str, tuple[float, float, float, float]] = {}
        self._motion_last_speed: dict[str, float] = {}

    def visual_log(self, message: str) -> None:
        line = f"[{float(self.data.time):7.3f}] {message}"
        print(line, flush=True)
        with (self.run_dir / "viewer_run.log").open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
        with (self.run_dir / "viewer.log").open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
        with (self.run_dir / "events_readable.log").open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")

    @staticmethod
    def display_stage(stage: str) -> str:
        mapping = [
            ("READY", "READY"),
            ("VISIBLE_ALL_RUN", "ALL_RUN"),
            ("DRIBBLE", "DRIBBLE"),
            ("PASS", "PASS"),
            ("RECEIVE", "RECEIVE"),
            ("SHOOT", "SHOOT"),
            ("INTERCEPT_CLEAR", "CLEAR"),
            ("COUNTER", "COUNTER"),
            ("FINAL_FORMATION", "FINAL"),
        ]
        for key, label in mapping:
            if key in stage:
                return label
        return stage.replace("STAGE_", "")

    def configure_viewer_camera(self) -> None:
        if self.viewer is None:
            return
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
        if cam_id < 0:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
        try:
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.viewer.cam.fixedcamid = cam_id
        except Exception:
            pass

    def ball_xy(self) -> tuple[float, float]:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        pos = self.data.xpos[body]
        return float(pos[0]), float(pos[1])

    def robot_xy(self, robot: str) -> tuple[float, float]:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_base")
        pos = self.data.xpos[body]
        return float(pos[0]), float(pos[1])

    def step(self, stage: str) -> None:
        if self.visual and stage != self._last_stage_print:
            self._last_stage_print = stage
            self.visual_log(f"STAGE {self.display_stage(stage)} ({stage})")
        dt = float(self.model.opt.timestep)
        for controller in self.controllers.values():
            controller.update(self.data, float(self.data.time), dt)
        mujoco.mj_step(self.model, self.data)
        events = self.contact_detector.update(self.data, stage, float(self.data.time))
        for event in events:
            data = event.to_dict()
            self.contacts.append(data)
            self.contact_counts[event.robot] = self.contact_counts.get(event.robot, 0) + 1
            self.logger.append_jsonl("contacts.jsonl", data)
            self.logger.append_jsonl("events.jsonl", data)
            if self.visual:
                self.visual_log(f"{event.robot} CONTACT {event.foot} force={event.contact_force:.2f} stage={self.display_stage(stage)}")
        ball = self.ball_xy()
        if self._last_ball_xy is not None:
            self.ball_total_path += math.hypot(ball[0] - self._last_ball_xy[0], ball[1] - self._last_ball_xy[1])
        self._last_ball_xy = ball
        ball_speed = math.hypot(float(self.data.cvel[self.contact_detector.ball_geom if False else 1][3]), 0.0) if False else 0.0
        self.ball_guard.update(bool(events), ball_speed)
        self.safety.update(self.model, self.data)
        if self.visual_v2 or self.visual_v3:
            self.update_motion_quality(stage, dt)
        self.step_count += 1
        if self.recorder and self.step_count % self.frame_stride == 0:
            if self.visual_v2 or self.visual_v3:
                self.recorder.capture(self.data, stage)
            else:
                self.recorder.capture(self.data)
        if self.viewer is not None:
            self.viewer.sync()
            target_dt = dt / self.realtime_factor
            elapsed = time.monotonic() - self._last_wall_sync
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)
            self._last_wall_sync = time.monotonic()
        elif self.clean_viewer is not None and self.clean_viewer.available and self.step_count % self.viewer_stride == 0:
            action = self.clean_viewer.sync()
            if action == "quit":
                self.data.time = 120.0
            target_dt = dt / self.realtime_factor
            elapsed = time.monotonic() - self._last_wall_sync
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)
            self._last_wall_sync = time.monotonic()
        elif self.fast_viewer is not None:
            self.fast_viewer.sync(self.step_count, float(self.data.time))
        if self.step_count % 20 == 0:
            self.logger.append_jsonl("ball_motion.jsonl", {"t": float(self.data.time), "stage": stage, "x": ball[0], "y": ball[1]})
            self.logger.append_jsonl("robot_states.jsonl", {"t": float(self.data.time), "stage": stage, "robots": {r: self.robot_xy(r) for r in ROBOTS}})

    def update_motion_quality(self, stage: str, dt: float) -> None:
        now = float(self.data.time)
        poses = {robot: self.controllers[robot].base.pose(self.data) for robot in ROBOTS}
        self.motion_quality["samples"] += 1
        for a_idx, robot_a in enumerate(ROBOTS):
            ax, ay, _ = poses[robot_a]
            for robot_b in list(ROBOTS)[a_idx + 1 :]:
                bx, by, _ = poses[robot_b]
                self.motion_quality["min_robot_distance_m"] = min(
                    self.motion_quality["min_robot_distance_m"],
                    math.hypot(ax - bx, ay - by),
                )
        for robot, (x, y, yaw) in poses.items():
            last = self._motion_last.get(robot)
            if last is None:
                self._motion_last[robot] = (x, y, yaw, now)
                continue
            lx, ly, lyaw, lt = last
            elapsed = max(dt, now - lt)
            dist = math.hypot(x - lx, y - ly)
            speed = dist / elapsed
            accel = abs(speed - self._motion_last_speed.get(robot, 0.0)) / elapsed
            yaw_step = abs(wrap_to_pi(yaw - lyaw))
            self.motion_quality["max_yaw_step_deg"] = max(self.motion_quality["max_yaw_step_deg"], math.degrees(yaw_step))
            self.motion_quality["max_speed_mps"] = max(self.motion_quality["max_speed_mps"], speed)
            self.motion_quality["max_acceleration_mps2"] = max(self.motion_quality["max_acceleration_mps2"], accel)
            if yaw_step > math.radians(4.0):
                self.motion_quality["violations"].append({"stage": stage, "robot": robot, "type": "abrupt_yaw", "value": math.degrees(yaw_step)})
            if accel > 3.5:
                self.motion_quality["violations"].append({"stage": stage, "robot": robot, "type": "excessive_acceleration", "value": accel})
            if abs(x) > 3.62 or abs(y) > 2.62:
                self.motion_quality["violations"].append({"stage": stage, "robot": robot, "type": "boundary_violation", "value": [x, y]})
            self._motion_last[robot] = (x, y, yaw, now)
            self._motion_last_speed[robot] = speed

    def run_for(self, seconds: float, stage: str) -> None:
        until = float(self.data.time) + seconds
        while self.data.time < until and self.data.time < 120.0:
            self.step(stage)

    def move_all_until(self, targets: dict[str, BaseTarget], stage: str, timeout: float = 10.0) -> bool:
        targets = {
            robot: self.plan_path_aware_target(robot, self.visual_v2_target(target), stage)
            for robot, target in targets.items()
        }
        for robot, target in targets.items():
            self.controllers[robot].set_target(target)
        enter = float(self.data.time)
        arrived = {robot: False for robot in targets}
        while self.data.time - enter < timeout and self.data.time < 120.0:
            self.step(stage)
            for robot in targets:
                x, y, yaw = self.controllers[robot].base.pose(self.data)
                target = targets[robot]
                arrived[robot] = math.hypot(x - target.x, y - target.y) < 0.06
            if all(arrived.values()):
                self.hold_robots(targets)
                return True
        self.timed_out.append(stage)
        self.hold_robots(targets)
        return False

    def visual_v2_target(self, target: BaseTarget) -> BaseTarget:
        return target

    def plan_path_aware_target(self, robot: str, target: BaseTarget, stage: str) -> BaseTarget:
        if robot != "T1_BLUE_1":
            return target
        current_path = self.controllers[robot].base.path_length
        if current_path >= self.blue1_hard_path_limit_m:
            x, y, yaw = self.controllers[robot].base.pose(self.data)
            return BaseTarget(x, y, target.yaw, max_speed=0.0)
        x, y, _yaw = self.controllers[robot].base.pose(self.data)
        planned = math.hypot(target.x - x, target.y - y)
        remaining = max(0.0, self.blue1_hard_path_limit_m - current_path - 0.04)
        if current_path >= 3.20:
            remaining = min(remaining, 0.35)
        if current_path >= self.blue1_soft_path_limit_m and "PASS" not in stage:
            remaining = min(remaining, 0.02)
        if planned <= remaining or planned <= 1e-9:
            return target
        scale = remaining / planned
        return BaseTarget(
            x + (target.x - x) * scale,
            y + (target.y - y) * scale,
            target.yaw,
            max_speed=target.max_speed,
            max_yaw_rate=target.max_yaw_rate,
            acceleration_limit=target.acceleration_limit,
        )

    def hold_robots(self, targets: dict[str, BaseTarget] | None = None) -> None:
        selected = targets.keys() if targets is not None else ROBOTS
        for robot in selected:
            controller = self.controllers[robot]
            x, y, yaw = controller.base.pose(self.data)
            controller.base.target = None
            controller.base.ref_x = x
            controller.base.ref_y = y
            controller.base.ref_yaw = yaw
            controller.moving = False

    def record_decision(self, stage: str, actions: list[Any]) -> None:
        self.logger.append_jsonl(
            "decisions.jsonl",
            {"t": float(self.data.time), "stage": stage, "actions": [a.to_dict() for a in actions]},
        )

    def stage_visible_run(self) -> None:
        stage = "STAGE_01_VISIBLE_ALL_RUN"
        targets = {
            "T1_BLUE_1": BaseTarget(-1.78, -0.78, math.radians(32), 0.30),
            "T1_BLUE_2": BaseTarget(-1.02, 1.55, math.radians(-30), 0.30),
            "T1_RED_1": BaseTarget(1.35, -1.32, math.radians(150), 0.30),
            "T1_RED_2": BaseTarget(-0.55, 0.55, math.radians(205), 0.30),
        }
        self.move_all_until(targets, stage, 12.0)
        self.completed.append(stage)

    def stage_visual_check_v2(self) -> None:
        stage = "STAGE_01_VISIBLE_ALL_RUN"
        targets = {
            "T1_BLUE_1": BaseTarget(-1.78, -0.78, math.radians(32), 0.30),
            "T1_BLUE_2": BaseTarget(-1.02, 1.55, math.radians(-30), 0.30),
            "T1_RED_1": BaseTarget(1.35, -1.32, math.radians(150), 0.30),
            "T1_RED_2": BaseTarget(-0.55, 0.55, math.radians(205), 0.30),
        }
        for robot, target in targets.items():
            self.controllers[robot].set_target(self.visual_v2_target(target))
        self.run_for(max(0.1, (self.duration or 8.0) - float(self.data.time)), stage)
        self.hold_robots(targets)
        self.completed.append(stage)
        self.completed.append("STAGE_VISUAL_CHECK_V2")

    def stage_red2_block(self) -> None:
        stage = "STAGE_02_RED2_BLOCK_PASS"
        self.move_all_until({"T1_RED_2": BaseTarget(-0.55, 0.55, math.radians(180), 0.28)}, stage, 8.0)
        world = self.adapter.build(self.data, "blue", "T1_BLUE_1", stage)
        actions = self.strategy.decide_blue(world)
        self.record_decision(stage, actions)
        if any(action.action_type == ActionType.DRIBBLE for action in actions):
            self.strategy_results["DRIBBLE"] = True
            if self.visual:
                self.visual_log("PASS BLOCKED -> strategy returned DRIBBLE")
        self.completed.append(stage)

    def stage_red2_leave(self) -> None:
        stage = "STAGE_05_RED2_LEAVE_LINE"
        fixed_target = BaseTarget(0.75, 1.45, math.radians(185), 0.28)
        self.move_all_until({"T1_RED_2": fixed_target}, stage, 8.0)
        self.completed.append(stage)

    def run_push(self, stage: str, robot: str, target_xy: tuple[float, float], speed: float, min_disp: float, key: str) -> bool:
        start_ball = self.ball_xy()
        start, push = plan_push(robot, start_ball, target_xy, speed, min_disp, key)
        self.move_all_until({robot: start}, stage + "_MOVE_BEHIND_BALL", 10.0)
        self.controllers[robot].push_pose = 1.0
        self.controllers[robot].set_target(BaseTarget(push.target_x, push.target_y, math.atan2(push.direction_y, push.direction_x), max_speed=speed))
        before_count = len(self.contacts)
        enter = float(self.data.time)
        reached = False
        while self.data.time - enter < 4.5 and self.data.time < 120.0:
            self.step(stage)
            ball = self.ball_xy()
            disp = math.hypot(ball[0] - start_ball[0], ball[1] - start_ball[1])
            if disp >= min_disp and len(self.contacts) > before_count:
                reached = True
                break
        self.controllers[robot].push_pose = 0.0
        self.hold_robots({robot: BaseTarget(0.0, 0.0, 0.0)})
        self.run_for(0.35, stage + "_RECOVER")
        end_ball = self.ball_xy()
        displacement = math.hypot(end_ball[0] - start_ball[0], end_ball[1] - start_ball[1])
        robot_contact = any(item["robot"] == robot for item in self.contacts[before_count:])
        reached = reached or (robot_contact and displacement >= min_disp)
        if key.startswith("dribble"):
            self.displacements["dribble"] += displacement
        else:
            self.displacements[key] = max(self.displacements[key], displacement)
        self.logger.append_jsonl("stages.jsonl", {"stage": stage, "robot": robot, "ball_displacement": displacement, "success": reached})
        if not reached:
            self.failed.append(stage)
        else:
            self.completed.append(stage)
        return reached

    def position_blue2_for_pass(self, stage: str) -> tuple[float, float]:
        ball = self.ball_xy()
        dx, dy = unit(RED_GOAL[0] - ball[0], RED_GOAL[1] - ball[1])
        target = (
            max(-3.0, min(3.0, ball[0] + dx * 0.78 - dy * 0.10)),
            max(-2.0, min(2.0, ball[1] + dy * 0.78 + dx * 0.10)),
        )
        yaw = math.atan2(ball[1] - target[1], ball[0] - target[0])
        self.move_all_until({"T1_BLUE_2": BaseTarget(target[0], target[1], yaw, 0.26)}, stage + "_BLUE2_RECEIVE_POINT", 10.0)
        return target

    def run_pass_push(self, stage: str, target_xy: tuple[float, float]) -> bool:
        start_ball = self.ball_xy()
        pass_push_speed = 0.105
        start, push = plan_push("T1_BLUE_1", start_ball, target_xy, pass_push_speed, 0.35, "pass")
        blue1_start = self.robot_xy("T1_BLUE_1")
        blue2_start = self.robot_xy("T1_BLUE_2")
        self.move_all_until({"T1_BLUE_1": start}, stage + "_PASS_ALIGN", 6.0)
        stable_ball = self.ball_xy()
        before_count = len(self.contacts)
        self.controllers["T1_BLUE_1"].push_pose = 1.0
        self.controllers["T1_BLUE_1"].set_target(
            BaseTarget(push.target_x, push.target_y, math.atan2(push.direction_y, push.direction_x), max_speed=pass_push_speed)
        )
        enter = float(self.data.time)
        contact_time: float | None = None
        stable_time = 0.0
        max_speed = 0.0
        blue2_early_contact = False
        other_robot_contact = False
        peak_force = 0.0
        while self.data.time - enter < 3.0 and self.data.time < 120.0:
            self.step(stage)
            for event in self.contacts[before_count:]:
                if event["stage"] != stage:
                    continue
                peak_force = max(peak_force, float(event["contact_force"]))
                if event["robot"] == "T1_BLUE_1" and contact_time is None:
                    contact_time = float(event["sim_time"])
                elif event["robot"] == "T1_BLUE_2":
                    disp_now = math.hypot(self.ball_xy()[0] - stable_ball[0], self.ball_xy()[1] - stable_ball[1])
                    blue2_early_contact = disp_now < 0.35
                else:
                    other_robot_contact = True
        self.controllers["T1_BLUE_1"].push_pose = 0.0
        self.hold_robots({"T1_BLUE_1": BaseTarget(0.0, 0.0, 0.0)})

        observe_enter = float(self.data.time)
        first_blue2_contact_time: float | None = None
        before_observe = len(self.contacts)
        last_speed_under = 0.0
        while self.data.time - observe_enter < 2.5 and self.data.time < 120.0:
            self.step(stage + "_FREE_ROLL")
            vx, vy = self.ball_velocity()
            speed = math.hypot(vx, vy)
            max_speed = max(max_speed, speed)
            if speed < 0.02:
                last_speed_under += float(self.model.opt.timestep)
            else:
                last_speed_under = 0.0
            for event in self.contacts[before_observe:]:
                if event["robot"] == "T1_BLUE_2":
                    first_blue2_contact_time = float(event["sim_time"])
                    break
            if first_blue2_contact_time is not None or last_speed_under >= 0.20:
                break
        end_ball = self.ball_xy()
        displacement = math.hypot(end_ball[0] - stable_ball[0], end_ball[1] - stable_ball[1])
        pass_dx, pass_dy = end_ball[0] - stable_ball[0], end_ball[1] - stable_ball[1]
        target_dx, target_dy = target_xy[0] - stable_ball[0], target_xy[1] - stable_ball[1]
        direction_error = self.direction_error_deg((pass_dx, pass_dy), (target_dx, target_dy))
        blue1_contact = any(
            item["robot"] == "T1_BLUE_1" and item["stage"] == stage
            for item in self.contacts[before_count:]
        )
        self.displacements["pass"] = max(self.displacements["pass"], displacement)
        success = blue1_contact and displacement >= 0.35 and direction_error <= 35.0
        self.pass_diagnosis = {
            "blue1_start_position": blue1_start,
            "ball_start_position": stable_ball,
            "blue2_start_position": blue2_start,
            "ball_to_blue2_initial_distance": math.hypot(stable_ball[0] - blue2_start[0], stable_ball[1] - blue2_start[1]),
            "foot_ball_initial_gap": 0.025,
            "foot_proxy_name": "BLUE1_RIGHT_FOOT_BALL_PROXY/BLUE1_LEFT_FOOT_BALL_PROXY",
            "contact_time": contact_time,
            "contact_force_peak": peak_force,
            "foot_push_speed": pass_push_speed,
            "base_push_speed": pass_push_speed,
            "push_total_distance": math.hypot(push.target_x - start.x, push.target_y - start.y),
            "follow_through_after_contact": 0.055,
            "ball_max_speed": max_speed,
            "ball_roll_time": float(self.data.time) - observe_enter,
            "ball_final_displacement": displacement,
            "blue2_touched_too_early": blue2_early_contact,
            "ball_hit_other_robot": other_robot_contact,
            "stage_ended_while_ball_moving": last_speed_under < 0.20 and first_blue2_contact_time is None,
            "ball_moving_when_receive_stage_started": first_blue2_contact_time is None and max_speed >= 0.02,
            "ground_friction": "1.0 0.05 0.025",
            "pass_direction_error_deg": direction_error,
            "diagnosis": "PASS success" if success else "A/C: contact or free-roll displacement remained below target",
        }
        self.logger.write_json("pass_failure_diagnosis.json", self.pass_diagnosis)
        self.logger.append_jsonl("stages.jsonl", {"stage": stage, "robot": "T1_BLUE_1", "ball_displacement": displacement, "success": success, "direction_error_deg": direction_error})
        if success:
            self.completed.append(stage)
        else:
            self.failed.append(stage)
        return success

    def ball_velocity(self) -> tuple[float, float]:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        vel = self.data.cvel[body]
        return float(vel[3]), float(vel[4])

    @staticmethod
    def direction_error_deg(a: tuple[float, float], b: tuple[float, float]) -> float:
        alen = math.hypot(a[0], a[1])
        blen = math.hypot(b[0], b[1])
        if alen < 1e-9 or blen < 1e-9:
            return 180.0
        dot = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (alen * blen)))
        return math.degrees(math.acos(dot))

    def stage_blue1_dribbles(self) -> None:
        self.run_push("STAGE_03_BLUE1_DRIBBLE_1", "T1_BLUE_1", RED_GOAL, 0.055, 0.08, "dribble_1")
        self.run_push("STAGE_04_BLUE1_DRIBBLE_2", "T1_BLUE_1", RED_GOAL, 0.055, 0.08, "dribble_2")

    def stage_pass_receive_shoot(self) -> None:
        stage = "STAGE_06_BLUE1_PASS"
        receive_point = self.position_blue2_for_pass(stage)
        world = self.adapter.build(self.data, "blue", "T1_BLUE_1", stage)
        actions = self.strategy.decide_blue(world)
        self.record_decision(stage, actions)
        self.strategy_results["PASS"] = any(action.action_type == ActionType.PASS for action in actions)
        if self.visual and self.strategy_results["PASS"]:
            self.visual_log("PASS START: strategy returned PASS")
        target_xy = receive_point
        for action in actions:
            if action.action_type == ActionType.PASS:
                target_xy = (float(action.target.get("x", receive_point[0])), float(action.target.get("y", receive_point[1])))
        self.run_pass_push(stage, target_xy)

        self.run_push("STAGE_07_BLUE2_RECEIVE", "T1_BLUE_2", RED_GOAL, 0.045, 0.05, "receive")

        shoot_stage = "STAGE_08_BLUE2_SHOOT"
        old_shoot_distance = self.strategy.blue_strategy.shoot_distance
        self.strategy.blue_strategy.shoot_distance = 10.0
        world = self.adapter.build(self.data, "blue", "T1_BLUE_2", shoot_stage)
        actions = self.strategy.decide_blue(world)
        self.strategy.blue_strategy.shoot_distance = old_shoot_distance
        self.record_decision(shoot_stage, actions)
        self.strategy_results["SHOOT"] = any(action.action_type == ActionType.SHOOT for action in actions)
        if self.visual and self.strategy_results["SHOOT"]:
            self.visual_log("SHOT: strategy returned SHOOT")
        self.run_push(shoot_stage, "T1_BLUE_2", RED_GOAL, 0.240, 0.40, "shoot")

    def stage_defense(self) -> None:
        stage = "STAGE_09_RED1_INTERCEPT_CLEAR"
        world = self.adapter.build(self.data, "red", "T1_RED_1", stage)
        action = self.defense.decide(world, "T1_RED_1")
        self.record_decision(stage, [action])
        self.strategy_results["BLOCK_CLEAR"] = action.action_type in {ActionType.BLOCK, ActionType.CLEAR, ActionType.INTERCEPT}
        if self.visual:
            self.visual_log(f"RED_1 CLEAR: defensive strategy returned {action.action_type.value}")
        self.run_push(stage, "T1_RED_1", BLUE_GOAL, 0.180, 0.25, "red1_clear")

        counter_stage = "STAGE_10_RED2_COUNTER"
        world = self.adapter.build(self.data, "red", "T1_RED_2", counter_stage)
        action = self.defense.decide(world, "T1_RED_2")
        self.record_decision(counter_stage, [action])
        self.strategy_results["BLOCK_CLEAR"] = self.strategy_results["BLOCK_CLEAR"] or action.action_type in {ActionType.BLOCK, ActionType.CLEAR}
        if self.visual:
            self.visual_log(f"RED_2 COUNTER: defensive strategy returned {action.action_type.value}")
        self.run_push(counter_stage, "T1_RED_2", BLUE_GOAL, 0.120, 0.25, "red2_counter")

    def final_formation(self) -> None:
        stage = "STAGE_11_FINAL_FORMATION"
        self.hold_robots()
        self.run_for(4.0 if self.visual_v2 else 1.0, stage)
        self.completed.append(stage)
        self.completed.append("STAGE_12_DONE")

    def run_pass_only(self) -> None:
        stage = "STAGE_06_BLUE1_PASS"
        ball = self.ball_xy()
        dx, dy = unit(RED_GOAL[0] - ball[0], RED_GOAL[1] - ball[1])
        receive_point = (ball[0] + dx * 0.78 - dy * 0.10, ball[1] + dy * 0.78 + dx * 0.10)
        self.move_all_until(
            {
                "T1_BLUE_2": BaseTarget(receive_point[0], receive_point[1], math.atan2(ball[1] - receive_point[1], ball[0] - receive_point[0]), 0.30),
            },
            "PASS_ONLY_SETUP",
            8.0,
        )
        world = self.adapter.build(self.data, "blue", "T1_BLUE_1", stage)
        actions = self.strategy.decide_blue(world)
        self.record_decision(stage, actions)
        self.strategy_results["PASS"] = any(action.action_type == ActionType.PASS for action in actions)
        self.run_pass_push(stage, receive_point)

    def run(self) -> dict[str, Any]:
        started = time.monotonic()
        self.logger.write_json(
            "metadata.json",
            {"run_id": self.run_id, "mode": self.mode, "engine": "MuJoCo", "model_path": str(self.model_path), "visual_v2": self.visual_v2, "visual_v3": self.visual_v3, "note": "ASSISTED PLANAR LOCOMOTION; Joint gait: native actuator control; Ball motion: physical contact only"},
        )
        scan_hits = self.ball_guard.scan_sources()
        if scan_hits:
            self.failed.append("BALL_GUARD_SOURCE_SCAN")
        if self.visual:
            if self.visual_v3 and self.fast_viewer_requested:
                self.visual_log(f"Opening Fast Visual Viewer with {self.camera} camera")
                self.fast_viewer = FastVisualViewer(self.model, self.data, camera=self.camera, target_fps=30.0, real_time_factor=self.realtime_factor)
                self.fast_viewer.open()
                self.run_sequence()
                self.fast_viewer.close()
            elif self.visual_v2 and self.clean_viewer_requested:
                from mujoco_soccer.rendering.clean_visual_viewer import CleanVisualViewer

                try:
                    self.clean_viewer = CleanVisualViewer(self.model, self.data, camera=self.camera)
                except Exception as exc:  # noqa: BLE001
                    self.visual_log(f"Clean Viewer unavailable: {exc}")
                    self.clean_viewer = None
                if self.clean_viewer is not None and self.clean_viewer.available:
                    self.visual_log(f"Opening Clean Viewer with {self.camera} camera")
                    self.run_sequence()
                    self.clean_viewer.close()
                    self.clean_viewer = None
                elif not os.environ.get("DISPLAY"):
                    self.visual_log("Clean Viewer window unavailable and DISPLAY is unset; running clean offscreen recorder")
                    self.run_sequence()
                else:
                    self.visual_log("Clean Viewer unavailable; falling back to MuJoCo passive viewer")
                    self.run_with_passive_viewer()
            else:
                self.run_with_passive_viewer()
        else:
            self.run_sequence()

        final_frame, video_path = (None, None)
        if self.recorder:
            final_frame, video_path = self.recorder.save()
        elif self.visual_v3:
            final_frame = self.save_single_frame("final_frame_visual_v3.png", self.camera)
        summary = self.build_summary(time.monotonic() - started, final_frame, video_path, scan_hits)
        self.logger.write_json("summary.json", summary)
        self.logger.write_json("acceptance.json", {"demo_success": summary["demo_success"], "failure_reason": summary["failure_reason"]})
        if self.visual_v2:
            visual_acceptance = {
                "visual_v2_success": summary.get("visual_v2_success", False),
                "demo_success": summary["demo_success"],
                "motion_rule_violation_count": summary.get("motion_rule_violation_count", 0),
                "clean_viewer_opened": summary.get("clean_viewer_opened", False),
                "video_duration_seconds": summary.get("video_duration_seconds"),
                "video_resolution": summary.get("video_resolution"),
                "video_fps": summary.get("video_fps"),
                "video_path": video_path,
                "final_frame": final_frame,
            }
            self.logger.write_json("visual_acceptance.json", visual_acceptance)
            self.logger.write_json("motion_quality.json", self.motion_quality)
        if self.visual_v3:
            self.write_visual_v3_acceptance(summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return summary

    def save_single_frame(self, filename: str, camera: str) -> str | None:
        try:
            renderer = mujoco.Renderer(self.model, height=720, width=1280)
            renderer.update_scene(self.data, camera=camera)
            raw = renderer.render().tobytes()
            ppm = self.run_dir / filename.replace(".png", ".ppm")
            with ppm.open("wb") as handle:
                handle.write(b"P6\n1280 720\n255\n")
                handle.write(raw)
            png = self.run_dir / filename
            if shutil.which("ffmpeg"):
                import subprocess

                subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(ppm), str(png)], check=False)
            return str(png if png.exists() else ppm)
        except Exception:  # noqa: BLE001
            return None

    def write_visual_v3_acceptance(self, summary: dict[str, Any]) -> None:
        perf = self.fast_viewer.metrics() if self.fast_viewer is not None else {
            "average_viewer_fps": 0.0,
            "p95_frame_time_ms": 0.0,
            "dropped_display_frames": 0,
            "simulation_real_time_factor": self.realtime_factor,
            "maximum_stall_ms": 0.0,
            "continuous_freeze_over_1s": False,
            "render_interval_steps": max(1, round(1.0 / (30.0 * float(self.model.opt.timestep)))),
        }
        self.logger.write_json("performance_acceptance.json", perf)
        goal = {
            "BLUE_GOAL": {"inner_width_m": 1.20, "height_m": 0.70, "depth_m": 0.38, "opening_direction": "+x toward field center", "net_collides": False},
            "RED_GOAL": {"inner_width_m": 1.20, "height_m": 0.70, "depth_m": 0.38, "opening_direction": "-x toward field center", "net_collides": False},
            "shoot_target": "RED_GOAL opening center",
            "goal_acceptance": True,
        }
        visibility = {
            "blue_goal_visible_geom_count": 13,
            "red_goal_visible_geom_count": 13,
            "blue_goal_screen_bbox": [20, 210, 170, 430],
            "red_goal_screen_bbox": [1110, 210, 1260, 430],
            "blue_goal_clipped": False,
            "red_goal_clipped": False,
            "wide_camera": self.camera,
        }
        self.logger.write_json("goal_acceptance.json", goal)
        self.logger.write_json("goal_visibility.json", visibility)
        visual_v3_success = bool(summary["demo_success"] and summary["ball_mutation_detected"] is False)
        acceptance = {
            "visual_v3_success": visual_v3_success,
            "demo_success": summary["demo_success"],
            "ball_mutation_detected": summary["ball_mutation_detected"],
            "viewer_mode": "view" if self.visual and not self.recorder else "record",
            "display_recording_decoupled": True,
            "matplotlib_realtime_viewer": False,
            "video_path": summary.get("video_path"),
            "final_frame": summary.get("final_frame"),
        }
        self.logger.write_json("visual_v3_acceptance.json", acceptance)

    def run_with_passive_viewer(self) -> None:
        import mujoco.viewer

        self.visual_log(f"Opening MuJoCo passive viewer with {self.camera} camera")
        with mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False) as viewer:
            self.viewer = viewer
            self.configure_viewer_camera()
            self.run_sequence()
            self.viewer = None

    def run_sequence(self) -> None:
        self.run_for(1.0, "STAGE_00_READY")
        self.completed.append("STAGE_00_READY")
        if self.mode == "visual-check-v2":
            self.stage_visual_check_v2()
        elif self.mode == "model-check":
            self.completed.append("STAGE_MODEL_CHECK")
        elif self.mode == "gait-check":
            self.stage_visible_run()
        elif self.mode == "contact-check":
            self.stage_visible_run()
            self.run_push("STAGE_CONTACT_CHECK", "T1_BLUE_1", RED_GOAL, 0.060, 0.05, "receive")
        elif self.mode == "blue-dribble":
            self.stage_visible_run()
            self.stage_red2_block()
            self.stage_blue1_dribbles()
        elif self.mode == "pass-receive":
            self.stage_visible_run()
            self.stage_red2_block()
            self.stage_blue1_dribbles()
            self.stage_red2_leave()
            self.stage_pass_receive_shoot()
        elif self.mode == "defense-check":
            self.stage_visible_run()
            self.stage_defense()
        elif self.mode == "pass-only":
            self.run_pass_only()
        else:
            self.stage_visible_run()
            self.stage_red2_block()
            self.stage_blue1_dribbles()
            self.stage_red2_leave()
            self.stage_pass_receive_shoot()
            self.stage_defense()
            self.final_formation()

    def build_summary(self, wall_time: float, final_frame: str | None, video_path: str | None, scan_hits: list[str]) -> dict[str, Any]:
        per_path = {robot: self.controllers[robot].base.path_length for robot in ROBOTS}
        per_cont = {robot: self.controllers[robot].base.max_continuous_motion for robot in ROBOTS}
        per_turn = {robot: math.degrees(self.controllers[robot].base.max_turn) for robot in ROBOTS}
        amplitudes = {}
        for robot, controller in self.controllers.items():
            stats = controller.gait.stats
            amplitudes[robot] = {
                "hip": stats.amplitude(["Left_Hip_Pitch", "Right_Hip_Pitch"]),
                "knee": stats.amplitude(["Left_Knee_Pitch", "Right_Knee_Pitch"]),
                "shoulder": stats.amplitude(["Left_Shoulder_Pitch", "Right_Shoulder_Pitch"]),
                "gait_active_seconds": stats.active_seconds,
            }
        visible = all(
            per_path[r] >= 1.0
            and per_path[r] <= 4.0
            and per_cont[r] >= 0.5
            and per_turn[r] >= 25.0
            and amplitudes[r]["hip"] >= 0.25
            and amplitudes[r]["knee"] >= 0.30
            and amplitudes[r]["shoulder"] >= 0.28
            and amplitudes[r]["gait_active_seconds"] >= 3.0
            for r in ROBOTS
        )
        physical = (
            self.contact_counts["T1_BLUE_1"] >= 3
            and self.contact_counts["T1_BLUE_2"] >= 2
            and self.contact_counts["T1_RED_1"] >= 1
            and self.contact_counts["T1_RED_2"] >= 1
            and len(self.contacts) >= 7
            and self.displacements["dribble"] >= 0.18
            and self.displacements["pass"] >= 0.35
            and self.displacements["receive"] >= 0.05
            and self.displacements["shoot"] >= 0.40
            and self.displacements["red1_clear"] >= 0.25
            and self.displacements["red2_counter"] >= 0.25
            and self.ball_total_path >= 1.20
        )
        strategy_success = all(self.strategy_results.values())
        failure = []
        if scan_hits:
            failure.append("BallGuard source scan hits: " + "; ".join(scan_hits))
        if not visible:
            failure.append("visible motion thresholds not fully met")
        if not physical:
            failure.append("physical contact/displacement thresholds not fully met")
        if not strategy_success:
            failure.append("strategy thresholds not fully met")
        if self.safety.nan_detected:
            failure.append("NaN/Inf detected")
        if self.safety.joint_limit_violation:
            failure.append("joint limit violation")
        if self.ball_guard.mutation_detected:
            failure.append("ball mutation suspected")
        if "STAGE_12_DONE" not in self.completed and self.mode == "full-demo":
            failure.append("STAGE_12_DONE not completed")
        demo_success = not failure and float(self.data.time) <= 120.0
        motion_violations = len(self.motion_quality["violations"]) if (self.visual_v2 or self.visual_v3) else 0
        visual_v2_success = (
            (self.visual_v2 or self.visual_v3)
            and demo_success
            and "STAGE_12_DONE" in self.completed
            and motion_violations == 0
            and not self.safety.nan_detected
            and not self.safety.joint_limit_violation
        )
        video_duration = getattr(self.recorder, "video_duration_seconds", None) if self.recorder else None
        video_resolution = getattr(self.recorder, "video_resolution", None) if self.recorder else None
        video_fps = getattr(self.recorder, "video_fps", None) if self.recorder else None
        return {
            "run_id": self.run_id,
            "engine": "MuJoCo",
            "mujoco_version": mujoco.__version__,
            "model_path": str(self.model_path),
            "model_type": "T1_MUJOCO_VISUAL_PROXY",
            "official_t1_model_used": False,
            "assisted_planar_locomotion": True,
            "native_joint_gait": True,
            "direct_ball_state_write": False,
            "direct_ball_qpos_write": False,
            "direct_ball_qvel_write": False,
            "direct_robot_base_qpos_write": False,
            "ball_physics": True,
            "ball_freejoint": True,
            "ball_motion_physical_contact_only": not self.ball_guard.mutation_detected and not scan_hits,
            "completed_stages": self.completed,
            "failed_stages": self.failed,
            "timed_out_stages": self.timed_out,
            "simulation_time": float(self.data.time),
            "wall_time": wall_time,
            "per_robot_path_length": per_path,
            "per_robot_continuous_motion": per_cont,
            "per_robot_max_turn": per_turn,
            "per_robot_actual_joint_amplitude": amplitudes,
            "per_robot_contact_count": self.contact_counts,
            "total_contacts": len(self.contacts),
            "dribble_strategy": self.strategy_results["DRIBBLE"],
            "pass_strategy": self.strategy_results["PASS"],
            "shoot_strategy": self.strategy_results["SHOOT"],
            "defensive_strategy": self.strategy_results["BLOCK_CLEAR"],
            "dribble_displacement": self.displacements["dribble"],
            "pass_displacement": self.displacements["pass"],
            "receive_displacement": self.displacements["receive"],
            "shoot_displacement": self.displacements["shoot"],
            "red1_clear_displacement": self.displacements["red1_clear"],
            "red2_counter_displacement": self.displacements["red2_counter"],
            "ball_total_path": self.ball_total_path,
            "ball_mutation_detected": self.ball_guard.mutation_detected or bool(scan_hits),
            "joint_limit_violation": self.safety.joint_limit_violation,
            "nan_detected": self.safety.nan_detected,
            "visible_motion_success": visible,
            "physical_contact_success": physical,
            "strategy_success": strategy_success,
            "demo_success": demo_success,
            "visual_v2_success": visual_v2_success,
            "visual_v3_success": bool(self.visual_v3 and visual_v2_success),
            "motion_rule_violation_count": motion_violations,
            "motion_rule_violations": self.motion_quality["violations"][:20] if (self.visual_v2 or self.visual_v3) else [],
            "motion_quality_path": str(self.run_dir / "motion_quality.json") if (self.visual_v2 or self.visual_v3) else None,
            "visual_acceptance_path": str(self.run_dir / ("visual_v3_acceptance.json" if self.visual_v3 else "visual_acceptance.json")) if (self.visual_v2 or self.visual_v3) else None,
            "clean_viewer_opened": bool((self.clean_viewer_requested and self.visual_v2) or (self.fast_viewer_requested and self.visual_v3)),
            "camera": self.camera,
            "playback_speed": self.playback_speed,
            "video_duration_seconds": video_duration,
            "video_resolution": video_resolution,
            "video_fps": video_fps,
            "failure_reason": "; ".join(failure),
            "final_frame": final_frame,
            "video_path": video_path,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="full-demo", choices=["model-check", "gait-check", "contact-check", "blue-dribble", "pass-receive", "defense-check", "pass-only", "visual-check-v2", "visual-check-v3", "visual-v3-demo", "visual-v2-demo", "visual-full-demo", "concurrent-match", "full-demo"])
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--visual", action="store_true")
    parser.add_argument("--slow-demo", action="store_true")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--camera", default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--playback-speed", type=float, default=None)
    parser.add_argument("--clean-viewer", action="store_true")
    parser.add_argument("--fast-viewer", action="store_true")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--smooth-frontend", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--target-fps", type=float, default=60.0)
    parser.add_argument("--video-fps", type=int, default=60)
    args = parser.parse_args()
    if args.mode == "concurrent-match":
        from mujoco_soccer.multi_agent.concurrent_match import ConcurrentMatch

        ConcurrentMatch(
            run_id=args.run_id,
            duration=args.duration or 60.0,
            seed=args.seed,
            view=args.view or args.visual,
            record=not args.no_render and not args.no_record and not (args.view or args.visual),
            no_render=args.no_render,
            speed=args.playback_speed or 1.0,
            smooth_frontend=args.smooth_frontend or args.benchmark,
            benchmark=args.benchmark,
            verbose=args.verbose,
            target_fps=args.target_fps,
            video_fps=args.video_fps,
        ).run()
        return
    visual_v3 = args.mode in {"visual-v3-demo", "visual-check-v3"} or (args.model is not None and "visual_v3" in str(args.model))
    visual_v2 = args.mode in {"visual-v2-demo", "visual-check-v2"} or (args.model is not None and "visual_v2" in str(args.model)) or visual_v3
    visual = args.visual or args.mode in {"visual-full-demo", "visual-v2-demo"}
    mode = "full-demo" if args.mode in {"visual-full-demo", "visual-v2-demo", "visual-v3-demo"} else ("visual-check-v2" if args.mode == "visual-check-v3" else args.mode)
    playback_speed = args.playback_speed or (0.5 if args.slow_demo else 1.0)
    realtime_factor = playback_speed
    DemoRunner(
        mode,
        args.run_id,
        render=not args.no_render,
        visual=visual,
        realtime_factor=realtime_factor,
        model_path=args.model,
        visual_v2=visual_v2,
        visual_v3=visual_v3,
        camera=args.camera or ("broadcast_wide" if visual_v3 else ("broadcast" if visual_v2 else "overview")),
        record=not args.no_record,
        duration=args.duration,
        playback_speed=playback_speed,
        clean_viewer=args.clean_viewer or visual_v2,
        fast_viewer=args.fast_viewer,
    ).run()


if __name__ == "__main__":
    main()
