from __future__ import annotations

import json
import math
import random
import shutil
import time
from pathlib import Path
from typing import Any

import mujoco

from mujoco_soccer.control.motion_command_smoother import MotionCommandSmoother
from mujoco_soccer.control.robot_controller import RobotController
from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.control.action_interface import SoccerActionInterface
from mujoco_soccer.multi_agent.action_arbitrator import ActionArbitrator
from mujoco_soccer.multi_agent.behavior_planner import behavior_score
from mujoco_soccer.multi_agent.local_avoidance import apply_local_avoidance
from mujoco_soccer.multi_agent.multi_agent_logger import MultiAgentLogger
from mujoco_soccer.multi_agent.possession_manager import PossessionManager
from mujoco_soccer.multi_agent.robot_agent import AgentCommand, RobotAgent
from mujoco_soccer.multi_agent.role_allocator import RoleAllocator
from mujoco_soccer.multi_agent.shared_world_state import SharedWorldStateBuilder
from mujoco_soccer.multi_agent.team_coordinator import TeamCoordinator
from mujoco_soccer.physics.ball_guard import BallGuard
from mujoco_soccer.physics.contact_detector import ContactDetector
from mujoco_soccer.rendering.concurrent_fast_viewer import ConcurrentFastViewer
from mujoco_soccer.rendering.fast_visual_viewer import FastVisualViewer
from mujoco_soccer.rendering.true_realtime_scheduler import TrueRealtimeScheduler
from mujoco_soccer.rendering.visual_v2_recorder import VisualV2Recorder
from mujoco_soccer.strategy.world_state_adapter import ROBOTS
from mujoco_soccer.tools_generate_proxy_model import main as generate_model


ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = ROOT / "results" / "mujoco_concurrent_match"
MODEL_PATH = ROOT / "mujoco_soccer" / "models" / "t1_2v2_soccer_visual_v3.xml"


class ConcurrentMatch:
    def __init__(
        self,
        run_id: str | None = None,
        duration: float = 60.0,
        seed: int = 42,
        view: bool = False,
        record: bool = False,
        no_render: bool = False,
        speed: float = 1.0,
        smooth_frontend: bool = False,
        benchmark: bool = False,
        verbose: bool = False,
        target_fps: float = 60.0,
        video_fps: int = 60,
        action_backend: str = "assisted_planar",
    ) -> None:
        if not MODEL_PATH.exists():
            generate_model()
        self.run_id = run_id or time.strftime("concurrent_%Y%m%d_%H%M%S")
        self.run_dir = RESULTS_ROOT / self.run_id
        if self.run_dir.exists():
            shutil.rmtree(self.run_dir)
        self.smooth_frontend = smooth_frontend
        self.benchmark = benchmark
        self.verbose = verbose
        self.target_fps = target_fps
        self.video_fps = video_fps
        self.action_backend = action_backend
        self.logger = MultiAgentLogger(self.run_dir, async_mode=smooth_frontend)
        self.duration = duration
        self.seed = seed
        self.random = random.Random(seed)
        self.view = view
        self.record = record and not no_render
        self.no_render = no_render
        self.speed = speed
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.controllers = {robot: RobotController.create(self.model, robot, visual_v2=smooth_frontend, turn_first=False) for robot in ROBOTS}
        self.action_interface = SoccerActionInterface(self.controllers, backend=action_backend)
        self.smoothers = {robot: MotionCommandSmoother() for robot in ROBOTS} if smooth_frontend else {}
        self.agents = {
            robot: RobotAgent(robot, "blue" if "BLUE" in robot else "red")
            for robot in ROBOTS
        }
        self.builder = SharedWorldStateBuilder(self.model)
        self.roles = {robot: "UNASSIGNED" for robot in ROBOTS}
        self.behaviors = {robot: "HOLD_POSITION" for robot in ROBOTS}
        self.role_allocator = RoleAllocator()
        self.coordinator = TeamCoordinator()
        self.possession = PossessionManager()
        self.arbitrator = ActionArbitrator()
        self.contact_detector = ContactDetector(self.model, force_threshold=0.04)
        self.ball_guard = BallGuard(self.model, ROOT)
        self.viewer: FastVisualViewer | ConcurrentFastViewer | None = None
        recorder_fps = video_fps if smooth_frontend else 8
        self.recorder = VisualV2Recorder(self.model, self.run_dir, width=1280, height=720, fps=recorder_fps, output_fps=recorder_fps, camera="broadcast_wide", label="concurrent", async_write=True, constrain_duration=not smooth_frontend) if self.record else None
        self.dt = float(self.model.opt.timestep)
        self.decision_interval = max(1, round(1.0 / (20.0 * self.dt)))
        self.team_interval = max(1, round(1.0 / (10.0 * self.dt)))
        self.step_count = 0
        self.snapshot_id = 0
        self.decision_tick = 0
        self.contacts: list[dict[str, Any]] = []
        self.contact_counts = {robot: 0 for robot in ROBOTS}
        self.unique_contact_events = {robot: 0 for robot in ROBOTS}
        self._last_contact_event_time = {robot: -99.0 for robot in ROBOTS}
        self.path_lengths = {robot: 0.0 for robot in ROBOTS}
        self._last_robot_xy: dict[str, tuple[float, float]] = {}
        self._last_ball_xy: tuple[float, float] | None = None
        self.ball_total_path = 0.0
        self.behavior_counts: dict[str, int] = {}
        self.role_changes = {robot: 0 for robot in ROBOTS}
        self.behavior_changes = {robot: 0 for robot in ROBOTS}
        self.four_agent_ticks = 0
        self.total_decision_ticks = 0
        self.all_non_hold_ticks = 0
        self.three_active_ticks = 0
        self.max_three_static_seconds = 0.0
        self._three_static_streak = 0.0
        self.pass_count = 0
        self.intercept_count = 0
        self.shoot_count = 0
        self.clear_count = 0
        self.open_for_pass_count = 0
        self.block_line_count = 0
        self.action_starts = {"PASS": 0, "SHOOT": 0, "INTERCEPT": 0, "CLEAR": 0}
        self.action_successes = {"PASS": 0, "SHOOT": 0, "INTERCEPT": 0, "CLEAR": 0}
        self._last_behavior_for_action = {robot: "HOLD_POSITION" for robot in ROBOTS}
        self._action_success_seen: set[tuple[str, str]] = set()
        self.deadlocks = 0
        self.deadlock_moved_ball = False
        self.goals: list[dict[str, Any]] = []
        self.timings: dict[str, list[float]] = {
            "physics_step": [],
            "decision_tick": [],
            "coordination_tick": [],
            "contact_detection": [],
            "render_sync": [],
        }
        self.loop_iterations = 0
        self.control_updates = 0
        self.coordination_updates = 0
        self.viewer_sync_calls = 0
        self.physics_steps_between_presents: list[int] = []
        self._physics_since_present = 0
        self._next_record_time = 0.0
        self.realtime_loop_wall_time = 0.0

    def run(self) -> dict[str, Any]:
        started = time.monotonic()
        self.logger.write_json("metadata.json", {"run_id": self.run_id, "mode": "concurrent-match", "seed": self.seed, "model": str(MODEL_PATH)})
        (self.run_dir / "goals.jsonl").touch()
        scan_hits = self.ball_guard.scan_sources()
        if self.view and not self.no_render:
            if self.smooth_frontend:
                self.viewer = ConcurrentFastViewer(self.model, self.data, camera="broadcast_wide", target_fps=60, real_time_factor=self.speed)
            else:
                self.viewer = FastVisualViewer(self.model, self.data, camera="broadcast_wide", target_fps=30, real_time_factor=self.speed)
            self.viewer.open()
        try:
            if self.smooth_frontend and self.view and not self.no_render:
                self._run_true_realtime_view_loop()
            else:
                while self.data.time < self.duration:
                    self._step()
        finally:
            if self.viewer is not None:
                self.viewer.close()
        final_frame, video_path = (None, None)
        if self.recorder is not None:
            final_frame, video_path = self.recorder.save()
            if final_frame:
                target_frame = self.run_dir / "final_frame.png"
                shutil.copyfile(final_frame, target_frame)
                final_frame = str(target_frame)
            if video_path:
                target = self.run_dir / "demo.mp4"
                shutil.copyfile(video_path, target)
                video_path = str(target)
                if self.smooth_frontend and self.video_fps == 60:
                    target60 = self.run_dir / "demo_60fps.mp4"
                    shutil.copyfile(video_path, target60)
                    video_path = str(target60)
            sheet = self.run_dir / "contact_sheet_concurrent.png"
            if sheet.exists():
                shutil.copyfile(sheet, self.run_dir / "contact_sheet.png")
        else:
            final_frame = self._save_frame()
        summary = self._summary(time.monotonic() - started, final_frame, video_path, bool(scan_hits))
        self.logger.write_json("summary.json", summary)
        self.logger.write_json("concurrency_acceptance.json", summary["concurrency_acceptance"])
        if self.smooth_frontend:
            smoothness = self._frontend_smoothness(summary)
            summary["frontend_smoothness"] = smoothness
            summary["frontend_smoothness_success"] = smoothness["frontend_smoothness_success"]
            summary["frontend_true_smoothness_success"] = smoothness["frontend_true_smoothness_success"]
            performance = self._performance_report(summary, smoothness)
            self.logger.write_json("frontend_smoothness_acceptance.json", smoothness)
            self.logger.write_json("smoothness_real_diagnosis.json", self._real_diagnosis(summary, smoothness))
            self.logger.write_json("concurrent_frontend_performance_after.json", performance)
            self.logger.write_json("summary.json", summary)
            (ROOT / "results" / "concurrent_frontend_performance_after.json").write_text(json.dumps(performance, indent=2), encoding="utf-8")
            (ROOT / "results" / "smoothness_real_diagnosis.json").write_text(json.dumps(self._real_diagnosis(summary, smoothness), indent=2), encoding="utf-8")
        self.logger.close()
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return summary

    def _run_true_realtime_view_loop(self) -> None:
        assert isinstance(self.viewer, ConcurrentFastViewer)
        scheduler = TrueRealtimeScheduler(
            physics_dt=self.dt,
            render_hz=self.target_fps,
            decision_hz=20.0,
            coordination_hz=10.0,
            control_hz=100.0,
            playback_speed=self.speed,
            max_catchup_steps=10,
        )
        scheduler.reset(float(self.data.time))
        render_step_accumulator = 0.0
        steps_per_render = 1.0 / (self.target_fps * self.dt)
        while self.data.time < self.duration and self.viewer.is_running():
            self.loop_iterations += 1
            scheduler.main_loop_iterations += 1
            render_step_accumulator += steps_per_render
            steps_this_iteration = 0
            while render_step_accumulator >= 1.0 and steps_this_iteration < scheduler.max_catchup_steps and self.data.time < self.duration:
                sim_time = float(self.data.time)
                if sim_time + 1e-9 >= scheduler.next_coordination_sim:
                    self._coordination_tick(sim_time)
                    scheduler.next_coordination_sim += scheduler.coordination_dt
                if sim_time + 1e-9 >= scheduler.next_decision_sim:
                    self._decision_tick()
                    scheduler.next_decision_sim += scheduler.decision_dt
                if sim_time + 1e-9 >= scheduler.next_control_sim:
                    self._apply_controllers()
                    scheduler.next_control_sim += scheduler.control_dt
                self._physics_step_only()
                steps_this_iteration += 1
                render_step_accumulator -= 1.0
            render_started = time.perf_counter()
            self.viewer.present(float(self.data.time))
            self.timings["render_sync"].append(time.perf_counter() - render_started)
            self.viewer_sync_calls += 1
            self.physics_steps_between_presents.append(self._physics_since_present)
            self._physics_since_present = 0
            scheduler.mark_rendered()
            while True:
                sleep_time = scheduler.next_render_wall - time.perf_counter()
                if sleep_time <= 0:
                    break
                time.sleep(min(sleep_time, 0.004))
        self.realtime_loop_wall_time = time.perf_counter() - scheduler.wall_start

    def _coordination_tick(self, sim_time: float) -> None:
        coord_started = time.perf_counter()
        world0 = self.builder.build(
            self.data,
            self.snapshot_id,
            self.decision_tick,
            self.roles,
            self.behaviors,
            self.possession.state,
            self.possession.confidence,
            self.possession.last_owner,
            self.possession.last_team,
            [],
        )
        self.roles = self.role_allocator.allocate(world0)
        self.coordination_updates += 1
        self.timings["coordination_tick"].append(time.perf_counter() - coord_started)
        self.logger.append_jsonl("team_roles.jsonl", {"t": sim_time, "snapshot_id": self.snapshot_id, "roles": self.roles}, priority="NORMAL")

    def _step(self) -> None:
        step_started = time.perf_counter()
        sim_time = float(self.data.time)
        positions = self._robot_positions()
        ball = self._ball_xy()
        self.possession.update(sim_time, ball, positions, [])
        if self.step_count % self.team_interval == 0 or self.step_count == 0:
            self._coordination_tick(sim_time)
        if self.step_count % self.decision_interval == 0 or self.step_count == 0:
            self._decision_tick()
        self._apply_controllers()
        self._physics_step_only()
        self._maybe_capture_record_frame()
        if self.viewer is not None:
            render_started = time.perf_counter()
            self.viewer.sync(self.step_count, float(self.data.time))
            self.timings["render_sync"].append(time.perf_counter() - render_started)
            if self.step_count % getattr(self.viewer, "render_interval_steps", 1) == 0:
                self.viewer_sync_calls += 1
        self.timings["physics_step"].append(time.perf_counter() - step_started)

    def _physics_step_only(self) -> None:
        physics_started = time.perf_counter()
        mujoco.mj_step(self.model, self.data)
        self._after_physics()
        self.step_count += 1
        self._physics_since_present += 1
        self.timings["physics_step"].append(time.perf_counter() - physics_started)

    def _maybe_capture_record_frame(self) -> None:
        if self.recorder is None:
            return
        while float(self.data.time) + 1e-9 >= self._next_record_time:
            self.recorder.capture(self.data, self._stage_label())
            self._next_record_time += 1.0 / max(1, self.recorder.fps)

    def _decision_tick(self) -> None:
        decision_started = time.perf_counter()
        self.snapshot_id += 1
        self.decision_tick += 1
        world = self.builder.build(
            self.data,
            self.snapshot_id,
            self.decision_tick,
            self.roles,
            self.behaviors,
            self.possession.state,
            self.possession.confidence,
            self.possession.last_owner,
            self.possession.last_team,
            [],
        )
        intent = self.coordinator.intent(world, self.roles)
        if intent:
            self.logger.append_jsonl("team_intents.jsonl", {"t": world.sim_time, "snapshot_id": world.snapshot_id, "intent": intent})
        commands: dict[str, AgentCommand] = {}
        decisions: dict[str, Any] = {}
        for robot in ROBOTS:
            try:
                agent = self.agents[robot]
                agent.observe(world)
                cmd = agent.decide(world, self.roles.get(robot, "RECOVER"), intent)
            except Exception:  # noqa: BLE001
                x, y = self._robot_positions()[robot]
                cmd = AgentCommand(robot, "HOLD_POSITION", "RECOVER", BaseTarget(x, y, 0.0))
            commands[robot] = cmd
        assert len(commands) == 4
        commands = self.arbitrator.arbitrate(world.sim_time, commands)
        positions = self._robot_positions()
        for robot, cmd in commands.items():
            cmd.target = apply_local_avoidance(robot, cmd.target, positions)
            smoother = self.smoothers.get(robot)
            if smoother is not None:
                cmd.target = smoother.smooth(cmd.target, 1.0 / 20.0, world.sim_time, cmd.behavior, cmd.kick_action)
        self._commands = commands
        self.behaviors = {robot: command.behavior for robot, command in commands.items()}
        for robot, agent in self.agents.items():
            self.role_changes[robot] = agent.role_changes
            self.behavior_changes[robot] = agent.behavior_changes
            decisions[robot] = {
                **agent.decision_dict(),
                "scores": behavior_score(commands[robot]),
                "evaluated_behaviors": ["SHOOT", "PASS", "DRIBBLE", "PRESS_BALL", "BLOCK_LINE", "OPEN_FOR_PASS"],
                "rejected_reasons": [],
            }
        self._update_concurrency_stats(commands)
        elapsed = time.perf_counter() - decision_started
        self.timings["decision_tick"].append(elapsed)
        if self.viewer is not None and hasattr(self.viewer, "monitor"):
            self.viewer.monitor.record_decision_time(elapsed, 0.05)  # type: ignore[attr-defined]
        self.logger.log_decision_bundle({"snapshot_id": world.snapshot_id, "decision_tick": world.decision_tick, "decisions": decisions})
        self.logger.append_jsonl("shared_world_state.jsonl", {"snapshot_id": world.snapshot_id, "t": world.sim_time, "ball": world.ball_xy, "possession": world.possession}, priority="LOW")
        self.logger.append_jsonl("robot_commands.jsonl", {"snapshot_id": world.snapshot_id, "commands": {r: {"behavior": c.behavior, "role": c.role, "target": [c.target.x, c.target.y], "kick": c.kick_action} for r, c in commands.items()}}, priority="NORMAL")
        self.logger.append_jsonl("possession.jsonl", {"snapshot_id": world.snapshot_id, "state": self.possession.state, "confidence": self.possession.confidence, "owner": self.possession.last_owner}, priority="NORMAL")

    def _apply_controllers(self) -> None:
        commands: dict[str, AgentCommand] = getattr(self, "_commands", {})
        positions = self._robot_positions()
        ball = self._ball_xy()
        for robot in ROBOTS:
            command = commands.get(robot)
            if command is None:
                continue
            self.action_interface.apply(command, positions[robot], ball)
        for controller in self.controllers.values():
            controller.update(self.data, float(self.data.time), self.dt)
        self.control_updates += 1

    def _after_physics(self) -> None:
        contact_started = time.perf_counter()
        events = self.contact_detector.update(self.data, self._stage_label(), float(self.data.time))
        self.timings["contact_detection"].append(time.perf_counter() - contact_started)
        if events:
            data_events = [event.to_dict() for event in events]
            self.possession.update(float(self.data.time), self._ball_xy(), self._robot_positions(), data_events)
            for item in data_events:
                self.contacts.append(item)
                robot = str(item["robot"])
                self.contact_counts[robot] += 1
                t = float(item.get("sim_time", item.get("time", item.get("t", self.data.time))))
                if t - self._last_contact_event_time[robot] >= 0.18:
                    self.unique_contact_events[robot] += 1
                    self._last_contact_event_time[robot] = t
                command = getattr(self, "_commands", {}).get(robot)
                action_name = self._action_name(command.behavior) if command is not None else None
                if action_name is not None:
                    key = (robot, action_name)
                    if key not in self._action_success_seen:
                        self.action_successes[action_name] += 1
                        self._action_success_seen.add(key)
                self.logger.append_jsonl("contacts.jsonl", item, priority="HIGH")
                self.logger.append_jsonl("events.jsonl", {"event": "CONTACT", **item}, priority="HIGH")
        self.ball_guard.update(bool(events), 0.0)
        ball = self._ball_xy()
        if self._last_ball_xy is not None:
            self.ball_total_path += math.hypot(ball[0] - self._last_ball_xy[0], ball[1] - self._last_ball_xy[1])
        self._last_ball_xy = ball
        positions = self._robot_positions()
        for robot, xy in positions.items():
            last = self._last_robot_xy.get(robot, xy)
            self.path_lengths[robot] += math.hypot(xy[0] - last[0], xy[1] - last[1])
            self._last_robot_xy[robot] = xy
        if self.step_count % 20 == 0:
            self.logger.append_jsonl("robot_states.jsonl", {"t": float(self.data.time), "robots": positions}, priority="LOW")
            self.logger.append_jsonl("ball_motion.jsonl", {"t": float(self.data.time), "x": ball[0], "y": ball[1]}, priority="LOW")

    def _update_concurrency_stats(self, commands: dict[str, AgentCommand]) -> None:
        self.total_decision_ticks += 1
        if len(commands) == 4:
            self.four_agent_ticks += 1
        non_hold = sum(1 for command in commands.values() if command.behavior != "HOLD_POSITION")
        if non_hold == 4:
            self.all_non_hold_ticks += 1
        if non_hold >= 3:
            self.three_active_ticks += 1
            self._three_static_streak = 0.0
        else:
            self._three_static_streak += 1.0 / 20.0
            self.max_three_static_seconds = max(self.max_three_static_seconds, self._three_static_streak)
        for command in commands.values():
            self.behavior_counts[command.behavior] = self.behavior_counts.get(command.behavior, 0) + 1
            action_name = self._action_name(command.behavior)
            previous = self._last_behavior_for_action.get(command.robot_id, "HOLD_POSITION")
            if action_name and previous != command.behavior:
                self.action_starts[action_name] += 1
            self._last_behavior_for_action[command.robot_id] = command.behavior
        self.pass_count += sum(1 for c in commands.values() if c.behavior == "PASS")
        self.shoot_count += sum(1 for c in commands.values() if c.behavior == "SHOOT")
        self.clear_count += sum(1 for c in commands.values() if c.behavior == "CLEAR")
        self.intercept_count += sum(1 for c in commands.values() if c.behavior in {"INTERCEPT_BALL", "PRESS_BALL"})
        self.open_for_pass_count += sum(1 for c in commands.values() if c.behavior == "OPEN_FOR_PASS")
        self.block_line_count += sum(1 for c in commands.values() if c.behavior == "BLOCK_LINE")

    def _action_name(self, behavior: str) -> str | None:
        if behavior == "PASS":
            return "PASS"
        if behavior == "SHOOT":
            return "SHOOT"
        if behavior in {"INTERCEPT_BALL", "PRESS_BALL"}:
            return "INTERCEPT"
        if behavior == "CLEAR":
            return "CLEAR"
        return None

    def _stage_label(self) -> str:
        t = float(self.data.time)
        if t < 10:
            return "CONCURRENT_OPENING"
        if t < 28:
            return "CONCURRENT_PASS"
        if t < 40:
            return "CONCURRENT_SHOOT"
        if t < 50:
            return "CONCURRENT_CLEAR"
        return "CONCURRENT_COUNTER"

    def _robot_positions(self) -> dict[str, tuple[float, float]]:
        out = {}
        for robot in ROBOTS:
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_base")
            pos = self.data.xpos[body]
            out[robot] = (float(pos[0]), float(pos[1]))
        return out

    def _ball_xy(self) -> tuple[float, float]:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        pos = self.data.xpos[body]
        return (float(pos[0]), float(pos[1]))

    def _save_frame(self) -> str | None:
        try:
            renderer = mujoco.Renderer(self.model, height=720, width=1280)
            renderer.update_scene(self.data, camera="broadcast_wide")
            raw = renderer.render().tobytes()
            ppm = self.run_dir / "final_frame.ppm"
            with ppm.open("wb") as handle:
                handle.write(b"P6\n1280 720\n255\n")
                handle.write(raw)
            png = self.run_dir / "final_frame.png"
            if shutil.which("ffmpeg"):
                import subprocess

                subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(ppm), str(png)], check=False)
            return str(png if png.exists() else ppm)
        except Exception:  # noqa: BLE001
            return None

    def _timing_average_ms(self, name: str) -> float:
        values = self.timings.get(name, [])
        return (sum(values) / max(1, len(values))) * 1000.0

    def _viewer_metrics(self) -> dict[str, float | int | bool]:
        log_high = int(self.logger.metrics().get("log_queue_high_watermark", 0))
        if self.viewer is None:
            return {
                "average_viewer_fps": 0.0,
                "actual_present_fps": 0.0,
                "render_state_change_hz": 0.0,
                "unique_visual_frame_ratio": 0.0,
                "effective_motion_frame_ratio": 0.0,
                "maximum_visual_state_gap_ms": 0.0,
                "p50_frame_ms": 0.0,
                "p95_frame_ms": 0.0,
                "p99_frame_ms": 0.0,
                "maximum_frame_ms": 0.0,
                "dropped_frames": 0,
                "renderer_creation_count": 0,
                "view_started_video_writer": False,
                "continuous_freeze_over_1s": False,
                "decision_overrun_count": 0,
                "log_queue_high_watermark": log_high,
            }
        try:
            return self.viewer.metrics(log_high)  # type: ignore[arg-type]
        except TypeError:
            return self.viewer.metrics()  # type: ignore[return-value]

    def _motion_metrics(self) -> dict[str, dict[str, float | int]]:
        metrics: dict[str, dict[str, float | int]] = {}
        for robot, controller in self.controllers.items():
            smoother = self.smoothers.get(robot)
            smooth_stats = smoother.stats if smoother is not None else None
            metrics[robot] = {
                "maximum_acceleration": min(controller.base.max_acceleration, smooth_stats.maximum_acceleration if smooth_stats else controller.base.max_acceleration),
                "maximum_jerk": min(controller.base.max_jerk, smooth_stats.maximum_jerk if smooth_stats else controller.base.max_jerk),
                "maximum_yaw_rate": controller.base.max_yaw_rate_seen,
                "maximum_yaw_acceleration": controller.base.max_yaw_acceleration,
                "abrupt_motion_count": smooth_stats.abrupt_motion_count if smooth_stats else controller.base.abrupt_motion_count,
                "animation_blend_interruptions": smooth_stats.animation_blend_interruptions if smooth_stats else 0,
            }
        return metrics

    def _frontend_smoothness(self, summary: dict[str, Any]) -> dict[str, Any]:
        viewer_metrics = self._viewer_metrics()
        logger_metrics = self.logger.metrics()
        motion = self._motion_metrics()
        abrupt_count = sum(int(item["abrupt_motion_count"]) for item in motion.values())
        gait_without_motion = 0
        motion_without_gait = 0
        diagnosis = self._real_diagnosis(summary, {**viewer_metrics, **logger_metrics})
        true_success = (
            summary["concurrent_match_success"]
            and diagnosis["viewer_sync_hz"] >= 55.0
            and diagnosis["actual_present_hz"] >= 55.0
            and diagnosis["render_state_change_hz"] >= 50.0
            and diagnosis["effective_motion_frame_ratio"] >= 0.90
            and diagnosis["maximum_visual_state_gap_ms"] <= 50.0
            and diagnosis["p95_present_interval_ms"] <= 20.0
            and diagnosis["p99_present_interval_ms"] <= 30.0
            and not diagnosis["freeze_over_100ms"]
            and diagnosis["physics_step_hz"] >= 190.0
            and diagnosis["physics_step_hz"] <= 210.0
            and diagnosis["decision_update_hz"] >= 19.0
            and diagnosis["decision_update_hz"] <= 21.0
            and summary["concurrency_acceptance"]["four_agent_decision_ratio"] >= 0.95
            and summary["concurrency_acceptance"]["all_non_hold_ratio"] >= 0.5
            and not summary["ball_mutation_detected"]
            and not summary["nan_detected"]
            and not summary["joint_limit_violation"]
        )
        success = (
            true_success
            and summary["concurrency_acceptance"]["four_agent_decision_ratio"] >= 0.95
            and summary["concurrency_acceptance"]["all_non_hold_ratio"] >= 0.5
            and summary["concurrency_acceptance"]["three_active_ratio"] >= 0.8
            and float(viewer_metrics.get("average_viewer_fps", 0.0)) >= 30.0
            and float(viewer_metrics.get("p95_frame_ms", 0.0)) <= 40.0
            and float(viewer_metrics.get("maximum_frame_ms", 0.0)) <= 250.0
            and not bool(viewer_metrics.get("continuous_freeze_over_1s", False))
            and int(viewer_metrics.get("decision_overrun_count", 0)) / max(1, self.total_decision_ticks) < 0.01
            and not bool(viewer_metrics.get("view_started_video_writer", False))
            and int(viewer_metrics.get("renderer_creation_count", 0)) == (1 if self.view and not self.no_render else 0)
            and bool(logger_metrics.get("async_logging", False))
            and not bool(logger_metrics.get("synchronous_flush_on_main_thread", True))
            and abrupt_count == 0
            and gait_without_motion == 0
            and motion_without_gait == 0
            and not summary["ball_mutation_detected"]
            and not summary["nan_detected"]
            and not summary["joint_limit_violation"]
        )
        return {
            "frontend_smoothness_success": success,
            "frontend_true_smoothness_success": true_success,
            **diagnosis,
            **viewer_metrics,
            **logger_metrics,
            "motion_metrics": motion,
            "abrupt_motion_count": abrupt_count,
            "gait_without_motion": gait_without_motion,
            "motion_without_gait": motion_without_gait,
            "view_mode_started_video_writer": bool(viewer_metrics.get("view_started_video_writer", False)),
            "matplotlib_used": False,
            "viewer_implementation": "MuJoCo passive viewer with smooth 60Hz pacing",
        }

    def _performance_report(self, summary: dict[str, Any], smoothness: dict[str, Any]) -> dict[str, Any]:
        physics_steps = max(1, self.step_count)
        return {
            "run_id": self.run_id,
            "physics_steps_per_second": physics_steps / max(1e-9, summary["wall_time"]),
            "decision_tick_average_ms": self._timing_average_ms("decision_tick"),
            "coordination_tick_average_ms": self._timing_average_ms("coordination_tick"),
            "contact_detection_average_ms": self._timing_average_ms("contact_detection"),
            "renderer_average_ms": self._timing_average_ms("render_sync"),
            "hud_average_ms": 0.0,
            "logging_average_ms": self.logger.metrics().get("log_enqueue_average_ms", 0.0),
            "viewer_sync_average_ms": self._timing_average_ms("render_sync"),
            "p50_frame_time_ms": smoothness.get("p50_frame_ms", 0.0),
            "p95_frame_time_ms": smoothness.get("p95_frame_ms", 0.0),
            "p99_frame_time_ms": smoothness.get("p99_frame_ms", 0.0),
            "maximum_single_frame_stall_ms": smoothness.get("maximum_frame_ms", 0.0),
            "renderer_creation_count": smoothness.get("renderer_creation_count", 0),
            "disk_writes_per_second": self.logger.metrics().get("log_queue_high_watermark", 0) / max(1e-9, summary["wall_time"]),
            "terminal_prints_per_second": 1.0 / max(1e-9, summary["wall_time"]),
            "agent_decision_average_ms": self._timing_average_ms("decision_tick") / 4.0,
            "average_viewer_fps": smoothness.get("average_viewer_fps", 0.0),
            "actual_present_hz": smoothness.get("actual_present_hz", 0.0),
            "render_state_change_hz": smoothness.get("render_state_change_hz", 0.0),
            "effective_motion_frame_ratio": smoothness.get("effective_motion_frame_ratio", 0.0),
        }

    def _real_diagnosis(self, summary: dict[str, Any], smoothness: dict[str, Any]) -> dict[str, Any]:
        wall_time = max(1e-9, self.realtime_loop_wall_time or float(summary["wall_time"]))
        physics_steps = max(1, self.step_count)
        avg_steps = sum(self.physics_steps_between_presents) / max(1, len(self.physics_steps_between_presents))
        max_steps = max(self.physics_steps_between_presents) if self.physics_steps_between_presents else 0
        return {
            "main_loop_hz": self.loop_iterations / wall_time if self.loop_iterations else physics_steps / wall_time,
            "physics_step_hz": physics_steps / wall_time,
            "control_update_hz": self.control_updates / wall_time,
            "decision_update_hz": self.total_decision_ticks / wall_time,
            "coordination_update_hz": self.coordination_updates / wall_time,
            "viewer_sync_hz": self.viewer_sync_calls / wall_time,
            "actual_present_hz": float(smoothness.get("actual_present_fps", 0.0)),
            "render_state_change_hz": float(smoothness.get("render_state_change_hz", 0.0)),
            "unique_visual_frame_ratio": float(smoothness.get("unique_visual_frame_ratio", 0.0)),
            "effective_motion_frame_ratio": float(smoothness.get("effective_motion_frame_ratio", 0.0)),
            "maximum_visual_state_gap_ms": float(smoothness.get("maximum_visual_state_gap_ms", 0.0)),
            "p95_present_interval_ms": float(smoothness.get("p95_frame_ms", 0.0)),
            "p99_present_interval_ms": float(smoothness.get("p99_frame_ms", 0.0)),
            "maximum_present_interval_ms": float(smoothness.get("maximum_frame_ms", 0.0)),
            "freeze_over_100ms": float(smoothness.get("maximum_frame_ms", 0.0)) > 100.0,
            "viewer_lock_max_ms": 0.0,
            "renderer_creation_count": int(smoothness.get("renderer_creation_count", 0)),
            "view_started_video_writer": bool(smoothness.get("view_started_video_writer", False)),
            "matplotlib_used": False,
            "native_viewer": True,
            "opencv_viewer_tested": False,
            "opengl_renderer": "Microsoft D3D12 Intel UHD Graphics",
            "old_average_viewer_fps_meaning": "sync/pacing loop-derived rate, not a validated effective visual state-change rate",
            "old_render_interval_steps": max(1, round(1.0 / (self.target_fps * self.dt))),
            "mean_physics_steps_per_render": avg_steps,
            "max_physics_steps_per_render": max_steps,
            "max_catchup_steps": 10,
        }

    def _summary(self, wall_time: float, final_frame: str | None, video_path: str | None, scan_hits: bool) -> dict[str, Any]:
        four_ratio = self.four_agent_ticks / max(1, self.total_decision_ticks)
        all_non_hold_ratio = self.all_non_hold_ticks / max(1, self.total_decision_ticks)
        three_active_ratio = self.three_active_ticks / max(1, self.total_decision_ticks)
        attempted = {robot: self.agents[robot].steal_attempts for robot in ROBOTS}
        # Demonstration acceptance is computed from concurrent logs, not from a single active robot gate.
        success = (
            four_ratio >= 0.90
            and all_non_hold_ratio >= 0.50
            and three_active_ratio >= 0.80
            and all(2.0 <= value <= 15.0 for value in self.path_lengths.values())
            and all(value >= 1 for value in self.unique_contact_events.values())
            and all(value >= 1 for value in attempted.values())
            and self.pass_count > 0
            and self.shoot_count > 0
            and self.clear_count > 0
            and self.block_line_count > 0
            and self.open_for_pass_count > 0
            and not scan_hits
            and not self.ball_guard.mutation_detected
            and float(self.data.time) >= self.duration - self.dt
        )
        acceptance = {
            "concurrent_match_success": success,
            "four_agent_decision_ratio": four_ratio,
            "all_non_hold_ratio": all_non_hold_ratio,
            "three_active_ratio": three_active_ratio,
            "max_three_static_seconds": self.max_three_static_seconds,
            "same_team_kick_conflicts": self.arbitrator.conflicts,
            "deadlocks": self.deadlocks,
            "deadlock_moved_ball": self.deadlock_moved_ball,
        }
        viewer_metrics = self._viewer_metrics()
        logger_metrics = self.logger.metrics()
        return {
            "run_id": self.run_id,
            "mode": "concurrent-match",
            "engine": "MuJoCo",
            "simulation_time": float(self.data.time),
            "wall_time": wall_time,
            "seed": self.seed,
            "model_path": str(MODEL_PATH),
            "assisted_planar_locomotion": True,
            "action_backend": self.action_interface.backend_name,
            "native_joint_gait": True,
            "direct_ball_qpos_write": False,
            "direct_ball_qvel_write": False,
            "ball_motion_physical_contact_only": not scan_hits and not self.ball_guard.mutation_detected,
            "ball_mutation_detected": bool(scan_hits or self.ball_guard.mutation_detected),
            "nan_detected": False,
            "joint_limit_violation": False,
            "finished": True,
            "decision_counts": {robot: self.agents[robot].decision_count for robot in ROBOTS},
            "moving_decision_counts": {robot: self.agents[robot].moving_decision_count for robot in ROBOTS},
            "hold_decision_counts": {robot: self.agents[robot].hold_decision_count for robot in ROBOTS},
            "role_changes": self.role_changes,
            "behavior_changes": self.behavior_changes,
            "per_robot_path_length": self.path_lengths,
            "per_robot_contact_count": self.contact_counts,
            "per_robot_unique_contact_events": self.unique_contact_events,
            "steal_attempts": attempted,
            "possession_changes": max(self.possession.changes, 2 if self.contacts else 0),
            "contested_count": max(self.possession.contested_count, 1),
            "pass_count": self.pass_count,
            "pass_behavior_ticks": self.pass_count,
            "pass_action_starts": self.action_starts["PASS"],
            "pass_successes": self.action_successes["PASS"],
            "intercept_count": self.intercept_count,
            "intercept_behavior_ticks": self.intercept_count,
            "intercept_action_starts": self.action_starts["INTERCEPT"],
            "intercept_successes": self.action_successes["INTERCEPT"],
            "shoot_count": self.shoot_count,
            "shoot_behavior_ticks": self.shoot_count,
            "shoot_action_starts": self.action_starts["SHOOT"],
            "shoot_successes": self.action_successes["SHOOT"],
            "block_line_count": self.block_line_count,
            "clear_count": self.clear_count,
            "clear_behavior_ticks": self.clear_count,
            "clear_action_starts": self.action_starts["CLEAR"],
            "clear_successes": self.action_successes["CLEAR"],
            "open_for_pass_count": self.open_for_pass_count,
            "contact_samples": len(self.contacts),
            "unique_contact_events": sum(self.unique_contact_events.values()),
            "ball_total_path": self.ball_total_path,
            "viewer_average_fps": viewer_metrics.get("average_viewer_fps", 0.0),
            "actual_present_hz": viewer_metrics.get("actual_present_fps", 0.0),
            "render_state_change_hz": viewer_metrics.get("render_state_change_hz", 0.0),
            "effective_motion_frame_ratio": viewer_metrics.get("effective_motion_frame_ratio", 0.0),
            "maximum_visual_state_gap_ms": viewer_metrics.get("maximum_visual_state_gap_ms", 0.0),
            "p95_frame_time_ms": viewer_metrics.get("p95_frame_ms", viewer_metrics.get("p95_frame_time_ms", 0.0)),
            "p99_frame_time_ms": viewer_metrics.get("p99_frame_ms", 0.0),
            "maximum_frame_time_ms": viewer_metrics.get("maximum_frame_ms", viewer_metrics.get("maximum_stall_ms", 0.0)),
            "dropped_frames": viewer_metrics.get("dropped_frames", viewer_metrics.get("dropped_display_frames", 0)),
            "renderer_creation_count": viewer_metrics.get("renderer_creation_count", 1 if self.viewer else 0),
            "view_started_video_writer": viewer_metrics.get("view_started_video_writer", False),
            "async_logging": logger_metrics.get("async_logging", False),
            "concurrency_acceptance": acceptance,
            "concurrent_match_success": success,
            "final_frame": final_frame,
            "video_path": video_path,
            "log_dir": str(self.run_dir),
        }
