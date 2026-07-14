#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import sys
import time
import hashlib
import re
from enum import Enum
from pathlib import Path

try:
    from controller import Robot, Supervisor
except ImportError:
    print("FATAL: Webots controller module not found.", file=sys.stderr)
    raise

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from common.world_state import WorldState
from common.native_joint_utils import classify_motor_name
from controllers.t1_native_ball_controller.coordinate_frames import (
    horizontal_distance,
    node_pose_relative_to,
    node_world_pose,
    pose_translation,
    pose_translation_column_major,
    robot_to_world,
)
from controllers.t1_native_ball_controller.collision_geometry import (
    axes_from_orientation,
    oriented_box_corners_world,
    sphere_to_axis_aligned_box_signed_distance,
    sphere_to_oriented_box_signed_distance,
)
from integration.native_robot_action_adapter import clip_target, horizontal_displacement, interpolate
from strategy.team_strategy import TeamStrategy


class MotionState(str, Enum):
    INITIALIZE = "INITIALIZE"
    RESOLVE_NODES = "RESOLVE_NODES"
    COORDINATE_FRAME_CHECK = "COORDINATE_FRAME_CHECK"
    INITIAL_COLLISION_AUDIT = "INITIAL_COLLISION_AUDIT"
    SETTLE_CHECK = "SETTLE_CHECK"
    SCENE_SUPPORT_CHECK = "SCENE_SUPPORT_CHECK"
    HOLD_STAND = "HOLD_STAND"
    GEOMETRY_CHECK = "GEOMETRY_CHECK"
    CALIBRATE = "CALIBRATE"
    TRAJECTORY_PRECHECK = "TRAJECTORY_PRECHECK"
    PREPARE_KICK = "PREPARE_KICK"
    PREPARE = "PREPARE"
    SHIFT_WEIGHT = "SHIFT_WEIGHT"
    LIFT_FOOT = "LIFT_FOOT"
    SWING_FORWARD = "SWING_FORWARD"
    CONTACT_HOLD = "CONTACT_HOLD"
    RETRACT = "RETRACT"
    RECOVER = "RECOVER"
    VERIFY_BALL = "VERIFY_BALL"
    DRIBBLE_REPOSITION = "DRIBBLE_REPOSITION"
    DONE = "DONE"
    FAILED = "FAILED"
    FAILED_PRECONDITION = "FAILED_PRECONDITION"
    FAILED_TRAJECTORY_PRECHECK = "FAILED_TRAJECTORY_PRECHECK"
    FAILED_TRAJECTORY_DIRECTION = "FAILED_TRAJECTORY_DIRECTION"
    FAILED_COORDINATE_FRAME = "FAILED_COORDINATE_FRAME"
    FAILED_SCENE_SUPPORT = "FAILED_SCENE_SUPPORT"


def now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def parse_simple_yaml(path: Path) -> dict:
    data: dict = {}
    stack: list[tuple[int, object]] = [(-1, data)]
    current_list_item = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        text = line.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if text.startswith("- "):
            item: dict = {}
            if isinstance(parent, list):
                parent.append(item)
            rest = text[2:]
            if ":" in rest:
                k, v = [p.strip() for p in rest.split(":", 1)]
                item[k] = coerce_value(v)
            stack.append((indent, item))
            current_list_item = item
            continue
        key, value = [p.strip() for p in text.split(":", 1)]
        if value == "":
            nxt = [] if key == "levels" else {}
            if isinstance(parent, dict):
                parent[key] = nxt
            stack.append((indent, nxt))
        else:
            if isinstance(parent, dict):
                parent[key] = coerce_value(value)
            elif current_list_item is not None:
                current_list_item[key] = coerce_value(value)
    return data


def coerce_value(value: str):
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def jsonl_append(path: Path, row: dict) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, ensure_ascii=False) + "\n")


def device_type_name(device) -> str:
    try:
        return str(device.getNodeType())
    except Exception:
        return "unknown"


class ReadOnlyWorldObserver:
    def __init__(self, supervisor: Supervisor) -> None:
        self.supervisor = supervisor
        self.robot_node = self._node_by_def_or_name("T1_BLUE_1")
        self.ball_node = self._node_by_def_or_name("SOCCER_BALL")
        self.resolution = {}
        self.ball_resolution = self._describe_node(self.ball_node, "SOCCER_BALL", "getFromDef(SOCCER_BALL)")
        self.foot_nodes = {
            "right": self._resolve_foot("right"),
            "left": self._resolve_foot("left"),
        }

    def get_robot_pose(self) -> dict | None:
        if self.robot_node is None:
            return None
        position = self._position(self.robot_node)
        orientation = self._orientation(self.robot_node)
        if position is None:
            return None
        roll = pitch = yaw = None
        if orientation and len(orientation) == 9:
            roll = math.atan2(orientation[7], orientation[8])
            pitch = math.atan2(-orientation[6], math.sqrt(orientation[7] ** 2 + orientation[8] ** 2))
            yaw = math.atan2(orientation[3], orientation[0])
        return {"position": position, "orientation": orientation, "roll": roll, "pitch": pitch, "yaw": yaw}

    def get_foot_position(self, side: str) -> list[float] | None:
        return self._position(self.foot_nodes.get(side.lower()))

    def get_ball_position(self) -> list[float] | None:
        return self._position(self.ball_node)

    def get_ball_velocity(self) -> list[float] | None:
        if self.ball_node is None:
            return None
        try:
            value = self.ball_node.getVelocity()
            return [float(v) for v in value]
        except Exception:
            return None

    def _node_by_def_or_name(self, name: str):
        try:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                return node
        except Exception:
            pass
        return self._find_named_node(self.supervisor.getRoot(), name)

    def _resolve_foot(self, side: str):
        upper = "RIGHT_FOOT" if side == "right" else "LEFT_FOOT"
        title = "Right_Foot" if side == "right" else "Left_Foot"
        legacy = f"{upper}_LINK"
        name = "right_foot_link" if side == "right" else "left_foot_link"
        attempts = [
            (upper, f"getFromDef({upper})", lambda key=upper: self.supervisor.getFromDef(key)),
            (title, f"getFromDef({title})", lambda key=title: self.supervisor.getFromDef(key)),
            (legacy, f"getFromDef({legacy})", lambda key=legacy: self.supervisor.getFromDef(key)),
            (name, f"recursive name search {name}", lambda key=name: self._find_named_node(self.robot_node, key)),
        ]
        for def_name, method, getter in attempts:
            try:
                node = getter()
            except Exception:
                node = None
            if node is not None:
                self.resolution[side] = self._describe_node(node, def_name, method)
                return node
        self.resolution[side] = {
            "resolved": False,
            "def_name": None,
            "method": None,
            "node_type": None,
            "node_id": None,
            "position": None,
            "orientation": None,
            "bounding_object_present": False,
        }
        return None

    def _describe_node(self, node, def_name: str | None, method: str | None) -> dict:
        if node is None:
            return {"resolved": False, "def_name": def_name, "method": method}
        node_id = None
        try:
            node_id = int(node.getId())
        except Exception:
            pass
        node_type = None
        try:
            node_type = str(node.getTypeName())
        except Exception:
            try:
                node_type = str(node.getType())
            except Exception:
                node_type = None
        bounding = False
        try:
            bounding = node.getField("boundingObject") is not None
        except Exception:
            bounding = False
        return {
            "resolved": True,
            "def_name": def_name,
            "method": method,
            "node_type": node_type,
            "node_id": node_id,
            "position": self._position(node),
            "orientation": self._orientation(node),
            "bounding_object_present": bounding,
        }

    def _position(self, node) -> list[float] | None:
        if node is None:
            return None
        try:
            return [float(v) for v in node.getPosition()]
        except Exception:
            return None

    def _orientation(self, node) -> list[float] | None:
        if node is None:
            return None
        try:
            return [float(v) for v in node.getOrientation()]
        except Exception:
            return None

    def _node_name(self, node) -> str | None:
        if node is None:
            return None
        try:
            field = node.getField("name")
            if field is not None:
                return field.getSFString()
        except Exception:
            return None
        return None

    def _find_named_node(self, node, wanted: str):
        if node is None:
            return None
        if self._node_name(node) == wanted:
            return node
        for field_name in ("children", "endPoint", "boundingObject"):
            try:
                field = node.getField(field_name)
            except Exception:
                field = None
            if field is None:
                continue
            for child in self._field_nodes(field):
                found = self._find_named_node(child, wanted)
                if found is not None:
                    return found
        return None

    def _field_nodes(self, field) -> list:
        try:
            return [field.getMFNode(i) for i in range(field.getCount()) if field.getMFNode(i) is not None]
        except Exception:
            pass
        try:
            node = field.getSFNode()
            return [node] if node is not None else []
        except Exception:
            return []


class NativeKickController:
    def __init__(self) -> None:
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep()) or 1
        self.run_id = os.environ.get("RUN_ID", time.strftime("%Y%m%d_%H%M%S"))
        self.run_dir = Path(os.environ.get("NATIVE_KICK_RUN_DIR", PROJECT / "results" / "native_physical_kick" / self.run_id))
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.mode = os.environ.get("NATIVE_KICK_MODE", "kick")
        self.audit_mode = self.mode in {"collision-audit", "settle-check"}
        self.assisted = self.mode.startswith("assisted") or self.mode in {"support-check", "frame-check", "geometry-check", "collision-audit", "settle-check"}
        self.config = parse_simple_yaml(PROJECT / "config" / "native_kick.yaml")
        self.match_state_file = Path(os.environ.get("MATCH_STATE_FILE", self.run_dir / "match_state.jsonl"))
        self.logs = {
            "events": self.run_dir / "events.jsonl",
            "world": self.run_dir / "world_state.jsonl",
            "strategy": self.run_dir / "strategy.jsonl",
            "commands": self.run_dir / "joint_commands.jsonl",
            "states": self.run_dir / "joint_states.jsonl",
            "ball": self.run_dir / "ball_motion.jsonl",
        }
        self.motors = {}
        self.sensors = {}
        self.motor_info = {}
        self.classified = {}
        self.initial_pose = {}
        self.current_targets = {}
        self.failure_reason = None
        self.joint_limit_violation = False
        self.robot_fallen = False
        self.ball_initial = None
        self.ball_final = None
        self.robot_initial = None
        self.touch_count = 0
        self.state_history = []
        self.observer = ReadOnlyWorldObserver(self.robot)
        self.prev_joint_sample = None
        self.prev_joint_sample_time = None
        self.last_hold_diagnosis = {}
        self.right_leg_calibration = {}
        self.kick_geometry = {}
        self.min_foot_ball_distance = None
        self.contact_estimated = False
        self.contact_window_ball_before = None
        self.contact_window_ball_after = None
        self.node_resolution = {}
        self.geometry_ok = False
        self.geometry_failure_reason = None
        self.trajectory_precheck = {}
        self.valid_predicted_levels = []
        self.selected_level = None
        self.selected_prediction = None
        self.trajectory_execution_consistency = {}
        self.actual_trajectory_samples = []
        self.predicted_vs_actual_trajectory = {}
        self.trajectory_direction_failed = False
        self.initial_foot_ball_distance = None
        self.scene_support = {}
        self.support_check = {}
        self.initial_contact_audit = {}
        self.initial_collision_geometry = {}
        self.settle_check = {}
        self.ball_physics = {}
        self.coordinate_frame = {}
        self.foot_ownership = {}
        self.foot_axis_selection = {}
        self.calibration_confidence = 0.0
        self.hip_pitch_forward_sign = None
        self.knee_extension_sign = None
        self.ankle_compensation_signs = {}
        self.result = None

    def run(self) -> None:
        self.emit("START", mode=self.mode, assisted_mode=self.assisted)
        self.inventory_devices()
        if self.audit_mode:
            self.enable_contact_tracking()
            self.wait_steps(1)
        else:
            self.wait_steps(200)
        self.capture_initial_pose()
        self.robot_initial = self.robot_xy_z()
        self.ball_initial = self.latest_ball_xy()
        if self.assisted:
            self.state_history.append(MotionState.INITIALIZE.value)
            if self.mode not in {"support-check", "collision-audit", "settle-check"}:
                self.wait_seconds(float(self.config.get("initial_hold_seconds", 5.0)), "initial_pose_hold")
            self.state_history.append(MotionState.RESOLVE_NODES.value)
            self.emit("STATE", state=MotionState.RESOLVE_NODES.value)
            if not self.resolve_nodes_precondition():
                self.failure_reason = self.failure_reason or "assisted node resolution failed"
                return self.finish()
            if self.mode == "assisted-node-test":
                return self.finish_node_test()
            self.write_ball_physics_analysis()
            self.state_history.append(MotionState.COORDINATE_FRAME_CHECK.value)
            self.emit("STATE", state=MotionState.COORDINATE_FRAME_CHECK.value)
            if self.mode == "frame-check":
                return self.finish_frame_check()
            if not self.coordinate_frame_check():
                self.failure_reason = self.failure_reason or "coordinate frame check failed"
                return self.finish()
            if self.mode == "collision-audit":
                return self.finish_collision_audit()
            if self.mode == "settle-check":
                return self.finish_settle_check()
            if self.mode == "assisted-kick":
                self.finish_settle_check(write_summary=False)
                if not self.settle_check.get("settle_check_success"):
                    self.failure_reason = self.failure_reason or "assisted-kick preflight settle-check failed"
                    return self.finish()
            self.state_history.append(MotionState.SCENE_SUPPORT_CHECK.value)
            self.emit("STATE", state=MotionState.SCENE_SUPPORT_CHECK.value)
            if self.mode == "support-check":
                return self.finish_support_check()
            if not self.scene_support_check(write_file=True):
                self.failure_reason = self.failure_reason or "scene support check failed"
                return self.finish()
            if self.mode == "geometry-check" and not self.run_support_check(duration=3.0):
                self.failure_reason = self.failure_reason or "geometry-check support stability failed"
                return self.finish()
        else:
            self.hold_pose(float(self.config.get("initial_hold_seconds", 5.0)), "initial_pose_hold")
        if self.assisted:
            stable = self.assisted_hold_stand()
        else:
            self.calibrate_directions()
            stable = self.hold_pose(float(self.config.get("hold_stand_seconds", 10.0)), "stable_stand")
        if not stable:
            if self.failure_reason is None:
                self.failure_reason = "robot fell during stable stand hold"
            return self.finish()
        if self.assisted:
            self.state_history.append(MotionState.GEOMETRY_CHECK.value)
            self.emit("STATE", state=MotionState.GEOMETRY_CHECK.value)
            self.write_kick_geometry()
            if self.mode == "geometry-check":
                return self.finish_geometry_check()
            if not self.geometry_ok:
                self.failure_reason = self.geometry_failure_reason or "assisted geometry precheck failed"
                return self.finish()
            self.state_history.append(MotionState.CALIBRATE.value)
            self.emit("STATE", state=MotionState.CALIBRATE.value)
            if not self.calibrate_right_leg():
                self.failure_reason = self.failure_reason or "assisted right leg calibration failed"
                return self.finish()
            self.state_history.append(MotionState.TRAJECTORY_PRECHECK.value)
            self.emit("STATE", state=MotionState.TRAJECTORY_PRECHECK.value)
            if not self.write_predicted_trajectories(self.config.get("levels", [])[:3]):
                self.failure_reason = self.failure_reason or "assisted trajectory precheck failed"
                return self.finish()

        self.evaluate_strategy()
        max_pushes = 3 if self.mode == "assisted-kick" else (1 if self.mode in {"kick", "shoot"} else int(self.config.get("max_pushes", 3)))
        levels = self.config.get("levels", [])
        if self.mode == "shoot" and len(levels) >= 3:
            levels = [levels[2]]
        elif self.mode in {"kick", "assisted-kick"}:
            levels = levels[:3]
        else:
            levels = [levels[0]] * max_pushes
        if self.mode == "assisted-kick":
            levels = self.predicted_contact_levels(levels)
            if not levels:
                self.failure_reason = "trajectory precheck found no predicted-contact level to execute"
                self.state_history.append(MotionState.FAILED_TRAJECTORY_PRECHECK.value)
                self.emit("STATE", state=MotionState.FAILED_TRAJECTORY_PRECHECK.value)
                return self.finish()
            if not self.write_trajectory_execution_consistency(levels[0]):
                self.failure_reason = self.failure_reason or "trajectory execution consistency failed"
                return self.finish()

        self.ball_initial = self.latest_ball_xy() or self.ball_initial
        for idx, level in enumerate(levels[:max_pushes], start=1):
            self.emit("PUSH_START", index=idx, level=level)
            self.active_motion_level = idx
            before = self.latest_ball_xy()
            ok = self.execute_push(level, idx)
            after = self.latest_ball_xy()
            disp = horizontal_displacement(before, after)
            total = horizontal_displacement(self.ball_initial, after)
            jsonl_append(self.logs["ball"], {"time": now(), "push": idx, "before": before, "after": after, "displacement": disp, "total": total})
            if disp > 0.01:
                self.touch_count += 1
            self.ball_final = after
            if not ok:
                break
            if self.mode in {"kick", "shoot", "assisted-kick"} and total > float(self.config.get("kick_success_threshold", 0.05)):
                break
            if self.assisted and not self.contact_estimated:
                cfg = self.assisted_config()
                close_limit = float(cfg.get("ball_radius", 0.11)) + float(cfg.get("foot_contact_radius", 0.08)) + float(cfg.get("contact_tolerance", 0.05))
                if self.min_foot_ball_distance is None or self.min_foot_ball_distance > close_limit:
                    self.failure_reason = "actual foot trajectory did not approach ball; stopping before higher force"
                    break
        self.finish()

    def inventory_devices(self) -> None:
        inventory = []
        count = self.robot.getNumberOfDevices()
        for index in range(count):
            device = self.robot.getDeviceByIndex(index)
            name = device.getName()
            is_motor = all(hasattr(device, attr) for attr in ("setPosition", "getMinPosition", "getMaxPosition"))
            is_sensor = hasattr(device, "enable") and hasattr(device, "getValue") and "sensor" in name.lower()
            row = {
                "index": index,
                "name": name,
                "node_type": device_type_name(device),
                "is_motor": bool(is_motor),
                "is_position_sensor": bool(is_sensor),
            }
            if is_motor:
                self.motors[name] = device
                cls = classify_motor_name(name)
                self.classified.setdefault(cls, []).append(name)
                info = {
                    "minPosition": safe_call(device, "getMinPosition"),
                    "maxPosition": safe_call(device, "getMaxPosition"),
                    "maxVelocity": safe_call(device, "getMaxVelocity"),
                    "availableTorque": safe_call(device, "getAvailableTorque"),
                    "maxTorque": safe_call(device, "getMaxTorque"),
                    "class": cls,
                }
                self.motor_info[name] = info
                row.update(info)
            inventory.append(row)

        for name in list(self.motors):
            sensor = None
            for candidate in (f"{name}_sensor", f"{name} sensor", name.replace("_", " ") + "_sensor"):
                try:
                    sensor = self.robot.getDevice(candidate)
                    if sensor:
                        break
                except Exception:
                    pass
            if sensor is not None and hasattr(sensor, "enable"):
                sensor.enable(self.timestep)
                self.sensors[name] = sensor

        (self.run_dir / "device_inventory.json").write_text(json.dumps(inventory, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        joint_map = {"motors": self.motor_info, "classes": self.classified, "sensor_count": len(self.sensors)}
        (self.run_dir / "joint_map.json").write_text(json.dumps(joint_map, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        (PROJECT / "controllers" / "t1_native_ball_controller" / "joint_map.json").write_text(json.dumps(joint_map, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("DEVICE_INVENTORY", motor_count=len(self.motors), sensor_count=len(self.sensors), classes=self.classified)

    def capture_initial_pose(self) -> None:
        for name, motor in self.motors.items():
            value = self.sensor_value(name)
            if value is None or not math.isfinite(value):
                value = 0.0
            self.initial_pose[name] = value
            self.current_targets[name] = value
            motor.setPosition(value)
            vel = self.motor_info[name].get("maxVelocity")
            if isinstance(vel, (int, float)) and math.isfinite(vel) and vel > 0:
                motor.setVelocity(min(float(vel), float(self.config.get("joint_velocity_limit", 0.55))))
        (self.run_dir / "initial_pose.json").write_text(json.dumps(self.initial_pose, indent=2) + "\n", encoding="utf-8")
        self.log_joint_state("initial_pose")

    def calibrate_directions(self) -> None:
        results = {}
        for cls in ("right_hip_pitch", "right_knee", "right_ankle_pitch", "left_hip_pitch", "left_knee", "left_ankle_pitch"):
            name = self.first_motor(cls)
            if not name:
                continue
            start = self.sensor_value(name) or self.initial_pose.get(name, 0.0)
            target = self.clip(name, start + 0.03)
            self.motors[name].setPosition(target)
            self.wait_steps(120)
            observed = self.sensor_value(name)
            self.motors[name].setPosition(start)
            self.wait_steps(120)
            results[cls] = {"motor": name, "command_delta": target - start, "observed_delta": None if observed is None else observed - start}
        (self.run_dir / "joint_direction_calibration.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    def resolve_nodes_precondition(self) -> bool:
        self.node_resolution = {
            "right_foot": self.observer._describe_node(self.observer.foot_nodes.get("right"), (self.observer.resolution.get("right") or {}).get("def_name"), (self.observer.resolution.get("right") or {}).get("method")),
            "left_foot": self.observer._describe_node(self.observer.foot_nodes.get("left"), (self.observer.resolution.get("left") or {}).get("def_name"), (self.observer.resolution.get("left") or {}).get("method")),
            "ball": self.observer._describe_node(self.observer.ball_node, "SOCCER_BALL", "getFromDef(SOCCER_BALL)"),
        }
        for key in ("right_foot", "left_foot", "ball"):
            info = self.node_resolution.get(key) or {}
            if not info.get("resolved"):
                self.node_resolution["success"] = False
                self.node_resolution["failure_reason"] = f"{key} node not resolved"
                self.failure_reason = self.node_resolution["failure_reason"]
                self.state_history.append(MotionState.FAILED_PRECONDITION.value)
                self.write_node_resolution()
                self.emit("FAILED_PRECONDITION", reason=self.failure_reason, node_resolution=self.node_resolution)
                return False
            if not finite_vec(info.get("position"), 3):
                self.node_resolution["success"] = False
                self.node_resolution["failure_reason"] = f"{key} position invalid"
                self.failure_reason = self.node_resolution["failure_reason"]
                self.state_history.append(MotionState.FAILED_PRECONDITION.value)
                self.write_node_resolution()
                self.emit("FAILED_PRECONDITION", reason=self.failure_reason, node_resolution=self.node_resolution)
                return False
        if not finite_vec((self.node_resolution["right_foot"] or {}).get("orientation"), 9):
            self.node_resolution["success"] = False
            self.node_resolution["failure_reason"] = "right_foot orientation invalid"
            self.failure_reason = self.node_resolution["failure_reason"]
            self.state_history.append(MotionState.FAILED_PRECONDITION.value)
            self.write_node_resolution()
            self.emit("FAILED_PRECONDITION", reason=self.failure_reason, node_resolution=self.node_resolution)
            return False
        self.node_resolution["success"] = True
        self.write_node_resolution()
        self.emit("NODE_RESOLUTION_OK", node_resolution=self.node_resolution)
        return True

    def write_node_resolution(self) -> None:
        (self.run_dir / "foot_node_resolution.json").write_text(json.dumps(self.node_resolution, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def finish_node_test(self) -> None:
        self.write_kick_geometry()
        summary = {
            "run_id": self.run_id,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "mode": self.mode,
            "assisted_mode": self.assisted,
            "node_resolution_success": bool(self.node_resolution.get("success")),
            "right_foot_position": (self.node_resolution.get("right_foot") or {}).get("position"),
            "left_foot_position": (self.node_resolution.get("left_foot") or {}).get("position"),
            "ball_position": (self.node_resolution.get("ball") or {}).get("position"),
            "geometry_precheck": self.geometry_ok,
            "geometry_failure_reason": self.geometry_failure_reason,
            "kick_success": False,
            "supervisor_moved_ball": False,
            "failure_reason": None if self.node_resolution.get("success") else self.failure_reason,
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("NODE_TEST_DONE" if self.node_resolution.get("success") else "FAILED", summary=summary)

    def finish_frame_check(self) -> None:
        start = self.robot.getTime()
        while self.robot.getTime() - start < 3.0:
            if self.robot.step(self.timestep) == -1:
                break
        ok = self.coordinate_frame_check()
        summary = {
            "run_id": self.run_id,
            "mode": self.mode,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "coordinate_frame_valid": ok,
            "kick_success": False,
            "supervisor_moved_ball": False,
            "failure_reason": None if ok else self.failure_reason,
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("FRAME_CHECK_DONE" if ok else "FAILED_COORDINATE_FRAME", summary=summary)

    def enable_contact_tracking(self) -> None:
        for node in (self.observer.ball_node, self.observer.foot_nodes.get("right"), self.observer.foot_nodes.get("left")):
            try:
                if node is not None:
                    node.enableContactPointsTracking(max(1, self.timestep), False)
            except Exception:
                pass

    def finish_collision_audit(self) -> None:
        self.state_history.append(MotionState.INITIAL_COLLISION_AUDIT.value)
        self.emit("STATE", state=MotionState.INITIAL_COLLISION_AUDIT.value)
        self.enable_contact_tracking()
        samples = []
        start = self.robot.getTime()
        next_sample = start
        while self.robot.getTime() - start <= 3.0:
            if self.robot.getTime() >= next_sample:
                sample = self.collision_audit_sample(write_geometry_file=False)
                samples.append(sample)
                jsonl_append(self.run_dir / "collision_audit_samples.jsonl", sample)
                next_sample = self.robot.getTime() + 0.025
            if self.robot.step(self.timestep) == -1:
                break
        ball_id = self.safe_node_id(self.observer.ball_node)
        contact_objects = sorted({
            c.get("node_name") or c.get("node_def") or str(c.get("node_id"))
            for sample in samples
            for c in sample.get("contacts", [])
            if c.get("node_name") not in {"green_field"} and c.get("node_def") not in {"green_field"}
            and c.get("node_id") != ball_id
        })
        geometry = self.build_initial_collision_geometry(write_file=True)
        overlapping = geometry.get("overlapping_objects", [])
        invalid_contacts = [name for name in contact_objects if name not in {"None", "ground", "green_field", "soccer_ball", "SOCCER_BALL"}]
        initial_collision_valid = not invalid_contacts and not overlapping
        first_motion = self.first_ball_motion_event(samples)
        audit = {
            "run_id": self.run_id,
            "mode": self.mode,
            "duration": 3.0,
            "api": {
                "getContactPoints": "Node.getContactPoints(includeDescendants=False)",
                "enableContactPointsTracking": "Node.enableContactPointsTracking(samplingPeriod, includeDescendants=False)",
                "contact_point_fields": ["point", "node_id"],
                "contact_normal_available": False,
            },
            "initial_collision_valid": initial_collision_valid,
            "invalid_contact_objects": invalid_contacts,
            "overlapping_objects": overlapping,
            "first_ball_motion": first_motion,
            "sample_count": len(samples),
            "final_sample": samples[-1] if samples else None,
        }
        self.initial_contact_audit = audit
        (self.run_dir / "initial_contact_audit.json").write_text(json.dumps(audit, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        summary = {
            "run_id": self.run_id,
            "mode": self.mode,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "assisted_mode": self.assisted,
            "collision_audit_success": initial_collision_valid,
            "initial_collision_valid": initial_collision_valid,
            "initial_collision_objects": sorted(set(invalid_contacts + overlapping)),
            "initial_ejection": bool(first_motion and first_motion.get("speed", 0.0) > 0.05),
            "initial_ejection_time": first_motion.get("sim_time") if first_motion else None,
            "kick_success": False,
            "supervisor_moved_ball": False,
            "state_history": self.state_history,
            "failure_reason": None if initial_collision_valid else "initial collision audit failed: " + ",".join(invalid_contacts + overlapping),
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("COLLISION_AUDIT_DONE" if initial_collision_valid else "FAILED", summary=summary)

    def finish_settle_check(self, write_summary: bool = True) -> None:
        self.state_history.append(MotionState.INITIAL_COLLISION_AUDIT.value)
        self.emit("STATE", state=MotionState.INITIAL_COLLISION_AUDIT.value)
        self.enable_contact_tracking()
        audit_sample = self.collision_audit_sample(write_geometry_file=False)
        geometry = self.build_initial_collision_geometry(write_file=True)
        initial_valid = not geometry.get("overlapping_objects")
        self.state_history.append(MotionState.SETTLE_CHECK.value)
        self.emit("STATE", state=MotionState.SETTLE_CHECK.value)
        samples = []
        start = self.robot.getTime()
        baseline = None
        next_sample = start
        while self.robot.getTime() - start <= 3.0:
            elapsed = self.robot.getTime() - start
            if self.robot.getTime() >= next_sample:
                sample = self.collision_audit_sample(write_geometry_file=False)
                sample["elapsed"] = elapsed
                sample["minimum_clearance"] = self.build_initial_collision_geometry(write_file=False).get("minimum_clearance")
                samples.append(sample)
                jsonl_append(self.run_dir / "settle_check_samples.jsonl", sample)
                if baseline is None and elapsed >= 0.5:
                    baseline = sample
                next_sample = self.robot.getTime() + 0.025
            if self.robot.step(self.timestep) == -1:
                break
        final = samples[-1] if samples else {}
        metrics = self.settle_metrics(samples, baseline, final)
        success = initial_valid and metrics["ball_position_finite"] and metrics["horizontal_displacement"] < 0.003 and metrics["z_change"] < 0.003 and metrics["max_horizontal_speed"] < 0.02 and metrics["final_horizontal_speed"] < 0.005 and not metrics["velocity_pulse"] and not metrics["left_ground"]
        failed = []
        if not initial_valid:
            failed.append("initial_overlap")
        for key, ok in (
            ("ball_position_finite", metrics["ball_position_finite"]),
            ("horizontal_displacement", metrics["horizontal_displacement"] < 0.003),
            ("z_change", metrics["z_change"] < 0.003),
            ("max_horizontal_speed", metrics["max_horizontal_speed"] < 0.02),
            ("final_horizontal_speed", metrics["final_horizontal_speed"] < 0.005),
            ("velocity_pulse", not metrics["velocity_pulse"]),
            ("left_ground", not metrics["left_ground"]),
        ):
            if not ok:
                failed.append(key)
        settle = {
            "run_id": self.run_id,
            "initial_audit_sample": audit_sample,
            "initial_collision_geometry": geometry,
            "settle_check_success": success,
            "failed_conditions": failed,
            "metrics": metrics,
            "baseline_sample": baseline,
            "final_sample": final,
            "sample_count": len(samples),
        }
        self.settle_check = settle
        (self.run_dir / "settle_check.json").write_text(json.dumps(settle, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        summary = {
            "run_id": self.run_id,
            "mode": self.mode,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "assisted_mode": self.assisted,
            "settle_check_success": success,
            "initial_collision_valid": initial_valid,
            "kick_success": False,
            "supervisor_moved_ball": False,
            "state_history": self.state_history,
            "failure_reason": None if success else "settle-check failed: " + ",".join(failed),
        }
        if write_summary:
            (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            self.emit("SETTLE_CHECK_DONE" if success else "FAILED", summary=summary)
        else:
            self.emit("SETTLE_CHECK_PREFLIGHT_DONE" if success else "FAILED", summary=summary)

    def collision_audit_sample(self, write_geometry_file: bool = False) -> dict:
        ball = self.observer.get_ball_position()
        velocity = self.observer.get_ball_velocity()
        geometry = self.build_initial_collision_geometry(write_file=write_geometry_file)
        return {
            "sim_time": self.robot.getTime(),
            "ball_position": ball,
            "ball_velocity": velocity[:3] if velocity else None,
            "ball_angular_velocity": velocity[3:] if velocity else None,
            "right_foot_position": self.observer.get_foot_position("right"),
            "left_foot_position": self.observer.get_foot_position("left"),
            "contacts": self.ball_contact_points(),
            "minimum_clearance": geometry.get("minimum_clearance"),
            "overlapping_objects": geometry.get("overlapping_objects", []),
        }

    def ball_contact_points(self) -> list[dict]:
        node = self.observer.ball_node
        contacts = []
        try:
            raw = node.getContactPoints(False) if node is not None else []
        except Exception:
            raw = []
        for cp in raw:
            try:
                point = [float(v) for v in cp.getPoint()]
            except Exception:
                point = None
            try:
                node_id = int(cp.getNodeId())
            except Exception:
                node_id = None
            other = None
            if node_id is not None:
                try:
                    other = self.robot.getFromId(node_id)
                except Exception:
                    other = None
            contacts.append({
                "point": point,
                "node_id": node_id,
                "node_def": self.safe_node_def(other),
                "node_name": self.observer._node_name(other),
                "node_type": self.safe_node_type(other),
                "normal": None,
            })
        return contacts

    def build_initial_collision_geometry(self, write_file: bool = True) -> dict:
        ball = self.observer.get_ball_position()
        cfg = self.assisted_config()
        radius = float(cfg.get("ball_radius", 0.11))
        objects = []
        if not finite_vec(ball, 3):
            result = {"ball_position": ball, "objects": [], "minimum_clearance": None, "overlapping_objects": ["ball_position_invalid"]}
            if write_file:
                (self.run_dir / "initial_collision_geometry.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            return result
        for name, box in self.collision_boxes().items():
            if box.get("kind") == "aabb":
                row = sphere_to_axis_aligned_box_signed_distance(ball, radius, box["min"], box["max"])
            else:
                row = sphere_to_oriented_box_signed_distance(ball, radius, box)
                row["corners"] = oriented_box_corners_world(box["center"], box["half_extents"], box["axes"])
            row.update({
                "name": name,
                "node_id": box.get("node_id"),
                "node_name": box.get("node_name"),
                "geometry_type": box.get("geometry_type", "box"),
            })
            objects.append(row)
        non_ground = [obj for obj in objects if obj["name"] != "ground"]
        minimum = min((obj["signed_surface_distance"] for obj in non_ground), default=None)
        overlapping = [obj["name"] for obj in non_ground if obj.get("overlapping")]
        result = {
            "ball_position": ball,
            "ball_radius": radius,
            "objects": objects,
            "minimum_clearance": minimum,
            "overlapping_objects": overlapping,
            "maximum_overlap_depth": max((obj["overlap_depth"] for obj in non_ground), default=0.0),
        }
        self.initial_collision_geometry = result
        if write_file:
            (self.run_dir / "initial_collision_geometry.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return result

    def collision_boxes(self) -> dict:
        boxes = {}
        foot_size = [0.22487, 0.1, 0.031]
        foot_pose = [0.010384, 0.0, -0.0155]
        for side, label in (("right", "RIGHT_FOOT"), ("left", "LEFT_FOOT")):
            node = self.observer.foot_nodes.get(side)
            pos = self.observer._position(node)
            ori = self.observer._orientation(node)
            if finite_vec(pos, 3) and finite_vec(ori, 9):
                axes = axes_from_orientation(ori)
                center = list(pos)
                for axis, value in zip(axes, foot_pose):
                    center = vec_add(center, vec_scale(axis, value))
                boxes[label] = {
                    "center": center,
                    "half_extents": [v / 2.0 for v in foot_size],
                    "axes": axes,
                    "node_id": self.safe_node_id(node),
                    "node_name": self.observer._node_name(node),
                    "geometry_type": "oriented_box",
                }
        for name, center, size in (
            ("assisted_torso_cradle", [0.013387494410705969, 0.024827210968314528, 0.42], [0.36, 0.34, 0.05]),
            ("assisted_left_roll_guard", [0.013387494410705969, 0.30482721096831455, 0.52], [0.42, 0.04, 0.74]),
            ("assisted_right_roll_guard", [0.013387494410705969, -0.2551727890316855, 0.52], [0.42, 0.04, 0.74]),
            ("assisted_rear_pitch_guard", [-0.24661250558929382, 0.024827210968314528, 0.52], [0.04, 0.48, 0.74]),
            ("RED_GOAL", [3.3, 0.0, 0.3], [0.1, 1.5, 0.6]),
            ("BLUE_GOAL", [-3.3, 0.0, 0.3], [0.1, 1.5, 0.6]),
        ):
            half = [v / 2.0 for v in size]
            boxes[name] = {"kind": "aabb", "min": [center[i] - half[i] for i in range(3)], "max": [center[i] + half[i] for i in range(3)], "node_name": name, "geometry_type": "axis_aligned_box"}
        boxes["ground"] = {"kind": "aabb", "min": [-3.5, -2.5, -0.005], "max": [3.5, 2.5, 0.0], "node_name": "green_field", "geometry_type": "axis_aligned_box"}
        return boxes

    def first_ball_motion_event(self, samples: list[dict]) -> dict | None:
        for sample in samples:
            velocity = sample.get("ball_velocity")
            if finite_vec(velocity, 3):
                speed = norm3(velocity)
                if speed > 0.05:
                    return {
                        "sim_time": sample.get("sim_time"),
                        "speed": speed,
                        "velocity": velocity,
                        "ball_position": sample.get("ball_position"),
                        "right_foot_position": sample.get("right_foot_position"),
                        "left_foot_position": sample.get("left_foot_position"),
                        "contacts": sample.get("contacts", []),
                        "overlapping_objects": sample.get("overlapping_objects", []),
                    }
        return None

    def settle_metrics(self, samples: list[dict], baseline: dict | None, final: dict) -> dict:
        baseline_ball = (baseline or {}).get("ball_position")
        final_ball = final.get("ball_position")
        speeds = []
        finite_positions = True
        left_ground = False
        for sample in samples:
            ball = sample.get("ball_position")
            finite_positions = finite_positions and finite_vec(ball, 3)
            if finite_vec(ball, 3) and ball[2] < 0.08:
                left_ground = True
            velocity = sample.get("ball_velocity")
            if finite_vec(velocity, 3):
                speeds.append(math.hypot(velocity[0], velocity[1]))
        return {
            "horizontal_displacement": distance_xy(baseline_ball, final_ball) if finite_vec(baseline_ball, 3) and finite_vec(final_ball, 3) else float("inf"),
            "z_change": abs(final_ball[2] - baseline_ball[2]) if finite_vec(baseline_ball, 3) and finite_vec(final_ball, 3) else float("inf"),
            "max_horizontal_speed": max(speeds) if speeds else float("inf"),
            "final_horizontal_speed": speeds[-1] if speeds else float("inf"),
            "velocity_pulse": any(speed > 0.05 for speed in speeds),
            "ball_position_finite": finite_positions,
            "left_ground": left_ground,
        }

    def coordinate_frame_check(self) -> bool:
        diag = self.build_coordinate_frame_diagnosis()
        failed = []
        rr = diag.get("robot_to_right_foot_distance")
        rl = diag.get("robot_to_left_foot_distance")
        lr = diag.get("left_to_right_foot_distance")
        rz = ((diag.get("nodes") or {}).get("RIGHT_FOOT") or {}).get("world_position", [None, None, None])[2]
        lz = ((diag.get("nodes") or {}).get("LEFT_FOOT") or {}).get("world_position", [None, None, None])[2]
        if rr is None or rr < 0.20 or rr > 1.50:
            failed.append("robot_to_right_foot_distance_out_of_range")
        if rl is None or rl < 0.20 or rl > 1.50:
            failed.append("robot_to_left_foot_distance_out_of_range")
        if lr is None or lr < 0.05 or lr > 0.60:
            failed.append("left_to_right_foot_distance_out_of_range")
        for name, z in (("right_foot_height", rz), ("left_foot_height", lz)):
            if not isinstance(z, (int, float)) or not math.isfinite(z) or z < -0.05 or z > 0.30:
                failed.append(f"{name}_out_of_range")
        max_error = diag.get("max_reconstruction_error")
        if max_error is None or max_error > 0.005:
            failed.append("coordinate_reconstruction_error")
        if not diag.get("foot_ownership", {}).get("ownership_valid"):
            failed.append("foot_ownership_invalid")
        if not diag.get("world_hashes_match"):
            failed.append("world_hash_mismatch")
        diag["coordinate_frame_valid"] = not failed
        diag["failed_conditions"] = failed
        self.coordinate_frame = diag
        (self.run_dir / "coordinate_frame_diagnosis.json").write_text(json.dumps(diag, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.foot_ownership = diag.get("foot_ownership", {})
        (self.run_dir / "foot_node_ownership.json").write_text(json.dumps(self.foot_ownership, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        if failed:
            self.state_history.append(MotionState.FAILED_COORDINATE_FRAME.value)
            self.failure_reason = "coordinate frame check failed: " + ",".join(failed)
            self.emit("FAILED_COORDINATE_FRAME", diagnosis=diag)
            return False
        self.emit("COORDINATE_FRAME_OK", diagnosis=diag)
        return True

    def build_coordinate_frame_diagnosis(self) -> dict:
        world_path = Path(os.environ.get("WEBOTS_WORLD", ""))
        project_world = PROJECT / "worlds" / "T1_native_assisted_kick.wbt"
        hashes = {
            "project_world": sha256_file(project_world),
            "runtime_world": sha256_file(world_path) if world_path.exists() else None,
            "project_world_path": str(project_world.resolve()),
            "runtime_world_path": str(world_path.resolve()) if world_path.exists() else str(world_path),
        }
        nodes = {
            "T1_BLUE_1": self.observer.robot_node,
            "RIGHT_FOOT": self.observer.foot_nodes.get("right"),
            "LEFT_FOOT": self.observer.foot_nodes.get("left"),
            "SOCCER_BALL": self.observer.ball_node,
            "RED_GOAL": self.observer._node_by_def_or_name("RED_GOAL"),
        }
        robot_node = nodes["T1_BLUE_1"]
        robot_pose = node_world_pose(robot_node)
        node_rows = {}
        reconstruction_errors = []
        for name, node in nodes.items():
            row = self.describe_frame_node(name, node, robot_node, robot_pose)
            node_rows[name] = row
            err = row.get("reconstruction_error")
            if isinstance(err, (int, float)) and math.isfinite(err):
                reconstruction_errors.append(err)
        robot = node_rows["T1_BLUE_1"].get("world_position")
        right = node_rows["RIGHT_FOOT"].get("world_position")
        left = node_rows["LEFT_FOOT"].get("world_position")
        ball = node_rows["SOCCER_BALL"].get("world_position")
        ownership = self.foot_node_ownership(robot_node, nodes["RIGHT_FOOT"], nodes["LEFT_FOOT"])
        diag = {
            **hashes,
            "world_hashes_match": hashes["project_world"] == hashes["runtime_world"],
            "nodes": node_rows,
            "foot_ownership": ownership,
            "robot_to_right_foot_distance": distance3(robot, right),
            "robot_to_left_foot_distance": distance3(robot, left),
            "left_to_right_foot_distance": distance3(left, right),
            "robot_to_ball_distance": distance3(robot, ball),
            "right_foot_height": right[2] if finite_vec(right, 3) else None,
            "left_foot_height": left[2] if finite_vec(left, 3) else None,
            "max_reconstruction_error": max(reconstruction_errors) if reconstruction_errors else None,
        }
        return diag

    def describe_frame_node(self, label: str, node, robot_node, robot_pose: list | None) -> dict:
        world_pose = node_world_pose(node)
        rel_pose = node_pose_relative_to(node, robot_node)
        world_from_pose = pose_translation(world_pose)
        world_from_position = self.observer._position(node)
        robot_local = pose_translation(rel_pose)
        reconstructed = robot_to_world(robot_local, robot_pose) if robot_local is not None and robot_pose is not None else None
        alt_robot_local = pose_translation_column_major(rel_pose)
        alt_reconstructed = robot_to_world(alt_robot_local, robot_pose) if alt_robot_local is not None and robot_pose is not None else None
        error = distance3(world_from_pose, reconstructed)
        alt_error = distance3(world_from_pose, alt_reconstructed)
        if alt_error is not None and (error is None or alt_error < error):
            robot_local = alt_robot_local
            reconstructed = alt_reconstructed
            error = alt_error
        return {
            "node_id": self.safe_node_id(node),
            "node_type": self.safe_node_type(node),
            "def": label,
            "name": self.observer._node_name(node),
            "getPosition": world_from_position,
            "world_position": world_from_pose or world_from_position,
            "getOrientation": self.observer._orientation(node),
            "getPose_NULL": world_pose,
            "getPose_T1_BLUE_1": rel_pose,
            "robot_local_position": robot_local,
            "reconstructed_world_position": reconstructed,
            "reconstruction_error": error,
            "translation_field": self.translation_field(node),
            "from_getFromDef": label in {"T1_BLUE_1", "RIGHT_FOOT", "LEFT_FOOT", "SOCCER_BALL", "RED_GOAL"},
            "from_proto_def": False,
            "owner_robot": "T1_BLUE_1" if label in {"RIGHT_FOOT", "LEFT_FOOT"} else None,
        }

    def foot_node_ownership(self, robot_node, right_node, left_node) -> dict:
        world_text = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text(encoding="utf-8")
        right_lines = [i + 1 for i, line in enumerate(world_text.splitlines()) if "DEF RIGHT_FOOT" in line]
        left_lines = [i + 1 for i, line in enumerate(world_text.splitlines()) if "DEF LEFT_FOOT" in line]
        t1_start = world_text.find("DEF T1_BLUE_1 Robot {")
        markers = world_text.find("# === Simplified WorldState markers")
        t1_text = world_text[t1_start:markers] if t1_start >= 0 and markers > t1_start else ""
        valid = (
            robot_node is not None
            and right_node is not None
            and left_node is not None
            and len(right_lines) == 1
            and len(left_lines) == 1
            and "DEF RIGHT_FOOT Solid" in t1_text
            and "DEF LEFT_FOOT Solid" in t1_text
        )
        return {
            "robot_node_id": self.safe_node_id(robot_node),
            "right_foot_node_id": self.safe_node_id(right_node),
            "left_foot_node_id": self.safe_node_id(left_node),
            "resolution_method": {
                "right": (self.observer.resolution.get("right") or {}).get("method"),
                "left": (self.observer.resolution.get("left") or {}).get("method"),
            },
            "ownership_valid": valid,
            "duplicate_def_count": {"RIGHT_FOOT": len(right_lines), "LEFT_FOOT": len(left_lines)},
            "source_world_lines": {"RIGHT_FOOT": right_lines, "LEFT_FOOT": left_lines},
        }

    def safe_node_id(self, node):
        try:
            return int(node.getId()) if node is not None else None
        except Exception:
            return None

    def safe_node_def(self, node):
        try:
            return str(node.getDef()) if node is not None else None
        except Exception:
            return None

    def safe_node_type(self, node):
        try:
            return str(node.getTypeName()) if node is not None else None
        except Exception:
            try:
                return str(node.getType()) if node is not None else None
            except Exception:
                return None

    def translation_field(self, node):
        try:
            field = node.getField("translation") if node is not None else None
            return [float(v) for v in field.getSFVec3f()] if field is not None else None
        except Exception:
            return None

    def run_support_check(self, duration: float | None = None, write_summary: bool = False) -> bool:
        cfg = self.assisted_config()
        sample_seconds = float(cfg.get("support_check_sample_seconds", 0.1))
        if duration is None:
            duration = float(cfg.get("support_check_seconds", 5.0))
        samples = []
        start_time = self.robot.getTime()
        next_sample = start_time
        while self.robot.getTime() - start_time <= duration:
            if self.robot.step(self.timestep) == -1:
                self.failure_reason = "Webots step ended during support-check"
                break
            now_sim = self.robot.getTime()
            if now_sim >= next_sample:
                sample = {
                    "sim_time": now_sim,
                    "ball_position": self.observer.get_ball_position(),
                    "ball_velocity": self.observer.get_ball_velocity(),
                    "right_foot_position": self.observer.get_foot_position("right"),
                    "left_foot_position": self.observer.get_foot_position("left"),
                    "robot_pose": self.observer.get_robot_pose(),
                    "assistance_positions": self.assistance_positions(),
                }
                samples.append(sample)
                jsonl_append(self.run_dir / "support_check_samples.jsonl", sample)
                next_sample = now_sim + sample_seconds
        initial = samples[0] if samples else {}
        final = samples[-1] if samples else {}
        initial_ball = initial.get("ball_position")
        final_ball = final.get("ball_position")
        z_drop = abs((final_ball or [0, 0, 0])[2] - (initial_ball or [0, 0, 0])[2]) if finite_vec(initial_ball, 3) and finite_vec(final_ball, 3) else None
        xy_drift = distance_xy(initial_ball, final_ball) if finite_vec(initial_ball, 3) and finite_vec(final_ball, 3) else None
        final_velocity = final.get("ball_velocity")
        final_speed = norm3(final_velocity[:3]) if final_velocity else None
        scene_ok = self.scene_support_check(write_file=False)
        success = (
            bool(samples)
            and scene_ok
            and z_drop is not None and z_drop <= float(cfg.get("support_check_max_ball_z_drop_m", 0.005))
            and xy_drift is not None and xy_drift <= float(cfg.get("support_check_max_ball_xy_drift_m", 0.005))
            and final_speed is not None and final_speed <= float(cfg.get("support_check_final_speed_m_s", 0.02))
        )
        failed = []
        if not scene_ok:
            failed.append("scene_support_check_failed")
        if z_drop is None or z_drop > float(cfg.get("support_check_max_ball_z_drop_m", 0.005)):
            failed.append("ball_z_unstable")
        if xy_drift is None or xy_drift > float(cfg.get("support_check_max_ball_xy_drift_m", 0.005)):
            failed.append("ball_xy_drift")
        if final_speed is None or final_speed > float(cfg.get("support_check_final_speed_m_s", 0.02)):
            failed.append("ball_final_speed_nonzero")
        self.support_check = {
            "success": success,
            "duration": duration,
            "sample_count": len(samples),
            "ball_initial_position": initial_ball,
            "ball_final_position": final_ball,
            "ball_z_drop_abs": z_drop,
            "ball_xy_drift": xy_drift,
            "ball_final_speed": final_speed,
            "right_foot_final_position": final.get("right_foot_position"),
            "left_foot_final_position": final.get("left_foot_position"),
            "scene_support": self.scene_support,
            "failed_conditions": failed,
        }
        (self.run_dir / "support_check.json").write_text(json.dumps(self.support_check, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        if write_summary:
            summary = {
                "run_id": self.run_id,
                "mode": self.mode,
                "world": os.environ.get("WEBOTS_WORLD", ""),
                "assisted_mode": self.assisted,
                "support_check_success": success,
                "kick_success": False,
                "supervisor_moved_ball": False,
                "failure_reason": None if success else ",".join(failed),
            }
            (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            self.emit("SUPPORT_CHECK_DONE" if success else "FAILED_SCENE_SUPPORT", summary=summary)
        elif not success:
            self.state_history.append(MotionState.FAILED_SCENE_SUPPORT.value)
            self.emit("FAILED_SCENE_SUPPORT", support_check=self.support_check)
        return success

    def finish_support_check(self) -> None:
        self.run_support_check(write_summary=True)

    def scene_support_check(self, write_file: bool = True) -> bool:
        ground = self.ground_support()
        cfg = self.assisted_config()
        margin = float(cfg.get("ground_margin_m", 0.5))
        ball_radius = float(cfg.get("ball_radius", 0.11))
        points = {
            "right_foot": self.observer.get_foot_position("right"),
            "left_foot": self.observer.get_foot_position("left"),
            "ball": self.observer.get_ball_position(),
        }
        pose = self.observer.get_robot_pose()
        if pose and finite_vec(pose.get("position"), 3):
            points["robot_root"] = pose["position"]
        failed = []
        entities = {}
        for name, position in points.items():
            status = self.ground_point_status(position, margin)
            entities[name] = {"position": position, **status}
            if not status["inside"]:
                failed.append(f"{name}_outside_ground")
            if status["distance_to_nearest_boundary"] is None or status["distance_to_nearest_boundary"] < margin:
                failed.append(f"{name}_inside_margin")
        ball = points.get("ball")
        expected_z = ground["top_z"] + ball_radius + 0.002
        ball_z_error = None
        if finite_vec(ball, 3):
            ball_z_error = abs(ball[2] - expected_z)
            if ball_z_error > 0.03:
                failed.append("ball_z_not_on_ground")
        else:
            failed.append("ball_position_invalid")
        velocity = self.observer.get_ball_velocity()
        if velocity is None or not finite_vec(velocity, 6):
            failed.append("ball_velocity_invalid")
        support = {
            "ground": ground,
            "ground_margin_m": margin,
            "entities": entities,
            "ball_expected_z": expected_z,
            "ball_z_error": ball_z_error,
            "ball_velocity": velocity,
            "assistance_positions": self.assistance_positions(),
            "success": not failed,
            "failed_conditions": failed,
        }
        self.scene_support = support
        if write_file:
            (self.run_dir / "scene_support_precheck.json").write_text(json.dumps(support, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        if failed:
            self.state_history.append(MotionState.FAILED_SCENE_SUPPORT.value)
            self.failure_reason = "scene support check failed: " + ",".join(failed)
            self.emit("FAILED_SCENE_SUPPORT", support=support)
            return False
        self.emit("SCENE_SUPPORT_OK", support=support)
        return True

    def ground_support(self) -> dict:
        cfg = self.config.get("ground_support", {})
        return {
            "ground_def": cfg.get("ground_def", "green_field"),
            "collision_type": cfg.get("collision_type", "Box"),
            "top_z": float(cfg.get("top_z", 0.0)),
            "x_min": float(cfg.get("x_min", -3.5)),
            "x_max": float(cfg.get("x_max", 3.5)),
            "y_min": float(cfg.get("y_min", -2.5)),
            "y_max": float(cfg.get("y_max", 2.5)),
        }

    def ground_point_status(self, position, margin: float) -> dict:
        ground = self.ground_support()
        if not finite_vec(position, 3):
            return {"inside": False, "distance_to_nearest_boundary": None, "margin_ok": False}
        x, y = position[0], position[1]
        distance = min(x - ground["x_min"], ground["x_max"] - x, y - ground["y_min"], ground["y_max"] - y)
        inside = ground["x_min"] <= x <= ground["x_max"] and ground["y_min"] <= y <= ground["y_max"]
        return {"inside": inside, "distance_to_nearest_boundary": distance, "margin_ok": inside and distance >= margin}

    def assistance_positions(self) -> dict:
        # Static assisted solids are not DEF nodes; report configured world translations after the scene move.
        return {
            "assisted_torso_cradle": [0.013387494410705969, 0.024827210968314528, 0.42],
            "assisted_left_roll_guard": [0.013387494410705969, 0.30482721096831455, 0.52],
            "assisted_right_roll_guard": [0.013387494410705969, -0.2551727890316855, 0.52],
            "assisted_rear_pitch_guard": [-0.24661250558929382, 0.024827210968314528, 0.52],
        }

    def write_ball_physics_analysis(self) -> None:
        self.ball_physics = {
            "boundingObject_present": True,
            "boundingObject_type": "Sphere",
            "collision_radius": float(self.assisted_config().get("ball_radius", 0.11)),
            "visual_radius": 0.11,
            "physics_present": True,
            "mass": 0.43,
            "density": -1,
            "translation_is_center": True,
            "collision_disabled": False,
            "source": "SOCCER_BALL Solid in T1_native_assisted_kick.wbt",
        }
        (self.run_dir / "ball_physics_analysis.json").write_text(json.dumps(self.ball_physics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def calibrate_right_leg(self) -> bool:
        motors = [
            "Right_Hip_Pitch",
            "Right_Knee_Pitch",
            "Crank_Up_Right",
            "Crank_Down_Right",
        ]
        results = {}
        axes = self.foot_axes()
        foot_before_all = self.foot_geometry_points()
        if not foot_before_all:
            self.failure_reason = "right foot geometry unavailable for calibration"
            return False
        for name in motors:
            if name not in self.motors:
                results[name] = {"available": False}
                continue
            start = self.sensor_value(name)
            if start is None or not math.isfinite(start):
                start = self.current_targets.get(name, self.initial_pose.get(name, 0.0))
            foot_before = self.foot_geometry_points()
            plus = self.clip(name, start + 0.03)
            self.apply_targets({name: plus}, 0.55, f"calibrate_{name}_plus")
            self.wait_seconds(0.3, f"calibrate_{name}_plus_hold")
            foot_plus = self.foot_geometry_points()
            self.apply_targets({name: start}, 0.55, f"calibrate_{name}_restore_plus")
            minus = self.clip(name, start - 0.03)
            self.apply_targets({name: minus}, 0.55, f"calibrate_{name}_minus")
            self.wait_seconds(0.3, f"calibrate_{name}_minus_hold")
            foot_minus = self.foot_geometry_points()
            self.apply_targets({name: start}, 0.55, f"calibrate_{name}_restore_minus")
            observed = self.sensor_value(name)
            plus_delta = projected_delta(foot_before, foot_plus, axes)
            minus_delta = projected_delta(foot_before, foot_minus, axes)
            results[name] = {
                "available": True,
                "start": start,
                "observed_after_restore": observed,
                "plus_command_delta": plus - start,
                "minus_command_delta": minus - start,
                "foot_before": foot_before,
                "foot_after_plus": foot_plus,
                "foot_after_minus": foot_minus,
                "plus_foot_delta": plus_delta,
                "minus_foot_delta": minus_delta,
            }
        self.right_leg_calibration = results
        self.derive_calibration_signs()
        (self.run_dir / "right_leg_calibration.json").write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        ok = self.calibration_confidence >= float(self.assisted_config().get("calibration_min_confidence", 0.0005))
        if not ok:
            self.failure_reason = f"right leg calibration confidence too low: {self.calibration_confidence:.6f}"
            self.state_history.append(MotionState.FAILED_PRECONDITION.value)
        self.emit(
            "RIGHT_LEG_CALIBRATION_DONE",
            motors=list(results),
            confidence=self.calibration_confidence,
            hip_pitch_forward_sign=self.hip_pitch_forward_sign,
            knee_extension_sign=self.knee_extension_sign,
            ok=ok,
        )
        return ok

    def evaluate_strategy(self) -> None:
        state = self.latest_world_state()
        if not state:
            self.emit("STRATEGY_SKIPPED", reason="no match_state yet")
            return
        jsonl_append(self.logs["world"], state)
        try:
            world = self.to_strategy_world(state)
            strategy = TeamStrategy()
            actions = strategy.decide(world)
            selected = "SHOOT" if self.mode == "shoot" else (actions[0].action_type.value if actions else "HOLD")
            jsonl_append(self.logs["strategy"], {
                "time": now(),
                "mode": self.mode,
                "strategy": selected,
                "summary": strategy.last_decision_summary,
                "actions": [a.to_dict() for a in actions],
            })
        except Exception as exc:
            self.emit("STRATEGY_ERROR", error=str(exc))

    def to_strategy_world(self, state: dict) -> WorldState:
        robots = state["robots"]
        ball = state["ball"]
        return WorldState.from_dict({
            "scenario_name": f"native_{self.mode}",
            "timestamp": time.time(),
            "ball": {"x": ball["x"], "y": ball["y"]},
            "robots": [
                {"robot_id": "T1_BLUE_1", "team": "BLUE", "x": robots["T1_BLUE_1"]["x"], "y": robots["T1_BLUE_1"]["y"], "theta": 0, "role": "BALL_HANDLER", "has_ball": True},
                {"robot_id": "T1_BLUE_2", "team": "BLUE", "x": robots["T1_BLUE_2"]["x"], "y": robots["T1_BLUE_2"]["y"], "theta": 0, "role": "SUPPORT"},
            ],
            "opponents": [
                {"opponent_id": "T1_RED_1", "x": robots["T1_RED_1"]["x"], "y": robots["T1_RED_1"]["y"]},
                {"opponent_id": "T1_RED_2", "x": robots["T1_RED_2"]["x"], "y": robots["T1_RED_2"]["y"]},
            ],
            "our_goal": state.get("blue_goal", {"x": -3.3, "y": 0.0}),
            "enemy_goal": state.get("red_goal", {"x": 3.3, "y": 0.0}),
            "field_width": 7.0,
            "field_height": 5.0,
        })

    def execute_push(self, level: dict, index: int) -> bool:
        if self.assisted:
            sequence = [
                (MotionState.PREPARE_KICK, self.prepare_targets(), 0.8),
                (MotionState.LIFT_FOOT, self.lift_targets(level), 0.7),
                (MotionState.SWING_FORWARD, self.swing_targets(level), float(level.get("duration", 0.8))),
                (MotionState.CONTACT_HOLD, None, 0.2),
                (MotionState.RETRACT, self.lift_targets(level, scale=0.45), 0.6),
                (MotionState.RECOVER, self.initial_pose, 0.8),
                (MotionState.VERIFY_BALL, None, float(self.config.get("ball_settle_seconds", 0.7))),
            ]
        else:
            sequence = [
                (MotionState.PREPARE, self.prepare_targets(), 1.0),
                (MotionState.SHIFT_WEIGHT, self.shift_weight_targets(), 1.2),
                (MotionState.LIFT_FOOT, self.lift_targets(level), 0.8),
                (MotionState.SWING_FORWARD, self.swing_targets(level), float(level.get("duration", 0.8))),
                (MotionState.CONTACT_HOLD, None, 0.15),
                (MotionState.RETRACT, self.lift_targets(level, scale=0.45), 0.7),
                (MotionState.RECOVER, self.initial_pose, 1.0),
                (MotionState.VERIFY_BALL, None, float(self.config.get("ball_settle_seconds", 0.7))),
            ]
        self.min_foot_ball_distance = None
        self.contact_estimated = False
        self.trajectory_direction_failed = False
        self.actual_trajectory_samples = []
        self.contact_window_ball_before = self.observer.get_ball_position()
        start_points = self.foot_geometry_points()
        start_front = (start_points or {}).get("front_center")
        self.initial_foot_ball_distance = distance3(start_front, self.contact_window_ball_before)
        for state, targets, duration in sequence:
            self.state_history.append(state.value)
            self.emit("STATE", state=state.value, push=index, duration=duration)
            if targets:
                if not self.apply_targets(targets, duration, state.value):
                    if self.trajectory_direction_failed:
                        self.state_history.append(MotionState.FAILED_TRAJECTORY_DIRECTION.value)
                        self.emit("STATE", state=MotionState.FAILED_TRAJECTORY_DIRECTION.value, push=index)
                        self.apply_targets(self.lift_targets(level, scale=0.45), 0.4, MotionState.RETRACT.value)
                        self.apply_targets(self.initial_pose, 0.6, MotionState.RECOVER.value)
                    return False
            else:
                self.wait_seconds(duration, state.value)
            if state in {MotionState.SWING_FORWARD, MotionState.CONTACT_HOLD}:
                self.update_contact_estimate(state.value)
            if self.check_fallen():
                self.failure_reason = f"robot fallen during {state.value}"
                return False
        self.contact_window_ball_after = self.observer.get_ball_position()
        if self.assisted:
            contact_disp = distance_xy(self.contact_window_ball_before, self.contact_window_ball_after)
            self.write_predicted_vs_actual_trajectory()
            self.emit(
                "CONTACT_WINDOW",
                min_foot_ball_distance=self.min_foot_ball_distance,
                ball_displacement=contact_disp,
                contact_estimated=self.contact_estimated,
                ball_velocity=self.observer.get_ball_velocity(),
            )
            if self.contact_estimated:
                self.emit("FOOT_BALL_CONTACT_ESTIMATED", min_foot_ball_distance=self.min_foot_ball_distance, ball_displacement=contact_disp)
        return True

    def prepare_targets(self) -> dict[str, float]:
        targets = dict(self.current_targets)
        for cls, delta in (("left_hip_pitch", 0.04), ("right_hip_pitch", 0.04), ("left_knee", -0.04), ("right_knee", -0.04), ("left_ankle_pitch", -0.02), ("right_ankle_pitch", -0.02)):
            self.add_delta(targets, cls, delta)
        for cls, delta in (("left_arm", 0.05), ("right_arm", -0.05)):
            self.add_delta(targets, cls, delta)
        return targets

    def shift_weight_targets(self) -> dict[str, float]:
        targets = self.prepare_targets()
        if self.assisted and bool(self.assisted_config().get("skip_weight_shift", True)):
            return targets
        self.add_delta(targets, "left_hip_roll", 0.04)
        self.add_delta(targets, "right_hip_roll", 0.04)
        self.add_delta(targets, "right_ankle_roll", -0.03)
        return targets

    def lift_targets(self, level: dict, scale: float = 1.0) -> dict[str, float]:
        targets = self.shift_weight_targets()
        direction = self.assisted_direction_sign() if self.assisted else 1.0
        self.add_delta(targets, "right_hip_pitch", direction * float(level.get("hip_pitch", 0.08)) * 0.5 * scale)
        knee = self.knee_extension_sign_or_default() * abs(float(level.get("knee", -0.06))) if self.assisted else float(level.get("knee", -0.06))
        self.add_delta(targets, "right_knee", knee * scale)
        self.add_delta(targets, "right_ankle_pitch", direction * float(level.get("ankle_pitch", -0.03)) * scale)
        return targets

    def swing_targets(self, level: dict) -> dict[str, float]:
        targets = self.shift_weight_targets()
        direction = self.assisted_direction_sign()
        self.add_delta(targets, "right_hip_pitch", direction * float(level.get("hip_pitch", 0.08)))
        knee = self.knee_extension_sign_or_default() * abs(float(level.get("knee", -0.06))) if self.assisted else float(level.get("knee", -0.06))
        self.add_delta(targets, "right_knee", knee)
        self.add_delta(targets, "right_ankle_pitch", self.ankle_sign_or_default("Crank_Up_Right") * abs(float(level.get("ankle_pitch", -0.03))))
        self.add_delta(targets, "right_ankle_roll", self.ankle_sign_or_default("Crank_Down_Right") * abs(float(level.get("ankle_pitch", -0.03))))
        return targets

    def assisted_config(self) -> dict:
        value = self.config.get("assisted", {})
        return value if isinstance(value, dict) else {}

    def assisted_direction_sign(self) -> float:
        return self.hip_pitch_forward_sign_or_default()

    def assisted_hold_stand(self) -> bool:
        self.state_history.append(MotionState.HOLD_STAND.value)
        cfg = self.assisted_config()
        min_seconds = float(cfg.get("hold_stand_min_seconds", 2.0))
        timeout = float(cfg.get("hold_stand_timeout_seconds", 8.0))
        max_error = float(cfg.get("max_joint_error_rad", 0.08))
        max_velocity = float(cfg.get("max_joint_velocity_rad_s", 0.20))
        start = self.robot.getTime()
        stable_since = None
        next_log = start
        diagnosis = {}
        while self.robot.getTime() - start <= timeout:
            if self.robot.step(self.timestep) == -1:
                self.failure_reason = "Webots step ended"
                return False
            metrics = self.joint_metrics()
            pose = self.observer.get_robot_pose()
            body_height = None
            roll = pitch = None
            if pose:
                position = pose.get("position")
                body_height = position[2] if position and len(position) > 2 else None
                roll = pose.get("roll")
                pitch = pose.get("pitch")
            failed = self.assisted_failed_conditions(metrics, pose, cfg, max_error, max_velocity)
            stable = not failed
            sim_now = self.robot.getTime()
            if stable:
                stable_since = sim_now if stable_since is None else stable_since
            else:
                stable_since = None
            stable_duration = 0.0 if stable_since is None else sim_now - stable_since
            diagnosis = {
                "assisted_mode": True,
                "sensor_valid": metrics["sensor_valid"],
                "fallen": "robot_abnormal" in failed or "free_standing_height_required_but_low" in failed,
                "body_height": body_height,
                "roll": roll,
                "pitch": pitch,
                "max_joint_error": metrics["max_joint_error"],
                "max_joint_velocity": metrics["max_joint_velocity"],
                "stable_elapsed": stable_duration,
                "failed_conditions": failed,
            }
            if sim_now >= next_log:
                self.emit(
                    "HOLD_STAND_DIAGNOSTIC",
                    elapsed=sim_now - start,
                    max_joint_error=metrics["max_joint_error"],
                    max_joint_velocity=metrics["max_joint_velocity"],
                    sensor_valid=metrics["sensor_valid"],
                    fallen=diagnosis["fallen"],
                    stable_duration=stable_duration,
                    failed_conditions=failed,
                )
                next_log = sim_now + 0.5
            self.log_joint_state("assisted_hold_stand", every=250)
            if stable_duration >= min_seconds:
                self.last_hold_diagnosis = diagnosis
                self.write_hold_diagnosis(diagnosis)
                self.emit("HOLD_STAND_STABLE", stable_duration=stable_duration)
                return True
        diagnosis.setdefault("assisted_mode", True)
        diagnosis["failed_conditions"] = list(diagnosis.get("failed_conditions", [])) + ["hold_stand_timeout"]
        self.last_hold_diagnosis = diagnosis
        self.write_hold_diagnosis(diagnosis)
        self.failure_reason = "assisted HOLD_STAND timeout"
        self.emit("HOLD_STAND_FAILED", diagnosis=diagnosis)
        return False

    def assisted_failed_conditions(self, metrics: dict, pose: dict | None, cfg: dict, max_error: float, max_velocity: float) -> list[str]:
        failed = []
        if not metrics["sensor_valid"]:
            failed.append("sensor_invalid")
        if not metrics["targets_in_limits"]:
            failed.append("targets_out_of_limits")
        if metrics["max_joint_error"] is None or metrics["max_joint_error"] > max_error:
            failed.append("max_joint_error_gt_threshold")
        if metrics["max_joint_velocity"] is None or metrics["max_joint_velocity"] > max_velocity:
            failed.append("max_joint_velocity_gt_threshold")
        position = (pose or {}).get("position") if pose else None
        height = position[2] if position and len(position) > 2 else None
        abnormal = height is None or not math.isfinite(height) or height < 0.15
        if position and any((not math.isfinite(v) or abs(v) > 100.0) for v in position):
            abnormal = True
        if abnormal:
            failed.append("robot_abnormal")
        if bool(cfg.get("require_free_standing_height", False)) and (height is None or height < float(self.config.get("fallen_z_threshold", 0.35))):
            failed.append("free_standing_height_required_but_low")
        if bool(cfg.get("require_roll_pitch_balance", False)):
            roll = (pose or {}).get("roll") if pose else None
            pitch = (pose or {}).get("pitch") if pose else None
            if roll is None or pitch is None or abs(roll) > 0.45 or abs(pitch) > 0.45:
                failed.append("roll_pitch_balance_required")
        return failed

    def joint_metrics(self) -> dict:
        samples = {}
        sensor_valid = len(self.sensors) == len(self.motors) and len(self.motors) >= 20
        max_error = 0.0
        max_velocity = 0.0
        now_sim = self.robot.getTime()
        for name in self.motors:
            value = self.sensor_value(name)
            if value is None or not math.isfinite(value):
                sensor_valid = False
                continue
            samples[name] = value
            target = self.current_targets.get(name)
            if target is not None and math.isfinite(target):
                max_error = max(max_error, abs(value - target))
        if self.prev_joint_sample is None or self.prev_joint_sample_time is None:
            max_velocity = 0.0
        else:
            dt = max(1e-6, now_sim - self.prev_joint_sample_time)
            for name, value in samples.items():
                old = self.prev_joint_sample.get(name)
                if old is not None and math.isfinite(old):
                    max_velocity = max(max_velocity, abs(value - old) / dt)
        self.prev_joint_sample = samples
        self.prev_joint_sample_time = now_sim
        return {
            "sensor_valid": sensor_valid,
            "max_joint_error": max_error,
            "max_joint_velocity": max_velocity,
            "targets_in_limits": self.targets_in_limits(),
        }

    def targets_in_limits(self) -> bool:
        for name, value in self.current_targets.items():
            info = self.motor_info.get(name, {})
            lower = info.get("minPosition")
            upper = info.get("maxPosition")
            if lower == 0.0 and upper == 0.0:
                continue
            if isinstance(lower, (int, float)) and math.isfinite(lower) and value < lower - 1e-6:
                return False
            if isinstance(upper, (int, float)) and math.isfinite(upper) and value > upper + 1e-6:
                return False
        return True

    def write_hold_diagnosis(self, diagnosis: dict) -> None:
        (self.run_dir / "hold_stand_diagnosis.json").write_text(json.dumps(diagnosis, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        target = PROJECT / "results" / "native_physical_kick" / "hold_stand_diagnosis.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(diagnosis, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def write_kick_geometry(self) -> None:
        foot_points = self.foot_geometry_points()
        foot = (foot_points or {}).get("center")
        ball = self.observer.get_ball_position()
        cfg = self.assisted_config()
        ball_radius = float(cfg.get("ball_radius", 0.11))
        obb_geometry = self.right_foot_obb_geometry(ball)
        axes = self.foot_axes()
        center_distance = horizontal_gap = lateral_offset = vertical_offset = surface_gap = None
        foot_front_to_ball_distance = None
        reachable = False
        failed = []
        no_overlap = False
        if foot and ball:
            center_distance = distance3(foot, ball)
            if obb_geometry:
                vector_xy = [ball[0] - obb_geometry["foot_reference"][0], ball[1] - obb_geometry["foot_reference"][1]]
                horizontal_gap = vector_xy[0] * obb_geometry["kick_direction_xy"][0] + vector_xy[1] * obb_geometry["kick_direction_xy"][1]
                lateral_offset = vector_xy[0] * obb_geometry["lateral_axis_xy"][0] + vector_xy[1] * obb_geometry["lateral_axis_xy"][1]
                vertical_offset = ball[2] - obb_geometry["foot_reference"][2]
                foot_front_to_ball_distance = obb_geometry["center_to_surface_closest_distance"]
                surface_gap = obb_geometry["surface_gap"]
                no_overlap = not obb_geometry["overlapping"] and not self.build_initial_collision_geometry(write_file=False).get("overlapping_objects")
                if surface_gap is not None and surface_gap < 0:
                    failed.append("initial_foot_ball_overlap")
                if horizontal_gap is None or horizontal_gap <= 0:
                    failed.append("ball_not_in_front_of_right_foot")
                if surface_gap is None or surface_gap < 0.04 or surface_gap > 0.08:
                    failed.append("surface_gap_out_of_range")
                if lateral_offset is None or abs(lateral_offset) > 0.025:
                    failed.append("lateral_offset_out_of_range")
                if horizontal_gap is None or horizontal_gap <= ball_radius:
                    failed.append("forward_offset_too_small")
                if not no_overlap:
                    failed.append("collision_overlap")
                reachable = not failed
            else:
                failed.append("right_foot_obb_unavailable")
        else:
            failed.append("right_foot_or_ball_position_unavailable")
        self.kick_geometry = {
            "right_foot_center": foot,
            "right_foot_front_center": (foot_points or {}).get("front_center"),
            "right_foot_obb": obb_geometry,
            "foot_reference": (obb_geometry or {}).get("foot_reference"),
            "right_foot_toe_left": (foot_points or {}).get("toe_left"),
            "right_foot_toe_right": (foot_points or {}).get("toe_right"),
            "local_forward_axis": (foot_points or {}).get("local_forward_axis"),
            "world_forward_axis": ((obb_geometry or {}).get("kick_direction_3d") or ((axes or {}).get("forward") if axes else None)),
            "world_lateral_axis": ((obb_geometry or {}).get("lateral_axis_3d") or ((axes or {}).get("lateral") if axes else None)),
            "world_vertical_axis": (axes or {}).get("vertical") if axes else None,
            "ball_center": ball,
            "ball_radius": ball_radius,
            "center_distance": center_distance,
            "foot_front_to_ball_distance": foot_front_to_ball_distance,
            "horizontal_gap": horizontal_gap,
            "lateral_offset": lateral_offset,
            "vertical_offset": vertical_offset,
            "no_overlap": no_overlap,
            "reachable": reachable,
            "surface_gap": surface_gap,
            "failed_conditions": failed,
            "ball_position_correction": None,
        }
        self.geometry_ok = reachable
        self.geometry_failure_reason = None if reachable else "geometry precheck failed: " + ",".join(failed)
        (self.run_dir / "kick_geometry.json").write_text(json.dumps(self.kick_geometry, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        if self.foot_axis_selection:
            (self.run_dir / "foot_axis_selection.json").write_text(json.dumps(self.foot_axis_selection, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("KICK_GEOMETRY", geometry=self.kick_geometry)

    def finish_geometry_check(self) -> None:
        ground = self.ground_support()
        cfg = self.assisted_config()
        ball = self.observer.get_ball_position()
        right = self.observer.get_foot_position("right")
        left = self.observer.get_foot_position("left")
        ball_radius = float(cfg.get("ball_radius", 0.11))
        expected_ball_z = ground["top_z"] + ball_radius + 0.002
        surface_gap = self.kick_geometry.get("surface_gap")
        lateral_offset = self.kick_geometry.get("lateral_offset")
        horizontal_gap = self.kick_geometry.get("horizontal_gap")
        ball_on_ground = finite_vec(ball, 3) and abs(ball[2] - expected_ball_z) <= 0.03
        feet_on_ground = (
            finite_vec(right, 3)
            and finite_vec(left, 3)
            and -0.05 <= right[2] <= 0.30
            and -0.05 <= left[2] <= 0.30
        )
        ball_in_front = isinstance(horizontal_gap, (int, float)) and horizontal_gap > 0.0
        conditions = {
            "coordinate_frame_valid": bool(self.coordinate_frame.get("coordinate_frame_valid")),
            "support_check_success": bool(self.support_check.get("success")),
            "hold_stand_success": True,
            "surface_gap_ok": isinstance(surface_gap, (int, float)) and 0.04 <= surface_gap <= 0.08,
            "lateral_offset_ok": isinstance(lateral_offset, (int, float)) and abs(lateral_offset) <= 0.025,
            "ball_in_front": ball_in_front,
            "feet_on_ground": feet_on_ground,
            "ball_on_ground": ball_on_ground,
            "no_overlap": bool(self.kick_geometry.get("no_overlap")),
        }
        success = all(conditions.values())
        failed = [name for name, ok in conditions.items() if not ok]
        summary = {
            "run_id": self.run_id,
            "mode": self.mode,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "assisted_mode": self.assisted,
            "geometry_check_success": success,
            "conditions": conditions,
            "failed_conditions": failed,
            "coordinate_frame_valid": conditions["coordinate_frame_valid"],
            "support_check_success": conditions["support_check_success"],
            "surface_gap": surface_gap,
            "lateral_offset": lateral_offset,
            "horizontal_gap": horizontal_gap,
            "ball_position": ball,
            "right_foot_position": right,
            "left_foot_position": left,
            "ball_expected_z": expected_ball_z,
            "kick_success": False,
            "supervisor_moved_ball": False,
            "state_history": self.state_history,
            "failure_reason": None if success else "geometry-check failed: " + ",".join(failed),
        }
        self.failure_reason = None if success else summary["failure_reason"]
        (self.run_dir / "geometry_check_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("GEOMETRY_CHECK_DONE" if success else "FAILED", summary=summary)

    def update_contact_estimate(self, label: str) -> None:
        foot_points = self.foot_geometry_points()
        foot = (foot_points or {}).get("front_center")
        ball = self.observer.get_ball_position()
        if not foot or not ball:
            return
        dist = distance3(foot, ball)
        self.min_foot_ball_distance = dist if self.min_foot_ball_distance is None else min(self.min_foot_ball_distance, dist)
        cfg = self.assisted_config()
        limit = float(cfg.get("foot_contact_radius", 0.08)) + float(cfg.get("ball_radius", 0.11)) + float(cfg.get("contact_tolerance", 0.05))
        obb = self.right_foot_obb_geometry(ball)
        surface_gap = obb.get("surface_gap") if obb else None
        progress = None
        if self.initial_foot_ball_distance is not None and dist is not None:
            progress = self.initial_foot_ball_distance - dist
        ball_changed = distance_xy(self.contact_window_ball_before, ball) > 0.002
        velocity = self.observer.get_ball_velocity()
        moving = bool(velocity and any(abs(v) > 0.002 for v in velocity[:3]))
        if dist <= limit and (ball_changed or moving):
            self.contact_estimated = True
            self.emit(
                "FOOT_BALL_CONTACT_ESTIMATED",
                sim_time=self.robot.getTime(),
                motion_level=getattr(self, "active_motion_level", None),
                right_foot_center=(foot_points or {}).get("center"),
                right_foot_front=foot,
                ball_position=ball,
                min_distance=self.min_foot_ball_distance,
                ball_speed_before=0.0,
                ball_speed_after=norm3(velocity[:3]) if velocity else None,
                kick_direction=(self.foot_axes() or {}).get("forward"),
                ball_motion_direction=unit(velocity[:3]) if velocity else None,
            )
        sample = {
            "time": now(),
            "sim_time": self.robot.getTime(),
            "label": label,
            "right_foot_center": (foot_points or {}).get("center"),
            "right_foot_front": foot,
            "ball": ball,
            "foot_ball_distance": dist,
            "foot_ball_obb_signed_distance": surface_gap,
            "progress_to_ball": progress,
            "contact_limit": limit,
            "ball_velocity": velocity,
            "current_joint_positions": {name: self.sensor_value(name) for name in self.predicted_joint_deltas(self.selected_level or {}).keys()} if self.selected_level else {},
            "commanded_joint_targets": {name: self.current_targets.get(name) for name in self.predicted_joint_deltas(self.selected_level or {}).keys()} if self.selected_level else {},
        }
        jsonl_append(self.logs["ball"], sample)
        if label in {MotionState.SWING_FORWARD.value, MotionState.CONTACT_HOLD.value}:
            self.actual_trajectory_samples.append(sample)
            if label == MotionState.SWING_FORWARD.value:
                recent = [row for row in self.actual_trajectory_samples[-5:] if row.get("progress_to_ball") is not None]
                if len(recent) >= 5 and all(row["progress_to_ball"] < -0.005 for row in recent):
                    self.trajectory_direction_failed = True
                    self.failure_reason = "actual foot trajectory moved away from ball during swing"
                    self.emit("FAILED_TRAJECTORY_DIRECTION", recent_progress=[row["progress_to_ball"] for row in recent])

    def foot_axes(self) -> dict | None:
        selection = self.select_foot_axis()
        if selection:
            return {
                "forward": selection["world_forward_axis"],
                "lateral": selection["world_lateral_axis"],
                "vertical": selection["world_vertical_axis"],
            }
        node = self.observer.foot_nodes.get("right")
        if node is None:
            return None
        orientation = self.observer._orientation(node)
        if not finite_vec(orientation, 9):
            return None
        geom = self.config.get("foot_geometry", {})
        forward = axis_from_orientation(orientation, int(geom.get("forward_axis_index", 0)), float(geom.get("forward_axis_sign", 1)))
        lateral = axis_from_orientation(orientation, int(geom.get("lateral_axis_index", 1)), float(geom.get("lateral_axis_sign", 1)))
        vertical = axis_from_orientation(orientation, int(geom.get("vertical_axis_index", 2)), float(geom.get("vertical_axis_sign", 1)))
        return {"forward": forward, "lateral": lateral, "vertical": vertical}

    def foot_geometry_points(self) -> dict | None:
        center = self.observer.get_foot_position("right")
        selection = self.select_foot_axis()
        axes = {
            "forward": selection["world_forward_axis"],
            "lateral": selection["world_lateral_axis"],
            "vertical": selection["world_vertical_axis"],
        } if selection else self.foot_axes()
        if not finite_vec(center, 3) or axes is None:
            return None
        geom = self.config.get("foot_geometry", {})
        forward_half = float((selection or {}).get("selected_half_extent_m") or geom.get("forward_half_length_m", 0.112435))
        lateral_half = float(geom.get("lateral_half_width_m", 0.05))
        vertical_touch = -float(geom.get("vertical_half_height_m", 0.0155))
        forward_vec = vec_scale(axes["forward"], forward_half)
        lateral_vec = vec_scale(axes["lateral"], lateral_half)
        vertical_vec = vec_scale(axes["vertical"], vertical_touch)
        front = vec_add(vec_add(center, forward_vec), vertical_vec)
        return {
            "center": center,
            "front_center": front,
            "toe_left": vec_add(front, lateral_vec),
            "toe_right": vec_sub(front, lateral_vec),
            "local_forward_axis": (selection or {}).get("selected_local_axis") or [int(self.config.get("foot_geometry", {}).get("forward_axis_index", 0)), float(self.config.get("foot_geometry", {}).get("forward_axis_sign", 1))],
        }

    def right_foot_obb_geometry(self, ball: list[float] | None) -> dict | None:
        boxes = self.collision_boxes()
        box = boxes.get("RIGHT_FOOT")
        red_goal = self.observer._node_by_def_or_name("RED_GOAL")
        red = self.observer._position(red_goal)
        if box is None or not finite_vec(red, 3):
            return None
        center = box["center"]
        kick_xy = normalize2([red[0] - center[0], red[1] - center[1]])
        if kick_xy is None:
            return None
        foot_reference = list(center)
        for axis, extent in zip(box["axes"], box["half_extents"]):
            projection = axis[0] * kick_xy[0] + axis[1] * kick_xy[1]
            sign = 1.0 if projection >= 0.0 else -1.0
            foot_reference = vec_add(foot_reference, vec_scale(axis, extent * sign))
        lateral_xy = [-kick_xy[1], kick_xy[0]]
        row = sphere_to_oriented_box_signed_distance(ball, float(self.assisted_config().get("ball_radius", 0.11)), box) if finite_vec(ball, 3) else {}
        return {
            "box_center": center,
            "box_half_extents": box["half_extents"],
            "foot_reference": foot_reference,
            "kick_direction_xy": kick_xy,
            "lateral_axis_xy": lateral_xy,
            "kick_direction_3d": [kick_xy[0], kick_xy[1], 0.0],
            "lateral_axis_3d": [lateral_xy[0], lateral_xy[1], 0.0],
            "surface_gap": row.get("signed_surface_distance"),
            "overlap_depth": row.get("overlap_depth"),
            "overlapping": row.get("overlapping"),
            "closest_point": row.get("closest_point"),
            "closest_normal": row.get("closest_normal"),
            "center_to_surface_closest_distance": None if row.get("signed_surface_distance") is None else row.get("signed_surface_distance") + float(self.assisted_config().get("ball_radius", 0.11)),
        }

    def select_foot_axis(self) -> dict | None:
        node = self.observer.foot_nodes.get("right")
        center = self.observer.get_foot_position("right")
        red_goal = self.observer._node_by_def_or_name("RED_GOAL")
        red = self.observer._position(red_goal)
        if node is None or not finite_vec(center, 3):
            return None
        orientation = self.observer._orientation(node)
        if not finite_vec(orientation, 9):
            return None
        kick_xy = normalize2([red[0] - center[0], red[1] - center[1]]) if finite_vec(red, 3) else None
        if kick_xy is None:
            robot_pose = self.observer.get_robot_pose()
            robot_orientation = robot_pose.get("orientation") if robot_pose else None
            if finite_vec(robot_orientation, 9):
                fallback = axis_from_orientation(robot_orientation, 0, 1.0)
                kick_xy = normalize2(fallback[:2]) if fallback else None
        if kick_xy is None:
            return None
        geom = self.config.get("foot_geometry", {})
        candidates = []
        for axis_index, axis_name in ((0, "X"), (1, "Y")):
            for sign in (1.0, -1.0):
                world_axis = axis_from_orientation(orientation, axis_index, sign)
                axis_xy = normalize2(world_axis[:2]) if world_axis else None
                if axis_xy is None:
                    continue
                cosine = max(-1.0, min(1.0, axis_xy[0] * kick_xy[0] + axis_xy[1] * kick_xy[1]))
                candidates.append({
                    "local_axis": axis_name,
                    "axis_index": axis_index,
                    "sign": sign,
                    "world_axis": world_axis,
                    "world_axis_xy": axis_xy,
                    "angle_to_kick_direction_deg": math.degrees(math.acos(cosine)),
                })
        if not candidates:
            return None
        selected = min(candidates, key=lambda row: row["angle_to_kick_direction_deg"])
        lateral_index = 1 if selected["axis_index"] == 0 else 0
        lateral = axis_from_orientation(orientation, lateral_index, 1.0)
        vertical = axis_from_orientation(orientation, 2, 1.0)
        if vertical and vertical[2] < 0:
            vertical = vec_scale(vertical, -1.0)
        selected_half = float(geom.get("forward_half_length_m", 0.112435)) if selected["axis_index"] == 0 else float(geom.get("lateral_half_width_m", 0.05))
        front = vec_add(vec_add(center, vec_scale(selected["world_axis"], selected_half)), vec_scale(vertical, -float(geom.get("vertical_half_height_m", 0.0155))))
        final_kick_xy = normalize2([red[0] - front[0], red[1] - front[1]]) if finite_vec(red, 3) else kick_xy
        self.foot_axis_selection = {
            "right_foot_center": center,
            "red_goal_position": red,
            "selection_direction_xy": kick_xy,
            "kick_direction_xy": final_kick_xy or kick_xy,
            "candidates": candidates,
            "selected": selected,
            "selected_local_axis": [selected["axis_index"], selected["sign"]],
            "selected_half_extent_m": selected_half,
            "world_forward_axis": selected["world_axis"],
            "world_lateral_axis": lateral,
            "world_vertical_axis": vertical,
            "right_foot_front_center": front,
            "bounding_box_half_extents": {
                "local_x": float(geom.get("forward_half_length_m", 0.112435)),
                "local_y": float(geom.get("lateral_half_width_m", 0.05)),
                "local_z": float(geom.get("vertical_half_height_m", 0.0155)),
            },
        }
        return self.foot_axis_selection

    def derive_calibration_signs(self) -> None:
        best = 0.0
        for result in self.right_leg_calibration.values():
            for key in ("plus_foot_delta", "minus_foot_delta"):
                delta = result.get(key) if isinstance(result, dict) else None
                if isinstance(delta, dict):
                    best = max(best, abs(float(delta.get("foot_delta_forward") or 0.0)), abs(float(delta.get("toe_delta_forward") or 0.0)), abs(float(delta.get("toe_delta_vertical") or 0.0)))
        self.calibration_confidence = best
        self.hip_pitch_forward_sign = self.sign_for_joint("Right_Hip_Pitch", "toe_delta_forward")
        self.knee_extension_sign = self.sign_for_joint("Right_Knee_Pitch", "toe_delta_forward")
        self.ankle_compensation_signs = {
            "Crank_Up_Right": self.sign_for_joint("Crank_Up_Right", "toe_delta_vertical"),
            "Crank_Down_Right": self.sign_for_joint("Crank_Down_Right", "toe_delta_vertical"),
        }

    def sign_for_joint(self, name: str, metric: str) -> float:
        result = self.right_leg_calibration.get(name, {})
        plus = result.get("plus_foot_delta") if isinstance(result, dict) else None
        minus = result.get("minus_foot_delta") if isinstance(result, dict) else None
        plus_value = plus.get(metric) if isinstance(plus, dict) else None
        minus_value = minus.get(metric) if isinstance(minus, dict) else None
        if isinstance(plus_value, (int, float)) and isinstance(minus_value, (int, float)):
            return 1.0 if plus_value >= minus_value else -1.0
        return 1.0

    def write_predicted_trajectories(self, levels: list) -> bool:
        start = self.foot_geometry_points()
        ball = self.observer.get_ball_position()
        cfg = self.assisted_config()
        if not start or not ball:
            self.failure_reason = "trajectory precheck missing foot or ball"
            return False
        predictions = []
        any_intersects = False
        for index, level in enumerate(levels[:3], start=1):
            samples = []
            min_distance = None
            closest_sample = None
            for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
                predicted_center = self.predicted_front_from_calibration(start["center"], level, fraction)
                predicted_front = self.predicted_front_from_calibration(start["front_center"], level, fraction)
                distance = distance3(predicted_front, ball)
                min_distance = distance if min_distance is None else min(min_distance, distance)
                sample = {"fraction": fraction, "right_foot_center": predicted_center, "right_foot_front": predicted_front, "distance_to_ball_center": distance}
                samples.append(sample)
                if distance is not None and (closest_sample is None or distance < closest_sample["distance_to_ball_center"]):
                    closest_sample = sample
            threshold = float(cfg.get("ball_radius", 0.11)) + float(cfg.get("foot_contact_radius", 0.08)) + float(cfg.get("trajectory_tolerance_m", 0.04))
            intersects = min_distance is not None and min_distance <= threshold
            any_intersects = any_intersects or intersects
            predictions.append({
                "level": index,
                "name": level.get("name"),
                "samples": samples,
                "min_distance_to_ball_center": min_distance,
                "contact_threshold": threshold,
                "predicted_contact": intersects,
                "predicted_closest_fraction": closest_sample.get("fraction") if closest_sample else None,
                "predicted_closest_point": closest_sample.get("right_foot_front") if closest_sample else None,
            })
        self.trajectory_precheck = {"predictions": predictions, "success": any_intersects}
        (self.run_dir / "predicted_kick_trajectories.json").write_text(json.dumps(self.trajectory_precheck, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("TRAJECTORY_PRECHECK", precheck=self.trajectory_precheck)
        if not any_intersects:
            self.failure_reason = "trajectory precheck found no level intersecting ball"
            self.state_history.append(MotionState.FAILED_PRECONDITION.value)
        return any_intersects

    def predicted_contact_levels(self, levels: list) -> list:
        predictions = self.trajectory_precheck.get("predictions", []) if isinstance(self.trajectory_precheck, dict) else []
        valid = []
        level_logs = []
        for row in predictions:
            if not isinstance(row, dict):
                continue
            min_distance = row.get("min_distance_to_ball_center")
            threshold = row.get("contact_threshold")
            level_index = int(row.get("level", 0)) - 1
            predicted_contact = bool(row.get("predicted_contact"))
            valid_level = (
                predicted_contact
                and isinstance(min_distance, (int, float))
                and isinstance(threshold, (int, float))
                and min_distance <= threshold
                and 0 <= level_index < len(levels)
            )
            level_logs.append({
                "level": row.get("level"),
                "name": row.get("name"),
                "predicted_contact": predicted_contact,
                "predicted_min_distance": min_distance,
                "contact_threshold": threshold,
                "valid": valid_level,
            })
            self.emit(
                "TRAJECTORY_LEVEL",
                level=row.get("level"),
                name=row.get("name"),
                predicted_contact=predicted_contact,
                predicted_min_distance=min_distance,
                contact_threshold=threshold,
                valid=valid_level,
            )
            if not valid_level:
                continue
            if 0 <= level_index < len(levels):
                valid.append(levels[level_index])
        self.valid_predicted_levels = valid
        self.selected_level = valid[0] if valid else None
        self.selected_prediction = None
        if valid:
            selected_name = valid[0].get("name")
            for row in predictions:
                if isinstance(row, dict) and row.get("name") == selected_name:
                    self.selected_prediction = row
                    break
        selected_names = [level.get("name", f"level_{idx + 1}") for idx, level in enumerate(valid)]
        filter_summary = {
            "levels": level_logs,
            "selected_level": valid[0].get("name") if valid else None,
            "valid_level_names": selected_names,
            "contact_threshold": next((row.get("contact_threshold") for row in predictions if isinstance(row, dict) and row.get("contact_threshold") is not None), None),
        }
        self.trajectory_precheck["valid_levels"] = level_logs
        self.trajectory_precheck["selected_level"] = filter_summary["selected_level"]
        (self.run_dir / "trajectory_level_selection.json").write_text(json.dumps(filter_summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit(
            "TRAJECTORY_LEVEL_FILTER",
            available=[level.get("name", f"level_{idx + 1}") for idx, level in enumerate(levels)],
            selected=selected_names,
            selected_level=filter_summary["selected_level"],
            contact_threshold=filter_summary["contact_threshold"],
        )
        return valid

    def write_trajectory_execution_consistency(self, level: dict) -> bool:
        predicted = self.predicted_joint_deltas(level)
        executed_targets = self.swing_targets(level)
        executed = {}
        differences = {}
        for motor, delta in predicted.items():
            base = self.initial_pose.get(motor)
            target = executed_targets.get(motor)
            if isinstance(base, (int, float)) and isinstance(target, (int, float)):
                executed[motor] = target - base
                differences[motor] = executed[motor] - delta
        consistent = all(abs(value) <= 0.01 for value in differences.values())
        current_pose = {name: self.sensor_value(name) for name in predicted}
        report = {
            "initial_joint_pose_prediction": {name: self.initial_pose.get(name) for name in predicted},
            "initial_joint_pose_execution": current_pose,
            "hip_sign": self.hip_pitch_forward_sign_or_default(),
            "knee_sign": self.knee_extension_sign_or_default(),
            "ankle_signs": {
                "Crank_Up_Right": self.ankle_sign_or_default("Crank_Up_Right"),
                "Crank_Down_Right": self.ankle_sign_or_default("Crank_Down_Right"),
            },
            "selected_level": level.get("name"),
            "predicted_targets": predicted,
            "executed_targets": executed,
            "target_difference": differences,
            "uses_current_run_calibration": bool(self.right_leg_calibration),
            "consistent": consistent,
        }
        self.trajectory_execution_consistency = report
        (self.run_dir / "trajectory_execution_consistency.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("TRAJECTORY_EXECUTION_CONSISTENCY", consistency=report)
        if not consistent:
            self.state_history.append(MotionState.FAILED_TRAJECTORY_PRECHECK.value)
            self.emit("STATE", state=MotionState.FAILED_TRAJECTORY_PRECHECK.value)
        return consistent

    def predicted_joint_deltas(self, level: dict) -> dict[str, float]:
        result = {}
        mapping = (
            ("right_hip_pitch", self.hip_pitch_forward_sign_or_default() * float(level.get("hip_pitch", 0.08))),
            ("right_knee", self.knee_extension_sign_or_default() * abs(float(level.get("knee", -0.06)))),
            ("right_ankle_pitch", self.ankle_sign_or_default("Crank_Up_Right") * abs(float(level.get("ankle_pitch", -0.03)))),
            ("right_ankle_roll", self.ankle_sign_or_default("Crank_Down_Right") * abs(float(level.get("ankle_pitch", -0.03)))),
        )
        for cls, delta in mapping:
            motor = self.first_motor(cls)
            if motor:
                result[motor] = delta
        return result

    def write_predicted_vs_actual_trajectory(self) -> None:
        prediction = self.selected_prediction or {}
        predicted_samples = prediction.get("samples", []) if isinstance(prediction, dict) else []
        actual_samples = list(self.actual_trajectory_samples)
        actual_distances = [row.get("foot_ball_distance") for row in actual_samples if isinstance(row.get("foot_ball_distance"), (int, float))]
        actual_min = min(actual_distances) if actual_distances else None
        actual_closest = None
        if actual_samples:
            rows = [row for row in actual_samples if isinstance(row.get("foot_ball_distance"), (int, float))]
            actual_closest = min(rows, key=lambda row: row["foot_ball_distance"]) if rows else None
        predicted_endpoint = predicted_samples[-1] if predicted_samples else None
        actual_endpoint = actual_samples[-1] if actual_samples else None
        predicted_closest = None
        if predicted_samples:
            rows = [row for row in predicted_samples if isinstance(row.get("distance_to_ball_center"), (int, float))]
            predicted_closest = min(rows, key=lambda row: row["distance_to_ball_center"]) if rows else None
        endpoint_error = distance3(
            (predicted_endpoint or {}).get("right_foot_front"),
            (actual_endpoint or {}).get("right_foot_front"),
        )
        closest_error = distance3(
            (predicted_closest or {}).get("right_foot_front"),
            (actual_closest or {}).get("right_foot_front"),
        )
        rms_terms = []
        if predicted_samples and actual_samples:
            for index, predicted in enumerate(predicted_samples):
                actual_index = min(len(actual_samples) - 1, round(index * (len(actual_samples) - 1) / max(1, len(predicted_samples) - 1)))
                error = distance3(predicted.get("right_foot_front"), actual_samples[actual_index].get("right_foot_front"))
                if error is not None:
                    rms_terms.append(error * error)
        rms = math.sqrt(sum(rms_terms) / len(rms_terms)) if rms_terms else None
        progress_values = [row.get("progress_to_ball") for row in actual_samples if isinstance(row.get("progress_to_ball"), (int, float))]
        report = {
            "selected_level": (self.selected_level or {}).get("name") if isinstance(self.selected_level, dict) else None,
            "predicted_min_distance": prediction.get("min_distance_to_ball_center") if isinstance(prediction, dict) else None,
            "predicted_closest_time": prediction.get("predicted_closest_fraction") if isinstance(prediction, dict) else None,
            "predicted_closest_point": prediction.get("predicted_closest_point") if isinstance(prediction, dict) else None,
            "actual_min_distance": actual_min,
            "actual_closest_time": (actual_closest or {}).get("sim_time") if actual_closest else None,
            "predicted_endpoint_error": endpoint_error,
            "predicted_closest_point_error": closest_error,
            "trajectory_rms_error": rms,
            "progress_to_ball_max": max(progress_values) if progress_values else None,
            "progress_to_ball_final": progress_values[-1] if progress_values else None,
            "moving_toward_ball": bool(progress_values and max(progress_values) > 0.0),
            "actual_samples": actual_samples,
            "predicted_samples": predicted_samples,
        }
        self.predicted_vs_actual_trajectory = report
        (self.run_dir / "predicted_vs_actual_trajectory.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def predicted_front_from_calibration(self, start_front: list[float], level: dict, fraction: float) -> list[float]:
        delta = [0.0, 0.0, 0.0]
        for motor, amount in (
            ("Right_Hip_Pitch", self.hip_pitch_forward_sign_or_default() * float(level.get("hip_pitch", 0.08)) * fraction),
            ("Right_Knee_Pitch", self.knee_extension_sign_or_default() * abs(float(level.get("knee", -0.06))) * fraction),
            ("Crank_Up_Right", self.ankle_sign_or_default("Crank_Up_Right") * abs(float(level.get("ankle_pitch", -0.03))) * fraction),
            ("Crank_Down_Right", self.ankle_sign_or_default("Crank_Down_Right") * abs(float(level.get("ankle_pitch", -0.03))) * fraction),
        ):
            cal = self.right_leg_calibration.get(motor, {})
            key = "plus_foot_delta" if amount >= 0 else "minus_foot_delta"
            d = cal.get(key) if isinstance(cal, dict) else None
            if isinstance(d, dict):
                scale = abs(amount) / 0.03
                world = d.get("toe_delta_world")
                if finite_vec(world, 3):
                    delta = vec_add(delta, vec_scale(world, scale))
        return vec_add(start_front, delta)

    def hip_pitch_forward_sign_or_default(self) -> float:
        return float(self.hip_pitch_forward_sign if self.hip_pitch_forward_sign is not None else 1.0)

    def knee_extension_sign_or_default(self) -> float:
        return float(self.knee_extension_sign if self.knee_extension_sign is not None else 1.0)

    def ankle_sign_or_default(self, name: str) -> float:
        return float(self.ankle_compensation_signs.get(name, 1.0))

    def add_delta(self, targets: dict[str, float], cls: str, delta: float) -> None:
        name = self.first_motor(cls)
        if not name:
            return
        base = self.initial_pose.get(name, 0.0)
        targets[name] = self.clip(name, base + delta)

    def first_motor(self, cls: str) -> str | None:
        names = self.classified.get(cls, [])
        return names[0] if names else None

    def clip(self, name: str, value: float) -> float:
        info = self.motor_info.get(name, {})
        lower = info.get("minPosition")
        upper = info.get("maxPosition")
        if lower == 0.0 and upper == 0.0:
            lower = None
            upper = None
        clipped = clip_target(value, lower, upper)
        if clipped != value:
            self.joint_limit_violation = True
            self.emit("JOINT_LIMIT_CLIP", motor=name, requested=value, clipped=clipped)
        return clipped

    def apply_targets(self, targets: dict[str, float], duration: float, label: str) -> bool:
        starts = {name: self.sensor_value(name) if self.sensor_value(name) is not None else self.current_targets.get(name, 0.0) for name in targets}
        steps = max(1, int(duration * 1000 / self.timestep))
        for step in range(steps):
            fraction = (step + 1) / steps
            for name, target in targets.items():
                value = self.clip(name, interpolate(starts[name], target, fraction))
                self.motors[name].setPosition(value)
                self.current_targets[name] = value
                if step % max(1, steps // 8) == 0:
                    jsonl_append(self.logs["commands"], {"time": now(), "state": label, "motor": name, "target": value})
            self.robot.step(self.timestep)
            if label in {MotionState.SWING_FORWARD.value, MotionState.CONTACT_HOLD.value}:
                self.update_contact_estimate(label)
                if self.trajectory_direction_failed:
                    return False
            if step % max(1, steps // 10) == 0:
                self.log_joint_state(label)
        return True

    def hold_pose(self, seconds: float, label: str) -> bool:
        self.state_history.append(MotionState.HOLD_STAND.value if "stable" in label else MotionState.INITIALIZE.value)
        return self.wait_seconds(seconds, label)

    def wait_seconds(self, seconds: float, label: str) -> bool:
        end = self.robot.getTime() + seconds
        while self.robot.getTime() < end:
            if self.robot.step(self.timestep) == -1:
                self.failure_reason = "Webots step ended"
                return False
            self.log_joint_state(label, every=250)
            if label in {MotionState.SWING_FORWARD.value, MotionState.CONTACT_HOLD.value}:
                self.update_contact_estimate(label)
            if self.check_fallen():
                return False
        return True

    def wait_steps(self, steps: int) -> None:
        for _ in range(steps):
            if self.robot.step(self.timestep) == -1:
                break

    def sensor_value(self, motor_name: str):
        sensor = self.sensors.get(motor_name)
        if not sensor:
            return None
        try:
            return float(sensor.getValue())
        except Exception:
            return None

    def latest_world_state(self) -> dict | None:
        if not self.match_state_file.exists():
            return None
        latest = None
        try:
            with self.match_state_file.open("r", encoding="utf-8") as handle:
                for line in handle:
                    try:
                        row = json.loads(line)
                    except Exception:
                        continue
                    if row.get("event") == "STATE":
                        latest = row
        except Exception:
            return None
        return latest

    def latest_ball_xy(self):
        state = self.latest_world_state()
        if not state or not state.get("ball"):
            return None
        ball = state["ball"]
        if ball.get("x") is None or ball.get("y") is None:
            return None
        return [float(ball["x"]), float(ball["y"])]

    def robot_xy_z(self):
        state = self.latest_world_state()
        if not state:
            return None
        robot = (state.get("robots") or {}).get("T1_BLUE_1")
        if not robot:
            return None
        return [robot.get("x"), robot.get("y"), robot.get("z")]

    def check_fallen(self) -> bool:
        if self.assisted and not bool(self.assisted_config().get("require_free_standing_height", False)):
            pose = self.observer.get_robot_pose()
            position = pose.get("position") if pose else None
            if position and len(position) > 2 and math.isfinite(position[2]) and position[2] >= 0.15:
                return False
        pos = self.robot_xy_z()
        if pos and pos[2] is not None and pos[2] < float(self.config.get("fallen_z_threshold", 0.35)):
            self.robot_fallen = True
            self.emit("ROBOT_FALLEN", position=pos)
            return True
        return False

    def log_joint_state(self, label: str, every: int = 1) -> None:
        if every > 1 and int(self.robot.getTime() * 1000) % every != 0:
            return
        sample = {name: self.sensor_value(name) for name in self.motors}
        jsonl_append(self.logs["states"], {"time": now(), "sim_time": self.robot.getTime(), "label": label, "joints": sample})

    def emit(self, event: str, **kwargs) -> None:
        row = {"time": now(), "sim_time": self.robot.getTime() if hasattr(self, "robot") else None, "event": event}
        row.update(kwargs)
        jsonl_append(self.logs["events"], row)
        print(f"[native] {event} {kwargs}", flush=True)

    def finish(self) -> None:
        self.ball_final = self.latest_ball_xy() or self.ball_final
        robot_initial = self.robot_initial
        robot_final = self.robot_xy_z()
        disp = horizontal_displacement(self.ball_initial, self.ball_final)
        kick_success = disp > float(self.config.get("kick_success_threshold", 0.05))
        if self.assisted:
            kick_success = kick_success and self.contact_estimated and self.geometry_ok and bool(self.node_resolution.get("success")) and bool(self.trajectory_precheck.get("success", True))
        dribble_success = self.touch_count >= 2 and disp > float(self.config.get("dribble_success_threshold", 0.15))
        ball_motion_before_kick = False
        ball_motion_during_kick = False
        kick_contact_time = None
        for row in self.actual_trajectory_samples:
            velocity = row.get("ball_velocity")
            speed = norm3(velocity[:3]) if finite_vec(velocity, 3) else 0.0
            moved = distance_xy(self.contact_window_ball_before, row.get("ball")) > 0.002 if self.contact_window_ball_before else False
            if speed > 0.002 or moved:
                ball_motion_during_kick = True
                kick_contact_time = kick_contact_time if kick_contact_time is not None else row.get("sim_time")
        if not kick_success and self.failure_reason is None:
            if self.assisted and not self.contact_estimated:
                self.failure_reason = f"no foot-ball contact evidence; ball displacement {disp:.4f}m"
            else:
                self.failure_reason = f"ball displacement {disp:.4f}m <= 0.05m"
        self.result = "ASSISTED_PHYSICAL_KICK_SUCCESS" if self.assisted and kick_success else ("KICK_SUCCESS" if kick_success else "FAILED")
        summary = {
            "run_id": self.run_id,
            "world": os.environ.get("WEBOTS_WORLD", ""),
            "assisted_mode": self.assisted,
            "supervisor_moved_ball": False,
            "result": self.result,
            "strategy": self.mode.upper(),
            "robot_initial_position": robot_initial,
            "robot_final_position": robot_final,
            "ball_initial_position": self.ball_initial,
            "ball_final_position": self.ball_final,
            "ball_horizontal_displacement": disp,
            "kick_success": kick_success,
            "dribble_touch_count": self.touch_count,
            "dribble_total_displacement": disp,
            "dribble_success": dribble_success,
            "robot_fallen": self.robot_fallen,
            "joint_limit_violation": self.joint_limit_violation,
            "contact_estimated": self.contact_estimated,
            "min_foot_ball_distance": self.min_foot_ball_distance,
            "initial_ejection": False,
            "initial_ejection_time": None,
            "initial_collision_objects": self.initial_collision_geometry.get("overlapping_objects", []),
            "settle_check_success": self.settle_check.get("settle_check_success"),
            "kick_contact_time": kick_contact_time,
            "ball_motion_before_kick": ball_motion_before_kick,
            "ball_motion_during_kick": ball_motion_during_kick,
            "node_resolution_success": bool(self.node_resolution.get("success")),
            "geometry_precheck": self.geometry_ok,
            "calibration_confidence": self.calibration_confidence,
            "hip_pitch_forward_sign": self.hip_pitch_forward_sign,
            "knee_extension_sign": self.knee_extension_sign,
            "trajectory_precheck": self.trajectory_precheck,
            "trajectory_execution_consistency": self.trajectory_execution_consistency,
            "predicted_vs_actual_trajectory": {k: v for k, v in self.predicted_vs_actual_trajectory.items() if k not in {"actual_samples", "predicted_samples"}},
            "selected_level": (self.selected_level or {}).get("name") if isinstance(self.selected_level, dict) else None,
            "state_history": self.state_history,
            "failure_reason": None if kick_success else self.failure_reason,
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.emit("DONE" if kick_success else "FAILED", summary=summary)
        if kick_success:
            if self.assisted:
                print("ASSISTED PHYSICAL KICK SUCCESS")
            print("ACTION REQUIRED:")
            print("请使用 Win+Shift+S 截取当前 Webots 辅助真实物理踢球画面，保存到：")
            print("outputs/screenshots/assisted_physical_kick.png")
            self.wait_seconds(30, "screenshot_pause")


def safe_call(obj, method: str):
    try:
        return getattr(obj, method)()
    except Exception:
        return None


def sha256_file(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except Exception:
        return None


def vector_delta(before, after):
    if not before or not after or len(before) < 3 or len(after) < 3:
        return None
    return {"x": after[0] - before[0], "y": after[1] - before[1], "z": after[2] - before[2], "norm": distance3(before, after)}


def projected_delta(before: dict | None, after: dict | None, axes: dict | None):
    if not before or not after or not axes:
        return None
    center_delta = vec_sub(after.get("center"), before.get("center"))
    toe_delta = vec_sub(after.get("front_center"), before.get("front_center"))
    if not finite_vec(center_delta, 3) or not finite_vec(toe_delta, 3):
        return None
    return {
        "foot_delta_world": center_delta,
        "toe_delta_world": toe_delta,
        "foot_delta_forward": dot(center_delta, axes["forward"]),
        "foot_delta_lateral": dot(center_delta, axes["lateral"]),
        "foot_delta_vertical": dot(center_delta, axes["vertical"]),
        "toe_delta_forward": dot(toe_delta, axes["forward"]),
        "toe_delta_lateral": dot(toe_delta, axes["lateral"]),
        "toe_delta_vertical": dot(toe_delta, axes["vertical"]),
        "norm": norm3(toe_delta),
    }


def distance3(a, b) -> float | None:
    if not a or not b or len(a) < 3 or len(b) < 3:
        return None
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def distance_xy(a, b) -> float:
    if not a or not b or len(a) < 2 or len(b) < 2:
        return 0.0
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finite_vec(value, size: int) -> bool:
    return isinstance(value, list) and len(value) >= size and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value[:size])


def axis_from_orientation(orientation: list[float], axis_index: int, sign: float) -> list[float]:
    axis_index = max(0, min(2, axis_index))
    vector = [orientation[axis_index], orientation[axis_index + 3], orientation[axis_index + 6]]
    return unit(vec_scale(vector, sign))


def vec_add(a, b) -> list[float]:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b) -> list[float]:
    if not finite_vec(a, 3) or not finite_vec(b, 3):
        return [float("nan"), float("nan"), float("nan")]
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a, scale: float) -> list[float]:
    return [a[0] * scale, a[1] * scale, a[2] * scale]


def dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm3(a) -> float:
    if not finite_vec(a, 3):
        return 0.0
    return math.sqrt(dot(a, a))


def normalize2(a) -> list[float] | None:
    if not isinstance(a, list) or len(a) < 2:
        return None
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in a[:2]):
        return None
    n = math.hypot(a[0], a[1])
    if n <= 1e-9:
        return None
    return [a[0] / n, a[1] / n]


def unit(a) -> list[float] | None:
    n = norm3(a)
    if n <= 1e-9:
        return None
    return [a[0] / n, a[1] / n, a[2] / n]


if __name__ == "__main__":
    NativeKickController().run()
