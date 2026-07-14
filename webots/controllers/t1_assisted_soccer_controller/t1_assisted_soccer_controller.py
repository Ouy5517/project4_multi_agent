from __future__ import annotations

import json
import os
import socket
import sys
import time
from pathlib import Path

try:
    from controller import Robot
except Exception:  # pragma: no cover - imported by unit tests without Webots
    Robot = None

from device_mapper import clip_joint_target, detect_devices
from foot_push_controller import PushState
from gait_generator import GaitGenerator


ROBOT_PORTS = {
    "BLUE_1": 18101,
    "BLUE_2": 18102,
    "RED_1": 18103,
    "RED_2": 18104,
}
SUPERVISOR_PORT = 18120


class AssistedSoccerController:
    def __init__(self) -> None:
        if Robot is None:
            raise RuntimeError("Webots controller API is unavailable")
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.identity = self._identity()
        self.port = ROBOT_PORTS[self.identity]
        self.run_dir = Path(os.environ.get("FOUR_ROBOT_DEMO_RUN_DIR", "."))
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.inventory = detect_devices(self.robot, self.timestep)
        self.gait = GaitGenerator()
        self.push = PushState()
        self.moving = False
        self.turn_rate = 0.0
        self.phase = "HOLD"
        self.last_seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("127.0.0.1", self.port))
        self.supervisor_addr = ("127.0.0.1", SUPERVISOR_PORT)
        self.joint_log = (self.run_dir / "joint_commands.jsonl").open("a", encoding="utf-8")
        self._send_status("READY", 0)

    def _identity(self) -> str:
        args = sys.argv[1:]
        if args:
            candidate = args[0].strip().upper()
            if candidate in ROBOT_PORTS:
                return candidate
        try:
            custom_data = self.robot.getCustomData().strip().upper()
        except Exception:
            custom_data = ""
        if custom_data in ROBOT_PORTS:
            return custom_data
        name = self.robot.getName().upper()
        for key in ROBOT_PORTS:
            if key in name:
                return key
        raise RuntimeError(f"Cannot identify robot controller instance from args={args!r}, customData={custom_data!r}, name={name!r}")

    def _send_status(self, state: str, seq: int) -> None:
        payload = {
            "robot": self.identity,
            "seq": seq,
            "command_ack": seq,
            "current_state": state,
            "gait_phase": self.phase,
            "motor_count": self.inventory.motor_count,
            "sensor_count": self.inventory.sensor_count,
            "missing_visible_joints": sorted(self.inventory.missing_visible_joints),
            "fallen": False,
            "joint_limit_violation": False,
            "current_joint_positions": self._sensor_values(),
        }
        self.sock.sendto(json.dumps(payload).encode("utf-8"), self.supervisor_addr)

    def _sensor_values(self) -> dict[str, float]:
        values: dict[str, float] = {}
        for name, sensor in self.inventory.sensors.items():
            if name.endswith("_sensor"):
                joint = name[: -len("_sensor")]
            else:
                joint = name
            try:
                values[joint] = float(sensor.getValue())
            except Exception:
                pass
        return values

    def _poll_command(self) -> None:
        while True:
            try:
                data, _addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            command = json.loads(data.decode("utf-8"))
            if command.get("robot") not in {self.identity, "ALL"}:
                continue
            self.last_seq = int(command.get("seq", self.last_seq))
            kind = str(command.get("command", "HOLD")).upper()
            self.phase = str(command.get("phase", kind))
            if kind in {"GAIT", "GAIT_START", "TURN"}:
                self.moving = True
                self.turn_rate = float(command.get("turn_rate", 0.0))
            elif kind in {"GAIT_STOP", "HOLD", "RECOVER"}:
                self.moving = False
                self.turn_rate = 0.0
            elif kind in {"FOOT_PUSH_RIGHT", "PREPARE_PUSH"}:
                self.push.start("RIGHT", self.robot.getTime(), float(command.get("duration", 0.9)))
            elif kind == "FOOT_PUSH_LEFT":
                self.push.start("LEFT", self.robot.getTime(), float(command.get("duration", 0.9)))
            elif kind == "EMERGENCY_STOP":
                self.moving = False
                self.push.active = False
            self._send_status(kind, self.last_seq)

    def _apply_targets(self, targets: dict[str, float]) -> None:
        written: dict[str, float] = {}
        for name, target in targets.items():
            motor = self.inventory.motors.get(name)
            if motor is None:
                continue
            value = clip_joint_target(target)
            try:
                motor.setPosition(value)
                written[name] = value
            except Exception:
                pass
        if written:
            self.joint_log.write(
                json.dumps(
                    {
                        "time": time.time(),
                        "sim_time": self.robot.getTime(),
                        "robot": self.identity,
                        "phase": self.phase,
                        "targets": written,
                    },
                    sort_keys=True,
                )
                + "\n"
            )
            self.joint_log.flush()

    def step(self) -> bool:
        if self.robot.step(self.timestep) == -1:
            return False
        self._poll_command()
        sim_time = self.robot.getTime()
        targets = self.gait.targets(sim_time, self.moving, self.turn_rate)
        if self.push.active:
            targets.update(self.gait.push_targets(self.push.foot, self.push.fraction(sim_time)))
            if self.push.update(sim_time):
                self._send_status("PUSH_DONE", self.last_seq)
        self._apply_targets(targets)
        if int(sim_time * 4) % 8 == 0:
            self._send_status("RUNNING", self.last_seq)
        return True

    def run(self) -> None:
        while self.step():
            pass
        self.joint_log.close()


def main() -> None:
    controller = AssistedSoccerController()
    controller.run()


if __name__ == "__main__":
    main()
