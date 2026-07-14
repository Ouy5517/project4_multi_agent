#!/usr/bin/python3.10
"""
Pass Execution Adapter — bridges pass strategy decisions to robot execution.

Receives PassDecision from pass_strategy.py and converts to an execution plan
with discrete phases: ROTATE → APPROACH → ALIGN → KICK → STOP → VERIFY.

Supports dry_run (no RPC) and simulation modes.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Callable, Dict, List, Optional


# ── Verified Kick API IDs (from booster::robot::b1::LocoApiId) ──
API_KICK = {
    "kShoot":       {"api_id": 2024, "body": "", "desc": "Powerful kick, no params"},
    "kVisualKick":  {"api_id": 2038, "body_template": '{"start": true, "version": 0}',
                     "desc": "Side-foot kick V1"},
    "kVisualKickV2": {"api_id": 2038, "body_template": '{"start": true, "version": 1}',
                      "desc": "Side-foot kick V2 (stronger)"},
}

# ── Robot modes ──
MODE_PREPARE = 1
MODE_WALKING = 2
MODE_SOCCER  = 4

# ── Safety limits ──
MAX_LINEAR_SPEED  = 0.2   # m/s
MAX_ANGULAR_SPEED = 0.5   # rad/s
MAX_TURN_DURATION = 2.0   # s
MAX_MOVE_DURATION = 2.0   # s


class Phase(Enum):
    """Execution phases for a pass."""
    ROTATE_TO_TARGET = "ROTATE_TO_TARGET"
    APPROACH_BALL    = "APPROACH_BALL"
    ALIGN_FOR_PASS   = "ALIGN_FOR_PASS"
    EXECUTE_KICK     = "EXECUTE_KICK"
    STOP             = "STOP"
    VERIFY           = "VERIFY"


@dataclass
class ExecutionStep:
    """One step in the execution plan."""
    phase: Phase
    command: str                 # "change_mode", "move", "kick", "stop"
    api_id: int = 0
    body: str = ""               # JSON body for RPC
    description: str = ""
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    duration: float = 0.0        # seconds to hold this command
    status: str = "pending"      # pending | running | success | failed | skipped
    result: Optional[Dict] = None
    timestamp: Optional[str] = None


@dataclass
class ExecutionPlan:
    """Complete execution plan for a pass."""
    decision: Any = None         # PassDecision
    steps: List[ExecutionStep] = field(default_factory=list)
    mode: str = "dry_run"        # dry_run | simulation
    can_kick: bool = False       # Whether kick API is available
    kick_api: str = ""           # Which kick API to use
    created_at: str = ""


class RpcClientInterface:
    """Minimal RPC client interface for dependency injection."""

    def call(self, api_id: int, body: str) -> Dict[str, Any]:
        """Call the RPC service. Returns dict with success, status_code, etc."""
        raise NotImplementedError


class DryRunRpcClient(RpcClientInterface):
    """Dry-run client — logs calls but does not execute."""

    def __init__(self):
        self.calls: List[Dict] = []

    def call(self, api_id: int, body: str) -> Dict[str, Any]:
        result = {
            "success": True,
            "status_code": 0,
            "status_name": "DRY_RUN",
            "response_body": "",
            "api_id": api_id,
            "request_body": body,
            "timestamp": datetime.now().isoformat(),
        }
        self.calls.append(result)
        return result


class PassExecutionAdapter:
    """
    Converts a PassDecision into a safe execution plan and (optionally)
    executes it via an injected RPC client.
    """

    def __init__(
        self,
        rpc_client: Optional[RpcClientInterface] = None,
        mode: str = "dry_run",
        kick_enabled: bool = False,
        kick_api: str = "kVisualKick",
    ):
        self.rpc = rpc_client or DryRunRpcClient()
        self.mode = mode
        self.kick_enabled = kick_enabled
        self.kick_api = kick_api if kick_api in API_KICK else "kVisualKick"
        self.execution_log: List[Dict] = []

    # ── Public API ──

    def build_plan(self, decision: Any, robot_position=None, ball_position=None) -> ExecutionPlan:
        """
        Build an execution plan from a PassDecision.

        Args:
            decision: PassDecision from pass_strategy.py
            robot_position: (x, y) of the passing robot
            ball_position: (x, y) of the ball

        Returns:
            ExecutionPlan with all phases
        """
        plan = ExecutionPlan(
            decision=decision,
            mode=self.mode,
            can_kick=self.kick_enabled,
            kick_api=self.kick_api,
            created_at=datetime.now().isoformat(),
        )

        if not decision.should_pass:
            plan.steps.append(self._make_step(Phase.STOP, "stop", description="No pass — hold ball"))
            return plan

        # Phase 1: ROTATE to face target
        target_angle = self._compute_target_angle(
            robot_position or (0, 0),
            decision.target_point or (0, 0),
        )
        plan.steps.append(self._make_step(
            Phase.ROTATE_TO_TARGET,
            "move",
            body='{"vx":0.0,"vy":0.0,"vyaw":%s}' % self._clamp_angular(target_angle * 0.5),
            description=f"Rotate toward receiver {decision.receiver_id}",
            vyaw=self._clamp_angular(target_angle * 0.5),
            duration=min(abs(target_angle) / MAX_ANGULAR_SPEED, MAX_TURN_DURATION),
        ))

        # Phase 2: APPROACH_BALL (if needed)
        if ball_position:
            dist_to_ball = self._compute_distance(robot_position or (0, 0), ball_position)
            if dist_to_ball > 0.1:
                plan.steps.append(self._make_step(
                    Phase.APPROACH_BALL,
                    "move",
                    body='{"vx":0.05,"vy":0.0,"vyaw":0.0}',
                    description=f"Approach ball (distance={dist_to_ball:.2f}m)",
                    vx=0.05,
                    duration=min(dist_to_ball / 0.05, MAX_MOVE_DURATION),
                ))

        # Phase 3: ALIGN_FOR_PASS
        plan.steps.append(self._make_step(
            Phase.ALIGN_FOR_PASS,
            "move",
            body='{"vx":0.0,"vy":0.0,"vyaw":%s}' % self._clamp_angular(target_angle * 0.3),
            description="Fine alignment for pass",
            vyaw=self._clamp_angular(target_angle * 0.3),
            duration=0.5,
        ))

        # Phase 4: EXECUTE_KICK
        if self.kick_enabled:
            kick_info = API_KICK[self.kick_api]
            plan.steps.append(self._make_step(
                Phase.EXECUTE_KICK,
                "kick",
                api_id=kick_info["api_id"],
                body=kick_info.get("body_template", kick_info["body"]),
                description=f"Execute kick: {kick_info['desc']}",
            ))
        else:
            plan.steps.append(self._make_step(
                Phase.EXECUTE_KICK,
                "kick",
                description="KICK NOT IMPLEMENTED — no kick API enabled",
                status="skipped",
            ))

        # Phase 5: STOP
        plan.steps.append(self._make_step(
            Phase.STOP,
            "stop",
            body='{"vx":0.0,"vy":0.0,"vyaw":0.0}',
            description="Stop after pass",
        ))

        # Phase 6: VERIFY
        plan.steps.append(self._make_step(
            Phase.VERIFY,
            "get_mode",
            api_id=2017,
            description="Verify robot mode after pass",
        ))

        return plan

    def execute_plan(self, plan: ExecutionPlan) -> List[Dict]:
        """Execute an entire plan, handling errors and ensuring Stop."""
        all_results = []

        # Enter Walking mode
        r = self._rpc_change_mode(MODE_WALKING)
        all_results.append(r)

        for step in plan.steps:
            if step.status == "skipped":
                step.result = {"skipped": True, "reason": step.description}
                all_results.append(step.result)
                continue

            step.timestamp = datetime.now().isoformat()
            try:
                if step.command == "move":
                    r = self._rpc_move(step.vx, step.vy, step.vyaw)
                elif step.command == "kick":
                    r = self._rpc_kick(step.api_id, step.body)
                elif step.command == "stop":
                    r = self._rpc_stop()
                elif step.command == "change_mode":
                    r = self._rpc_change_mode(MODE_PREPARE)
                elif step.command == "get_mode":
                    r = self._rpc_get_mode()
                else:
                    r = {"success": True, "status_code": 0, "note": f"no-op: {step.command}"}

                step.result = r
                step.status = "success" if r.get("success") else "failed"

            except Exception as e:
                step.result = {"success": False, "error": str(e)}
                step.status = "failed"
                # Emergency stop
                try:
                    self._rpc_stop()
                except Exception:
                    pass
                break

            if step.duration > 0:
                time.sleep(step.duration)

            all_results.append(step.result)

        # Final Stop
        try:
            all_results.append(self._rpc_stop())
        except Exception:
            pass

        # VERIFY
        try:
            all_results.append(self._rpc_get_mode())
        except Exception:
            pass

        self.execution_log.extend(all_results)
        return all_results

    # ── Internal RPC helpers ──

    def _rpc_change_mode(self, mode: int) -> Dict:
        body = json.dumps({"mode": mode})
        return self.rpc.call(2000, body)

    def _rpc_move(self, vx: float, vy: float, vyaw: float) -> Dict:
        body = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw})
        return self.rpc.call(2001, body)

    def _rpc_stop(self) -> Dict:
        return self.rpc.call(2001, '{"vx":0.0,"vy":0.0,"vyaw":0.0}')

    def _rpc_get_mode(self) -> Dict:
        return self.rpc.call(2017, "")

    def _rpc_kick(self, api_id: int, body: str) -> Dict:
        if not self.kick_enabled:
            return {
                "success": False,
                "status_code": -99,
                "status_name": "NOT_IMPLEMENTED",
                "response_body": "",
                "error": "Kick API not enabled. Set kick_enabled=True to execute kicks.",
            }
        return self.rpc.call(api_id, body)

    # ── Geometry helpers ──

    @staticmethod
    def _compute_target_angle(robot_pos, target_pos) -> float:
        """Compute yaw angle (radians) from robot to target. Handles tuples and Point objects."""
        tx = target_pos[0] if hasattr(target_pos, '__getitem__') and not isinstance(target_pos, str) else (target_pos.x if hasattr(target_pos, 'x') else target_pos[0])
        ty = target_pos[1] if hasattr(target_pos, '__getitem__') and not isinstance(target_pos, str) else (target_pos.y if hasattr(target_pos, 'y') else target_pos[1])
        rx = robot_pos[0] if hasattr(robot_pos, '__getitem__') and not isinstance(robot_pos, str) else (robot_pos.x if hasattr(robot_pos, 'x') else robot_pos[0])
        ry = robot_pos[1] if hasattr(robot_pos, '__getitem__') and not isinstance(robot_pos, str) else (robot_pos.y if hasattr(robot_pos, 'y') else robot_pos[1])
        dx = tx - rx
        dy = ty - ry
        return math.atan2(dy, dx)

    @staticmethod
    def _compute_distance(a, b) -> float:
        ax = a[0] if hasattr(a, '__getitem__') and not isinstance(a, str) else (a.x if hasattr(a, 'x') else a[0])
        ay = a[1] if hasattr(a, '__getitem__') and not isinstance(a, str) else (a.y if hasattr(a, 'y') else a[1])
        bx = b[0] if hasattr(b, '__getitem__') and not isinstance(b, str) else (b.x if hasattr(b, 'x') else b[0])
        by = b[1] if hasattr(b, '__getitem__') and not isinstance(b, str) else (b.y if hasattr(b, 'y') else b[1])
        return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)

    @staticmethod
    def _clamp_angular(value: float) -> float:
        return max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, value))

    @staticmethod
    def _make_step(
        phase: Phase,
        command: str,
        body: str = "",
        api_id: int = 0,
        description: str = "",
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
        duration: float = 0.0,
        status: str = "pending",
    ) -> ExecutionStep:
        return ExecutionStep(
            phase=phase,
            command=command,
            api_id=api_id,
            body=body,
            description=description,
            vx=vx,
            vy=vy,
            vyaw=vyaw,
            duration=duration,
            status=status,
        )


def create_adapter(
    rpc_client: Optional[RpcClientInterface] = None,
    mode: str = "dry_run",
    kick_enabled: bool = False,
    kick_api: str = "kVisualKick",
) -> PassExecutionAdapter:
    """Factory function — convenience."""
    return PassExecutionAdapter(
        rpc_client=rpc_client,
        mode=mode,
        kick_enabled=kick_enabled,
        kick_api=kick_api,
    )
