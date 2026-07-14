from __future__ import annotations

import math
import re
from pathlib import Path

from controllers.t1_native_ball_controller.collision_geometry import (
    axes_from_orientation,
    sphere_to_axis_aligned_box_signed_distance,
    sphere_to_oriented_box_signed_distance,
)


BALL_RADIUS = 0.11
FOOT_HALF_EXTENTS = [0.112435, 0.05, 0.0155]
FOOT_LOCAL_CENTER = [0.010384, 0.0, -0.0155]


class BallMutationGuard:
    forbidden = [
        "setVelocity",
        "resetPhysics",
        "addForce",
        "addForceWithOffset",
    ]

    def __init__(self, root: Path) -> None:
        self.root = root

    def scan(self) -> list[str]:
        findings: list[str] = []
        scan_roots = [
            self.root / "controllers" / "four_robot_match_supervisor",
            self.root / "controllers" / "t1_assisted_soccer_controller",
        ]
        for root in scan_roots:
            if not root.exists():
                continue
            for path in root.rglob("*.py"):
                findings.extend(self._scan_file(path))
        return findings

    def _scan_file(self, path: Path) -> list[str]:
        findings: list[str] = []
        text = path.read_text(encoding="utf-8", errors="replace")
        if "SOCCER_BALL" not in text and "ball_node" not in text and "self.ball" not in text:
            return findings
        for token in self.forbidden:
            if re.search(rf"\.{re.escape(token)}\s*\(", text):
                findings.append(f"{path.relative_to(self.root)}:{token}")
        if re.search(r"ball[^\n]{0,80}\.setSFRotation\s*\(", text, re.I):
            findings.append(f"{path.relative_to(self.root)}:ball setSFRotation")
        if re.search(r"ball[^\n]{0,80}\.setSFVec3f\s*\(", text, re.I):
            findings.append(f"{path.relative_to(self.root)}:ball setSFVec3f")
        return findings


class BallContactVerifier:
    def __init__(self, robot_foot_nodes: dict[str, dict[str, object]], assistance_nodes: dict[str, object], ball_node: object) -> None:
        self.robot_foot_nodes = robot_foot_nodes
        self.assistance_nodes = assistance_nodes
        self.ball_node = ball_node
        self.contacts: list[dict] = []
        self.last_ball_pos = list(ball_node.getPosition())

    def foot_box(self, foot_node) -> dict:
        center = list(foot_node.getPosition())
        axes = axes_from_orientation(list(foot_node.getOrientation()))
        for axis, value in zip(axes, FOOT_LOCAL_CENTER):
            center = [center[i] + axis[i] * value for i in range(3)]
        return {"center": center, "half_extents": FOOT_HALF_EXTENTS, "axes": axes}

    def signed_gaps(self) -> dict[str, dict[str, float]]:
        ball = self.ball_node.getPosition()
        out: dict[str, dict[str, float]] = {}
        for robot, feet in self.robot_foot_nodes.items():
            out[robot] = {}
            for side, node in feet.items():
                gap = sphere_to_oriented_box_signed_distance(ball, BALL_RADIUS, self.foot_box(node))
                out[robot][side] = float(gap["signed_surface_distance"])
        return out

    def assistance_clearance(self) -> dict[str, float]:
        ball = self.ball_node.getPosition()
        clearances = {}
        for name, node in self.assistance_nodes.items():
            pos = node.getPosition()
            box_min = [pos[0] - 0.16, pos[1] - 0.16, pos[2] - 0.04]
            box_max = [pos[0] + 0.16, pos[1] + 0.16, pos[2] + 0.04]
            clearances[name] = float(
                sphere_to_axis_aligned_box_signed_distance(ball, BALL_RADIUS, box_min, box_max)[
                    "signed_surface_distance"
                ]
            )
        return clearances

    def confirm_contact(
        self,
        robot: str,
        strategy: str,
        stage: str,
        sim_time: float,
        ball_before: list[float],
        speed_before: float,
        direction: tuple[float, float],
        observed_min_gap: float | None = None,
        observed_foot: str | None = None,
        observed_peak_speed: float | None = None,
    ) -> dict:
        ball_after = list(self.ball_node.getPosition())
        velocity = self.ball_node.getVelocity()
        speed_after = math.hypot(velocity[0], velocity[1])
        disp = math.hypot(ball_after[0] - ball_before[0], ball_after[1] - ball_before[1])
        gaps = self.signed_gaps().get(robot, {})
        foot, signed_gap = min(gaps.items(), key=lambda item: item[1])
        if observed_min_gap is not None and observed_min_gap < signed_gap:
            signed_gap = observed_min_gap
            foot = observed_foot or foot
        ball_dir = [ball_after[0] - ball_before[0], ball_after[1] - ball_before[1]]
        ball_len = math.hypot(ball_dir[0], ball_dir[1])
        dot = 0.0
        if ball_len > 1e-9:
            dot = (ball_dir[0] * direction[0] + ball_dir[1] * direction[1]) / ball_len
        assistance_clear = min(self.assistance_clearance().values())
        peak_speed = max(speed_after, observed_peak_speed if observed_peak_speed is not None else 0.0)
        motion_evidence = peak_speed > speed_before + 0.005 or disp > 0.02
        confirmed = signed_gap <= 0.002 and disp > 0.02 and dot > 0.45 and assistance_clear > 0.02 and motion_evidence
        event = {
            "event": "FOOT_BALL_CONTACT_CONFIRMED" if confirmed else "FOOT_BALL_CONTACT_REJECTED",
            "robot": robot,
            "foot": foot,
            "sim_time": sim_time,
            "signed_gap": signed_gap,
            "ball_speed_before": speed_before,
            "ball_speed_after": speed_after,
            "ball_speed_peak": peak_speed,
            "motion_evidence": motion_evidence,
            "ball_displacement": disp,
            "foot_direction": list(direction),
            "ball_direction": ball_dir,
            "direction_dot": dot,
            "assistance_min_clearance": assistance_clear,
            "strategy": strategy,
            "stage": stage,
        }
        self.contacts.append(event)
        self.last_ball_pos = ball_after
        return event
