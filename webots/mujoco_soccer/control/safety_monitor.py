from __future__ import annotations

import math

import mujoco
import numpy as np


class SafetyMonitor:
    def __init__(self) -> None:
        self.nan_detected = False
        self.joint_limit_violation = False

    def update(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            self.nan_detected = True
        for jid in range(model.njnt):
            if model.jnt_limited[jid]:
                adr = model.jnt_qposadr[jid]
                value = float(data.qpos[adr])
                low, high = model.jnt_range[jid]
                if value < low - 0.08 or value > high + 0.08:
                    self.joint_limit_violation = True

