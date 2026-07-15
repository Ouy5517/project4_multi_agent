from __future__ import annotations


REQUIRED_LEG_JOINT_CLASSES = {
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
}


def normalize_joint_name(name: str) -> str:
    return name.lower().replace(" ", "_").replace("-", "_")


def classify_motor_name(name: str) -> str:
    n = normalize_joint_name(name)
    side = "left" if "left" in n or n.startswith("l_") else "right" if "right" in n or n.startswith("r_") else ""
    if "hip" in n and "yaw" in n:
        part = "hip_yaw"
    elif "hip" in n and "roll" in n:
        part = "hip_roll"
    elif "hip" in n and "pitch" in n:
        part = "hip_pitch"
    elif "knee" in n:
        part = "knee"
    elif ("ankle" in n and "pitch" in n) or "crank_up" in n:
        part = "ankle_pitch"
    elif ("ankle" in n and "roll" in n) or "crank_down" in n:
        part = "ankle_roll"
    elif "waist" in n or "torso" in n:
        return "torso"
    elif "shoulder" in n or "elbow" in n or "arm" in n:
        return f"{side + '_' if side else ''}arm"
    elif "head" in n:
        return "head"
    else:
        return "other"
    return f"{side}_{part}" if side else part


def missing_required_leg_joints(classes: dict[str, list[str]]) -> set[str]:
    return {name for name in REQUIRED_LEG_JOINT_CLASSES if not classes.get(name)}
