from __future__ import annotations

from dataclasses import dataclass


REQUIRED_VISIBLE_JOINTS = {
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Crank_Up_Left",
    "Crank_Down_Left",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Crank_Up_Right",
    "Crank_Down_Right",
    "Left_Shoulder_Pitch",
    "Right_Shoulder_Pitch",
    "Left_Elbow_Pitch",
    "Right_Elbow_Pitch",
}


@dataclass
class DeviceInventory:
    motors: dict[str, object]
    sensors: dict[str, object]

    @property
    def motor_count(self) -> int:
        return len(self.motors)

    @property
    def sensor_count(self) -> int:
        return len(self.sensors)

    @property
    def missing_visible_joints(self) -> set[str]:
        return REQUIRED_VISIBLE_JOINTS.difference(self.motors)


def detect_devices(robot, timestep: int) -> DeviceInventory:
    motors: dict[str, object] = {}
    sensors: dict[str, object] = {}
    for index in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(index)
        name = device.getName()
        node_type = device.getNodeType()
        type_name = device.getNodeTypeName() if hasattr(device, "getNodeTypeName") else ""
        if "Motor" in type_name or node_type == 10:
            motors[name] = device
        elif "PositionSensor" in type_name or node_type == 31:
            sensors[name] = device
            try:
                device.enable(timestep)
            except Exception:
                pass
    return DeviceInventory(motors=motors, sensors=sensors)


def clip_joint_target(value: float, lo: float = -0.65, hi: float = 0.65) -> float:
    return max(lo, min(hi, float(value)))
