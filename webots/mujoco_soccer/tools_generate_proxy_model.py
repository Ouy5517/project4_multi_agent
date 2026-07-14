from __future__ import annotations

from pathlib import Path


ROBOTS = {
    "T1_BLUE_1": {"prefix": "BLUE1", "pos": (-2.3, -0.45, 0.0), "rgba": "0.04 0.12 0.85 1"},
    "T1_BLUE_2": {"prefix": "BLUE2", "pos": (-2.1, 1.0, -0.1), "rgba": "0.18 0.62 1 1"},
    "T1_RED_1": {"prefix": "RED1", "pos": (2.25, -0.65, 3.14159), "rgba": "0.75 0.03 0.04 1"},
    "T1_RED_2": {"prefix": "RED2", "pos": (-1.25, 1.25, 3.14159), "rgba": "1 0.22 0.08 1"},
}

JOINTS = [
    ("AAHead_yaw", "-1.57 1.57", 8),
    ("Head_pitch", "-0.35 1.22", 8),
    ("Left_Shoulder_Pitch", "-1.2 1.2", 24),
    # 与官方 T1 一致, 才能下垂 (零位为横伸 T 字)
    ("Left_Shoulder_Roll", "-1.74 1.57", 16),
    ("Left_Elbow_Pitch", "-2.27 2.27", 16),
    ("Left_Elbow_Yaw", "-1.0 1.0", 12),
    ("Right_Shoulder_Pitch", "-1.2 1.2", 24),
    ("Right_Shoulder_Roll", "-1.57 1.74", 16),
    ("Right_Elbow_Pitch", "-2.27 2.27", 16),
    ("Right_Elbow_Yaw", "-1.0 1.0", 12),
    ("Waist", "-0.6 0.6", 20),
    ("Left_Hip_Pitch", "-0.9 0.9", 40),
    ("Left_Hip_Roll", "-0.45 0.45", 22),
    ("Left_Hip_Yaw", "-0.35 0.35", 18),
    ("Left_Knee_Pitch", "0 1.25", 48),
    ("Left_Ankle_Pitch", "-0.5 0.5", 20),
    ("Left_Ankle_Roll", "-0.35 0.35", 16),
    ("Right_Hip_Pitch", "-0.9 0.9", 40),
    ("Right_Hip_Roll", "-0.45 0.45", 22),
    ("Right_Hip_Yaw", "-0.35 0.35", 18),
    ("Right_Knee_Pitch", "0 1.25", 48),
    ("Right_Ankle_Pitch", "-0.5 0.5", 20),
    ("Right_Ankle_Roll", "-0.35 0.35", 16),
]


def robot_visual_v2_shell_xml(name: str, cfg: dict[str, object]) -> str:
    prefix = str(cfg["prefix"])
    rgba = str(cfg["rgba"])
    white = "0.92 0.95 0.96 1"
    dark = "0.025 0.03 0.04 1"
    visor = "0.01 0.025 0.05 1"
    return f"""
            <geom name="{name}_visual_torso_shell" type="ellipsoid" size="0.155 0.105 0.195" pos="0.015 0 0.015" rgba="{white}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_chest_panel" type="box" size="0.014 0.075 0.085" pos="0.155 0 0.045" rgba="{rgba}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_pelvis_shell" type="ellipsoid" size="0.125 0.09 0.06" pos="0 0 -0.245" rgba="{dark}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_left_shoulder_shell" type="sphere" size="0.055" pos="0.035 0.137 0.155" rgba="{white}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_right_shoulder_shell" type="sphere" size="0.055" pos="0.035 -0.137 0.155" rgba="{white}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_head_neck" type="capsule" fromto="0.02 0 0.222 0.02 0 0.275" size="0.035" rgba="{dark}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_head_shell" type="ellipsoid" size="0.088 0.074 0.083" pos="0.035 0 0.325" rgba="{white}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_visor" type="box" size="0.012 0.058 0.020" pos="0.115 0 0.333" rgba="{visor}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_left_ear_panel" type="sphere" size="0.025" pos="0.035 0.071 0.328" rgba="{rgba}" contype="0" conaffinity="0" group="2"/>
            <geom name="{name}_visual_right_ear_panel" type="sphere" size="0.025" pos="0.035 -0.071 0.328" rgba="{rgba}" contype="0" conaffinity="0" group="2"/>
"""


def robot_xml(name: str, cfg: dict[str, object], visual_v2: bool = False) -> str:
    prefix = str(cfg["prefix"])
    x, y, _yaw = cfg["pos"]  # type: ignore[misc]
    rgba = str(cfg["rgba"])
    dark = "0.08 0.08 0.08 1"
    foot_rgba = "0.02 0.02 0.02 1"
    ghost = "0.02 0.02 0.02 1" if visual_v2 else "0.1 1 0.1 0.22"
    base_group = ' group="3"' if visual_v2 else ""
    shell = robot_visual_v2_shell_xml(name, cfg) if visual_v2 else ""
    return f"""
    <body name="{name}_base" pos="{x:.4f} {y:.4f} 0">
      <joint name="{name}_base_x" type="slide" axis="1 0 0" limited="true" range="-4 4" damping="45"/>
      <joint name="{name}_base_y" type="slide" axis="0 1 0" limited="true" range="-3 3" damping="45"/>
      <joint name="{name}_base_yaw" type="hinge" axis="0 0 1" limited="true" range="-7 7" damping="20"/>
      <body name="{name}_torso" pos="0 0 0.98">
        <inertial pos="0 0 0" mass="18" diaginertia="0.32 0.28 0.18"/>
        <geom name="{name}_torso_geom" type="box" size="0.13 0.09 0.18" rgba="{rgba}" contype="0" conaffinity="0"{base_group}/>
{shell}
        <body name="{name}_head_mount" pos="0.03 0 0.24">
          <joint name="{prefix}_AAHead_yaw" type="hinge" axis="0 0 1" range="-1.57 1.57" damping="1.5"/>
          <inertial pos="0 0 0.02" mass="0.35" diaginertia="0.004 0.004 0.004"/>
          <body name="{name}_head" pos="0 0 0.06">
            <joint name="{prefix}_Head_pitch" type="hinge" axis="0 1 0" range="-0.35 1.22" damping="1.5"/>
            <geom name="{name}_head_geom" type="sphere" size="0.075" rgba="0.9 0.9 0.85 1" contype="0" conaffinity="0"{base_group}/>
          </body>
        </body>
        <body name="{name}_waist" pos="0 0 -0.22">
          <joint name="{prefix}_Waist" type="hinge" axis="0 0 1" range="-0.6 0.6" damping="2"/>
          <geom name="{name}_waist_geom" type="box" size="0.11 0.085 0.055" rgba="{dark}" contype="0" conaffinity="0"{base_group}/>
          <body name="{name}_left_hip_pitch" pos="0 0.072 -0.055">
            <joint name="{prefix}_Left_Hip_Pitch" type="hinge" axis="0 1 0" range="-0.9 0.9" damping="2"/>
            <inertial pos="0 0 -0.01" mass="0.45" diaginertia="0.005 0.005 0.005"/>
            <body name="{name}_left_hip_roll" pos="0 0 -0.02">
              <joint name="{prefix}_Left_Hip_Roll" type="hinge" axis="1 0 0" range="-0.45 0.45" damping="1.5"/>
              <inertial pos="0 0 -0.01" mass="0.45" diaginertia="0.005 0.005 0.005"/>
              <body name="{name}_left_hip_yaw" pos="0 0 -0.02">
                <joint name="{prefix}_Left_Hip_Yaw" type="hinge" axis="0 0 1" range="-0.35 0.35" damping="1.5"/>
                <geom name="{name}_left_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.28" size="0.04" rgba="{rgba}" contype="0" conaffinity="0"{base_group}/>
                <geom name="{name}_visual_left_thigh_shell" type="capsule" fromto="0.012 0 0.015 0.012 0 -0.245" size="0.054" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
                <body name="{name}_left_shin" pos="0 0 -0.28">
                  <joint name="{prefix}_Left_Knee_Pitch" type="hinge" axis="0 1 0" range="0 1.25" damping="2"/>
                  <geom name="{name}_left_shin_geom" type="capsule" fromto="0 0 0 0 0 -0.25" size="0.036" rgba="{dark}" contype="0" conaffinity="0"{base_group}/>
                  <geom name="{name}_visual_left_knee_shell" type="sphere" size="0.052" pos="0.008 0 0.005" rgba="{dark}" contype="0" conaffinity="0" group="2"/>
                  <geom name="{name}_visual_left_shin_shell" type="capsule" fromto="0.012 0 -0.015 0.025 0 -0.225" size="0.046" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
                  <body name="{name}_left_ankle" pos="0 0 -0.25">
                    <joint name="{prefix}_Left_Ankle_Pitch" type="hinge" axis="0 1 0" range="-0.5 0.5" damping="1.5"/>
                    <inertial pos="0 0 -0.01" mass="0.35" diaginertia="0.004 0.004 0.004"/>
                    <body name="{name}_left_foot" pos="0.055 0 -0.035">
                      <joint name="{prefix}_Left_Ankle_Roll" type="hinge" axis="1 0 0" range="-0.35 0.35" damping="1.2"/>
                      <geom name="{name}_LEFT_FOOT_SOLE" type="box" size="0.11 0.045 0.018" pos="0 0 0" rgba="{foot_rgba}" contype="0" conaffinity="0"{base_group}/>
                      <geom name="{name}_visual_left_foot_shell" type="box" size="0.122 0.052 0.028" pos="0.006 0 0.008" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
                      <geom name="{prefix}_LEFT_FOOT_BALL_PROXY" type="box" size="0.10 0.043 0.027" pos="0.018 0 0.012" rgba="{ghost}" contype="4" conaffinity="2"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
          <body name="{name}_right_hip_pitch" pos="0 -0.072 -0.055">
            <joint name="{prefix}_Right_Hip_Pitch" type="hinge" axis="0 1 0" range="-0.9 0.9" damping="2"/>
            <inertial pos="0 0 -0.01" mass="0.45" diaginertia="0.005 0.005 0.005"/>
            <body name="{name}_right_hip_roll" pos="0 0 -0.02">
              <joint name="{prefix}_Right_Hip_Roll" type="hinge" axis="1 0 0" range="-0.45 0.45" damping="1.5"/>
              <inertial pos="0 0 -0.01" mass="0.45" diaginertia="0.005 0.005 0.005"/>
              <body name="{name}_right_hip_yaw" pos="0 0 -0.02">
                <joint name="{prefix}_Right_Hip_Yaw" type="hinge" axis="0 0 1" range="-0.35 0.35" damping="1.5"/>
                <geom name="{name}_right_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.28" size="0.04" rgba="{rgba}" contype="0" conaffinity="0"{base_group}/>
                <geom name="{name}_visual_right_thigh_shell" type="capsule" fromto="0.012 0 0.015 0.012 0 -0.245" size="0.054" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
                <body name="{name}_right_shin" pos="0 0 -0.28">
                  <joint name="{prefix}_Right_Knee_Pitch" type="hinge" axis="0 1 0" range="0 1.25" damping="2"/>
                  <geom name="{name}_right_shin_geom" type="capsule" fromto="0 0 0 0 0 -0.25" size="0.036" rgba="{dark}" contype="0" conaffinity="0"{base_group}/>
                  <geom name="{name}_visual_right_knee_shell" type="sphere" size="0.052" pos="0.008 0 0.005" rgba="{dark}" contype="0" conaffinity="0" group="2"/>
                  <geom name="{name}_visual_right_shin_shell" type="capsule" fromto="0.012 0 -0.015 0.025 0 -0.225" size="0.046" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
                  <body name="{name}_right_ankle" pos="0 0 -0.25">
                    <joint name="{prefix}_Right_Ankle_Pitch" type="hinge" axis="0 1 0" range="-0.5 0.5" damping="1.5"/>
                    <inertial pos="0 0 -0.01" mass="0.35" diaginertia="0.004 0.004 0.004"/>
                    <body name="{name}_right_foot" pos="0.055 0 -0.035">
                      <joint name="{prefix}_Right_Ankle_Roll" type="hinge" axis="1 0 0" range="-0.35 0.35" damping="1.2"/>
                      <geom name="{name}_RIGHT_FOOT_SOLE" type="box" size="0.11 0.045 0.018" pos="0 0 0" rgba="{foot_rgba}" contype="0" conaffinity="0"{base_group}/>
                      <geom name="{name}_visual_right_foot_shell" type="box" size="0.122 0.052 0.028" pos="0.006 0 0.008" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
                      <geom name="{prefix}_RIGHT_FOOT_BALL_PROXY" type="box" size="0.10 0.043 0.027" pos="0.018 0 0.012" rgba="{ghost}" contype="4" conaffinity="2"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
        <body name="{name}_left_shoulder" pos="0.02 0.125 0.15">
          <joint name="{prefix}_Left_Shoulder_Pitch" type="hinge" axis="0 1 0" range="-1.2 1.2" damping="1.2"/>
          <inertial pos="0 0 -0.02" mass="0.35" diaginertia="0.004 0.004 0.004"/>
          <body name="{name}_left_upper_arm" pos="0 0.03 -0.08">
            <joint name="{prefix}_Left_Shoulder_Roll" type="hinge" axis="1 0 0" range="-1.74 1.57" damping="1.2"/>
            <geom name="{name}_left_upper_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.16" size="0.03" rgba="{rgba}" contype="0" conaffinity="0"{base_group}/>
            <geom name="{name}_visual_left_upper_arm_shell" type="capsule" fromto="0 0 0 0 0 -0.15" size="0.039" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
            <body name="{name}_left_lower_arm" pos="0 0 -0.16">
              <joint name="{prefix}_Left_Elbow_Pitch" type="hinge" axis="0 1 0" range="-2.27 2.27" damping="1"/>
              <joint name="{prefix}_Left_Elbow_Yaw" type="hinge" axis="0 0 1" range="-1 1" damping="1"/>
              <geom name="{name}_left_lower_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.14" size="0.026" rgba="{dark}" contype="0" conaffinity="0"{base_group}/>
              <geom name="{name}_visual_left_forearm_shell" type="capsule" fromto="0 0 0 0 0 -0.13" size="0.034" rgba="{dark}" contype="0" conaffinity="0" group="2"/>
            </body>
          </body>
        </body>
        <body name="{name}_right_shoulder" pos="0.02 -0.125 0.15">
          <joint name="{prefix}_Right_Shoulder_Pitch" type="hinge" axis="0 1 0" range="-1.2 1.2" damping="1.2"/>
          <inertial pos="0 0 -0.02" mass="0.35" diaginertia="0.004 0.004 0.004"/>
          <body name="{name}_right_upper_arm" pos="0 -0.03 -0.08">
            <joint name="{prefix}_Right_Shoulder_Roll" type="hinge" axis="1 0 0" range="-1.57 1.74" damping="1.2"/>
            <geom name="{name}_right_upper_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.16" size="0.03" rgba="{rgba}" contype="0" conaffinity="0"{base_group}/>
            <geom name="{name}_visual_right_upper_arm_shell" type="capsule" fromto="0 0 0 0 0 -0.15" size="0.039" rgba="0.92 0.95 0.96 1" contype="0" conaffinity="0" group="2"/>
            <body name="{name}_right_lower_arm" pos="0 0 -0.16">
              <joint name="{prefix}_Right_Elbow_Pitch" type="hinge" axis="0 1 0" range="-2.27 2.27" damping="1"/>
              <joint name="{prefix}_Right_Elbow_Yaw" type="hinge" axis="0 0 1" range="-1 1" damping="1"/>
              <geom name="{name}_right_lower_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.14" size="0.026" rgba="{dark}" contype="0" conaffinity="0"{base_group}/>
              <geom name="{name}_visual_right_forearm_shell" type="capsule" fromto="0 0 0 0 0 -0.13" size="0.034" rgba="{dark}" contype="0" conaffinity="0" group="2"/>
            </body>
          </body>
        </body>
      </body>
    </body>"""


def actuator_xml(name: str, prefix: str) -> str:
    items = [
        f'    <position name="{name}_base_x_act" joint="{name}_base_x" kp="260" ctrlrange="-4 4" forcerange="-140 140"/>',
        f'    <position name="{name}_base_y_act" joint="{name}_base_y" kp="260" ctrlrange="-3 3" forcerange="-140 140"/>',
        f'    <position name="{name}_base_yaw_act" joint="{name}_base_yaw" kp="240" ctrlrange="-7 7" forcerange="-140 140"/>',
    ]
    for joint, _range, force in JOINTS:
        full = f"{prefix}_{joint}"
        items.append(
            f'    <position name="{full}_act" joint="{full}" kp="32" forcerange="-{force} {force}"/>'
        )
    return "\n".join(items)


def visual_v2_field_xml() -> str:
    return """
    <geom name="visual_v2_surround_north" type="box" pos="0 2.88 0.003" size="4.15 0.30 0.004" rgba="0.015 0.025 0.045 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_surround_south" type="box" pos="0 -2.88 0.003" size="4.15 0.30 0.004" rgba="0.015 0.025 0.045 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_surround_west" type="box" pos="-3.88 0 0.003" size="0.30 2.55 0.004" rgba="0.015 0.025 0.045 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_surround_east" type="box" pos="3.88 0 0.003" size="0.30 2.55 0.004" rgba="0.015 0.025 0.045 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_center_spot" type="cylinder" pos="0 0 0.016" size="0.035 0.004" rgba="1 1 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_penalty_back" type="box" pos="-2.82 0 0.014" size="0.012 0.82 0.005" material="line_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_penalty_top" type="box" pos="-3.16 0.82 0.014" size="0.34 0.012 0.005" material="line_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_penalty_bottom" type="box" pos="-3.16 -0.82 0.014" size="0.34 0.012 0.005" material="line_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_penalty_back" type="box" pos="2.82 0 0.014" size="0.012 0.82 0.005" material="line_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_penalty_top" type="box" pos="3.16 0.82 0.014" size="0.34 0.012 0.005" material="line_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_penalty_bottom" type="box" pos="3.16 -0.82 0.014" size="0.34 0.012 0.005" material="line_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_goal_left_post" type="cylinder" pos="-3.55 0.50 0.33" size="0.025 0.33" rgba="0.94 0.96 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_goal_right_post" type="cylinder" pos="-3.55 -0.50 0.33" size="0.025 0.33" rgba="0.94 0.96 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_goal_crossbar" type="capsule" fromto="-3.55 -0.50 0.66 -3.55 0.50 0.66" size="0.025" rgba="0.94 0.96 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_blue_goal_net_1" type="box" pos="-3.61 0 0.33" size="0.006 0.50 0.31" rgba="0.45 0.60 1 0.35" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_goal_left_post" type="cylinder" pos="3.55 0.50 0.33" size="0.025 0.33" rgba="0.94 0.96 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_goal_right_post" type="cylinder" pos="3.55 -0.50 0.33" size="0.025 0.33" rgba="0.94 0.96 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_goal_crossbar" type="capsule" fromto="3.55 -0.50 0.66 3.55 0.50 0.66" size="0.025" rgba="0.94 0.96 1 1" contype="0" conaffinity="0" group="2"/>
    <geom name="visual_v2_red_goal_net_1" type="box" pos="3.61 0 0.33" size="0.006 0.50 0.31" rgba="1 0.45 0.38 0.35" contype="0" conaffinity="0" group="2"/>
"""


def visual_v3_goal_xml() -> str:
    return """
    <geom name="BLUE_GOAL_left_post" type="capsule" fromto="-3.42 -0.60 0.035 -3.42 -0.60 0.70" size="0.035" rgba="0.95 0.96 0.98 1" contype="1" conaffinity="2" group="2" density="0"/>
    <geom name="BLUE_GOAL_right_post" type="capsule" fromto="-3.42 0.60 0.035 -3.42 0.60 0.70" size="0.035" rgba="0.95 0.96 0.98 1" contype="1" conaffinity="2" group="2" density="0"/>
    <geom name="BLUE_GOAL_crossbar" type="capsule" fromto="-3.42 -0.60 0.70 -3.42 0.60 0.70" size="0.035" rgba="0.95 0.96 0.98 1" contype="1" conaffinity="2" group="2" density="0"/>
    <geom name="BLUE_GOAL_back_left_post" type="capsule" fromto="-3.80 -0.60 0.035 -3.80 -0.60 0.70" size="0.025" rgba="0.78 0.84 1 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_back_right_post" type="capsule" fromto="-3.80 0.60 0.035 -3.80 0.60 0.70" size="0.025" rgba="0.78 0.84 1 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_top_left_depth" type="capsule" fromto="-3.42 -0.60 0.70 -3.80 -0.60 0.70" size="0.022" rgba="0.88 0.92 1 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_top_right_depth" type="capsule" fromto="-3.42 0.60 0.70 -3.80 0.60 0.70" size="0.022" rgba="0.88 0.92 1 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_back_bottom" type="capsule" fromto="-3.80 -0.60 0.04 -3.80 0.60 0.04" size="0.018" rgba="0.70 0.80 1 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_left_net" type="box" pos="-3.61 -0.60 0.36" size="0.19 0.006 0.34" rgba="0.45 0.65 1 0.28" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_right_net" type="box" pos="-3.61 0.60 0.36" size="0.19 0.006 0.34" rgba="0.45 0.65 1 0.28" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_top_net" type="box" pos="-3.61 0 0.70" size="0.19 0.60 0.006" rgba="0.45 0.65 1 0.24" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_back_net" type="box" pos="-3.80 0 0.36" size="0.006 0.60 0.34" rgba="0.45 0.65 1 0.30" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="BLUE_GOAL_base_panel" type="box" pos="-3.62 0 0.018" size="0.22 0.67 0.012" rgba="0.04 0.28 0.82 0.65" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_left_post" type="capsule" fromto="3.42 -0.60 0.035 3.42 -0.60 0.70" size="0.035" rgba="0.95 0.96 0.98 1" contype="1" conaffinity="2" group="2" density="0"/>
    <geom name="RED_GOAL_right_post" type="capsule" fromto="3.42 0.60 0.035 3.42 0.60 0.70" size="0.035" rgba="0.95 0.96 0.98 1" contype="1" conaffinity="2" group="2" density="0"/>
    <geom name="RED_GOAL_crossbar" type="capsule" fromto="3.42 -0.60 0.70 3.42 0.60 0.70" size="0.035" rgba="0.95 0.96 0.98 1" contype="1" conaffinity="2" group="2" density="0"/>
    <geom name="RED_GOAL_back_left_post" type="capsule" fromto="3.80 -0.60 0.035 3.80 -0.60 0.70" size="0.025" rgba="1 0.82 0.78 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_back_right_post" type="capsule" fromto="3.80 0.60 0.035 3.80 0.60 0.70" size="0.025" rgba="1 0.82 0.78 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_top_left_depth" type="capsule" fromto="3.42 -0.60 0.70 3.80 -0.60 0.70" size="0.022" rgba="1 0.90 0.86 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_top_right_depth" type="capsule" fromto="3.42 0.60 0.70 3.80 0.60 0.70" size="0.022" rgba="1 0.90 0.86 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_back_bottom" type="capsule" fromto="3.80 -0.60 0.04 3.80 0.60 0.04" size="0.018" rgba="1 0.74 0.70 1" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_left_net" type="box" pos="3.61 -0.60 0.36" size="0.19 0.006 0.34" rgba="1 0.48 0.42 0.28" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_right_net" type="box" pos="3.61 0.60 0.36" size="0.19 0.006 0.34" rgba="1 0.48 0.42 0.28" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_top_net" type="box" pos="3.61 0 0.70" size="0.19 0.60 0.006" rgba="1 0.48 0.42 0.24" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_back_net" type="box" pos="3.80 0 0.36" size="0.006 0.60 0.34" rgba="1 0.48 0.42 0.30" contype="0" conaffinity="0" group="2" density="0"/>
    <geom name="RED_GOAL_base_panel" type="box" pos="3.62 0 0.018" size="0.22 0.67 0.012" rgba="0.82 0.08 0.09 0.65" contype="0" conaffinity="0" group="2" density="0"/>
"""


def build_model(visual_v2: bool = False, visual_v3: bool = False) -> str:
    visual_shell = visual_v2 or visual_v3
    robot_bodies = "\n".join(robot_xml(name, cfg, visual_v2=visual_shell) for name, cfg in ROBOTS.items())
    robot_actuators = "\n".join(
        actuator_xml(name, str(cfg["prefix"])) for name, cfg in ROBOTS.items()
    )
    model_name = "T1_2v2_assisted_physical_soccer_visual_v3" if visual_v3 else ("T1_2v2_assisted_physical_soccer_visual_v2" if visual_v2 else "T1_2v2_assisted_physical_soccer")
    field_rgb1 = "0.10 0.48 0.16" if visual_v3 else ("0.02 0.60 0.17" if visual_v2 else "0.05 0.45 0.12")
    field_rgb2 = "0.07 0.38 0.12" if visual_v3 else ("0.03 0.50 0.13" if visual_v2 else "0.04 0.36 0.10")
    extra_field = (visual_v2_field_xml() + (visual_v3_goal_xml() if visual_v3 else "")) if visual_shell else ""
    extra_lights = """
    <light name="visual_v2_fill_left" directional="true" pos="-2 3 5" dir="0.35 -0.25 -1" diffuse="0.42 0.45 0.50"/>
    <light name="visual_v2_fill_right" directional="true" pos="2 -3 5" dir="-0.35 0.25 -1" diffuse="0.38 0.40 0.46"/>
""" if visual_shell else ""
    broadcast_camera = """
    <camera name="broadcast" pos="0 -6.4 3.25" xyaxes="1 0 0 0 0.47 0.88" fovy="42"/>
    <camera name="follow_ball" pos="-1.6 -4.0 2.3" xyaxes="1 0 0 0 0.50 0.86" fovy="45"/>
    <camera name="broadcast_wide" pos="0 -7.2 4.25" xyaxes="1 0 0 0 0.54 0.84" fovy="46"/>
    <camera name="broadcast_action" pos="0.15 -5.65 3.15" xyaxes="1 0 0 0 0.50 0.86" fovy="39"/>
""" if visual_shell else ""
    text = f"""<mujoco model="{model_name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.005" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <joint armature="0.03" damping="3.0"/>
    <geom density="320"/>
  </default>
  <size njmax="300" nconmax="120"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture name="field_tex" type="2d" builtin="checker" rgb1="{field_rgb1}" rgb2="{field_rgb2}" width="512" height="512"/>
    <material name="field_mat" texture="field_tex" texrepeat="8 6" texuniform="true" reflectance="0.08"/>
    <material name="line_mat" rgba="1 1 1 1"/>
    <material name="ball_mat" rgba="0.96 0.96 0.90 1"/>
  </asset>
  <worldbody>
    <light name="sun" directional="true" pos="0 -4 6" dir="0.2 0.35 -1" diffuse="0.85 0.85 0.8"/>
{extra_lights}
    <camera name="overview" pos="0 -5.3 4.2" xyaxes="1 0 0 0 0.62 0.78" fovy="48"/>
{broadcast_camera}
    <geom name="ground" type="plane" pos="0 0 0" size="4 3 0.1" material="field_mat" contype="1" conaffinity="2" friction="1.0 0.02 0.002"/>
    <geom name="line_left" type="box" pos="0 -2.5 0.006" size="3.5 0.018 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="line_right" type="box" pos="0 2.5 0.006" size="3.5 0.018 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="line_blue" type="box" pos="-3.5 0 0.006" size="0.018 2.5 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="line_red" type="box" pos="3.5 0 0.006" size="0.018 2.5 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="line_mid" type="box" pos="0 0 0.007" size="0.012 2.5 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_0" type="box" pos="0.360 0.000 0.012" euler="0 0 0.000" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_1" type="box" pos="0.255 0.255 0.012" euler="0 0 0.785" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_2" type="box" pos="0.000 0.360 0.012" euler="0 0 1.571" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_3" type="box" pos="-0.255 0.255 0.012" euler="0 0 2.356" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_4" type="box" pos="-0.360 0.000 0.012" euler="0 0 3.142" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_5" type="box" pos="-0.255 -0.255 0.012" euler="0 0 3.927" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_6" type="box" pos="0.000 -0.360 0.012" euler="0 0 4.712" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="center_circle_7" type="box" pos="0.255 -0.255 0.012" euler="0 0 5.498" size="0.120 0.010 0.006" material="line_mat" contype="0" conaffinity="0"/>
    <geom name="field_wall_blue" type="box" pos="-3.58 0 0.12" size="0.03 2.55 0.12" rgba="1 1 1 0.08" contype="1" conaffinity="2"/>
    <geom name="field_wall_red" type="box" pos="3.58 0 0.12" size="0.03 2.55 0.12" rgba="1 1 1 0.08" contype="1" conaffinity="2"/>
    <geom name="field_wall_left" type="box" pos="0 -2.58 0.12" size="3.58 0.03 0.12" rgba="1 1 1 0.08" contype="1" conaffinity="2"/>
    <geom name="field_wall_right" type="box" pos="0 2.58 0.12" size="3.58 0.03 0.12" rgba="1 1 1 0.08" contype="1" conaffinity="2"/>
    {'' if visual_v3 else '<geom name="BLUE_GOAL" type="box" pos="-3.52 0 0.42" size="0.03 0.48 0.42" rgba="0.0 0.12 1 0.35" contype="0" conaffinity="0"/>'}
    {'' if visual_v3 else '<geom name="RED_GOAL" type="box" pos="3.52 0 0.42" size="0.03 0.48 0.42" rgba="1 0.05 0.02 0.35" contype="0" conaffinity="0"/>'}
{extra_field}
    <body name="soccer_ball" pos="-1.35 -0.05 0.115">
      <joint name="soccer_ball_free" type="free" damping="0.04"/>
      <geom name="soccer_ball_geom" type="sphere" size="0.11" mass="0.43" material="ball_mat" contype="2" conaffinity="5" friction="1.0 0.05 0.025" solref="0.006 1.0" solimp="0.9 0.95 0.001"/>
    </body>
{robot_bodies}
  </worldbody>
  <actuator>
{robot_actuators}
  </actuator>
</mujoco>
"""
    if visual_shell:
        text = text.replace('group="2"', 'group="2" density="0"')
        text = text.replace('density="0" density="0"', 'density="0"')
    return text


def main() -> None:
    root = Path(__file__).resolve().parent
    model = root / "models" / "t1_2v2_soccer.xml"
    single = root / "models" / "t1_single_contact_test.xml"
    text = build_model()
    model.write_text(text, encoding="utf-8")
    single.write_text(text.replace('model="T1_2v2_assisted_physical_soccer"', 'model="T1_single_contact_test"'), encoding="utf-8")
    visual_v2 = root / "models" / "t1_2v2_soccer_visual_v2.xml"
    visual_v2.write_text(build_model(visual_v2=True), encoding="utf-8")
    visual_v3 = root / "models" / "t1_2v2_soccer_visual_v3.xml"
    visual_v3.write_text(build_model(visual_v3=True), encoding="utf-8")
    print(model)
    print(single)
    print(visual_v2)
    print(visual_v3)


if __name__ == "__main__":
    main()
