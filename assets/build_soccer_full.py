"""Generate assets/soccer_full.xml with hinged limb robots."""
from pathlib import Path


def robot_xml(rid: int, pos: str, leg: str, torso: str) -> str:
    return f"""
    <body name="robot_{rid}" mocap="true" pos="{pos}">
      <geom name="robot_{rid}_ring" type="cylinder" size="0.14 0.004" pos="0 0 0.003"
            rgba="0.5 0.5 0.5 0.7" contype="0" conaffinity="0"/>
      <geom name="robot_{rid}_torso" type="box" pos="0 0 0.33" size="0.065 0.05 0.11"
            rgba="{torso}" contype="0" conaffinity="0"/>
      <geom name="robot_{rid}_head" type="sphere" pos="0 0 0.48" size="0.045"
            rgba="0.85 0.85 0.85 1" contype="0" conaffinity="0"/>
      <geom name="robot_{rid}_face" type="capsule" fromto="0 0 0.33 0.12 0 0.33" size="0.012"
            rgba="1 1 1 0.9" contype="0" conaffinity="0"/>

      <body name="robot_{rid}_r_shoulder" pos="0.08 0 0.38">
        <joint name="robot_{rid}_r_shoulder" type="hinge" axis="0 1 0" range="-2.5 2.5" damping="1"/>
        <geom name="robot_{rid}_r_arm" type="capsule" fromto="0 0 0 0.12 0 -0.02" size="0.018"
              rgba="{torso}" contype="0" conaffinity="0"/>
      </body>
      <body name="robot_{rid}_l_shoulder" pos="-0.08 0 0.38">
        <joint name="robot_{rid}_l_shoulder" type="hinge" axis="0 1 0" range="-2.5 2.5" damping="1"/>
        <geom name="robot_{rid}_l_arm" type="capsule" fromto="0 0 0 -0.12 0 -0.02" size="0.018"
              rgba="{torso}" contype="0" conaffinity="0"/>
      </body>

      <body name="robot_{rid}_r_hip" pos="0.045 0 0.24">
        <joint name="robot_{rid}_r_hip" type="hinge" axis="0 1 0" range="-2.2 1.2" damping="1"/>
        <geom name="robot_{rid}_r_thigh" type="capsule" fromto="0 0 0 0 0 -0.12" size="0.028"
              rgba="{leg}" contype="0" conaffinity="0"/>
        <body name="robot_{rid}_r_knee" pos="0 0 -0.12">
          <joint name="robot_{rid}_r_knee" type="hinge" axis="0 1 0" range="-0.05 2.2" damping="1"/>
          <geom name="robot_{rid}_r_shank" type="capsule" fromto="0 0 0 0 0 -0.12" size="0.025"
                rgba="{leg}" contype="0" conaffinity="0"/>
        </body>
      </body>

      <body name="robot_{rid}_l_hip" pos="-0.045 0 0.24">
        <joint name="robot_{rid}_l_hip" type="hinge" axis="0 1 0" range="-2.2 1.2" damping="1"/>
        <geom name="robot_{rid}_l_thigh" type="capsule" fromto="0 0 0 0 0 -0.12" size="0.028"
              rgba="{leg}" contype="0" conaffinity="0"/>
        <body name="robot_{rid}_l_knee" pos="0 0 -0.12">
          <joint name="robot_{rid}_l_knee" type="hinge" axis="0 1 0" range="-0.05 2.2" damping="1"/>
          <geom name="robot_{rid}_l_shank" type="capsule" fromto="0 0 0 0 0 -0.12" size="0.025"
                rgba="{leg}" contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
"""


def main() -> None:
    blue_leg, blue_torso = "0.18 0.42 0.92 1", "0.2 0.45 0.95 1"
    yellow_leg, yellow_torso = "0.85 0.65 0.08 1", "0.95 0.75 0.1 1"
    robots = [
        (0, "-1 0 0", blue_leg, blue_torso),
        (1, "-2 1.5 0", blue_leg, blue_torso),
        (2, "-2.5 0 0", blue_leg, blue_torso),
        (10, "1 0 0", yellow_leg, yellow_torso),
        (11, "2 -1.5 0", yellow_leg, yellow_torso),
        (12, "2.5 0 0", yellow_leg, yellow_torso),
    ]

    header = """<mujoco model="soccer_full">
  <!-- Booster T1 multi-robot soccer — hinged limbs for kick/walk animation -->
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <visual>
    <headlight diffuse="0.8 0.8 0.8" ambient="0.35 0.35 0.35"/>
    <rgba haze="0.1 0.2 0.3 1"/>
    <map fogstart="8" fogend="25"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient"
             rgb1="0.4 0.6 0.85" rgb2="0.05 0.1 0.2" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker"
             rgb1="0.18 0.55 0.18" rgb2="0.12 0.42 0.12" width="512" height="512"/>
    <material name="matplane" texture="texplane" texrepeat="8 6" reflectance="0.08"/>
    <material name="matgoal_post" rgba="1 1 1 0.9"/>
    <material name="matgoal_net" rgba="1 1 1 0.2"/>
    <material name="matline" rgba="1 1 1 0.7"/>
  </asset>

  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" diffuse="1 1 1" castshadow="true"/>
    <light pos="5 3 8" dir="-0.5 -0.3 -1" diffuse="0.3 0.35 0.4"/>
    <light pos="-5 -3 6" dir="0.5 0.3 -1" diffuse="0.2 0.2 0.25"/>

    <geom name="floor" type="plane" size="6 5 0.1" material="matplane"/>

    <geom name="line_top"    type="box" pos="0 3.0 0.006" size="4.6 0.025 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_bottom" type="box" pos="0 -3.0 0.006" size="4.6 0.025 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_left"   type="box" pos="-4.5 0 0.006" size="0.025 3.0 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_right"  type="box" pos="4.5 0 0.006" size="0.025 3.0 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_center" type="box" pos="0 0 0.006" size="0.02 3.0 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="circle_center" type="cylinder" pos="0 0 0.005" size="0.8 0.003" rgba="1 1 1 0.45" contype="0" conaffinity="0"/>

    <geom name="goal_left_post_top"    type="box" pos="-4.5 1.0 0.35" size="0.03 0.03 0.35" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_left_post_bottom" type="box" pos="-4.5 -1.0 0.35" size="0.03 0.03 0.35" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_left_bar"         type="box" pos="-4.5 0 0.70" size="0.03 1.0 0.03" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_left_net"         type="box" pos="-4.75 0 0.35" size="0.01 1.0 0.35" material="matgoal_net" contype="0" conaffinity="0"/>
    <geom name="goal_right_post_top"    type="box" pos="4.5 1.0 0.35" size="0.03 0.03 0.35" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_right_post_bottom" type="box" pos="4.5 -1.0 0.35" size="0.03 0.03 0.35" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_right_bar"         type="box" pos="4.5 0 0.70" size="0.03 1.0 0.03" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_right_net"         type="box" pos="4.75 0 0.35" size="0.01 1.0 0.35" material="matgoal_net" contype="0" conaffinity="0"/>

    <body name="ball" pos="0 0 0.05">
      <freejoint name="ball_joint"/>
      <geom name="ball_geom" type="sphere" size="0.05"
            rgba="1 1 1 1" mass="0.045" friction="0.3 0.3 0.001"/>
    </body>

    <body name="pass_line_0" mocap="true" pos="0 0 0.03">
      <geom name="pass_line_0_geom" type="box" size="0.5 0.012 0.004"
            rgba="0.3 1.0 0.15 0" contype="0" conaffinity="0"/>
    </body>
    <body name="pass_line_1" mocap="true" pos="0 0 0.03">
      <geom name="pass_line_1_geom" type="box" size="0.5 0.012 0.004"
            rgba="0.3 1.0 0.15 0" contype="0" conaffinity="0"/>
    </body>
    <body name="pass_line_2" mocap="true" pos="0 0 0.03">
      <geom name="pass_line_2_geom" type="box" size="0.5 0.012 0.004"
            rgba="0.3 1.0 0.15 0" contype="0" conaffinity="0"/>
    </body>
"""
    footer = """
  </worldbody>
</mujoco>
"""
    out = Path(__file__).resolve().parent / "soccer_full.xml"
    out.write_text(header + "\n".join(robot_xml(*r) for r in robots) + footer, encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
