"""
生成 assets/soccer_full.xml
============================
挂载官方 Booster T1 STL (booster_assets/robots/T1)，
仍用 mocap + LimbAnimator 短名关节 + 冲量踢球。

依赖: 仓库旁/内的 booster_assets 克隆
  project4_multi_agent/booster_assets/robots/T1/meshes/*.STL
"""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "booster_assets" / "robots" / "T1" / "meshes"
# 相对 soccer_full.xml (在 assets/) 的 mesh 目录
MESH_DIR_REL = "../booster_assets/robots/T1/meshes"

# 官方模型 Trunk 默认高度约 0.7m；略抬/降可调脚贴地
TRUNK_Z = 0.68


def _require_meshes() -> None:
    need = ["Trunk.STL", "H1.STL", "H2.STL", "Waist.STL", "Hip_Pitch_Right.STL"]
    missing = [n for n in need if not (MESH_DIR / n).is_file()]
    if missing:
        raise FileNotFoundError(
            "缺少官方 T1 网格。请先克隆:\n"
            "  git clone https://github.com/BoosterRobotics/booster_assets.git\n"
            f"到 {ROOT / 'booster_assets'}\n"
            f"缺失: {missing}"
        )


def _asset_meshes() -> str:
    names = [
        "Trunk", "H1", "H2", "Logo",
        "AL1", "AL2", "AL3", "left_hand_link",
        "AR1", "AR2", "AR3", "right_hand_link",
        "Waist",
        "Hip_Pitch_Left", "Hip_Roll_Left", "Hip_Yaw_Left", "Shank_Left",
        "Ankle_Cross_Left", "left_foot_link",
        "Hip_Pitch_Right", "Hip_Roll_Right", "Hip_Yaw_Right", "Shank_Right",
        "Ankle_Cross_Right", "right_foot_link",
    ]
    lines = [f'    <mesh name="{n}" file="{n}.STL"/>' for n in names]
    return "\n".join(lines)


def _inertial(mass: float, pos: str = "0 0 0") -> str:
    i = max(1e-4, mass * 0.02)
    return f'<inertial pos="{pos}" mass="{mass}" diaginertia="{i} {i} {i}"/>'


def t1_robot_xml(rid: int, pos: str, team_rgba: str) -> str:
    """
    官方 T1_23dof 运动学树，嵌在 mocap robot_{id} 下。
    动画短名: l/r_hip, l/r_knee, l/r_shoulder
    其余关节保留官方语义后缀 (跟随驱动用)。
    """
    p = f"robot_{rid}"
    # 主外壳色 = 队色；深色部件保持深灰
    shell = team_rgba
    dark = "0.35 0.35 0.35 1"
    mid = "0.55 0.55 0.55 1"

    return f"""
    <!-- Official Booster T1 STL  rid={rid} -->
    <body name="{p}" mocap="true" pos="{pos}">
      <geom name="{p}_ring" type="cylinder" size="0.22 0.004" pos="0 0 0.003"
            rgba="0.5 0.5 0.5 0.7" contype="0" conaffinity="0"/>

      <body name="{p}_Trunk" pos="0 0 {TRUNK_Z}">
        {_inertial(11.7, "0.055 0 0.105")}
        <geom type="mesh" mesh="Trunk" rgba="{shell}" contype="0" conaffinity="0" group="1" density="0"/>
        <geom type="mesh" mesh="Logo" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0" group="1" density="0"/>

        <!-- Head -->
        <body name="{p}_H1" pos="0.0625 0 0.243">
          {_inertial(0.44)}
          <joint name="{p}_AAHead_yaw" type="hinge" axis="0 0 1" range="-1.57 1.57" damping="1"/>
          <geom type="mesh" mesh="H1" rgba="{mid}" contype="0" conaffinity="0" group="1" density="0"/>
          <body name="{p}_H2" pos="0 0 0.06185">
            {_inertial(0.63)}
            <joint name="{p}_Head_pitch" type="hinge" axis="0 1 0" range="-0.35 1.22" damping="1"/>
            <geom type="mesh" mesh="H2" rgba="{mid}" contype="0" conaffinity="0" group="1" density="0"/>
          </body>
        </body>

        <!-- Left Arm: l_shoulder = Shoulder_Pitch -->
        <body name="{p}_AL1" pos="0.0575 0.1063 0.219">
          {_inertial(0.53)}
          <joint name="{p}_l_shoulder" type="hinge" axis="0 1 0" range="-3.31 1.22" damping="1"/>
          <geom type="mesh" mesh="AL1" rgba="{shell}" contype="0" conaffinity="0" group="1" density="0"/>
          <body name="{p}_AL2" pos="0 0.047 0">
            {_inertial(0.16)}
            <joint name="{p}_Left_Shoulder_Roll" type="hinge" axis="1 0 0" range="-1.74 1.57" damping="1"/>
            <geom type="mesh" mesh="AL2" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
            <body name="{p}_AL3" pos="0.00025 0.0605 0">
              {_inertial(1.02)}
              <joint name="{p}_Left_Elbow_Pitch" type="hinge" axis="0 1 0" range="-2.27 2.27" damping="1"/>
              <geom type="mesh" mesh="AL3" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
              <body name="{p}_left_hand" pos="0 0.1471 0">
                {_inertial(0.33)}
                <joint name="{p}_Left_Elbow_Yaw" type="hinge" axis="0 0 1" range="-2.44 0" damping="1"/>
                <geom type="mesh" mesh="left_hand_link" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
              </body>
            </body>
          </body>
        </body>

        <!-- Right Arm: r_shoulder -->
        <body name="{p}_AR1" pos="0.0575 -0.1063 0.219">
          {_inertial(0.53)}
          <joint name="{p}_r_shoulder" type="hinge" axis="0 1 0" range="-3.31 1.22" damping="1"/>
          <geom type="mesh" mesh="AR1" rgba="{shell}" contype="0" conaffinity="0" group="1" density="0"/>
          <body name="{p}_AR2" pos="0 -0.047 0">
            {_inertial(0.16)}
            <joint name="{p}_Right_Shoulder_Roll" type="hinge" axis="1 0 0" range="-1.57 1.74" damping="1"/>
            <geom type="mesh" mesh="AR2" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
            <body name="{p}_AR3" pos="0.00025 -0.0605 0">
              {_inertial(1.02)}
              <joint name="{p}_Right_Elbow_Pitch" type="hinge" axis="0 1 0" range="-2.27 2.27" damping="1"/>
              <geom type="mesh" mesh="AR3" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
              <body name="{p}_right_hand" pos="0 -0.1471 0">
                {_inertial(0.33)}
                <joint name="{p}_Right_Elbow_Yaw" type="hinge" axis="0 0 1" range="0 2.44" damping="1"/>
                <geom type="mesh" mesh="right_hand_link" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
              </body>
            </body>
          </body>
        </body>

        <!-- Waist + Legs -->
        <body name="{p}_Waist" pos="0.0625 0 -0.1155">
          {_inertial(2.58)}
          <joint name="{p}_Waist" type="hinge" axis="0 0 1" range="-1.57 1.57" damping="1"/>
          <geom type="mesh" mesh="Waist" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>

          <!-- Left Leg: l_hip / l_knee -->
          <body name="{p}_Hip_Pitch_Left" pos="0 0.106 0">
            {_inertial(1.02)}
            <joint name="{p}_l_hip" type="hinge" axis="0 1 0" range="-3.0 2.2" damping="1"/>
            <geom type="mesh" mesh="Hip_Pitch_Left" rgba="{shell}" contype="0" conaffinity="0" group="1" density="0"/>
            <body name="{p}_Hip_Roll_Left" pos="0 0 -0.02">
              {_inertial(0.39)}
              <joint name="{p}_Left_Hip_Roll" type="hinge" axis="1 0 0" range="-0.2 1.57" damping="1"/>
              <geom type="mesh" mesh="Hip_Roll_Left" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
              <body name="{p}_Hip_Yaw_Left" pos="0 0 -0.081854">
                {_inertial(2.17)}
                <joint name="{p}_Left_Hip_Yaw" type="hinge" axis="0 0 1" range="-1 1" damping="1"/>
                <geom type="mesh" mesh="Hip_Yaw_Left" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                <body name="{p}_Shank_Left" pos="-0.014 0 -0.134">
                  {_inertial(1.82)}
                  <joint name="{p}_l_knee" type="hinge" axis="0 1 0" range="-0.05 2.18" damping="1"/>
                  <geom type="mesh" mesh="Shank_Left" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                  <body name="{p}_Ankle_Cross_Left" pos="0 0 -0.28">
                    {_inertial(0.07)}
                    <joint name="{p}_Left_Ankle_Pitch" type="hinge" axis="0 1 0" range="-0.87 0.35" damping="1"/>
                    <geom type="mesh" mesh="Ankle_Cross_Left" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                    <body name="{p}_left_foot" pos="0 0.00025 -0.012">
                      {_inertial(0.69)}
                      <joint name="{p}_Left_Ankle_Roll" type="hinge" axis="1 0 0" range="-0.44 0.44" damping="1"/>
                      <geom type="mesh" mesh="left_foot_link" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>

          <!-- Right Leg: r_hip / r_knee (kick leg) -->
          <body name="{p}_Hip_Pitch_Right" pos="0 -0.106 0">
            {_inertial(1.02)}
            <joint name="{p}_r_hip" type="hinge" axis="0 1 0" range="-3.0 2.2" damping="1"/>
            <geom type="mesh" mesh="Hip_Pitch_Right" rgba="{shell}" contype="0" conaffinity="0" group="1" density="0"/>
            <body name="{p}_Hip_Roll_Right" pos="0 0 -0.02">
              {_inertial(0.39)}
              <joint name="{p}_Right_Hip_Roll" type="hinge" axis="1 0 0" range="-1.57 0.2" damping="1"/>
              <geom type="mesh" mesh="Hip_Roll_Right" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
              <body name="{p}_Hip_Yaw_Right" pos="0 0 -0.081854">
                {_inertial(2.17)}
                <joint name="{p}_Right_Hip_Yaw" type="hinge" axis="0 0 1" range="-1 1" damping="1"/>
                <geom type="mesh" mesh="Hip_Yaw_Right" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                <body name="{p}_Shank_Right" pos="-0.014 0 -0.134">
                  {_inertial(1.82)}
                  <joint name="{p}_r_knee" type="hinge" axis="0 1 0" range="-0.05 2.18" damping="1"/>
                  <geom type="mesh" mesh="Shank_Right" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                  <body name="{p}_Ankle_Cross_Right" pos="0 0 -0.28">
                    {_inertial(0.07)}
                    <joint name="{p}_Right_Ankle_Pitch" type="hinge" axis="0 1 0" range="-0.87 0.35" damping="1"/>
                    <geom type="mesh" mesh="Ankle_Cross_Right" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                    <body name="{p}_right_foot" pos="0 -0.00025 -0.012">
                      {_inertial(0.69)}
                      <joint name="{p}_Right_Ankle_Roll" type="hinge" axis="1 0 0" range="-0.44 0.44" damping="1"/>
                      <geom type="mesh" mesh="right_foot_link" rgba="{dark}" contype="0" conaffinity="0" group="1" density="0"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
"""


def main() -> None:
    _require_meshes()

    blue = "0.15 0.40 0.95 1"
    yellow = "0.95 0.70 0.12 1"
    robots = [
        (0, "-1.2 0 0", blue),
        (1, "-2.2 1.4 0", blue),
        (2, "-2.6 0 0", blue),
        (10, "1.2 0 0", yellow),
        (11, "2.2 -1.4 0", yellow),
        (12, "2.6 0 0", yellow),
    ]

    header = f"""<mujoco model="soccer_full">
  <!-- Official Booster T1 STL meshes from booster_assets -->
  <!-- See docs/T1_ROOT_REPLACEMENT_STEPS.md step 6 -->
  <compiler angle="radian" meshdir="{MESH_DIR_REL}" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <visual>
    <headlight diffuse="0.8 0.8 0.8" ambient="0.4 0.4 0.4"/>
    <rgba haze="0.1 0.2 0.3 1"/>
    <map fogstart="10" fogend="30"/>
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
{_asset_meshes()}
  </asset>

  <worldbody>
    <light pos="0 0 12" dir="0 0 -1" diffuse="1 1 1" castshadow="true"/>
    <light pos="5 3 8" dir="-0.5 -0.3 -1" diffuse="0.35 0.38 0.42"/>
    <light pos="-5 -3 6" dir="0.5 0.3 -1" diffuse="0.25 0.25 0.28"/>

    <geom name="floor" type="plane" size="6 5 0.1" material="matplane"/>

    <geom name="line_top"    type="box" pos="0 3.0 0.006" size="4.6 0.025 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_bottom" type="box" pos="0 -3.0 0.006" size="4.6 0.025 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_left"   type="box" pos="-4.5 0 0.006" size="0.025 3.0 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_right"  type="box" pos="4.5 0 0.006" size="0.025 3.0 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="line_center" type="box" pos="0 0 0.006" size="0.02 3.0 0.005" material="matline" contype="0" conaffinity="0"/>
    <geom name="circle_center" type="cylinder" pos="0 0 0.005" size="0.8 0.003" rgba="1 1 1 0.45" contype="0" conaffinity="0"/>

    <geom name="goal_left_post_top"    type="box" pos="-4.5 1.0 0.55" size="0.04 0.04 0.55" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_left_post_bottom" type="box" pos="-4.5 -1.0 0.55" size="0.04 0.04 0.55" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_left_bar"         type="box" pos="-4.5 0 1.10" size="0.04 1.0 0.04" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_left_net"         type="box" pos="-4.8 0 0.55" size="0.01 1.0 0.55" material="matgoal_net" contype="0" conaffinity="0"/>
    <geom name="goal_right_post_top"    type="box" pos="4.5 1.0 0.55" size="0.04 0.04 0.55" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_right_post_bottom" type="box" pos="4.5 -1.0 0.55" size="0.04 0.04 0.55" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_right_bar"         type="box" pos="4.5 0 1.10" size="0.04 1.0 0.04" material="matgoal_post" contype="0" conaffinity="0"/>
    <geom name="goal_right_net"         type="box" pos="4.8 0 0.55" size="0.01 1.0 0.55" material="matgoal_net" contype="0" conaffinity="0"/>

    <body name="ball" pos="0 0 0.055">
      <freejoint name="ball_joint"/>
      <geom name="ball_geom" type="sphere" size="0.055"
            rgba="1 1 1 1" mass="0.045" friction="0.3 0.3 0.001"/>
    </body>

    <body name="pass_line_0" mocap="true" pos="0 0 0.05">
      <geom name="pass_line_0_geom" type="box" size="0.5 0.015 0.005"
            rgba="0.3 1.0 0.15 0" contype="0" conaffinity="0"/>
    </body>
    <body name="pass_line_1" mocap="true" pos="0 0 0.05">
      <geom name="pass_line_1_geom" type="box" size="0.5 0.015 0.005"
            rgba="0.3 1.0 0.15 0" contype="0" conaffinity="0"/>
    </body>
    <body name="pass_line_2" mocap="true" pos="0 0 0.05">
      <geom name="pass_line_2_geom" type="box" size="0.5 0.015 0.005"
            rgba="0.3 1.0 0.15 0" contype="0" conaffinity="0"/>
    </body>
"""
    footer = """
  </worldbody>
</mujoco>
"""
    out = Path(__file__).resolve().parent / "soccer_full.xml"
    text = header + "\n".join(t1_robot_xml(*r) for r in robots) + footer
    out.write_text(text, encoding="utf-8")
    print(f"wrote {out}")
    print(f"meshdir -> {MESH_DIR}")
    print(f"robots: {len(robots)} x official T1 STL")


if __name__ == "__main__":
    main()
