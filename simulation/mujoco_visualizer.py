"""
MuJoCo 3D 可视化器
==================
封装 MuJoCo viewer 的生命周期, 提供与 ASCIIVisualizer / MatplotlibVisualizer
兼容的 render() / close() 接口。

使用方式:
    simulator = MuJoCoSimulator(...)
    visualizer = MuJoCoVisualizer(simulator)
    for tick in range(total_ticks):
        visualizer.render(world_state, fsm)
    visualizer.close()
"""

from __future__ import annotations

import time
from typing import Optional

import mujoco.viewer

from common.world_state import WorldState


class MuJoCoVisualizer:
    """
    MuJoCo 3D 可视化器。

    封装 mujoco.viewer.launch_passive, 提供:
    - render(ws, fsm): 同步状态 + 刷新光圈/传球线 + 刷新画面
    - close(): 关闭 viewer
    - is_running: 检查 viewer 是否仍在运行 (用户关闭窗口)
    """

    def __init__(self, simulator, title: str = "Booster T1 — MuJoCo 3D"):
        """
        Args:
            simulator: MuJoCoSimulator 实例
            title: 窗口标题
        """
        self._sim = simulator
        self._title = title
        self._viewer: Optional[mujoco.viewer.Handle] = None
        self._frame_count: int = 0
        self._start_time: float = 0.0

    # ================================================================
    # 生命周期
    # ================================================================

    def render(self, ws: WorldState, fsm, yellow_fsm=None) -> bool:
        """
        渲染当前帧。

        首次调用时启动 MuJoCo viewer。
        后续调用执行: 同步状态 → 更新光圈 → 更新传球线 → 刷新画面。

        Args:
            ws: 当前 WorldState 快照
            fsm: 蓝队 DecisionFSM
            yellow_fsm: 黄队 DecisionFSM (可选)

        Returns:
            True 如果 viewer 仍在运行, False 如果用户已关闭窗口
        """
        # 延迟启动 viewer (在第一次 render 时)
        if self._viewer is None:
            self._start_viewer()

        if self._viewer is None:
            return False  # viewer 启动失败

        if not self._viewer.is_running():
            return False

        # 同步物理状态到 MuJoCo
        self._sim.sync_to_mujoco()

        # 更新视觉元素
        self._sim.update_status_rings(fsm, yellow_fsm)
        self._sim.update_pass_lines(ws, fsm, yellow_fsm)

        # 刷新画面
        self._viewer.sync()

        self._frame_count += 1
        return True

    def _start_viewer(self):
        """启动 MuJoCo passive viewer"""
        try:
            self._viewer = mujoco.viewer.launch_passive(
                self._sim.model, self._sim.data,
                show_left_ui=False,
                show_right_ui=False,
            )
            # 设置初始相机角度 (俯视视角, 能看到整个场地)
            self._viewer.cam.lookat[:] = [0, 0, 0.5]
            self._viewer.cam.distance = 14
            self._viewer.cam.elevation = -35
            self._viewer.cam.azimuth = 90

            self._start_time = time.time()

            print(f"  MuJoCo Viewer 已启动 (窗口可能在其他窗口后面)")
            print(f"  操作: 鼠标拖拽=旋转 | 滚轮=缩放 | 右键拖拽=平移")
            print(f"  关闭 3D 窗口即可结束仿真")

        except Exception as e:
            print(f"  错误: 无法启动 MuJoCo viewer: {e}")
            self._viewer = None

    def close(self):
        """关闭 viewer"""
        if self._viewer is not None:
            elapsed = time.time() - self._start_time if self._start_time > 0 else 0
            if elapsed > 0:
                print(f"\n  MuJoCo Viewer 已关闭 (渲染 {self._frame_count} 帧, "
                      f"实际耗时 {elapsed:.1f}s)")
            self._viewer = None

    # ================================================================
    # 查询
    # ================================================================

    @property
    def is_running(self) -> bool:
        """viewer 是否仍在运行"""
        if self._viewer is None:
            return False
        return self._viewer.is_running()

    @property
    def frame_count(self) -> int:
        """已渲染帧数"""
        return self._frame_count
