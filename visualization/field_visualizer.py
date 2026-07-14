"""
2D 足球场 Matplotlib 可视化
============================
实时显示机器人、足球、传球连线和决策状态。
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle, Wedge
from matplotlib.lines import Line2D

from common.config import (
    FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH, GOAL_DEPTH,
    GOAL_X, OUR_GOAL_X, ROBOT_RADIUS, BALL_RADIUS,
    GOAL_POST_WIDTH, CIRCLE_RADIUS,
    PENALTY_AREA_LENGTH, PENALTY_AREA_WIDTH,
    GOAL_AREA_LENGTH, GOAL_AREA_WIDTH,
)
from common.world_state import WorldState, Team
from decision.decision_fsm import DecisionFSM, DecisionState


# 状态颜色
STATE_COLORS = {
    DecisionState.IDLE: "#888888",
    DecisionState.CHASE: "#2196F3",
    DecisionState.DRIBBLE: "#FF9800",
    DecisionState.PASS: "#4CAF50",
    DecisionState.SHOOT: "#E91E63",
    DecisionState.BLOCK: "#9C27B0",
}


class MatplotlibVisualizer:
    """Matplotlib 实时 2D 可视化"""

    def __init__(self, title: str = "Booster T1 Soccer Sim", headless: bool = False,
                 save_gif: Optional[str] = None,
                 frame_skip: int = 3):
        self.title = title
        self.save_gif = save_gif
        self.frame_skip = max(1, frame_skip)
        self._frame_count = 0
        self._ball_trail: List[Tuple[float, float]] = []
        self._trail_max = 40
        self.headless = headless
        self._gif_frames: List = []

        if not self.headless:
            plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(11, 7.5))
        if not self.headless:
            self.fig.canvas.manager.set_window_title(title)

    def render(self, ws: WorldState, fsm: DecisionFSM):
        """渲染当前帧"""
        self._frame_count += 1

        # 记录球轨迹
        self._ball_trail.append((ws.ball.x, ws.ball.y))
        if len(self._ball_trail) > self._trail_max:
            self._ball_trail.pop(0)

        self.ax.clear()
        self._draw_field()
        self._draw_ball_trail()
        self._draw_pass_lines(ws, fsm)
        self._draw_robots(ws, fsm)
        self._draw_ball(ws)
        self._draw_hud(ws, fsm)

        self.ax.set_xlim(-FIELD_WIDTH / 2 - 0.8, FIELD_WIDTH / 2 + 0.8)
        self.ax.set_ylim(-FIELD_HEIGHT / 2 - 0.5, FIELD_HEIGHT / 2 + 0.5)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        self.fig.canvas.draw_idle()
        if not self.headless:
            plt.pause(0.001)

        if self.save_gif and self._frame_count % self.frame_skip == 0:
            self.fig.canvas.draw()
            img = self._fig_to_array()
            if img is not None:
                self._gif_frames.append(img)

    def close(self):
        """关闭窗口，可选导出 GIF"""
        if self.save_gif and self._gif_frames:
            self._export_gif(self.save_gif)
        plt.ioff()
        plt.close(self.fig)

    # ------------------------------------------------------------------

    def _draw_field(self):
        hw, hh = FIELD_WIDTH / 2, FIELD_HEIGHT / 2

        # 草地
        self.ax.add_patch(Rectangle(
            (-hw, -hh), FIELD_WIDTH, FIELD_HEIGHT,
            facecolor="#2E7D32", edgecolor="white", linewidth=2, zorder=0
        ))

        # 中线
        self.ax.plot([0, 0], [-hh, hh], color="white", linewidth=1.5,
                     alpha=0.7, zorder=1)

        # 中圈
        self.ax.add_patch(Circle((0, 0), CIRCLE_RADIUS, fill=False,
                               edgecolor="white", linewidth=1, alpha=0.6, zorder=1))

        # 罚球区 & 球门区
        for gx, color in [(OUR_GOAL_X, "#1565C0"), (GOAL_X, "#F9A825")]:
            sign = 1 if gx < 0 else -1
            # 罚球区
            pa_x = gx + sign * PENALTY_AREA_LENGTH if gx < 0 else gx
            self.ax.add_patch(Rectangle(
                (pa_x, -PENALTY_AREA_WIDTH / 2),
                PENALTY_AREA_LENGTH * sign if gx < 0 else sign * PENALTY_AREA_LENGTH,
                PENALTY_AREA_WIDTH,
                facecolor="none", edgecolor=color, linewidth=1,
                linestyle="--", alpha=0.5, zorder=1
            ))
            # 球门区
            ga_x = gx + sign * GOAL_AREA_LENGTH if gx < 0 else gx
            self.ax.add_patch(Rectangle(
                (ga_x, -GOAL_AREA_WIDTH / 2),
                GOAL_AREA_LENGTH * sign if gx < 0 else sign * GOAL_AREA_LENGTH,
                GOAL_AREA_WIDTH,
                facecolor="none", edgecolor=color, linewidth=1,
                linestyle=":", alpha=0.4, zorder=1
            ))

        # 球门
        for gx, color in [(OUR_GOAL_X, "#1565C0"), (GOAL_X, "#F9A825")]:
            self.ax.add_patch(Rectangle(
                (gx - GOAL_DEPTH if gx < 0 else gx, -GOAL_WIDTH / 2),
                GOAL_DEPTH, GOAL_WIDTH,
                facecolor=color, edgecolor="white", alpha=0.5, zorder=1
            ))
            # 门柱 (左右各一根)
            for post_y in [-GOAL_WIDTH / 2, GOAL_WIDTH / 2]:
                self.ax.add_patch(Rectangle(
                    (gx - GOAL_POST_WIDTH / 2, post_y - GOAL_POST_WIDTH / 2),
                    GOAL_POST_WIDTH, GOAL_POST_WIDTH,
                    facecolor="white", edgecolor="#333333",
                    linewidth=1, zorder=2
                ))

    def _draw_ball_trail(self):
        if len(self._ball_trail) < 2:
            return
        xs, ys = zip(*self._ball_trail)
        self.ax.plot(xs, ys, color="#FFEB3B", linewidth=1.5, alpha=0.5, zorder=2)

    def _draw_pass_lines(self, ws: WorldState, fsm: DecisionFSM):
        """绘制传球箭头：持球者 PASS 状态 → 接球队友"""
        for robot in ws.teammates:
            if fsm.get_state(robot.id) != DecisionState.PASS:
                continue
            target_id = fsm.get_pass_target_id(robot.id)
            if target_id is None:
                continue
            receiver = ws.get_robot_by_id(target_id)
            if receiver is None:
                continue

            arrow = FancyArrowPatch(
                (robot.x, robot.y), (receiver.x, receiver.y),
                arrowstyle="-|>", mutation_scale=18,
                color="#76FF03", linewidth=3, linestyle="--",
                alpha=0.9, zorder=4
            )
            self.ax.add_patch(arrow)

            # 接球区域标记
            self.ax.add_patch(Circle(
                (receiver.x, receiver.y), 0.35,
                fill=False, edgecolor="#76FF03", linewidth=2,
                linestyle=":", zorder=4
            ))
            self.ax.text(receiver.x, receiver.y + 0.45,
                         f"Receiver #{target_id}",
                         ha="center", fontsize=8, color="#76FF03",
                         fontweight="bold", zorder=6)

    def _draw_robots(self, ws: WorldState, fsm: DecisionFSM):
        for robot in ws.all_robots():
            is_blue = robot.team == Team.BLUE
            base_color = "#1976D2" if is_blue else "#FBC02D"
            edge_color = "white"

            state = fsm.get_state(robot.id) if is_blue else None
            if state and state in STATE_COLORS:
                edge_color = STATE_COLORS[state]

            # 机器人圆
            self.ax.add_patch(Circle(
                (robot.x, robot.y), ROBOT_RADIUS,
                facecolor=base_color, edgecolor=edge_color,
                linewidth=2.5, zorder=5
            ))

            # 朝向箭头
            dx = math.cos(robot.theta) * ROBOT_RADIUS * 1.8
            dy = math.sin(robot.theta) * ROBOT_RADIUS * 1.8
            self.ax.annotate(
                "", xy=(robot.x + dx, robot.y + dy),
                xytext=(robot.x, robot.y),
                arrowprops=dict(arrowstyle="-|>", color="white", lw=1.5),
                zorder=6
            )

            # 标签
            label = f"#{robot.id}"
            if is_blue and state:
                label += f"\n{state.value}"
            self.ax.text(robot.x, robot.y - 0.35, label,
                         ha="center", va="top", fontsize=7,
                         color="white", fontweight="bold", zorder=6)

    def _draw_ball(self, ws: WorldState):
        b = ws.ball
        self.ax.add_patch(Circle(
            (b.x, b.y), BALL_RADIUS * 3,
            facecolor="white", edgecolor="black", linewidth=1, zorder=7
        ))

        # 球速箭头
        if b.speed > 0.05:
            scale = min(b.speed * 0.5, 1.0)
            self.ax.annotate(
                "", xy=(b.x + b.vx * scale, b.y + b.vy * scale),
                xytext=(b.x, b.y),
                arrowprops=dict(arrowstyle="-|>", color="#FF5722", lw=2),
                zorder=7
            )

    def _draw_hud(self, ws: WorldState, fsm: DecisionFSM):
        self.ax.set_title(
            f"{self.title}  |  t={ws.timestamp:.1f}s  tick={fsm.tick_count}",
            fontsize=12, fontweight="bold", color="white",
            bbox=dict(boxstyle="round", facecolor="#1B5E20", alpha=0.8)
        )

        # 图例
        legend_items = [
            Line2D([0], [0], marker="o", color="w", markerfacecolor="#1976D2",
                   markersize=8, label="Blue (us)"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="#FBC02D",
                   markersize=8, label="Yellow (opponent)"),
            Line2D([0], [0], color="#76FF03", linewidth=2, linestyle="--",
                   label="Pass line"),
        ]
        for state, color in [
            (DecisionState.CHASE, STATE_COLORS[DecisionState.CHASE]),
            (DecisionState.PASS, STATE_COLORS[DecisionState.PASS]),
            (DecisionState.BLOCK, STATE_COLORS[DecisionState.BLOCK]),
        ]:
            legend_items.append(
                Line2D([0], [0], marker="o", color="w", markeredgecolor=color,
                       markerfacecolor="none", markeredgewidth=2,
                       markersize=8, label=state.value)
            )
        self.ax.legend(handles=legend_items, loc="upper right",
                       fontsize=7, framealpha=0.85)

        # 最近状态转换
        if fsm.transitions:
            lines = []
            for t in fsm.transitions[-3:]:
                lines.append(
                    f"R{t.robot_id}: {t.from_state.value}->{t.to_state.value}"
                )
            self.ax.text(
                -FIELD_WIDTH / 2 + 0.1, FIELD_HEIGHT / 2 - 0.15,
                "\n".join(lines),
                fontsize=7, color="white", va="top",
                bbox=dict(boxstyle="round", facecolor="black", alpha=0.5),
                zorder=8
            )

    def _fig_to_array(self):
        try:
            self.fig.canvas.draw()
            buf = self.fig.canvas.buffer_rgba()
            import numpy as np
            w, h = self.fig.canvas.get_width_height()
            return np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)
        except Exception:
            return None

    def _export_gif(self, path: str):
        try:
            from PIL import Image
            import os
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            images = [Image.fromarray(f[:, :, :3]) for f in self._gif_frames]
            if not images:
                return
            duration_ms = int(1000 / 15)  # ~15 fps
            images[0].save(
                path, save_all=True, append_images=images[1:],
                duration=duration_ms, loop=0
            )
            print(f"\n  GIF saved: {path} ({len(images)} frames)")
        except Exception as e:
            print(f"\n  Warning: GIF export failed: {e}")
