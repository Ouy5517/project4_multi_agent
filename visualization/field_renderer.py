from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

from common.config import FIELD_HEIGHT, FIELD_WIDTH, GOAL_WIDTH


def _draw_field(ax):
    ax.set_xlim(-FIELD_WIDTH / 2 - 0.5, FIELD_WIDTH / 2 + 0.5)
    ax.set_ylim(-FIELD_HEIGHT / 2 - 0.5, FIELD_HEIGHT / 2 + 0.5)
    ax.set_aspect("equal")
    ax.set_facecolor("#1f7a4d")
    ax.plot(
        [-FIELD_WIDTH / 2, FIELD_WIDTH / 2, FIELD_WIDTH / 2, -FIELD_WIDTH / 2, -FIELD_WIDTH / 2],
        [-FIELD_HEIGHT / 2, -FIELD_HEIGHT / 2, FIELD_HEIGHT / 2, FIELD_HEIGHT / 2, -FIELD_HEIGHT / 2],
        color="white",
        linewidth=1.5,
    )
    ax.axvline(0, color="white", linewidth=1)
    ax.plot([FIELD_WIDTH / 2, FIELD_WIDTH / 2], [-GOAL_WIDTH / 2, GOAL_WIDTH / 2], color="#ffdd55", linewidth=4)
    ax.plot([-FIELD_WIDTH / 2, -FIELD_WIDTH / 2], [-GOAL_WIDTH / 2, GOAL_WIDTH / 2], color="#ffdd55", linewidth=4)
    for x in (-3, -1.5, 1.5, 3):
        ax.axvline(x, color="white", linewidth=0.35, alpha=0.25)
    for y in (-2, -1, 1, 2):
        ax.axhline(y, color="white", linewidth=0.35, alpha=0.25)
    ax.set_xticks([])
    ax.set_yticks([])


def _read_rows(trajectory: Path):
    with Path(trajectory).open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def render_png_from_trajectory(trajectory: str | Path, output: str | Path) -> None:
    rows = _read_rows(Path(trajectory))
    if not rows:
        raise ValueError("trajectory is empty")
    first_tick = rows[0]["tick"]
    frame_rows = [row for row in rows if row["tick"] == first_tick]
    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    _draw_field(ax)
    _draw_entities(ax, frame_rows)
    fig.tight_layout(pad=0.2)
    fig.savefig(output)
    plt.close(fig)


def render_mp4_from_trajectory(trajectory: str | Path, output: str | Path, fps: int = 30) -> None:
    rows = _read_rows(Path(trajectory))
    ticks = []
    by_tick = {}
    for row in rows:
        tick = row["tick"]
        if tick not in by_tick:
            ticks.append(tick)
            by_tick[tick] = []
        by_tick[tick].append(row)

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=1800)
    with writer.saving(fig, str(output), dpi=100):
        for tick in ticks:
            ax.clear()
            _draw_field(ax)
            _draw_entities(ax, by_tick[tick])
            writer.grab_frame()
    plt.close(fig)


def _draw_entities(ax, rows):
    for row in rows:
        x = float(row["x"])
        y = float(row["y"])
        if row["entity"] == "ball":
            ax.scatter([x], [y], s=120, c="#f7f7f7", edgecolors="#111111", linewidths=1.5, zorder=5)
        elif row["entity"] == "robot":
            robot_id = row["robot_id"]
            is_blue = robot_id and int(robot_id) < 10
            color = "#1f77ff" if is_blue else "#ffcc33"
            ax.scatter([x], [y], s=260, c=color, edgecolors="#111111", linewidths=1.2, zorder=4)
            ax.text(x, y + 0.22, f"{robot_id} {row.get('state', '')}", ha="center", va="bottom", fontsize=8, color="white")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.suffix.lower() == ".png":
        render_png_from_trajectory(args.trajectory, output)
    else:
        render_mp4_from_trajectory(args.trajectory, output)


if __name__ == "__main__":
    main()
