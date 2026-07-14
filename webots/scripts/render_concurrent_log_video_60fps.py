#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
from bisect import bisect_right
from pathlib import Path

import cv2
import numpy as np


ROBOTS = ["T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"]
COLORS = {
    "T1_BLUE_1": (40, 120, 255),
    "T1_BLUE_2": (80, 180, 255),
    "T1_RED_1": (255, 70, 70),
    "T1_RED_2": (255, 130, 80),
}


def read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def interp_series(samples: list[tuple[float, np.ndarray]], t: float) -> np.ndarray:
    if not samples:
        return np.zeros(2, dtype=np.float32)
    times = [item[0] for item in samples]
    idx = bisect_right(times, t)
    if idx <= 0:
        return samples[0][1]
    if idx >= len(samples):
        return samples[-1][1]
    t0, v0 = samples[idx - 1]
    t1, v1 = samples[idx]
    alpha = (t - t0) / max(1e-9, t1 - t0)
    return v0 + (v1 - v0) * alpha


def to_px(x: float, y: float, width: int, height: int) -> tuple[int, int]:
    px = int((x + 3.5) / 7.0 * width)
    py = int((2.4 - y) / 4.8 * height)
    return px, py


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    run_dir = args.run_dir
    summary = json.loads((run_dir / "summary.json").read_text())
    duration = float(summary["simulation_time"])

    robot_samples = {robot: [] for robot in ROBOTS}
    for item in read_jsonl(run_dir / "robot_states.jsonl"):
        t = float(item["t"])
        for robot, xy in item["robots"].items():
            robot_samples[robot].append((t, np.asarray(xy, dtype=np.float32)))
    ball_samples = [(float(item["t"]), np.asarray([item["x"], item["y"]], dtype=np.float32)) for item in read_jsonl(run_dir / "ball_motion.jsonl")]

    out = run_dir / "demo_60fps.mp4"
    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s:v",
        f"{args.width}x{args.height}",
        "-r",
        str(args.fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        str(out),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    total_frames = int(round(duration * args.fps))
    trails = {robot: [] for robot in ROBOTS}
    for frame_idx in range(total_frames):
        t = frame_idx / args.fps
        frame = np.full((args.height, args.width, 3), (44, 122, 72), dtype=np.uint8)
        cv2.rectangle(frame, to_px(-3.35, -2.25, args.width, args.height), to_px(3.35, 2.25, args.width, args.height), (230, 245, 235), 3)
        cv2.line(frame, to_px(0, -2.25, args.width, args.height), to_px(0, 2.25, args.width, args.height), (210, 235, 220), 2)
        cv2.circle(frame, to_px(0, 0, args.width, args.height), 70, (210, 235, 220), 2)
        cv2.rectangle(frame, to_px(-3.5, -0.55, args.width, args.height), to_px(-3.35, 0.55, args.width, args.height), (255, 255, 255), 3)
        cv2.rectangle(frame, to_px(3.35, -0.55, args.width, args.height), to_px(3.5, 0.55, args.width, args.height), (255, 255, 255), 3)
        ball = interp_series(ball_samples, t)
        for robot in ROBOTS:
            xy = interp_series(robot_samples[robot], t)
            trails[robot].append(tuple(xy))
            trails[robot] = trails[robot][-36:]
            pts = [to_px(float(p[0]), float(p[1]), args.width, args.height) for p in trails[robot]]
            for a, b in zip(pts, pts[1:]):
                cv2.line(frame, a, b, COLORS[robot], 2)
            px, py = to_px(float(xy[0]), float(xy[1]), args.width, args.height)
            bx, by = to_px(float(ball[0]), float(ball[1]), args.width, args.height)
            cv2.circle(frame, (px, py), 24, COLORS[robot], -1)
            cv2.circle(frame, (px, py), 26, (255, 255, 255), 2)
            cv2.line(frame, (px, py), (bx, by), COLORS[robot], 1)
            cv2.putText(frame, robot.replace("T1_", "").replace("_", ""), (px - 34, py - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)
        bx, by = to_px(float(ball[0]), float(ball[1]), args.width, args.height)
        cv2.circle(frame, (bx, by), 15, (245, 245, 245), -1)
        cv2.circle(frame, (bx, by), 17, (20, 20, 20), 2)
        cv2.putText(frame, f"MuJoCo Concurrent Match  {t:05.2f}s  60 FPS validation", (32, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        proc.stdin.write(frame.tobytes())
    proc.stdin.close()
    code = proc.wait()
    if code != 0:
        raise SystemExit(f"ffmpeg exited with {code}")
    summary["video_path"] = str(out)
    summary["demo_60fps_path"] = str(out)
    (run_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    provenance = {
        "video_type": "trajectory_replay",
        "source_physics_run_id": summary.get("run_id", run_dir.name),
        "video_run_id": run_dir.name,
        "physics_hz": 200,
        "agent_decision_hz": 20,
        "output_fps": args.fps,
        "trajectory_interpolation": True,
        "direct_realtime_screen_recording": False,
        "source_is_physical_simulation": True,
        "ball_mutation_detected": bool(summary.get("ball_mutation_detected", False)),
        "source_summary": str(run_dir / "summary.json"),
    }
    (run_dir / "video_provenance.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2), encoding="utf-8")
    print(out)


if __name__ == "__main__":
    main()
