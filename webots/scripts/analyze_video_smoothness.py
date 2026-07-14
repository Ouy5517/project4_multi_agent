#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: python scripts/analyze_video_smoothness.py <video.mp4>")
    path = Path(sys.argv[1])
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"failed to open {path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    previous: np.ndarray | None = None
    duplicate_frames = 0
    longest_duplicate_run = 0
    current_duplicate_run = 0
    changed_frames = 0
    sampled = 0
    diffs: list[float] = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (320, 180), interpolation=cv2.INTER_AREA)
        if previous is not None:
            diff = float(np.mean(cv2.absdiff(previous, small)))
            diffs.append(diff)
            if diff < 0.005:
                duplicate_frames += 1
                current_duplicate_run += 1
                longest_duplicate_run = max(longest_duplicate_run, current_duplicate_run)
            else:
                changed_frames += 1
                current_duplicate_run = 0
        previous = small
        sampled += 1
    cap.release()
    comparable = max(1, sampled - 1)
    duplicate_ratio = duplicate_frames / comparable
    active_change_ratio = changed_frames / comparable
    report = {
        "video_path": str(path),
        "fps": fps,
        "frame_count": frame_count or sampled,
        "sampled_frames": sampled,
        "duplicate_frame_ratio": duplicate_ratio,
        "longest_consecutive_duplicate_frames": longest_duplicate_run,
        "active_change_ratio": active_change_ratio,
        "mean_frame_diff": sum(diffs) / max(1, len(diffs)),
        "p05_frame_diff": sorted(diffs)[int(0.05 * (len(diffs) - 1))] if diffs else 0.0,
        "video_smoothness_success": fps >= 59.0 and duplicate_ratio < 0.05 and longest_duplicate_run <= 2,
    }
    out = path.with_name("video_smoothness_report.json")
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
