from __future__ import annotations

import queue
import shutil
import subprocess
import threading
from pathlib import Path


class AsyncVideoWriter:
    def __init__(self, path: Path, width: int, height: int, input_fps: int, output_fps: int = 30) -> None:
        self.path = path
        self.width = width
        self.height = height
        self.input_fps = input_fps
        self.output_fps = output_fps
        self.queue: queue.Queue[bytes | None] = queue.Queue(maxsize=16)
        self.dropped_frames = 0
        self.frames_written = 0
        self.error: str | None = None
        self._proc: subprocess.Popen[bytes] | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if shutil.which("ffmpeg") is None:
            self.error = "ffmpeg not found"
            return
        vf_args = []
        if self.output_fps != self.input_fps:
            vf_args = ["-vf", f"minterpolate=fps={self.output_fps}:mi_mode=blend"]
        cmd = [
            "ffmpeg",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s:v",
            f"{self.width}x{self.height}",
            "-r",
            str(self.input_fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            *vf_args,
            "-preset",
            "veryfast",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            str(self.path),
        ]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self._thread = threading.Thread(target=self._consume, name="visual-video-writer", daemon=True)
        self._thread.start()

    def write(self, raw_rgb: bytes) -> None:
        if self.error:
            return
        try:
            self.queue.put_nowait(raw_rgb)
        except queue.Full:
            self.dropped_frames += 1

    def close(self) -> None:
        if self._thread is None:
            return
        self.queue.put(None)
        self._thread.join(timeout=30)
        self._thread = None

    def _consume(self) -> None:
        assert self._proc is not None
        assert self._proc.stdin is not None
        while True:
            item = self.queue.get()
            if item is None:
                break
            try:
                self._proc.stdin.write(item)
                self.frames_written += 1
            except Exception as exc:  # noqa: BLE001
                self.error = str(exc)
                break
        self._proc.stdin.close()
        code = self._proc.wait(timeout=30)
        if code != 0 and self.error is None:
            self.error = f"ffmpeg exited with {code}"
