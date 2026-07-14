from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import Any

import mujoco

from mujoco_soccer.rendering.async_video_writer import AsyncVideoWriter


KEY_STAGES = {
    "opening_frame": "STAGE_00_READY",
    "all_run_frame": "STAGE_01_VISIBLE_ALL_RUN",
    "dribble_frame": "STAGE_03_BLUE1_DRIBBLE_1",
    "pass_frame": "STAGE_06_BLUE1_PASS",
    "receive_frame": "STAGE_07_BLUE2_RECEIVE",
    "shoot_frame": "STAGE_08_BLUE2_SHOOT",
    "clear_frame": "STAGE_09_RED1_INTERCEPT_CLEAR",
    "counter_frame": "STAGE_10_RED2_COUNTER",
}


class VisualV2Recorder:
    def __init__(
        self,
        model: mujoco.MjModel,
        run_dir: Path,
        width: int = 1280,
        height: int = 720,
        fps: int = 8,
        output_fps: int = 30,
        camera: str = "broadcast",
        enabled: bool = True,
        label: str = "visual_v2",
        async_write: bool = False,
        constrain_duration: bool = True,
    ) -> None:
        self.model = model
        self.run_dir = run_dir
        self.width = width
        self.height = height
        self.fps = fps
        self.output_fps = output_fps
        self.camera = camera
        self.enabled = enabled
        self.label = label
        self.async_write = async_write
        self.renderer: mujoco.Renderer | None = None
        self.ffmpeg: subprocess.Popen[bytes] | None = None
        self.async_writer: AsyncVideoWriter | None = None
        self.error: str | None = None
        self.video_path = run_dir / f"demo_{label}.mp4"
        self.key_frames: dict[str, Path] = {}
        self.last_ppm: Path | None = None
        self.last_raw: bytes | None = None
        self.frames_written = 0
        self.video_duration_seconds: float | None = None
        self.video_resolution = f"{width}x{height}"
        self.video_fps = output_fps
        self.constrain_duration = constrain_duration
        if not enabled:
            return
        try:
            self.renderer = mujoco.Renderer(model, height=height, width=width)
            if async_write:
                self.async_writer = AsyncVideoWriter(self.video_path, width, height, fps, output_fps)
                self.async_writer.start()
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)

    def capture(self, data: mujoco.MjData, stage: str) -> None:
        if not self.enabled or self.renderer is None:
            return
        try:
            self.renderer.update_scene(data, camera=self.camera)
            frame: Any = self.renderer.render()
            raw = frame.tobytes()
            if self.async_writer is not None:
                self.async_writer.write(raw)
                self.frames_written += 1
            else:
                self._ensure_video_pipe()
            if self.async_writer is None and self.ffmpeg and self.ffmpeg.stdin:
                self.ffmpeg.stdin.write(raw)
                self.frames_written += 1
            self._capture_key(stage, raw)
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self._close_pipe()

    def _ensure_video_pipe(self) -> None:
        if self.ffmpeg is not None:
            return
        if shutil.which("ffmpeg") is None:
            self.error = "ffmpeg not found"
            return
        vf_args = []
        if self.output_fps != self.fps:
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
            str(self.fps),
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
            str(self.video_path),
        ]
        self.ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def _capture_key(self, stage: str, raw: bytes) -> None:
        for key, marker in KEY_STAGES.items():
            if key in self.key_frames:
                continue
            if marker in stage:
                ppm = self.run_dir / f"{key}.ppm"
                self._write_ppm(ppm, raw)
                png = self.run_dir / f"{key}.png"
                self._ppm_to_png(ppm, png)
                self.key_frames[key] = png
        self.last_raw = raw

    def _write_ppm(self, path: Path, raw: bytes) -> None:
        with path.open("wb") as handle:
            handle.write(f"P6\n{self.width} {self.height}\n255\n".encode("ascii"))
            handle.write(raw)

    def _ppm_to_png(self, ppm: Path, png: Path) -> None:
        if shutil.which("ffmpeg") is None:
            return
        subprocess.run(
            ["ffmpeg", "-loglevel", "error", "-y", "-i", str(ppm), str(png)],
            check=False,
        )

    def _close_pipe(self) -> None:
        if self.ffmpeg is None:
            return
        if self.ffmpeg.stdin:
            self.ffmpeg.stdin.close()
        code = self.ffmpeg.wait(timeout=30)
        if code != 0 and self.error is None:
            self.error = f"ffmpeg exited with {code}"
        self.ffmpeg = None

    def save(self) -> tuple[str | None, str | None]:
        if self.async_writer is not None:
            self.async_writer.close()
            if self.async_writer.error and self.error is None:
                self.error = self.async_writer.error
        else:
            self._close_pipe()
        final_png = self.run_dir / f"final_frame_{self.label}.png"
        if self.last_raw is not None:
            self.last_ppm = self.run_dir / f"final_frame_{self.label}.ppm"
            self._write_ppm(self.last_ppm, self.last_raw)
        if self.last_ppm is not None:
            self._ppm_to_png(self.last_ppm, final_png)
        if final_png.exists():
            for missing in KEY_STAGES:
                self.key_frames.setdefault(missing, final_png)
        self._make_contact_sheet()
        video = self.video_path if self.video_path.exists() else None
        if video is not None:
            if self.constrain_duration:
                self._constrain_video_duration(video)
            self.video_duration_seconds = self._probe_duration(video)
        return (str(final_png) if final_png.exists() else None, str(video) if video else None)

    def _probe_duration(self, video: Path) -> float | None:
        if shutil.which("ffprobe") is None:
            return None
        result = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-show_entries",
                "format=duration",
                "-of",
                "default=noprint_wrappers=1:nokey=1",
                str(video),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        try:
            return float(result.stdout.strip())
        except ValueError:
            return None

    def _constrain_video_duration(self, video: Path) -> None:
        duration = self._probe_duration(video)
        if duration is None or duration <= 55.0 or shutil.which("ffmpeg") is None:
            return
        target = 53.0
        factor = max(0.70, min(0.98, target / duration))
        raw = video.with_name(video.stem + "_raw_%.0fs.mp4" % duration)
        video.rename(raw)
        subprocess.run(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-y",
                "-i",
                str(raw),
                "-filter:v",
                f"setpts={factor:.6f}*PTS,fps={self.output_fps}",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                str(video),
            ],
            check=False,
        )
        if not video.exists():
            raw.rename(video)

    def _make_contact_sheet(self) -> None:
        if shutil.which("ffmpeg") is None:
            return
        ordered = list(KEY_STAGES)
        images = [self.key_frames.get(key) for key in ordered]
        if not images or any(path is None or not path.exists() for path in images):
            return
        cmd = ["ffmpeg", "-loglevel", "error", "-y"]
        for path in images:
            cmd.extend(["-i", str(path)])
        filters = []
        for idx in range(len(images)):
            filters.append(f"[{idx}:v]scale=320:180[v{idx}]")
        layout = "0_0|320_0|640_0|960_0|0_180|320_180|640_180|960_180"
        filters.append("".join(f"[v{idx}]" for idx in range(len(images))) + f"xstack=inputs=8:layout={layout}:fill=black[out]")
        cmd.extend(["-filter_complex", ";".join(filters), "-map", "[out]", str(self.run_dir / f"contact_sheet_{self.label}.png")])
        subprocess.run(cmd, check=False)
