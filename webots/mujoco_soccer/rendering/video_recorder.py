from __future__ import annotations

from pathlib import Path

import mujoco


class VideoRecorder:
    def __init__(self, model: mujoco.MjModel, run_dir: Path, width: int = 1280, height: int = 720, fps: int = 30) -> None:
        self.model = model
        self.run_dir = run_dir
        self.width = width
        self.height = height
        self.fps = fps
        self.frames: list[object] = []
        self.renderer: mujoco.Renderer | None = None
        self.error: str | None = None
        try:
            self.renderer = mujoco.Renderer(model, height=height, width=width)
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)

    def capture(self, data: mujoco.MjData) -> None:
        if self.renderer is None:
            return
        try:
            self.renderer.update_scene(data, camera="overview")
            self.frames.append(self.renderer.render())
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.renderer = None

    def save(self) -> tuple[str | None, str | None]:
        if not self.frames:
            return None, None
        import imageio.v2 as imageio

        final_frame = self.run_dir / "final_frame.png"
        imageio.imwrite(final_frame, self.frames[-1])
        video_path = self.run_dir / "demo.mp4"
        try:
            imageio.mimsave(video_path, self.frames, fps=self.fps)
            return str(final_frame), str(video_path)
        except Exception:
            frames_dir = self.run_dir / "frames"
            frames_dir.mkdir(exist_ok=True)
            for idx, frame in enumerate(self.frames):
                imageio.imwrite(frames_dir / f"frame_{idx:04d}.png", frame)
            return str(final_frame), str(frames_dir)

