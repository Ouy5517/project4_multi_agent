from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco


@dataclass
class CleanVisualViewer:
    model: mujoco.MjModel
    data: mujoco.MjData
    camera: str = "broadcast"
    width: int = 1280
    height: int = 720

    def __post_init__(self) -> None:
        self.renderer: mujoco.Renderer | None = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.cv2: Any | None = None
        self.window_opened = False
        try:
            import cv2  # type: ignore[import-not-found]

            self.cv2 = cv2
            cv2.namedWindow("MuJoCo Soccer Visual V2", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("MuJoCo Soccer Visual V2", self.width, self.height)
            self.window_opened = True
        except Exception:  # noqa: BLE001
            self.cv2 = None
            self.window_opened = False

    @property
    def available(self) -> bool:
        return self.cv2 is not None and self.renderer is not None and self.window_opened

    def sync(self) -> str | None:
        if not self.available or self.cv2 is None or self.renderer is None:
            return None
        self.renderer.update_scene(self.data, camera=self.camera)
        frame = self.renderer.render()
        self.cv2.imshow("MuJoCo Soccer Visual V2", frame[:, :, ::-1])
        key = self.cv2.waitKey(1) & 0xFF
        if key == 27:
            return "quit"
        if key == ord(" "):
            return "pause"
        if key == ord("r"):
            return "restart"
        if key == ord("c"):
            return "camera"
        return None

    def close(self) -> None:
        if self.cv2 is not None:
            self.cv2.destroyWindow("MuJoCo Soccer Visual V2")
