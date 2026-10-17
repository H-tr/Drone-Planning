"""Offscreen MP4 recorder for :class:`PyBulletEnv`.

Records MP4 clips from a headless PyBullet session by rendering frames
via ``getCameraImage`` and piping them into ``ffmpeg``.  Works with a
``DIRECT`` connection (no GUI window required), which makes it safe to
use inside build pipelines, CI, or a doc-rendering driver.

Typical usage::

    from drone_planning.utils.video_recorder import CameraView, VideoRecorder

    with VideoRecorder(env, "output.mp4", camera=CameraView()) as rec:
        rec.hold(seconds=0.5)            # pause on the start pose
        rec.play(configurations)         # one frame per configuration
        rec.hold(seconds=1.0)            # pause on the final pose

The recorder assumes ``ffmpeg`` is on ``PATH``.  Frames are rendered with
the CPU ``TinyRenderer``.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pybullet as pb

if TYPE_CHECKING:
    from drone_planning.envs.pybullet_env import PyBulletEnv


@dataclass
class CameraView:
    """Static orbit-camera parameters passed to ``computeViewMatrixFromYawPitchRoll``.

    ``target`` points at a world-space location, ``distance`` is the
    orbit radius, and ``yaw``/``pitch`` orbit around ``target`` in
    degrees.  ``fov`` is the vertical field of view of the projection.
    The camera does not move during a clip.
    """

    target: tuple[float, float, float] = (2.6, 0.0, 1.0)
    distance: float = 6.0
    yaw: float = -35.0
    pitch: float = -25.0
    roll: float = 0.0
    fov: float = 45.0
    near: float = 0.05
    far: float = 30.0


@dataclass
class VideoRecorder:
    """Render frames from a :class:`PyBulletEnv` into an H.264 MP4.

    The recorder owns a running ``ffmpeg`` subprocess while the context is
    open; frames are written directly to its stdin in ``rgba`` format.
    """

    env: "PyBulletEnv"
    output_path: Path | str
    fps: int = 30
    resolution: tuple[int, int] = (960, 540)
    camera: CameraView = field(default_factory=CameraView)
    light_direction: tuple[float, float, float] = (0.55, -0.40, 1.30)
    crf: int = 28

    _proc: subprocess.Popen | None = field(default=None, init=False, repr=False)
    _view: list[float] = field(default_factory=list, init=False, repr=False)
    _proj: list[float] = field(default_factory=list, init=False, repr=False)

    def __enter__(self) -> "VideoRecorder":
        client = self.env.sim.client
        self._view = client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=list(self.camera.target),
            distance=self.camera.distance,
            yaw=self.camera.yaw,
            pitch=self.camera.pitch,
            roll=self.camera.roll,
            upAxisIndex=2,
        )
        w, h = self.resolution
        self._proj = client.computeProjectionMatrixFOV(
            fov=self.camera.fov,
            aspect=w / h,
            nearVal=self.camera.near,
            farVal=self.camera.far,
        )
        Path(self.output_path).parent.mkdir(parents=True, exist_ok=True)
        # fmt: off
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w}x{h}",
            "-r", str(self.fps), "-i", "-",
            "-an", "-vcodec", "libx264", "-preset", "slow", "-crf", str(self.crf),
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(self.output_path),
        ]
        # fmt: on
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._proc.stdin.close()
        self._proc.wait()

    def capture(self) -> None:
        """Render the current env state as a single frame into the clip."""
        w, h = self.resolution
        img = self.env.sim.client.getCameraImage(
            width=w,
            height=h,
            viewMatrix=self._view,
            projectionMatrix=self._proj,
            shadow=1,
            lightDirection=list(self.light_direction),
            renderer=pb.ER_TINY_RENDERER,
        )
        rgba = np.asarray(img[2], dtype=np.uint8).reshape(h, w, 4)
        self._proc.stdin.write(rgba.tobytes())

    def hold(self, seconds: float = 0.5) -> None:
        """Freeze on the current configuration for *seconds*."""
        for _ in range(max(1, int(round(seconds * self.fps)))):
            self.capture()

    def play(self, configurations: np.ndarray) -> None:
        """Capture one frame per full-DOF configuration."""
        for q in configurations:
            self.env.set_configuration(q)
            self.capture()
