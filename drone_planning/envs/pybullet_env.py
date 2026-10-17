import os
import re
import tempfile
import time
import xml.etree.ElementTree as ET
from typing import Any

import numpy as np
import pybullet as pb

from drone_planning.envs.base_env import BaseEnv
from drone_planning.types import RobotConfig
from drone_planning.utils import pybullet_interface as vpb


def _prepare_urdf_for_pybullet(urdf_path: str) -> str:
    """Resolve ``package://`` URIs to absolute mesh paths.

    PyBullet resolves ``package://`` relative to the URDF file's directory,
    which breaks for ROS-style ``package://pkg_name/...`` references — so
    this helper rewrites every ``<mesh filename="...">`` in the URDF to an
    absolute path, then writes the result to a temporary URDF file.
    """
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    pkg_re = re.compile(r"package://(.+)")

    tree = ET.parse(urdf_path)
    for mesh in tree.getroot().iter("mesh"):
        m = pkg_re.match(mesh.get("filename", ""))
        if m:
            mesh.set("filename", os.path.join(urdf_dir, m.group(1)))

    tmp = tempfile.NamedTemporaryFile(
        suffix=".urdf", prefix="viz_", delete=False, mode="wb"
    )
    tree.write(tmp.name, encoding="utf-8", xml_declaration=True)
    tmp.close()
    return tmp.name


class PyBulletEnv(BaseEnv):
    """PyBullet scene with the X500.

    The URDF carries the body pose as joints (``base_x_joint`` …
    ``base_yaw_joint`` from ``world``), so a full 6-DOF configuration is
    applied by setting joint positions.
    """

    def __init__(self, config: RobotConfig, visualize: bool = True):
        self.config = config
        urdf = _prepare_urdf_for_pybullet(config.urdf_path)
        self.sim = vpb.PyBulletSimulator(urdf, config.joint_names, visualize=visualize)
        self.joint_names = config.joint_names
        self.camera_link_idx = self.sim.link_map[config.camera.link_name]

        from drone_planning.x500 import HOME_JOINTS

        self.set_joint_states(HOME_JOINTS)

    def get_joint_states(self) -> np.ndarray:
        states = []
        for joint_idx in self.sim.joints:
            state = self.sim.client.getJointState(self.sim.skel_id, joint_idx)
            states.append(state[0])
        return np.array(states)

    def set_joint_states(self, config: np.ndarray):
        self.sim.set_joint_positions(np.asarray(config))

    def set_configuration(self, config: np.ndarray):
        """Apply a full 6-DOF config (4 body + 2 gimbal) to the visualization."""
        self.set_joint_states(config)

    def get_localization(self) -> np.ndarray:
        return self.get_joint_states()[:4]

    def get_rgbd(self):
        ls = self.sim.client.getLinkState(self.sim.skel_id, self.camera_link_idx)
        pos = np.asarray(ls[0])
        rot_mat = np.array(self.sim.client.getMatrixFromQuaternion(ls[1])).reshape(3, 3)

        # Camera sensor frame: X is the optical axis, Z is up (Gazebo).
        forward = rot_mat[:, 0]
        up = rot_mat[:, 2]

        view_matrix = self.sim.client.computeViewMatrix(
            cameraEyePosition=pos, cameraTargetPosition=pos + forward, cameraUpVector=up
        )

        width = self.config.camera.width
        height = self.config.camera.height
        proj_matrix = self.sim.client.computeProjectionMatrixFOV(
            self.config.camera.fov,
            width / height,
            self.config.camera.near,
            self.config.camera.far,
        )

        img = self.sim.client.getCameraImage(width, height, view_matrix, proj_matrix)

        # img[2] is RGB, img[3] is Depth
        rgb = np.array(img[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
        depth = np.array(img[3], dtype=np.float32).reshape(height, width)

        return {"rgb": rgb, "depth": depth}

    def get_obs(self) -> Any:
        return {
            "joint_states": self.get_joint_states(),
            "localization": self.get_localization(),
            "camera": self.get_rgbd(),
        }

    def step(self):
        self.sim.client.stepSimulation()

    def wait_key(self, key: str | int, message: str = "") -> None:
        """Block until *key* is pressed in the PyBullet GUI.

        Prints *message* to stdout, then polls keyboard events until the
        requested key is triggered.  Accepts either a single character
        (``"n"``) or a raw PyBullet key code (e.g. ``pb.B3G_RIGHT_ARROW``).
        Returns early if the GUI window is closed.
        """
        client = self.sim.client
        key_code = ord(key) if isinstance(key, str) else int(key)
        if message:
            print(message)
        while client.isConnected():
            keys = client.getKeyboardEvents()
            if key_code in keys and keys[key_code] & pb.KEY_WAS_TRIGGERED:
                break
            time.sleep(0.01)

    def wait_for_close(self) -> None:
        """Block until the user closes the PyBullet GUI window."""
        client = self.sim.client
        while client.isConnected():
            client.getKeyboardEvents()  # keeps the event queue alive
            time.sleep(0.05)

    def animate_path(
        self,
        path: np.ndarray,
        fps: float = 60.0,
        next_key: str | int | None = None,
    ) -> bool:
        """Interactively play back a full-DOF path, VAMP-style.

        Each row of *path* is handed to :meth:`set_configuration`.  The
        method blocks on the PyBullet GUI with these controls:

        * ``SPACE``             — toggle auto-play (loops end → start)
        * ``← / →``             — step one waypoint back / forward (while paused)
        * ``next_key`` (opt.)   — exit and return ``True`` — useful when
          the caller wants to drive a sequence of demos

        Exits when the user closes the GUI window (returns ``False``) or
        presses *next_key* if one is provided (returns ``True``).  The
        animation starts paused so the caller can read the banner before
        anything moves.

        Args:
          path: ``(N, dof)`` array of full-DOF configurations.
          fps:  playback frame rate while auto-playing.
          next_key: optional single-character string or raw PyBullet key
            code; pressing it exits the viewer.

        Returns:
          ``True`` if *next_key* was pressed, ``False`` otherwise.
        """
        client = self.sim.client
        dt = 1.0 / fps
        n = int(len(path))
        idx = 0
        playing = False

        left = pb.B3G_LEFT_ARROW
        right = pb.B3G_RIGHT_ARROW
        space = ord(" ")
        next_code: int | None = None
        if next_key is not None:
            next_code = ord(next_key) if isinstance(next_key, str) else int(next_key)

        parts = ["SPACE play/pause", "←/→ step"]
        if next_code is not None:
            parts.append(f"'{next_key}' next")
        parts.append("close window to exit")
        print("  " + "  |  ".join(parts))

        while client.isConnected():
            self.set_configuration(path[idx])

            keys = client.getKeyboardEvents()
            if space in keys and keys[space] & pb.KEY_WAS_TRIGGERED:
                playing = not playing
            elif (
                next_code is not None
                and next_code in keys
                and keys[next_code] & pb.KEY_WAS_TRIGGERED
            ):
                return True
            elif not playing and left in keys and keys[left] & pb.KEY_WAS_TRIGGERED:
                idx = (idx - 1) % n
            elif not playing and right in keys and keys[right] & pb.KEY_WAS_TRIGGERED:
                idx = (idx + 1) % n
            elif playing:
                idx = (idx + 1) % n

            time.sleep(dt)
        return False

    def add_pointcloud(
        self, points: np.ndarray, lifetime: float = 0.0, pointsize: int = 3
    ):
        self.sim.draw_pointcloud(points, lifetime, pointsize)

    # ── scene-setup primitives ────────────────────────────────────────

    def draw_box(
        self,
        center,
        half_extents,
        color: tuple[float, float, float, float] = (0.75, 0.75, 0.78, 1.0),
    ) -> int:
        """Visual-only axis-aligned box — e.g. a wall of the scene."""
        client = self.sim.client
        vid = client.createVisualShape(
            shapeType=pb.GEOM_BOX,
            halfExtents=[float(h) for h in half_extents],
            rgbaColor=list(color),
        )
        return client.createMultiBody(
            baseVisualShapeIndex=vid,
            basePosition=list(np.asarray(center, dtype=float)),
        )
