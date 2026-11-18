"""PHANTOM's drone: the bundled description.

This module collects every concrete value that describes the second
robot this project ships: joint groupings, the home pose, the VAMP
planning subgroups, the kinodynamic limits and the top-level
``RobotConfig`` instance.  It has the layout of :mod:`drone_planning.x500`.

The model is the drone of the PHANTOM planner (H-tr/PHANTOM-Planner,
``phantom/robot/drone.urdf``): a stabilized multirotor.  Its flight
controller holds roll and pitch, so the body is position and yaw.  The
collision model is ONE sphere of radius 0.30 m at the body centre.  A
camera hangs 6 cm below the body centre on a pan-tilt gimbal.  The full
configuration vector is 6-DOF, in PHANTOM's order:

* ``[0:4]`` — body ``(x, y, z, yaw)``: virtual joints in the world frame
  (z up, yaw counter-clockwise from +x)
* ``[4:6]`` — gimbal: pan (``camera_pan``) and tilt (``camera_tilt``)

Positive pan turns the camera left (counter-clockwise seen from above),
positive tilt raises it, and zero looks straight ahead.  The gimbal links
have no collision geometry, so the gimbal angles never change whether a
configuration collides.
"""

import math
import os

import numpy as np

from drone_planning.types.robot import CameraConfig, RobotConfig

_PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
_RESOURCES_DIR = os.path.join(_PKG_ROOT, "resources", "robot", "drone")

# The robot name of ``OmplVampPlanner(ROBOT_NAME)`` and ``create_planner``.
ROBOT_NAME = "drone"

# Atomic joint groups — indices into the full 6-DOF configuration array.
# Order matches VAMP's URDF tree traversal: body → gimbal.
JOINT_GROUPS = {
    "base": slice(0, 4),  # base_x, base_y, base_z, base_yaw
    "gimbal": slice(4, 6),  # pan, tilt
}

# ── Kinodynamic limits ───────────────────────────────────────────────
# PHANTOM's actuator limits (the <actuator> entries of PHANTOM's
# drone.urdf).  They are simulated capabilities, not hardware ratings.
BASE_MAX_VELOCITY = 1.5  # m/s, each of x, y and z
BASE_MAX_ACCELERATION = 1.0  # m/s^2, each of x, y and z
YAW_MAX_VELOCITY = 1.5  # rad/s
YAW_MAX_ACCELERATION = 3.0  # rad/s^2
GIMBAL_MAX_VELOCITY = 1.0  # rad/s, pan and tilt
GIMBAL_MAX_ACCELERATION = 2.0  # rad/s^2, pan and tilt

# Kinodynamic limits used by ``MotionPlanner.plan_kinodynamic`` (FLASK):
# (lower, upper) per joint, all symmetric.
JOINT_VELOCITY_LIMITS: dict[str, tuple[float, float]] = {
    "base_x_joint": (-BASE_MAX_VELOCITY, BASE_MAX_VELOCITY),
    "base_y_joint": (-BASE_MAX_VELOCITY, BASE_MAX_VELOCITY),
    "base_z_joint": (-BASE_MAX_VELOCITY, BASE_MAX_VELOCITY),
    "base_yaw_joint": (-YAW_MAX_VELOCITY, YAW_MAX_VELOCITY),
    "camera_pan": (-GIMBAL_MAX_VELOCITY, GIMBAL_MAX_VELOCITY),
    "camera_tilt": (-GIMBAL_MAX_VELOCITY, GIMBAL_MAX_VELOCITY),
}
JOINT_ACCELERATION_LIMITS: dict[str, tuple[float, float]] = {
    "base_x_joint": (-BASE_MAX_ACCELERATION, BASE_MAX_ACCELERATION),
    "base_y_joint": (-BASE_MAX_ACCELERATION, BASE_MAX_ACCELERATION),
    "base_z_joint": (-BASE_MAX_ACCELERATION, BASE_MAX_ACCELERATION),
    "base_yaw_joint": (-YAW_MAX_ACCELERATION, YAW_MAX_ACCELERATION),
    "camera_pan": (-GIMBAL_MAX_ACCELERATION, GIMBAL_MAX_ACCELERATION),
    "camera_tilt": (-GIMBAL_MAX_ACCELERATION, GIMBAL_MAX_ACCELERATION),
}
# FLASK also bounds the length (norm) of the horizontal velocity and
# acceleration vectors (x, y).  PHANTOM gives one limit per axis, so the
# norm bounds take the per-axis values.  This is stricter than PHANTOM
# when the drone flies diagonally: its horizontal speed is then at most
# 1.5 m/s in all, not 1.5 m/s along each of x and y.
HORIZONTAL_MAX_SPEED: float = BASE_MAX_VELOCITY  # |(x_dot, y_dot)|, m/s
HORIZONTAL_MAX_ACCELERATION: float = BASE_MAX_ACCELERATION  # m/s^2

# The URDF with the collision sphere only, the source of the VAMP model
# (cricket).  Written by hand: the collision model already is one sphere,
# so there is no foam step.
SPHERIZED_URDF_PATH = os.path.join(_RESOURCES_DIR, "drone_spherized.urdf")

# PHANTOM's drone.urdf with the virtual base joints (and its visual boxes).
drone_robot_config = RobotConfig(
    urdf_path=os.path.join(_RESOURCES_DIR, "drone.urdf"),
    joint_names=[
        # [0:4]   body (virtual joints)
        "base_x_joint",
        "base_y_joint",
        "base_z_joint",
        "base_yaw_joint",
        # [4:6]   gimbal pan, tilt
        "camera_pan",
        "camera_tilt",
    ],
    # PHANTOM's sensor: a cone of 1.0821 rad (62 degrees) seeing up to 5 m,
    # x along the optical axis.  PHANTOM gives no image size; the square
    # image is this package's choice, so the view cone fits it.
    camera=CameraConfig(
        link_name="camera",
        width=640,
        height=640,
        fov=math.degrees(1.0821041362364843),
        near=0.05,
        far=5.0,
    ),
)

# VAMP subgroup robot names for planning.
_BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint"]
_GIMBAL_JOINTS = ["camera_pan", "camera_tilt"]

PLANNING_SUBGROUPS = {
    # Body only (x, y, z, yaw); the gimbal holds its pose
    "drone_base": {"dof": 4, "joints": _BASE_JOINTS},
    # Body + gimbal pan and tilt (full)
    "drone_whole_body": {"dof": 6, "joints": _BASE_JOINTS + _GIMBAL_JOINTS},
}

# Hover 1 m above the origin, heading +x, camera level and forward.
HOME_JOINTS = np.array(
    [
        # [0:4]   body
        0.0,
        0.0,
        1.0,
        0.0,
        # [4:6]   gimbal pan, tilt
        0.0,
        0.0,
    ]
)
