"""The Holybro X500 V2 robot's bundled description.

This module collects every concrete value that describes the one robot
this project ships: joint groupings, the home pose, the VAMP planning
subgroups, the kinodynamic limits and the top-level ``RobotConfig``
instance.

The dataclass *types* themselves live in :mod:`drone_planning.types.robot`
— this file holds *values* of those types.

The model is PX4's ``x500_gimbal`` Gazebo model (PX4-gazebo-models
``x500`` + ``gimbal``), converted to URDF by ``scripts/px4_sdf_to_urdf.py``.
Its rotors sit 0.348 m apart on a square (0.49 m diagonal), matching the
X500 V2's 500 mm wheelbase, and its skids hang 0.23 m below the body
frame (Holybro: 215 mm landing gear).  The model keeps PX4's 11.5-inch
propeller mesh; Holybro ships 10-inch (1045) propellers.

The flight controller holds roll and pitch, so the body is a
velocity-commanded point with a heading.  The full configuration vector is
6-DOF:

* ``[0:4]`` — body ``(x, y, z, yaw)``: virtual joints in the world frame
  (z up, yaw counter-clockwise from +x)
* ``[4:6]`` — gimbal: pan (``cgo3_vertical_arm_joint``) and tilt
  (``cgo3_camera_joint``); gimbal roll is held level

The gimbal keeps the PX4 model's joint axes, so the joint values are
PX4's gimbal angles in the body frame: positive pan turns the camera
clockwise seen from above (to the right), positive tilt raises it, and
zero looks straight ahead.
"""

import math
import os

import numpy as np

from drone_planning.types.robot import CameraConfig, RobotConfig

_PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
_RESOURCES_DIR = os.path.join(_PKG_ROOT, "resources", "robot", "x500")

# Atomic joint groups — indices into the full 6-DOF configuration array.
# Order matches VAMP's URDF tree traversal: body → gimbal.
JOINT_GROUPS = {
    "base": slice(0, 4),  # base_x, base_y, base_z, base_yaw
    "gimbal": slice(4, 6),  # pan, tilt
}

# ── Kinodynamic limits ───────────────────────────────────────────────
# PX4 v1.17.0 default parameters (src/modules/mc_pos_control/
# *_params.c, src/modules/gimbal/gimbal_params.c).  The PX4 X500 airframes
# (4001_gz_x500, 4019_gz_x500_gimbal, 4019_x500_v2) leave all of them at
# their defaults.
#
# MPC_XY_VEL_MAX: "Maximum horizontal velocity.  Absolute maximum for all
# velocity controlled modes."
MPC_XY_VEL_MAX = 12.0  # m/s
# MPC_Z_VEL_MAX_UP / _DN: "Maximum ascent / descent velocity.  Absolute
# maximum for all climb rate controlled modes."
MPC_Z_VEL_MAX_UP = 3.0  # m/s
MPC_Z_VEL_MAX_DN = 1.5  # m/s
# MPC_ACC_HOR_MAX: "Maximum horizontal acceleration".
MPC_ACC_HOR_MAX = 5.0  # m/s^2
# MPC_ACC_UP_MAX / MPC_ACC_DOWN_MAX: "Maximum upwards / downwards
# acceleration in climb rate controlled modes".
MPC_ACC_UP_MAX = 4.0  # m/s^2
MPC_ACC_DOWN_MAX = 3.0  # m/s^2
# MPC_YAWRAUTO_MAX / MPC_YAWRAUTO_ACC: "Maximum yaw rate / yaw acceleration
# in autonomous modes".
MPC_YAWRAUTO_MAX = math.radians(60.0)  # rad/s
MPC_YAWRAUTO_ACC = math.radians(20.0)  # rad/s^2
# MNT_RATE_YAW / MNT_RATE_PITCH: "Angular yaw / pitch rate for manual
# input in degrees/second" — PX4's gimbal slew rates.
MNT_RATE_YAW = math.radians(30.0)  # rad/s
MNT_RATE_PITCH = math.radians(30.0)  # rad/s
# PX4 has no gimbal acceleration parameter.  The x500_gimbal model drives
# each gimbal joint with a gz JointPositionController whose torque is
# capped at cmd_max = 0.3 N m; over the SDF inertia about the joint axis
# (three 0.1 kg links of 0.001 kg m^2 about pan, offset from its axis;
# the camera's 0.001 kg m^2 about tilt) that bounds the acceleration.
GIMBAL_PAN_MAX_ACCELERATION = 0.3 / 0.0030915  # rad/s^2
GIMBAL_TILT_MAX_ACCELERATION = 0.3 / 0.001  # rad/s^2

# Kinodynamic limits used by ``MotionPlanner.plan_kinodynamic`` (FLASK):
# (lower, upper) per joint; the climb rate and vertical acceleration are
# asymmetric.  The horizontal pair (base_x, base_y) is also bounded in
# norm, as PX4 bounds it.
JOINT_VELOCITY_LIMITS: dict[str, tuple[float, float]] = {
    "base_x_joint": (-MPC_XY_VEL_MAX, MPC_XY_VEL_MAX),
    "base_y_joint": (-MPC_XY_VEL_MAX, MPC_XY_VEL_MAX),
    "base_z_joint": (-MPC_Z_VEL_MAX_DN, MPC_Z_VEL_MAX_UP),
    "base_yaw_joint": (-MPC_YAWRAUTO_MAX, MPC_YAWRAUTO_MAX),
    "cgo3_vertical_arm_joint": (-MNT_RATE_YAW, MNT_RATE_YAW),
    "cgo3_camera_joint": (-MNT_RATE_PITCH, MNT_RATE_PITCH),
}
JOINT_ACCELERATION_LIMITS: dict[str, tuple[float, float]] = {
    "base_x_joint": (-MPC_ACC_HOR_MAX, MPC_ACC_HOR_MAX),
    "base_y_joint": (-MPC_ACC_HOR_MAX, MPC_ACC_HOR_MAX),
    "base_z_joint": (-MPC_ACC_DOWN_MAX, MPC_ACC_UP_MAX),
    "base_yaw_joint": (-MPC_YAWRAUTO_ACC, MPC_YAWRAUTO_ACC),
    "cgo3_vertical_arm_joint": (
        -GIMBAL_PAN_MAX_ACCELERATION,
        GIMBAL_PAN_MAX_ACCELERATION,
    ),
    "cgo3_camera_joint": (
        -GIMBAL_TILT_MAX_ACCELERATION,
        GIMBAL_TILT_MAX_ACCELERATION,
    ),
}
HORIZONTAL_MAX_SPEED: float = MPC_XY_VEL_MAX  # |(x_dot, y_dot)|, m/s
HORIZONTAL_MAX_ACCELERATION: float = MPC_ACC_HOR_MAX  # |(x_ddot, y_ddot)|, m/s^2

# The URDF with sphere collision geometry (foam), the source of the VAMP
# model (cricket).
SPHERIZED_URDF_PATH = os.path.join(_RESOURCES_DIR, "x500_spherized.urdf")

# The URDF converted from the PX4 SDF (meshes, virtual base joints).
x500_robot_config = RobotConfig(
    urdf_path=os.path.join(_RESOURCES_DIR, "x500.urdf"),
    joint_names=[
        # [0:4]   body (virtual joints)
        "base_x_joint",
        "base_y_joint",
        "base_z_joint",
        "base_yaw_joint",
        # [4:6]   gimbal pan, tilt
        "cgo3_vertical_arm_joint",
        "cgo3_camera_joint",
    ],
    # The SDF camera sensor: x along the optical axis (Gazebo convention).
    camera=CameraConfig(
        link_name="camera_sensor_link",
        width=1280,
        height=720,
        # vertical field of view of the SDF's horizontal_fov = 2.0 rad
        fov=math.degrees(2.0 * math.atan(math.tan(2.0 / 2.0) * 720 / 1280)),
        near=0.05,
        far=15000.0,
    ),
)

# VAMP subgroup robot names for planning.
_BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint"]
_GIMBAL_JOINTS = ["cgo3_vertical_arm_joint", "cgo3_camera_joint"]

PLANNING_SUBGROUPS = {
    # Body only (x, y, z, yaw); the gimbal holds its pose
    "x500_base": {"dof": 4, "joints": _BASE_JOINTS},
    # Body + gimbal pan and tilt (full)
    "x500_whole_body": {"dof": 6, "joints": _BASE_JOINTS + _GIMBAL_JOINTS},
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
