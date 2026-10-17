import numpy as np
import pinocchio as pin

from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.kinematics import (
    compute_forward_kinematics,
    create_pinocchio_context,
)
from drone_planning.planning import available_robots
from drone_planning.x500 import (
    HOME_JOINTS,
    JOINT_ACCELERATION_LIMITS,
    JOINT_GROUPS,
    JOINT_VELOCITY_LIMITS,
    PLANNING_SUBGROUPS,
    SPHERIZED_URDF_PATH,
    x500_robot_config,
)


def test_home_joints_has_six_dof():
    assert HOME_JOINTS.shape == (6,)


def test_joint_groups_cover_all_dof():
    idx = set()
    for group_slice in JOINT_GROUPS.values():
        idx.update(range(*group_slice.indices(HOME_JOINTS.shape[0])))
    assert idx == set(range(HOME_JOINTS.shape[0]))


def test_planning_subgroups_exposed():
    expected = {"x500_base", "x500_whole_body"}
    assert expected == set(PLANNING_SUBGROUPS.keys())
    assert expected.issubset(set(available_robots()))
    for sg in PLANNING_SUBGROUPS.values():
        assert sg["dof"] == len(sg["joints"])


def test_compiled_model_matches_urdf():
    """The VAMP model (cricket) has the URDF's joints, order and ranges."""
    for urdf in (x500_robot_config.urdf_path, SPHERIZED_URDF_PATH):
        model = pin.buildModelFromUrdf(urdf)
        names = [model.names[i] for i in range(1, model.njoints)]
        assert names == x500_robot_config.joint_names
        lower, upper = OmplVampPlanner.default_joint_limits()
        np.testing.assert_allclose(lower, model.lowerPositionLimit, atol=1e-6)
        np.testing.assert_allclose(upper, model.upperPositionLimit, atol=1e-6)


def test_limits_cover_every_joint():
    for limits in (JOINT_VELOCITY_LIMITS, JOINT_ACCELERATION_LIMITS):
        assert list(limits) == x500_robot_config.joint_names
        for lo, hi in limits.values():
            assert lo < 0.0 < hi


def test_camera_fk():
    """The camera hangs below the body and looks along the gimbal angles:
    positive pan turns it clockwise seen from above, positive tilt up."""
    ctx = create_pinocchio_context(
        x500_robot_config.urdf_path,
        x500_robot_config.camera.link_name,
        x500_robot_config.joint_names,
    )
    home = compute_forward_kinematics(ctx, HOME_JOINTS)
    assert home.position[2] < HOME_JOINTS[2]
    np.testing.assert_allclose(home.rotation[:, 0], [1.0, 0.0, 0.0], atol=1e-2)

    q = HOME_JOINTS.copy()
    q[4], q[5] = 0.5, -0.5
    axis = compute_forward_kinematics(ctx, q).rotation[:, 0]
    np.testing.assert_allclose(
        axis,
        [np.cos(0.5) * np.cos(0.5), -np.sin(0.5) * np.cos(0.5), -np.sin(0.5)],
        atol=1e-2,
    )
