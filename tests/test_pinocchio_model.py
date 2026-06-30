"""
Tests for the pinocchio backend (PinocchioWheeledUR5eModel).

Verifies:
  - The MJCF preprocessing produces a 10-DOF pinocchio model
  - FK matches the hand-written FK in robot_model.py to machine precision
  - FK matches MuJoCo's site_xpos["ee_site"]
  - The position Jacobian matches finite-difference of the hand-written FK
  - The reduced arm model is 6-DOF and locks the base joints at nominal
"""

import os
import sys

import numpy as np
import pytest

# Inject ALIGATOR build path (matching the rest of the test suite).
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path[:0] = [
    os.path.join(_REPO_ROOT, "build", "bindings", "python"),
    os.path.join(_REPO_ROOT, "study_example", "wheeled_ur5e_aligator_mpc"),
]

from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel


@pytest.fixture(scope="module")
def pin_model():
    return PinocchioWheeledUR5eModel()


@pytest.fixture(scope="module")
def rm():
    return WheeledUR5eModel()


def test_dof_count(pin_model):
    assert pin_model.model.nq == 10
    assert pin_model.model.nv == 10


def test_fk_at_nominal(pin_model, rm):
    q = rm.q_nominal.copy()
    p_pin, R_pin = pin_model.fk_pose(q)
    p_rm = rm.fk_numpy(q)
    assert np.max(np.abs(p_pin - p_rm)) < 1e-10
    # Sanity: rotation matrix is orthonormal
    assert np.allclose(R_pin @ R_pin.T, np.eye(3), atol=1e-10)
    assert np.isclose(np.linalg.det(R_pin), 1.0, atol=1e-10)


def test_fk_random_configs_match_robot_model(pin_model, rm):
    rng = np.random.default_rng(0)
    max_err = 0.0
    for _ in range(50):
        q = rng.uniform(rm.q_min * 0.5, rm.q_max * 0.5)
        p_pin, _ = pin_model.fk_pose(q)
        p_rm = rm.fk_numpy(q)
        max_err = max(max_err, np.max(np.abs(p_pin - p_rm)))
    assert max_err < 1e-10, f"max FK diff {max_err}"


def test_fk_matches_mujoco(pin_model, rm):
    """Cross-check: pinocchio FK == MuJoCo site_xpos (ground truth simulator FK)."""
    import mujoco

    mjcf = os.path.join(
        _REPO_ROOT, "study_example", "wheeled_ur5e_aligator_mpc",
        "assets", "wheeled_ur5e.xml",
    )
    m = mujoco.MjModel.from_xml_path(mjcf)
    d = mujoco.MjData(m)
    ee_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    jnames = [
        "base_x", "base_y", "base_z", "base_yaw",
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]
    qaddr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in jnames]

    rng = np.random.default_rng(11)
    max_err = 0.0
    for _ in range(20):
        q = rng.uniform(rm.q_min * 0.5, rm.q_max * 0.5)
        for i, addr in enumerate(qaddr):
            d.qpos[addr] = q[i]
        mujoco.mj_kinematics(m, d)
        ee_mj = d.site_xpos[ee_site_id].copy()
        p_pin, _ = pin_model.fk_pose(q)
        max_err = max(max_err, np.max(np.abs(p_pin - ee_mj)))
    assert max_err < 1e-10, f"pin vs mujoco max err {max_err}"


def test_position_jacobian_matches_finite_difference(pin_model, rm):
    rng = np.random.default_rng(3)
    for _ in range(5):
        q = rng.uniform(rm.q_min * 0.5, rm.q_max * 0.5)
        J_pin = pin_model.position_jacobian(q)
        J_fd = rm.finite_difference_jacobian_fk(q)
        assert J_pin.shape == (3, 10)
        # FD noise floor ~1e-6
        assert np.max(np.abs(J_pin - J_fd)) < 1e-5


def test_reordering_helpers_are_self_inverse(pin_model, rm):
    rng = np.random.default_rng(5)
    for _ in range(5):
        q = rng.uniform(rm.q_min, rm.q_max)
        q_pin = pin_model.q_rm_to_pin(q)
        q_back = pin_model.q_pin_to_rm(q_pin)
        assert np.allclose(q, q_back)


def test_reduced_arm_model_is_six_dof(pin_model):
    assert pin_model.arm_model.nq == 6
    assert pin_model.arm_model.nv == 6
