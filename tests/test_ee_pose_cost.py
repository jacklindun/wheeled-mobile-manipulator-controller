"""
Tests for EE pose cost (position + orientation tracking).

Verifies:
  - evaluate() returns correct value for position + orientation error
  - computeGradients() matches finite-difference
  - computeHessians() is positive definite (Gauss-Newton)
  - w_orientation=0 recovers position-only cost
  - set_reference() updates the target pose
"""

import os
import sys

import numpy as np
import pytest

# Inject ALIGATOR build path.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path[:0] = [
    os.path.join(_REPO_ROOT, "build", "bindings", "python"),
    os.path.join(_REPO_ROOT, "study_example", "wheeled_ur5e_aligator_mpc"),
]

try:
    import aligator
    import pinocchio as pin
except ImportError:
    pytest.skip("aligator or pinocchio not available", allow_module_level=True)

from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.ee_pose_cost import EEPoseCost


@pytest.fixture(scope="module")
def pin_model():
    return PinocchioWheeledUR5eModel()


@pytest.fixture(scope="module")
def rm():
    return WheeledUR5eModel()


def test_pose_cost_evaluate(pin_model, rm):
    space = aligator.manifolds.VectorSpace(10)
    nu = 10
    w_p = 100.0
    w_o = 50.0

    q = rm.q_nominal.copy()
    p_ref, R_ref = pin_model.fk_pose(q)

    cost = EEPoseCost(space, nu, pin_model, w_p, w_o, p_ref, R_ref)
    data = cost.createData()
    u = np.zeros(nu)

    # At the reference: cost should be zero
    cost.evaluate(q, u, data)
    assert data.value < 1e-10, f"cost at reference = {data.value}, expected ~0"

    # Perturb position: cost = 0.5 * w_p * ||dp||^2
    q2 = q.copy()
    q2[0] += 0.1  # base_x
    p2, R2 = pin_model.fk_pose(q2)
    dp = np.linalg.norm(p2 - p_ref)
    cost.evaluate(q2, u, data)
    expected = 0.5 * w_p * dp**2
    assert abs(data.value - expected) < 1e-6, f"position cost mismatch: {data.value} vs {expected}"

    # Perturb orientation: cost = 0.5 * w_p * ||dp||^2 + 0.5 * w_o * ||log3(R_ref^T @ R)||^2
    # (moving a wrist joint changes both position and orientation via the kinematic chain)
    q3 = q.copy()
    q3[8] += 0.2  # wrist_2
    p3, R3 = pin_model.fk_pose(q3)
    dp3 = p3 - p_ref
    e_o = pin.log3(R_ref.T @ R3)
    cost.evaluate(q3, u, data)
    expected = 0.5 * w_p * np.dot(dp3, dp3) + 0.5 * w_o * np.dot(e_o, e_o)
    assert abs(data.value - expected) < 1e-5, f"pose cost mismatch: {data.value} vs {expected}"


def test_pose_cost_gradient_finite_difference(pin_model, rm):
    space = aligator.manifolds.VectorSpace(10)
    nu = 10
    w_p = 100.0
    w_o = 50.0

    rng = np.random.default_rng(42)
    q = rng.uniform(rm.q_min * 0.5, rm.q_max * 0.5)
    p_ref, R_ref = pin_model.fk_pose(q)
    # Perturb reference slightly so gradient is nonzero
    p_ref += rng.normal(0, 0.05, 3)
    R_ref = R_ref @ pin.exp3(rng.normal(0, 0.1, 3))

    cost = EEPoseCost(space, nu, pin_model, w_p, w_o, p_ref, R_ref)
    data = cost.createData()
    u = np.zeros(nu)

    cost.computeGradients(q, u, data)
    grad_analytic = data.Lx.copy()

    # Finite-difference gradient
    eps = 1e-7
    grad_fd = np.zeros(10)
    for i in range(10):
        q_p = q.copy(); q_p[i] += eps
        q_m = q.copy(); q_m[i] -= eps
        cost.evaluate(q_p, u, data); cost_p = data.value
        cost.evaluate(q_m, u, data); cost_m = data.value
        grad_fd[i] = (cost_p - cost_m) / (2 * eps)

    max_err = np.max(np.abs(grad_analytic - grad_fd))
    assert max_err < 1e-4, f"gradient max err = {max_err}"


def test_pose_cost_hessian_is_positive_definite(pin_model, rm):
    space = aligator.manifolds.VectorSpace(10)
    nu = 10
    w_p = 100.0
    w_o = 50.0

    q = rm.q_nominal.copy()
    p_ref, R_ref = pin_model.fk_pose(q)

    cost = EEPoseCost(space, nu, pin_model, w_p, w_o, p_ref, R_ref)
    data = cost.createData()
    u = np.zeros(nu)

    cost.computeHessians(q, u, data)
    H = data.Lxx.copy()

    # Gauss-Newton Hessian should be PSD
    eigvals = np.linalg.eigvalsh(H)
    assert np.all(eigvals >= -1e-8), f"Hessian has negative eigenvalues: {eigvals.min()}"
    assert np.all(eigvals < 1e10), f"Hessian is ill-conditioned: {eigvals.max()}"


def test_orientation_weight_zero_recovers_position_only(pin_model, rm):
    space = aligator.manifolds.VectorSpace(10)
    nu = 10
    w_p = 100.0
    w_o = 0.0  # orientation weight = 0

    q = rm.q_nominal.copy()
    p_ref, R_ref = pin_model.fk_pose(q)

    cost = EEPoseCost(space, nu, pin_model, w_p, w_o, p_ref, R_ref)
    data = cost.createData()
    u = np.zeros(nu)

    # Perturb orientation heavily
    q2 = q.copy()
    q2[8] += 1.0  # large wrist_2 rotation
    p2, R2 = pin_model.fk_pose(q2)
    e_o = pin.log3(R_ref.T @ R2)

    cost.evaluate(q2, u, data)
    # Cost should only reflect position error (which is tiny since base didn't move much)
    dp = np.linalg.norm(p2 - p_ref)
    expected = 0.5 * w_p * dp**2
    assert abs(data.value - expected) < 1e-6, f"w_ori=0 didn't ignore orientation: {data.value} vs {expected}"


def test_set_reference_updates_target(pin_model, rm):
    space = aligator.manifolds.VectorSpace(10)
    nu = 10
    w_p = 100.0
    w_o = 50.0

    q = rm.q_nominal.copy()
    p_ref1, R_ref1 = pin_model.fk_pose(q)

    cost = EEPoseCost(space, nu, pin_model, w_p, w_o, p_ref1, R_ref1)
    data = cost.createData()
    u = np.zeros(nu)

    # Cost at reference should be zero
    cost.evaluate(q, u, data)
    assert data.value < 1e-10

    # Update reference
    p_ref2 = p_ref1 + np.array([0.1, 0.0, 0.0])
    R_ref2 = R_ref1 @ pin.exp3(np.array([0.0, 0.0, 0.1]))
    cost.set_reference(p_ref2, R_ref2)

    # Now cost should be nonzero
    cost.evaluate(q, u, data)
    dp = 0.1
    e_o = pin.log3(R_ref2.T @ R_ref1)
    expected = 0.5 * w_p * dp**2 + 0.5 * w_o * np.dot(e_o, e_o)
    assert abs(data.value - expected) < 1e-5, f"set_reference didn't update: {data.value} vs {expected}"
