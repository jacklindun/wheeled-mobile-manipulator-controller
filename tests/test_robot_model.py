"""Tests for robot_model.py: FK, dynamics, linearization."""

import sys
from pathlib import Path
import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel


@pytest.fixture
def model():
    return WheeledUR5eModel()


def test_fk_runs(model):
    q = model.q_nominal.copy()
    ee = model.fk_numpy(q)
    assert ee.shape == (3,), f"Expected (3,), got {ee.shape}"
    assert not np.any(np.isnan(ee)), "FK returned NaN"


def test_fk_zero_base(model):
    q = np.zeros(10)
    ee = model.fk_numpy(q)
    assert ee.shape == (3,)
    # At zero config the arm is unfolded; EE should be non-trivial
    assert np.linalg.norm(ee) > 0.05


def test_dynamics_zero_control(model):
    q = model.q_nominal.copy()
    u = np.zeros(10)
    q_next = model.dynamics_numpy(q, u, dt=0.05)
    assert q_next.shape == (10,)
    assert np.allclose(q_next, q, atol=1e-10)


def test_dynamics_nonzero_control(model):
    q = model.q_nominal.copy()
    u = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dt = 0.05
    q_next = model.dynamics_numpy(q, u, dt)
    # base_x should increase
    expected_dx = dt * np.cos(q[3]) * u[0]
    assert abs(q_next[0] - (q[0] + expected_dx)) < 1e-9


def test_linearize_shapes(model):
    q = model.q_nominal.copy()
    u = np.zeros(10)
    A, B = model.linearize_dynamics(q, u, dt=0.05)
    assert A.shape == (10, 10)
    assert B.shape == (10, 10)


def test_linearize_vs_fd(model):
    q = model.q_nominal.copy()
    u = np.array([0.1, 0.05, 0.0, 0.2, 0.1, -0.1, 0.05, 0.0, 0.0, 0.0])
    dt = 0.05
    A, B = model.linearize_dynamics(q, u, dt)

    eps = 1e-6
    A_fd = np.zeros((10, 10))
    for i in range(10):
        qp = q.copy(); qp[i] += eps
        qm = q.copy(); qm[i] -= eps
        A_fd[:, i] = (model.dynamics_numpy(qp, u, dt) - model.dynamics_numpy(qm, u, dt)) / (2 * eps)

    B_fd = np.zeros((10, 10))
    for i in range(10):
        up = u.copy(); up[i] += eps
        um = u.copy(); um[i] -= eps
        B_fd[:, i] = (model.dynamics_numpy(q, up, dt) - model.dynamics_numpy(q, um, dt)) / (2 * eps)

    assert np.max(np.abs(A - A_fd)) < 1e-5, f"A linearization error: {np.max(np.abs(A - A_fd)):.2e}"
    assert np.max(np.abs(B - B_fd)) < 1e-5, f"B linearization error: {np.max(np.abs(B - B_fd)):.2e}"


def test_fk_jacobian_shape(model):
    q = model.q_nominal.copy()
    J = model.finite_difference_jacobian_fk(q)
    assert J.shape == (3, 10)
    assert not np.any(np.isnan(J))
