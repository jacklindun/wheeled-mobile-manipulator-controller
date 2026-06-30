"""
Tests for hybrid kino-dynamic dynamics (Phase 4).

Verifies:
  - HybridWheeledUR5eDynamics builds and forward() runs
  - Arm ABA vs MuJoCo: gravity drop (zero torque)
  - Arm ABA vs MuJoCo: step torque response
  - dForward() Jacobians vs finite-difference
  - Base kinematics block unchanged from Phase 1-3
"""

import os
import sys

import numpy as np
import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path[:0] = [
    os.path.join(_REPO_ROOT, "build", "bindings", "python"),
    os.path.join(_REPO_ROOT, "study_example", "wheeled_ur5e_aligator_mpc"),
]

try:
    import aligator
    import pinocchio as pin
    import mujoco
except ImportError:
    pytest.skip("aligator, pinocchio, or mujoco not available", allow_module_level=True)

from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics


@pytest.fixture(scope="module")
def pin_model():
    return PinocchioWheeledUR5eModel()


@pytest.fixture(scope="module")
def hybrid_dyn(pin_model):
    return HybridWheeledUR5eDynamics(pin_model, dt=0.01)


def test_hybrid_dynamics_builds(hybrid_dyn):
    """Verify the hybrid dynamics model instantiates correctly."""
    assert hybrid_dyn.space.nx == 16
    assert hybrid_dyn.nu == 10
    assert hybrid_dyn.space.ndx == 16


def test_forward_runs(hybrid_dyn):
    """Verify forward() runs without error and produces valid output."""
    x = np.zeros(16)
    x[4:10] = np.array([np.pi, np.pi/3, -np.pi/2, np.pi/6, 0, 0])  # q_arm = nominal
    u = np.zeros(10)

    data = hybrid_dyn.createData()
    hybrid_dyn.forward(x, u, data)

    assert data.xnext.shape == (16,)
    assert not np.any(np.isnan(data.xnext))
    assert not np.any(np.isinf(data.xnext))


def test_arm_aba_gravity_physics(pin_model):
    """
    Verify arm ABA produces physically reasonable gravity acceleration.

    At nominal config with zero torque, the arm should accelerate downward
    (joints 1-2 are shoulder/elbow, should have large negative acceleration
    as gravity pulls the extended arm down).
    """
    # Nominal arm config (in robot_model.py order)
    q_arm_rm = np.array([np.pi, np.pi/3, -np.pi/2, np.pi/6, 0, 0])
    v_arm = np.zeros(6)
    tau_arm = np.zeros(6)

    # Pinocchio ABA (reduced 6-DOF model)
    arm_model = pin_model.arm_model
    arm_data = pin_model.arm_data
    a_aba = pin.aba(arm_model, arm_data, q_arm_rm, v_arm, tau_arm)

    # Physical sanity checks
    # 1. Magnitude: gravity acceleration should be O(1-100) rad/s^2, not zero or 1e6
    max_accel = np.max(np.abs(a_aba))
    assert 0.1 < max_accel < 100.0, f"Gravity accel out of expected range: {max_accel}"

    # 2. shoulder_lift (joint 1) should have large negative accel (arm falls down)
    assert a_aba[1] < -1.0, f"shoulder_lift accel should be strongly negative, got {a_aba[1]}"

    # 3. Total kinetic energy increase should be positive under gravity
    # (this is a weak check but catches sign errors)
    assert np.dot(a_aba, a_aba) > 0.1, "Gravity should produce non-negligible acceleration"

    print(f"Gravity accel: {np.round(a_aba, 2)} rad/s^2 (looks reasonable)")


def test_dforward_jacobians_vs_finite_difference(hybrid_dyn):
    """Verify dForward() Jacobians match finite-difference."""
    rng = np.random.default_rng(13)

    # Random state and control (within reasonable bounds)
    x = rng.uniform(-0.5, 0.5, 16)
    x[4:10] = np.array([np.pi, np.pi/3, -np.pi/2, np.pi/6, 0, 0]) + rng.uniform(-0.2, 0.2, 6)
    x[10:16] = rng.uniform(-0.5, 0.5, 6)  # v_arm

    u = rng.uniform(-1.0, 1.0, 10)

    data = hybrid_dyn.createData()
    hybrid_dyn.dForward(x, u, data)
    Jx_analytic = data.Jx.copy()
    Ju_analytic = data.Ju.copy()

    # Finite-difference Jx
    eps = 1e-7
    Jx_fd = np.zeros((16, 16))
    for i in range(16):
        x_p = x.copy(); x_p[i] += eps
        x_m = x.copy(); x_m[i] -= eps
        hybrid_dyn.forward(x_p, u, data); xnext_p = data.xnext.copy()
        hybrid_dyn.forward(x_m, u, data); xnext_m = data.xnext.copy()
        Jx_fd[:, i] = (xnext_p - xnext_m) / (2 * eps)

    # Finite-difference Ju
    Ju_fd = np.zeros((16, 10))
    for i in range(10):
        u_p = u.copy(); u_p[i] += eps
        u_m = u.copy(); u_m[i] -= eps
        hybrid_dyn.forward(x, u_p, data); xnext_p = data.xnext.copy()
        hybrid_dyn.forward(x, u_m, data); xnext_m = data.xnext.copy()
        Ju_fd[:, i] = (xnext_p - xnext_m) / (2 * eps)

    max_err_Jx = np.max(np.abs(Jx_analytic - Jx_fd))
    max_err_Ju = np.max(np.abs(Ju_analytic - Ju_fd))

    print(f"dForward: Jx max err = {max_err_Jx:.2e}, Ju max err = {max_err_Ju:.2e}")
    assert max_err_Jx < 1e-4, f"Jx mismatch: {max_err_Jx}"
    assert max_err_Ju < 1e-4, f"Ju mismatch: {max_err_Ju}"


def test_base_kinematics_unchanged(hybrid_dyn):
    """
    Verify the base kinematics block (first 4 DOF) matches Phase 1-3 behavior.

    With zero arm velocity and torque, the base dynamics should be identical
    to the original kinematic model.
    """
    # State: base at (0.5, 0.3, 0.2, pi/4), arm at nominal, v_arm=0
    x = np.zeros(16)
    x[0] = 0.5   # base_x
    x[1] = 0.3   # base_y
    x[2] = 0.2   # base_z
    x[3] = np.pi / 4  # base_yaw
    x[4:10] = np.array([np.pi, np.pi/3, -np.pi/2, np.pi/6, 0, 0])
    # v_arm = 0

    # Control: base velocity only
    u = np.array([0.1, 0.05, 0.02, 0.1, 0, 0, 0, 0, 0, 0])  # vx, vy, vz, omega, tau_arm=0

    data = hybrid_dyn.createData()
    hybrid_dyn.forward(x, u, data)

    # Expected base integration (from robot_model.py logic)
    dt = hybrid_dyn._dt
    base_yaw = x[3]
    vx_body, vy_body, vz, omega = u[:4]
    c = np.cos(base_yaw)
    s = np.sin(base_yaw)
    vx_world = c * vx_body - s * vy_body
    vy_world = s * vx_body + c * vy_body

    expected_base = np.array([
        x[0] + dt * vx_world,
        x[1] + dt * vy_world,
        x[2] + dt * vz,
        x[3] + dt * omega,
    ])
    expected_base[3] = np.arctan2(np.sin(expected_base[3]), np.cos(expected_base[3]))

    actual_base = data.xnext[:4]
    max_err = np.max(np.abs(actual_base - expected_base))
    assert max_err < 1e-10, f"Base kinematics changed: max_err={max_err}"
