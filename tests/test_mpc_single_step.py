"""Tests for AligatorWholeBodyMPC single-step solve."""

import sys
from pathlib import Path
import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))
_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


@pytest.fixture
def mpc_setup():
    robot = WheeledUR5eModel()
    mpc = AligatorWholeBodyMPC(robot, horizon=5, dt=0.05, max_iters=5)
    ref_gen = ReferenceGenerator("ee_circle")
    return robot, mpc, ref_gen


def test_mpc_build(mpc_setup):
    robot, mpc, ref_gen = mpc_setup
    assert mpc is not None
    assert mpc.horizon == 5


def test_mpc_single_step(mpc_setup):
    robot, mpc, ref_gen = mpc_setup
    q0 = robot.q_nominal.copy()
    ref_traj = ref_gen.get_reference(t=0.0, horizon=mpc.horizon, dt=mpc.dt)

    u0, q_pred, info = mpc.solve(q_current=q0, ref_traj=ref_traj)

    assert u0.shape == (10,), f"u0 shape: {u0.shape}"
    assert q_pred.shape == (6, 10), f"q_pred shape: {q_pred.shape}"
    assert "success" in info
    assert "status" in info
    assert "solve_time" in info
    assert "iter" in info
    assert "fallback" in info


def test_u0_within_bounds(mpc_setup):
    robot, mpc, ref_gen = mpc_setup
    q0 = robot.q_nominal.copy()
    ref_traj = ref_gen.get_reference(t=0.0, horizon=mpc.horizon, dt=mpc.dt)
    u0, _, _ = mpc.solve(q_current=q0, ref_traj=ref_traj)
    assert np.all(u0 >= robot.u_min - 1e-6), f"u0 below u_min: {u0}"
    assert np.all(u0 <= robot.u_max + 1e-6), f"u0 above u_max: {u0}"


def test_fallback_on_bad_state(mpc_setup):
    """MPC should not crash on a degenerate initial state."""
    robot, mpc, ref_gen = mpc_setup
    q_bad = np.zeros(10)
    q_bad[2] = -1.0  # base_z outside bounds
    ref_traj = ref_gen.get_reference(t=0.0, horizon=mpc.horizon, dt=mpc.dt)
    u0, q_pred, info = mpc.solve(q_current=q_bad, ref_traj=ref_traj)
    assert u0.shape == (10,)


def test_warm_start_consistency(mpc_setup):
    """Two consecutive solves should both return valid shapes."""
    robot, mpc, ref_gen = mpc_setup
    q0 = robot.q_nominal.copy()
    dt = mpc.dt
    for step in range(3):
        ref_traj = ref_gen.get_reference(t=step * dt, horizon=mpc.horizon, dt=dt)
        u0, q_pred, info = mpc.solve(q_current=q0, ref_traj=ref_traj)
        assert u0.shape == (10,)
        q0 = q_pred[1].copy()
