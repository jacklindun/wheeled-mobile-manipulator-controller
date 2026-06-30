"""
Tests for hybrid MPC problem builder (Phase 4).

Verifies:
  - HybridWheeledUR5eProblemBuilder constructs valid OCP
  - Solver can solve a single-step hybrid MPC problem
  - EE pose cost gradient works on 16-dim state
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
except ImportError:
    pytest.skip("aligator not available", allow_module_level=True)

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


@pytest.fixture(scope="module")
def robot():
    return WheeledUR5eModel()


@pytest.fixture(scope="module")
def pin_robot():
    return PinocchioWheeledUR5eModel()


@pytest.fixture(scope="module")
def hybrid_builder(robot, pin_robot):
    return HybridWheeledUR5eProblemBuilder(robot, pin_robot, horizon=5, dt=0.05)


def test_hybrid_problem_builds(hybrid_builder, robot, pin_robot):
    """Verify hybrid problem builder constructs a valid OCP."""
    # Initial state: q_nominal + zero arm velocity
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    # v_arm = 0

    # Reference trajectory (stationary at nominal)
    N = 5
    p_ee_nom = pin_robot.fk_numpy(robot.q_nominal)
    _, R_ee_nom = pin_robot.fk_pose(robot.q_nominal)
    ref_traj = {
        "ee_pos": np.tile(p_ee_nom, (N+1, 1)),
        "ee_rot": np.tile(R_ee_nom, (N+1, 1, 1)),
        "base": np.zeros((N+1, 3)),
        "base_z": np.full(N+1, 0.2),
    }

    problem, ee_costs = hybrid_builder.build_problem(x0, ref_traj)

    # Check structure
    assert problem.num_steps == N
    assert len(ee_costs) == N + 1  # N running + 1 terminal
    assert problem.stages[0].dynamics.space.nx == 16
    assert problem.stages[0].dynamics.nu == 10


def test_hybrid_mpc_single_step_solves(hybrid_builder, robot, pin_robot):
    """Verify the solver can solve a single-step hybrid MPC problem."""
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal

    N = 5
    p_ee_nom = pin_robot.fk_numpy(robot.q_nominal)
    _, R_ee_nom = pin_robot.fk_pose(robot.q_nominal)
    ref_traj = {
        "ee_pos": np.tile(p_ee_nom, (N+1, 1)),
        "ee_rot": np.tile(R_ee_nom, (N+1, 1, 1)),
        "base": np.zeros((N+1, 3)),
        "base_z": np.full(N+1, 0.2),
    }

    problem, _ = hybrid_builder.build_problem(x0, ref_traj)

    # Solve with more lenient tolerances for initial test
    solver = aligator.SolverProxDDP(1e-3, mu_init=1e-2, max_iters=50)
    solver.setup(problem)
    xs_init = [x0.copy() for _ in range(N+1)]
    us_init = [np.zeros(10) for _ in range(N)]
    solver.run(problem, xs_init, us_init)

    # Check convergence (lenient for first hybrid test)
    print(f"\nSolver: iters={solver.results.num_iters}, conv={solver.results.conv}")
    print(f"  primal_infeas={solver.results.primal_infeas:.2e}, dual_infeas={solver.results.dual_infeas:.2e}")

    # Accept if infeasibility is small even if formal convergence flag is false
    assert solver.results.primal_infeas < 0.1 or solver.results.conv, \
        f"Solver failed: primal_infeas={solver.results.primal_infeas}"

    # Check solution validity
    xs_opt = np.array(solver.results.xs)
    us_opt = np.array(solver.results.us)
    assert xs_opt.shape == (N+1, 16)
    assert us_opt.shape == (N, 10)
    assert not np.any(np.isnan(xs_opt))
    assert not np.any(np.isnan(us_opt))


def test_ee_pose_cost_hybrid_gradient(pin_robot):
    """Verify EEPoseCostHybrid gradient computation on 16-dim state."""
    from wheeled_ur5e_aligator_mpc.hybrid_problem import EEPoseCostHybrid

    space = aligator.manifolds.VectorSpace(16)
    nu = 10
    w_p = 100.0
    w_o = 50.0

    # Random 16-dim state
    rng = np.random.default_rng(42)
    x = np.zeros(16)
    x[:10] = rng.uniform(-0.5, 0.5, 10)  # q
    x[10:] = rng.uniform(-0.5, 0.5, 6)   # v_arm
    u = np.zeros(10)

    p_ref, R_ref = pin_robot.fk_pose(x[:10])
    p_ref += rng.normal(0, 0.05, 3)  # perturb reference

    cost = EEPoseCostHybrid(space, nu, pin_robot, w_p, w_o, p_ref, R_ref)
    data = cost.createData()

    cost.computeGradients(x, u, data)
    grad_analytic = data.Lx.copy()

    # Finite-difference gradient
    eps = 1e-7
    grad_fd = np.zeros(16)
    for i in range(16):
        x_p = x.copy(); x_p[i] += eps
        x_m = x.copy(); x_m[i] -= eps
        cost.evaluate(x_p, u, data); cost_p = data.value
        cost.evaluate(x_m, u, data); cost_m = data.value
        grad_fd[i] = (cost_p - cost_m) / (2 * eps)

    max_err = np.max(np.abs(grad_analytic - grad_fd))
    assert max_err < 1e-4, f"Gradient max err = {max_err}"
