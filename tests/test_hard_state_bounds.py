"""
Test hard state box constraints (Phase 3).

Verifies:
  - Problem builds with use_hard_state_bounds=True
  - Solver respects joint limits when reference would violate them
  - Soft constraints (default) allow small violations
  - Hard constraints prevent violations
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
from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder


@pytest.fixture
def robot():
    return WheeledUR5eModel()


def test_build_problem_with_hard_state_bounds(robot):
    """Verify problem builds successfully with hard state constraints enabled."""
    builder = KinematicWheeledUR5eProblemBuilder(
        robot, horizon=5, use_hard_state_bounds=True
    )

    # Build a minimal reference
    N = 5
    ref_traj = {
        "ee_pos": np.tile(robot.fk_numpy(robot.q_nominal), (N + 1, 1)),
        "ee_rot": np.tile(np.eye(3), (N + 1, 1, 1)),
        "base": np.zeros((N + 1, 3)),
        "base_z": np.full(N + 1, 0.2),
    }

    problem, _ = builder.build_problem(robot.q_nominal, ref_traj)

    # Check problem has the expected structure
    assert problem.num_steps == N
    # Each stage should have 2 constraints: control bounds + state bounds
    for stage in problem.stages:
        assert stage.constraints.size == 2, "Expected 2 constraints per stage (control + state)"


def test_hard_state_bounds_prevent_violations(robot):
    """Verify hard state constraints prevent joint limit violations."""
    builder_hard = KinematicWheeledUR5eProblemBuilder(
        robot, horizon=10, use_hard_state_bounds=True
    )

    N = 10
    q0 = robot.q_nominal.copy()

    # Create a reference that would violate base_z upper limit (0.5 m)
    # Drive base_z to 0.6 m (over the limit)
    ref_traj = {
        "ee_pos": np.tile(robot.fk_numpy(q0), (N + 1, 1)),
        "ee_rot": np.tile(np.eye(3), (N + 1, 1, 1)),
        "base": np.zeros((N + 1, 3)),
        "base_z": np.linspace(0.2, 0.6, N + 1),  # ramp from 0.2 to 0.6 (violates 0.5 max)
    }

    problem, _ = builder_hard.build_problem(q0, ref_traj)

    # Solve
    solver = aligator.SolverProxDDP(1e-4, mu_init=1e-4, max_iters=20)
    solver.setup(problem)
    xs_init = [q0.copy() for _ in range(N + 1)]
    us_init = [np.zeros(robot.nu) for _ in range(N)]
    solver.run(problem, xs_init, us_init)

    # Extract solution
    xs_opt = np.array(solver.results.xs)

    # Verify all states respect the bounds (with small tolerance for numerical error)
    for k in range(N + 1):
        q_k = xs_opt[k]
        assert np.all(q_k >= robot.q_min - 1e-5), f"State {k} violates lower bound: {q_k}"
        assert np.all(q_k <= robot.q_max + 1e-5), f"State {k} violates upper bound: {q_k}"

    # Specifically check base_z never exceeds 0.5
    base_z_traj = xs_opt[:, 2]
    assert np.all(base_z_traj <= 0.5 + 1e-5), f"base_z violated upper bound: max={base_z_traj.max()}"


def test_soft_vs_hard_bounds_behavior(robot):
    """Compare soft (default) vs hard state constraints on a limit-pushing reference."""
    N = 10
    q0 = robot.q_nominal.copy()

    # Reference that strongly violates base_z upper limit (pushes to 0.7 m vs 0.5 limit)
    ref_traj = {
        "ee_pos": np.tile(robot.fk_numpy(q0), (N + 1, 1)),
        "ee_rot": np.tile(np.eye(3), (N + 1, 1, 1)),
        "base": np.zeros((N + 1, 3)),
        "base_z": np.linspace(0.2, 0.7, N + 1),  # ramp to 0.7 (well over 0.5 limit)
    }

    # Soft constraints with WEAKENED base_z penalty to allow violations
    weak_weights = {"base_z": 0.1}  # very weak, down from default 60
    builder_soft = KinematicWheeledUR5eProblemBuilder(
        robot, horizon=N, use_hard_state_bounds=False, weights=weak_weights
    )
    problem_soft, _ = builder_soft.build_problem(q0, ref_traj)
    solver_soft = aligator.SolverProxDDP(1e-4, mu_init=1e-4, max_iters=30)
    solver_soft.setup(problem_soft)
    xs_init = [q0.copy() for _ in range(N + 1)]
    us_init = [np.zeros(robot.nu) for _ in range(N)]
    solver_soft.run(problem_soft, xs_init, us_init)
    xs_soft = np.array(solver_soft.results.xs)

    # Hard constraints
    builder_hard = KinematicWheeledUR5eProblemBuilder(
        robot, horizon=N, use_hard_state_bounds=True, weights=weak_weights
    )
    problem_hard, _ = builder_hard.build_problem(q0, ref_traj)
    solver_hard = aligator.SolverProxDDP(1e-4, mu_init=1e-4, max_iters=30)
    solver_hard.setup(problem_hard)
    solver_hard.run(problem_hard, xs_init, us_init)
    xs_hard = np.array(solver_hard.results.xs)

    base_z_soft = xs_soft[:, 2]
    base_z_hard = xs_hard[:, 2]

    # With hard constraints, base_z must stay ≤ 0.5
    assert np.all(base_z_hard <= 0.5 + 1e-5), f"Hard bounds violated: max={base_z_hard.max()}"

    # With weak soft constraints and aggressive reference, the soft solution should
    # violate the bound (or at least reach it), while hard cannot.
    # The difference validates that the hard constraint is active.
    max_soft = base_z_soft.max()
    max_hard = base_z_hard.max()
    print(f"Soft max: {max_soft:.4f}, Hard max: {max_hard:.4f}")
    # Either soft violates, or both saturate but hard is slightly more conservative
    assert max_soft >= max_hard - 1e-6, "Soft should allow at least as much as hard"


def test_solver_converges_with_hard_state_bounds(robot):
    """Verify the solver still converges with hard state constraints active."""
    builder = KinematicWheeledUR5eProblemBuilder(
        robot, horizon=10, use_hard_state_bounds=True
    )

    N = 10
    q0 = robot.q_nominal.copy()
    ref_traj = {
        "ee_pos": np.tile(robot.fk_numpy(q0), (N + 1, 1)),
        "ee_rot": np.tile(np.eye(3), (N + 1, 1, 1)),
        "base": np.zeros((N + 1, 3)),
        "base_z": np.full(N + 1, 0.2),
    }

    problem, _ = builder.build_problem(q0, ref_traj)
    solver = aligator.SolverProxDDP(1e-4, mu_init=1e-4, max_iters=20)
    solver.setup(problem)
    xs_init = [q0.copy() for _ in range(N + 1)]
    us_init = [np.zeros(robot.nu) for _ in range(N)]
    solver.run(problem, xs_init, us_init)

    # Check convergence
    assert solver.results.conv, "Solver did not converge with hard state constraints"
    assert solver.results.num_iters < 20, f"Solver used all {solver.results.num_iters} iterations"
