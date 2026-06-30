"""
Unit tests for DualArmAligatorProblem.

Tests problem building, cost evaluation, and gradient computation.
"""

import numpy as np
import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheeled_ur5e_aligator_mpc.dual_arm_aligator_problem import (
    DualArmAligatorProblem,
    DualArmKinDynamics,
    DualEEPosCost,
)
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel

try:
    import aligator
    import aligator.manifolds
except ImportError:
    pytest.skip("aligator not available", allow_module_level=True)


@pytest.fixture
def pin_model():
    """Fixture providing DualArmPinocchioModel."""
    return DualArmPinocchioModel()


@pytest.fixture
def problem_builder():
    """Fixture providing DualArmAligatorProblem."""
    return DualArmAligatorProblem(horizon=10, dt=0.05)


class TestDynamics:
    """Test dual-arm kinematic dynamics."""

    def test_dynamics_integration(self):
        """Test simple Euler integration."""
        space = aligator.manifolds.VectorSpace(16)
        dyn = DualArmKinDynamics(space, dt=0.1)
        data = dyn.createData()

        q = np.zeros(16)
        u = np.ones(16) * 0.1

        dyn.forward(q, u, data)

        # q_next = q + dt * u = 0 + 0.1 * 0.1 = 0.01
        assert np.allclose(data.xnext, np.ones(16) * 0.01)

    def test_dynamics_linearization(self):
        """Test Jacobians A and B."""
        space = aligator.manifolds.VectorSpace(16)
        dt = 0.05
        dyn = DualArmKinDynamics(space, dt)
        data = dyn.createData()

        q = np.random.randn(16)
        u = np.random.randn(16)

        dyn.dForward(q, u, data)

        # A = I, B = dt * I
        assert np.allclose(data.Jx, np.eye(16))
        assert np.allclose(data.Ju, np.eye(16) * dt)

    def test_yaw_wrapping(self):
        """Test yaw angle wrapping to [-π, π]."""
        space = aligator.manifolds.VectorSpace(16)
        dyn = DualArmKinDynamics(space, dt=0.1)
        data = dyn.createData()

        q = np.zeros(16)
        q[2] = 3.0  # yaw close to π
        u = np.zeros(16)
        u[2] = 5.0  # velocity that would push yaw > π

        dyn.forward(q, u, data)

        # Yaw should wrap back to [-π, π]
        assert -np.pi <= data.xnext[2] <= np.pi


class TestDualEECost:
    """Test dual EE position cost."""

    def test_cost_evaluation_at_target(self, pin_model):
        """Cost should be zero when both EEs are at target."""
        space = aligator.manifolds.VectorSpace(16)
        nu = 16

        q = pin_model.get_q_nominal()
        p_left = pin_model.fk_left_ee(q)
        p_right = pin_model.fk_right_ee(q)

        cost = DualEEPosCost(space, nu, pin_model, 1000.0, 1000.0, p_left, p_right)
        data = cost.createData()

        cost.evaluate(q, np.zeros(nu), data)

        assert np.isclose(data.value, 0.0, atol=1e-6)

    def test_cost_evaluation_with_error(self, pin_model):
        """Cost should increase quadratically with error."""
        space = aligator.manifolds.VectorSpace(16)
        nu = 16

        q = pin_model.get_q_nominal()
        p_left = pin_model.fk_left_ee(q)
        p_right = pin_model.fk_right_ee(q)

        # Set targets 0.1m away from actual positions
        p_left_ref = p_left + np.array([0.1, 0.0, 0.0])
        p_right_ref = p_right + np.array([0.0, 0.1, 0.0])

        w_left = 1000.0
        w_right = 2000.0
        cost = DualEEPosCost(space, nu, pin_model, w_left, w_right, p_left_ref, p_right_ref)
        data = cost.createData()

        cost.evaluate(q, np.zeros(nu), data)

        # Expected: 0.5 * 1000 * 0.1² + 0.5 * 2000 * 0.1²
        expected = 0.5 * w_left * 0.01 + 0.5 * w_right * 0.01
        assert np.isclose(data.value, expected, rtol=1e-3)

    def test_cost_gradient_shape(self, pin_model):
        """Gradient should have correct shape."""
        space = aligator.manifolds.VectorSpace(16)
        nu = 16

        q = pin_model.get_q_nominal()
        p_left = pin_model.fk_left_ee(q)
        p_right = pin_model.fk_right_ee(q)

        cost = DualEEPosCost(space, nu, pin_model, 1000.0, 1000.0, p_left, p_right)
        data = cost.createData()

        cost.computeGradients(q, np.zeros(nu), data)

        assert data.Lx.shape == (16,)
        assert data.Lu.shape == (16,)

    def test_cost_hessian_shape(self, pin_model):
        """Hessian should have correct shape."""
        space = aligator.manifolds.VectorSpace(16)
        nu = 16

        q = pin_model.get_q_nominal()
        p_left = pin_model.fk_left_ee(q)
        p_right = pin_model.fk_right_ee(q)

        cost = DualEEPosCost(space, nu, pin_model, 1000.0, 1000.0, p_left, p_right)
        data = cost.createData()

        cost.computeHessians(q, np.zeros(nu), data)

        assert data.Lxx.shape == (16, 16)
        assert data.Luu.shape == (16, 16)
        assert data.Lxu.shape == (16, 16)

    def test_gradient_finite_diff(self, pin_model):
        """Verify gradient with finite differences."""
        space = aligator.manifolds.VectorSpace(16)
        nu = 16

        q = pin_model.get_q_nominal()
        p_left = pin_model.fk_left_ee(q) + np.array([0.05, 0.0, 0.0])
        p_right = pin_model.fk_right_ee(q) + np.array([0.0, 0.05, 0.0])

        cost = DualEEPosCost(space, nu, pin_model, 1000.0, 1000.0, p_left, p_right)
        data = cost.createData()

        cost.computeGradients(q, np.zeros(nu), data)
        grad_analytic = data.Lx.copy()

        # Finite differences
        eps = 1e-7
        grad_fd = np.zeros(16)
        for i in range(16):
            q_plus = q.copy()
            q_plus[i] += eps
            cost.evaluate(q_plus, np.zeros(nu), data)
            cost_plus = data.value

            cost.evaluate(q, np.zeros(nu), data)
            cost_0 = data.value

            grad_fd[i] = (cost_plus - cost_0) / eps

        assert np.allclose(grad_analytic, grad_fd, atol=1e-4)


class TestProblemBuilder:
    """Test DualArmAligatorProblem."""

    def test_initialization(self, problem_builder):
        """Test problem builder initialization."""
        assert problem_builder.horizon == 10
        assert problem_builder.dt == 0.05
        assert problem_builder.q_nominal.shape == (16,)

    def test_build_simple_problem(self, problem_builder):
        """Build a simple problem with straight-line trajectories."""
        N = problem_builder.horizon
        x0 = problem_builder.q_nominal

        # Straight-line trajectories
        p_left_traj = np.tile([0.4, 0.3, 0.0], (N+1, 1))
        p_right_traj = np.tile([0.4, -0.3, 0.0], (N+1, 1))

        problem = problem_builder.build(x0, p_left_traj, p_right_traj)

        assert isinstance(problem, aligator.TrajOptProblem)
        assert len(problem.stages) == N
        assert problem.num_steps == N

    def test_build_with_base_trajectory(self, problem_builder):
        """Build problem with base movement."""
        N = problem_builder.horizon
        x0 = problem_builder.q_nominal

        # EE trajectories
        p_left_traj = np.zeros((N+1, 3))
        p_right_traj = np.zeros((N+1, 3))

        # Base moves forward
        base_traj = np.zeros((N+1, 3))
        base_traj[:, 0] = np.linspace(0, 1.0, N+1)  # x from 0 to 1

        problem = problem_builder.build(
            x0, p_left_traj, p_right_traj,
            base_traj=base_traj
        )

        assert len(problem.stages) == N

    def test_get_ee_positions(self, problem_builder):
        """Test EE position helper."""
        q = problem_builder.q_nominal
        p_left, p_right = problem_builder.get_ee_positions(q)

        assert p_left.shape == (3,)
        assert p_right.shape == (3,)

        # Should match direct FK calls
        pin = problem_builder._pin
        assert np.allclose(p_left, pin.fk_left_ee(q))
        assert np.allclose(p_right, pin.fk_right_ee(q))


class TestWeights:
    """Test weight configuration."""

    def test_default_weights(self, problem_builder):
        """Check default weights are set."""
        w = problem_builder.weights
        assert "ee_left" in w
        assert "ee_right" in w
        assert w["ee_left"] > 0
        assert w["ee_right"] > 0

    def test_custom_weights(self):
        """Test custom weight override."""
        custom_weights = {
            "ee_left": 5000.0,
            "ee_right": 3000.0,
        }
        builder = DualArmAligatorProblem(weights=custom_weights)

        assert builder.weights["ee_left"] == 5000.0
        assert builder.weights["ee_right"] == 3000.0
        # Other defaults should still be present
        assert "base_xy" in builder.weights


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
