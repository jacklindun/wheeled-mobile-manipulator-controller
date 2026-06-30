"""
ALIGATOR OCP builder for dual-arm kinematic MPC of wheeled dual UR5e.

Extends the single-arm approach to 16-DOF dual-arm system with:
  - Independent left/right EE tracking costs
  - Configurable weight balance between arms
  - Optional coordinated manipulation constraints (future work)
"""

import sys
import os
from pathlib import Path

import numpy as np

try:
    import aligator
    import aligator.dynamics
    import aligator.manifolds
    import aligator.constraints
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [
        str(_repo_root / "build" / "bindings" / "python"),
        str(_repo_root / "bindings" / "python"),
    ]
    import aligator
    import aligator.dynamics
    import aligator.manifolds
    import aligator.constraints

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel


# ---------------------------------------------------------------------------
# Dual-arm kinematic dynamics
# ---------------------------------------------------------------------------

class DualArmKinDynamics(aligator.dynamics.ExplicitDynamicsModel):
    """
    Discrete kinematic dynamics for the dual-arm wheeled UR5e.
    State space: R^16 (VectorSpace(16)), nu=16.

    Simple kinematic integration: q_{k+1} = q_k + dt * u_k
    """

    def __init__(self, space_or_dim, dt: float):
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
        else:
            space = space_or_dim
        nu = 16
        super().__init__(space, nu)
        self._space_nx = space.nx
        self._dt = dt

    def __reduce__(self):
        """Support deepcopy (called by StageModel constructor)."""
        return (self.__class__, (self._space_nx, self._dt))

    def forward(self, x, u, data) -> None:
        """Simple Euler integration: q_next = q + dt * u"""
        q = np.asarray(x)
        v = np.asarray(u)
        q_next = q + self._dt * v

        # Clamp yaw to [-π, π]
        q_next[2] = np.arctan2(np.sin(q_next[2]), np.cos(q_next[2]))

        data.xnext[:] = q_next

    def dForward(self, x, u, data) -> None:
        """Linearization: A = I, B = dt * I"""
        data.Jx[:] = np.eye(16)
        data.Ju[:] = np.eye(16) * self._dt


# ---------------------------------------------------------------------------
# Dual EE position cost
# ---------------------------------------------------------------------------

class DualEEPosCost(aligator.CostAbstract):
    """
    Dual end-effector position tracking cost.

    cost = 0.5 * w_left * ||fk_left(q) - p_left_ref||^2
         + 0.5 * w_right * ||fk_right(q) - p_right_ref||^2

    Uses Gauss-Newton approximation for Hessian.
    """

    def __init__(
        self,
        space_or_dim,
        nu: int,
        pin_model: DualArmPinocchioModel,
        w_left: float,
        w_right: float,
        p_left_ref: np.ndarray,
        p_right_ref: np.ndarray,
    ):
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
            self._space_nx = space_or_dim
        else:
            space = space_or_dim
            self._space_nx = space.nx

        super().__init__(space, nu)
        self._pin = pin_model
        self._w_left = w_left
        self._w_right = w_right
        self._p_left_ref = np.asarray(p_left_ref).copy()
        self._p_right_ref = np.asarray(p_right_ref).copy()

    def __reduce__(self):
        """Support deepcopy (called by StageModel constructor)."""
        return (
            self.__class__,
            (
                self._space_nx,
                self.nu,
                self._pin,
                self._w_left,
                self._w_right,
                self._p_left_ref,
                self._p_right_ref,
            )
        )

    def evaluate(self, x, u, data) -> None:
        q = np.asarray(x)

        # Compute FK for both arms
        p_left = self._pin.fk_left_ee(q)
        p_right = self._pin.fk_right_ee(q)

        # Position errors
        e_left = p_left - self._p_left_ref
        e_right = p_right - self._p_right_ref

        # Total cost
        cost = 0.5 * self._w_left * np.dot(e_left, e_left) \
             + 0.5 * self._w_right * np.dot(e_right, e_right)

        data.value = cost

    def computeGradients(self, x, u, data) -> None:
        q = np.asarray(x)

        # FK and Jacobians
        p_left = self._pin.fk_left_ee(q)
        p_right = self._pin.fk_right_ee(q)
        J_left = self._pin.position_jacobian_left_ee(q)   # (3, 16)
        J_right = self._pin.position_jacobian_right_ee(q) # (3, 16)

        # Errors
        e_left = p_left - self._p_left_ref
        e_right = p_right - self._p_right_ref

        # Gradients: dL/dq = w_left * J_left^T * e_left + w_right * J_right^T * e_right
        data.Lx[:] = self._w_left * (J_left.T @ e_left) \
                   + self._w_right * (J_right.T @ e_right)
        data.Lu[:] = 0.0

    def computeHessians(self, x, u, data) -> None:
        q = np.asarray(x)

        # Jacobians
        J_left = self._pin.position_jacobian_left_ee(q)
        J_right = self._pin.position_jacobian_right_ee(q)

        # Gauss-Newton approximation: d²L/dq² ≈ J^T W J
        data.Lxx[:] = self._w_left * (J_left.T @ J_left) \
                    + self._w_right * (J_right.T @ J_right)
        data.Luu[:] = 0.0
        data.Lxu[:] = 0.0


# ---------------------------------------------------------------------------
# Default weights
# ---------------------------------------------------------------------------

DEFAULT_WEIGHTS = {
    # Dual EE tracking
    "ee_left": 1000.0,    # Left EE position tracking
    "ee_right": 1000.0,   # Right EE position tracking

    # Base tracking (shared for both arms)
    "base_xy": 10.0,
    "base_yaw": 5.0,
    "base_z": 50.0,

    # Posture regularization
    "posture": 0.1,       # Keep arms near nominal config

    # Control regularization
    "u": 0.01,            # Control effort
    "du": 0.1,            # Control smoothness

    # Terminal cost
    "terminal_ee_left": 2000.0,
    "terminal_ee_right": 2000.0,
    "terminal_posture": 1.0,
}


# ---------------------------------------------------------------------------
# MPC Problem Builder
# ---------------------------------------------------------------------------

class DualArmAligatorProblem:
    """
    Build ALIGATOR trajectory optimization problem for dual-arm kinematic MPC.

    State: q ∈ R^16  [base(4), left_arm(6), right_arm(6)]
    Control: u ∈ R^16 (joint velocities)

    Running cost l_k:
      w_ee_left   * ||p_left_k - p_left_ref_k||^2
      w_ee_right  * ||p_right_k - p_right_ref_k||^2
      w_base_xy   * ||base_xy_k - base_xy_ref_k||^2
      w_base_yaw  * |yaw_k - yaw_ref_k|^2
      w_base_z    * |z_k - z_ref_k|^2
      w_posture   * ||q_arm_k - q_arm_nominal||^2
      w_u         * ||u_k||^2
      w_du        * ||u_k - u_{k-1}||^2

    Terminal cost l_N:
      w_terminal_ee_left  * ||p_left_N - p_left_ref_N||^2
      w_terminal_ee_right * ||p_right_N - p_right_ref_N||^2
      w_terminal_posture  * ||q_arm_N - q_arm_nominal||^2
    """

    def __init__(
        self,
        horizon: int = 20,
        dt: float = 0.05,
        weights: dict | None = None,
    ):
        self.horizon = horizon
        self.dt = dt
        self.weights = {**DEFAULT_WEIGHTS, **(weights or {})}

        self._space = aligator.manifolds.VectorSpace(16)
        self._nu = 16
        self._dynamics = DualArmKinDynamics(16, dt)

        # Pinocchio model for FK/Jacobian
        self._pin = DualArmPinocchioModel()

        # Nominal configuration
        self.q_nominal = self._pin.get_q_nominal()

    # ------------------------------------------------------------------
    # Internal helper builders
    # ------------------------------------------------------------------

    def _make_running_cost(
        self,
        p_left_ref: np.ndarray,
        p_right_ref: np.ndarray,
        base_ref: np.ndarray,
        base_z_ref: float,
        u_prev: np.ndarray | None,
    ) -> aligator.CostStack:
        """Build running cost for one stage."""
        w = self.weights
        space = self._space
        nu = self._nu
        nq = 16

        rcost = aligator.CostStack(space, nu)

        # 1. Dual EE position cost
        ee_cost = DualEEPosCost(
            space, nu, self._pin,
            w["ee_left"], w["ee_right"],
            p_left_ref, p_right_ref
        )
        rcost.addCost("dual_ee_pos", ee_cost)

        # 2. Base xy tracking
        W_base_xy = np.zeros(nq)
        W_base_xy[0] = w["base_xy"]
        W_base_xy[1] = w["base_xy"]
        q_base_xy_ref = self.q_nominal.copy()
        q_base_xy_ref[0] = base_ref[0]
        q_base_xy_ref[1] = base_ref[1]
        Wq_base = np.diag(W_base_xy)
        rcost.addCost("base_xy", aligator.QuadraticStateCost(space, nu, q_base_xy_ref, Wq_base))

        # 3. Base yaw tracking
        W_yaw = np.zeros(nq)
        W_yaw[2] = w["base_yaw"]
        q_yaw_ref = self.q_nominal.copy()
        q_yaw_ref[2] = base_ref[2]  # base_ref[2] = yaw
        rcost.addCost("base_yaw", aligator.QuadraticStateCost(space, nu, q_yaw_ref, np.diag(W_yaw)))

        # 4. Base z tracking
        W_bz = np.zeros(nq)
        W_bz[3] = w["base_z"]
        q_bz_ref = self.q_nominal.copy()
        q_bz_ref[3] = base_z_ref
        rcost.addCost("base_z", aligator.QuadraticStateCost(space, nu, q_bz_ref, np.diag(W_bz)))

        # 5. Posture regularization (both arms)
        W_posture = np.zeros(nq)
        W_posture[4:16] = w["posture"]  # Both left and right arms
        rcost.addCost(
            "posture",
            aligator.QuadraticStateCost(space, nu, self.q_nominal, np.diag(W_posture)),
        )

        # 6. Control regularization (u ~ 0)
        W_u = np.eye(nu) * w["u"]
        rcost.addCost("u_reg", aligator.QuadraticControlCost(space, np.zeros(nu), W_u))

        # 7. Control smoothness (if u_prev given)
        if u_prev is not None and w["du"] > 0:
            W_du = np.eye(nu) * w["du"]
            rcost.addCost("du_reg", aligator.QuadraticControlCost(space, u_prev, W_du))

        return rcost

    def _make_terminal_cost(
        self,
        p_left_ref: np.ndarray,
        p_right_ref: np.ndarray,
    ) -> aligator.CostStack:
        """Build terminal cost."""
        w = self.weights
        space = self._space
        nu = self._nu
        nq = 16

        tcost = aligator.CostStack(space, nu)

        # 1. Terminal EE cost
        ee_term_cost = DualEEPosCost(
            space, nu, self._pin,
            w["terminal_ee_left"], w["terminal_ee_right"],
            p_left_ref, p_right_ref
        )
        tcost.addCost("terminal_dual_ee", ee_term_cost)

        # 2. Terminal posture
        W_term_posture = np.zeros(nq)
        W_term_posture[4:16] = w["terminal_posture"]
        tcost.addCost(
            "terminal_posture",
            aligator.QuadraticStateCost(space, nu, self.q_nominal, np.diag(W_term_posture)),
        )

        return tcost

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(
        self,
        x0: np.ndarray,
        p_left_traj: np.ndarray,
        p_right_traj: np.ndarray,
        base_traj: np.ndarray | None = None,
        base_z_traj: np.ndarray | None = None,
        u_prev_traj: np.ndarray | None = None,
    ) -> aligator.TrajOptProblem:
        """
        Build the trajectory optimization problem.

        Args:
            x0: Initial state (16,)
            p_left_traj: Left EE reference trajectory (N+1, 3)
            p_right_traj: Right EE reference trajectory (N+1, 3)
            base_traj: Base [x, y, yaw] reference (N+1, 3), default zeros
            base_z_traj: Base z reference (N+1,), default 0.2
            u_prev_traj: Previous control trajectory (N, 16), default zeros

        Returns:
            TrajOptProblem ready for ProxDDP solver
        """
        N = self.horizon
        x0 = np.asarray(x0)

        if base_traj is None:
            base_traj = np.zeros((N+1, 3))
        if base_z_traj is None:
            base_z_traj = np.full(N+1, 0.2)
        if u_prev_traj is None:
            u_prev_traj = np.zeros((N, self._nu))

        # Terminal cost
        term_cost = self._make_terminal_cost(p_left_traj[-1], p_right_traj[-1])

        # Create problem
        problem = aligator.TrajOptProblem(x0, self._nu, self._space, term_cost)

        # Add running stages
        for k in range(N):
            u_prev = u_prev_traj[k] if k < len(u_prev_traj) else None
            rcost = self._make_running_cost(
                p_left_traj[k],
                p_right_traj[k],
                base_traj[k],
                base_z_traj[k],
                u_prev,
            )

            stage = aligator.StageModel(rcost, self._dynamics)

            # Optional: Add control box constraints
            # u_min = np.full(self._nu, -0.5)
            # u_max = np.full(self._nu, 0.5)
            # stage.addConstraint(
            #     aligator.ControlErrorResidual(self._space.ndx, np.zeros(self._nu)),
            #     aligator.constraints.BoxConstraint(u_min, u_max)
            # )

            problem.addStage(stage)

        return problem

    def get_ee_positions(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Helper: compute both EE positions from configuration.

        Returns: (p_left, p_right) each (3,)
        """
        return self._pin.fk_left_ee(q), self._pin.fk_right_ee(q)
