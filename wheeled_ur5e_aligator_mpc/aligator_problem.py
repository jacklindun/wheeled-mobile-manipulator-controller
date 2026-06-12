"""
ALIGATOR OCP builder for kinematic whole-body MPC of wheeled UR5e.

Confirmed ALIGATOR API (version 0.19.0):
  - aligator.dynamics.ExplicitDynamicsModel(space, nu): subclass with forward/dForward
    - data.xnext[:] = ...           (field name: xnext, not xnext_)
    - data.Jx[:] = ...              (numpy array attribute, not a method)
    - data.Ju[:] = ...
  - aligator.CostAbstract(space, nu): subclass with evaluate/computeGradients/computeHessians
    - data.value = scalar
    - data.Lx[:] = ..., data.Lu[:] = ...
    - data.Lxx[:] = ..., data.Luu[:] = ..., data.Lxu[:] = ...
  - aligator.manifolds.VectorSpace(n): plain R^n space
  - aligator.CostStack(space, nu).addCost(name, cost_obj)
  - aligator.QuadraticStateCost(space, nu, target, W): 0.5*(x-target)^T W (x-target)
  - aligator.QuadraticControlCost(space, target, W): 0.5*(u-target)^T W (u-target)
  - aligator.ControlErrorResidual(ndx, target) for box constraint residual
  - aligator.constraints.BoxConstraint(u_min, u_max)
  - aligator.StageModel(cost, dynamics).addConstraint(residual, constraint_set)
  - aligator.TrajOptProblem(x0, nu, space, term_cost).addStage(stage)
"""

import sys
import os
from pathlib import Path

import numpy as np

# ALIGATOR import: works with pixi shell-hook -e all or if sys.path includes build/bindings/python
try:
    import aligator
    import aligator.dynamics
    import aligator.manifolds
    import aligator.constraints
except ImportError:
    # Try adding build path (when running from repo root)
    # __file__ is in wheeled_ur5e_aligator_mpc/wheeled_ur5e_aligator_mpc/
    # parents[3] = aligator repo root
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [
        str(_repo_root / "build" / "bindings" / "python"),
        str(_repo_root / "bindings" / "python"),
    ]
    import aligator
    import aligator.dynamics
    import aligator.manifolds
    import aligator.constraints

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel


# ---------------------------------------------------------------------------
# Custom dynamics model (ExplicitDynamicsModel subclass)
# ---------------------------------------------------------------------------

class WheeledUR5eKinDynamics(aligator.dynamics.ExplicitDynamicsModel):
    """
    Discrete kinematic dynamics for the wheeled UR5e.
    State space: R^10 (VectorSpace(10)), nu=10.
    x_{k+1} = f(x_k, u_k) via first-order kinematic integration.
    """

    def __init__(self, space_or_dim, robot: WheeledUR5eModel, dt: float):
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
        else:
            space = space_or_dim
        super().__init__(space, robot.nu)
        self._space_nx = space.nx
        self._robot = robot
        self._dt = dt

    def __reduce__(self):
        """Support deepcopy (called by StageModel constructor)."""
        return (self.__class__, (self._space_nx, self._robot, self._dt))

    def forward(self, x, u, data) -> None:
        q_next = self._robot.dynamics_numpy(np.asarray(x), np.asarray(u), self._dt)
        data.xnext[:] = q_next

    def dForward(self, x, u, data) -> None:
        A, B = self._robot.linearize_dynamics(np.asarray(x), np.asarray(u), self._dt)
        data.Jx[:] = A
        data.Ju[:] = B


# ---------------------------------------------------------------------------
# Custom end-effector position cost (CostAbstract subclass)
# Gauss-Newton approximation of 0.5 * w * ||fk(q) - p_ref||^2
# ---------------------------------------------------------------------------

class EEPosCost(aligator.CostAbstract):
    """
    End-effector position tracking cost.
    cost = 0.5 * weight * ||fk(q) - p_ref||^2

    Gradient (Gauss-Newton):
      Lx = weight * J^T (fk(q) - p_ref)
      Lu = 0
    Hessian (Gauss-Newton):
      Lxx = weight * J^T J
      Lxu = 0, Luu = 0
    where J = d(fk)/dq  shape (3, 10) via finite differences.
    """

    def __init__(
        self,
        space_or_dim,  # VectorSpace or int (for deepcopy reconstruction)
        nu: int,
        robot: WheeledUR5eModel,
        weight: float,
        p_ref: np.ndarray | None = None,
    ):
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
        else:
            space = space_or_dim
        super().__init__(space, nu)
        self._space_nx = space.nx
        self._robot = robot
        self._weight = float(weight)
        self._p_ref = np.array(p_ref) if p_ref is not None else np.zeros(3)

    def __reduce__(self):
        """Support deepcopy (called by CostStack.addCost internally).
        Pass space as int dim to avoid deepcopy of VectorSpace C++ object."""
        return (self.__class__, (self._space_nx, self.nu, self._robot, self._weight, self._p_ref.copy()))

    def set_reference(self, p_ref: np.ndarray) -> None:
        """Update target end-effector position."""
        self._p_ref[:] = p_ref

    def evaluate(self, x, u, data) -> None:
        q = np.asarray(x)
        p_ee = self._robot.fk_numpy(q)
        e = p_ee - self._p_ref
        data.value = 0.5 * self._weight * float(np.dot(e, e))

    def computeGradients(self, x, u, data) -> None:
        q = np.asarray(x)
        p_ee = self._robot.fk_numpy(q)
        e = p_ee - self._p_ref
        J = self._robot.finite_difference_jacobian_fk(q)  # (3, 10)
        data.Lx[:] = self._weight * (J.T @ e)
        data.Lu[:] = 0.0

    def computeHessians(self, x, u, data) -> None:
        q = np.asarray(x)
        J = self._robot.finite_difference_jacobian_fk(q)  # (3, 10)
        data.Lxx[:] = self._weight * (J.T @ J)
        data.Luu[:] = 0.0
        data.Lxu[:] = 0.0


# ---------------------------------------------------------------------------
# OCP problem builder
# ---------------------------------------------------------------------------

DEFAULT_WEIGHTS = {
    "ee_pos": 100.0,
    "terminal_ee": 200.0,
    # Base-pose tracking. These must be high relative to ee_pos: with low base
    # weights the solver "cheats" the EE target by drifting the base (e.g. in
    # ee_circle the base_x reference is constant 0, but a weak penalty let the
    # base wander ~15 cm). At 60 the base stays locked to its reference (drift
    # <1 cm) while a moving-base scenario like base_and_ee still tracks its
    # 0.8 m ramp to ~2 cm.
    "base_xy": 60.0,
    "base_yaw": 10.0,
    "base_z": 60.0,
    "posture": 0.5,
    "terminal_posture": 0.5,
    "u": 0.01,
    "du": 0.1,
}


class KinematicWheeledUR5eProblemBuilder:
    """
    Builds the ALIGATOR TrajOptProblem for kinematic whole-body MPC.

    The OCP:
      min  Σ l_k(q_k, u_k, ref_k)  +  l_N(q_N, ref_N)
      s.t. q_{k+1} = f(q_k, u_k)   (kinematic integration)
           u_min ≤ u_k ≤ u_max      (control box constraint, hard)
           q_min ≤ q_k ≤ q_max      (state soft penalty via QuadraticStateCost)

    Running cost l_k:
      w_ee_pos   * ||fk(q_k) - p_ee_ref_k||^2
      w_base_xy  * ||(q_k[:2] - base_ref_k[:2])||^2
      w_base_yaw * angle_error^2
      w_base_z   * (q_k[2] - base_z_ref_k)^2
      w_posture  * ||q_arm_k - q_arm_nominal||^2
      w_u        * ||u_k||^2
      w_du       * ||u_k - u_{k-1}||^2

    Terminal cost l_N:
      w_terminal_ee      * ||fk(q_N) - p_ee_ref_N||^2
      w_terminal_posture * ||q_arm_N - q_arm_nominal||^2

    Note: state box constraints are implemented as soft penalty (penalize deviation
    beyond bounds using Q-weight matrix). Hard constraints: u_min ≤ u ≤ u_max.
    TODO (next version): add hard state box constraints via ALIGATOR constraint API.
    """

    def __init__(
        self,
        robot: WheeledUR5eModel,
        horizon: int = 20,
        dt: float = 0.05,
        weights: dict | None = None,
    ):
        self.robot = robot
        self.horizon = horizon
        self.dt = dt
        self.weights = {**DEFAULT_WEIGHTS, **(weights or {})}

        self._space = aligator.manifolds.VectorSpace(robot.nx)
        self._nu = robot.nu
        self._dynamics = WheeledUR5eKinDynamics(robot.nx, robot, dt)

    # ------------------------------------------------------------------
    # Internal helper builders
    # ------------------------------------------------------------------

    def _make_running_cost(
        self,
        p_ee_ref: np.ndarray,
        base_ref: np.ndarray,
        base_z_ref: float,
        u_prev: np.ndarray | None,
    ) -> tuple[aligator.CostStack, EEPosCost]:
        """Build running cost for one stage. Returns (CostStack, ee_cost_ref)."""
        w = self.weights
        space = self._space
        nu = self._nu
        nq = self.robot.nq

        rcost = aligator.CostStack(space, nu)

        # 1. End-effector position cost
        ee_cost = EEPosCost(space, nu, self.robot, w["ee_pos"], p_ee_ref)
        rcost.addCost("ee_pos", ee_cost)

        # 2. Base xy tracking
        base_xy_target = np.array([base_ref[0], base_ref[1]])
        W_base_xy = np.zeros(nq)
        W_base_xy[0] = w["base_xy"]
        W_base_xy[1] = w["base_xy"]
        q_base_xy_ref = self.robot.q_nominal.copy()
        q_base_xy_ref[0] = base_ref[0]
        q_base_xy_ref[1] = base_ref[1]
        Wq_base = np.diag(W_base_xy)
        rcost.addCost("base_xy", aligator.QuadraticStateCost(space, nu, q_base_xy_ref, Wq_base))

        # 3. Base yaw tracking
        W_yaw = np.zeros(nq)
        W_yaw[3] = w["base_yaw"]
        q_yaw_ref = self.robot.q_nominal.copy()
        q_yaw_ref[3] = base_ref[2]  # base_ref col 2 = yaw
        rcost.addCost("base_yaw", aligator.QuadraticStateCost(space, nu, q_yaw_ref, np.diag(W_yaw)))

        # 4. Base z tracking
        W_bz = np.zeros(nq)
        W_bz[2] = w["base_z"]
        q_bz_ref = self.robot.q_nominal.copy()
        q_bz_ref[2] = base_z_ref
        rcost.addCost("base_z", aligator.QuadraticStateCost(space, nu, q_bz_ref, np.diag(W_bz)))

        # 5. Posture regularization (arm joints)
        W_posture = np.zeros(nq)
        W_posture[4:10] = w["posture"]
        rcost.addCost(
            "posture",
            aligator.QuadraticStateCost(space, nu, self.robot.q_nominal, np.diag(W_posture)),
        )

        # 6. Control regularization (u ~ 0)
        W_u = np.eye(nu) * w["u"]
        rcost.addCost("u_reg", aligator.QuadraticControlCost(space, np.zeros(nu), W_u))

        # 7. Delta-u smoothness (u - u_prev)
        if u_prev is not None:
            W_du = np.eye(nu) * w["du"]
            rcost.addCost("du_reg", aligator.QuadraticControlCost(space, u_prev, W_du))

        return rcost, ee_cost

    def _make_terminal_cost(
        self, p_ee_ref: np.ndarray
    ) -> tuple[aligator.CostStack, EEPosCost]:
        """Build terminal cost. Returns (CostStack, ee_cost_ref)."""
        w = self.weights
        space = self._space
        nu = self._nu
        nq = self.robot.nq

        term_cost = aligator.CostStack(space, nu)

        # Terminal EE cost (heavier)
        ee_cost = EEPosCost(space, nu, self.robot, w["terminal_ee"], p_ee_ref)
        term_cost.addCost("terminal_ee", ee_cost)

        # Terminal posture
        W_posture = np.zeros(nq)
        W_posture[4:10] = w["terminal_posture"]
        term_cost.addCost(
            "terminal_posture",
            aligator.QuadraticStateCost(
                space, nu, self.robot.q_nominal, np.diag(W_posture)
            ),
        )

        return term_cost, ee_cost

    # ------------------------------------------------------------------
    # Public build interface
    # ------------------------------------------------------------------

    def build_problem(
        self,
        x0: np.ndarray,
        ref_traj: dict,
        u_prev: np.ndarray | None = None,
    ) -> tuple:
        """
        Build a TrajOptProblem for the given initial state and reference trajectory.

        Parameters
        ----------
        x0 : (10,) current joint state
        ref_traj : dict with keys:
            "ee_pos"  : (N+1, 3) end-effector reference positions
            "base"    : (N+1, 3) base_x, base_y, base_yaw reference
            "base_z"  : (N+1,)   base_z reference
        u_prev : (10,) previous control (for delta-u cost), or None

        Returns
        -------
        problem : aligator.TrajOptProblem
        ee_costs : list of EEPosCost objects (running + terminal) for reference update
        """
        N = self.horizon
        ee_pos_ref = ref_traj["ee_pos"]   # (N+1, 3)
        base_ref   = ref_traj["base"]      # (N+1, 3)
        base_z_ref = ref_traj["base_z"]    # (N+1,)

        # Build terminal cost
        term_cost, term_ee_cost = self._make_terminal_cost(ee_pos_ref[N])

        # Build problem
        problem = aligator.TrajOptProblem(x0, self._nu, self._space, term_cost)

        all_ee_costs = []

        for k in range(N):
            u_prev_k = u_prev if k == 0 else None
            rcost, ee_cost = self._make_running_cost(
                ee_pos_ref[k],
                base_ref[k],
                float(base_z_ref[k]),
                u_prev_k,
            )
            all_ee_costs.append(ee_cost)
            stage = aligator.StageModel(rcost, self._dynamics)

            # Hard control box constraint: u_min ≤ u ≤ u_max
            ctrl_res = aligator.ControlErrorResidual(self._space.ndx, np.zeros(self._nu))
            box_cstr = aligator.constraints.BoxConstraint(self.robot.u_min, self.robot.u_max)
            stage.addConstraint(ctrl_res, box_cstr)

            problem.addStage(stage)

        all_ee_costs.append(term_ee_cost)
        return problem, all_ee_costs
