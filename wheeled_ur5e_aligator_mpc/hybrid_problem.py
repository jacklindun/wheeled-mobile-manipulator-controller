"""
Hybrid kino-dynamic MPC problem builder (Phase 4).

Builds ALIGATOR OCP for the 16-dim hybrid state space:
  x = [q_base(4), q_arm(6), v_arm(6)] = 16
  u = [v_base(4), tau_arm(6)] = 10

Costs:
  - EE pose tracking (via pinocchio FK on q_base + q_arm)
  - Base pose tracking (q_base)
  - Arm posture regularization (q_arm)
  - Arm velocity regularization (v_arm)
  - Torque regularization (tau_arm)
  - Torque smoothness (delta tau_arm)

Dynamics: HybridWheeledUR5eDynamics (kinematic base + arm ABA)
Constraints: control box (v_base, tau_arm), optional state box
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

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.ee_pose_cost import EEPoseCost
from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics


# Default weights for hybrid MPC
DEFAULT_HYBRID_WEIGHTS = {
    "ee_pos": 100.0,
    "ee_ori": 0.0,
    "terminal_ee_pos": 200.0,
    "terminal_ee_ori": 0.0,
    "base_xy": 60.0,
    "base_yaw": 10.0,
    "base_z": 60.0,
    "arm_posture": 0.5,
    "terminal_arm_posture": 0.5,
    "v_arm": 0.01,      # arm velocity regularization
    "tau_arm": 0.001,   # torque regularization
    "dtau_arm": 0.01,   # torque smoothness
    "v_base": 0.01,     # base velocity regularization (same as Phase 1-3 "u")
}


class HybridWheeledUR5eProblemBuilder:
    """
    ALIGATOR OCP builder for hybrid kino-dynamic MPC.

    State: x = [q_base(4), q_arm(6), v_arm(6)] = 16
    Control: u = [v_base(4), tau_arm(6)] = 10

    Dynamics: kinematic base + arm ABA (semi-implicit Euler)
    """

    def __init__(
        self,
        robot: WheeledUR5eModel,
        pin_robot: PinocchioWheeledUR5eModel,
        horizon: int = 20,
        dt: float = 0.05,
        weights: dict | None = None,
        use_hard_state_bounds: bool = False,
    ):
        """
        Parameters
        ----------
        robot : WheeledUR5eModel
            Provides q_min, q_max, u_min, u_max, q_nominal (for reference).
        pin_robot : PinocchioWheeledUR5eModel
            Provides FK, Jacobian, and the reduced arm model for ABA.
        horizon : int
            MPC horizon N.
        dt : float
            Integration timestep.
        weights : dict | None
            Cost weights (merged with DEFAULT_HYBRID_WEIGHTS).
        use_hard_state_bounds : bool
            If True, add hard state box constraints.
        """
        self.robot = robot
        self.pin_robot = pin_robot
        self.horizon = horizon
        self.dt = dt
        self.weights = {**DEFAULT_HYBRID_WEIGHTS, **(weights or {})}
        self.use_hard_state_bounds = use_hard_state_bounds

        self._space = aligator.manifolds.VectorSpace(16)  # 16-dim state
        self._nu = 10
        self._dynamics = HybridWheeledUR5eDynamics(pin_robot, dt)

        # Control bounds: [v_base(4), tau_arm(6)]
        # v_base bounds from robot.u_min/max (first 4)
        # tau_arm bounds: assume ±100 Nm (typical UR5e joint torque limits)
        self._u_min = np.concatenate([robot.u_min[:4], np.full(6, -100.0)])
        self._u_max = np.concatenate([robot.u_max[:4], np.full(6, 100.0)])

    # ------------------------------------------------------------------
    # Cost builders
    # ------------------------------------------------------------------

    def _make_running_cost(
        self,
        p_ee_ref: np.ndarray,
        R_ee_ref: np.ndarray,
        base_ref: np.ndarray,
        base_z_ref: float,
        u_prev: np.ndarray | None,
    ) -> tuple:
        """Build running cost for one stage."""
        w = self.weights
        space = self._space
        nu = self._nu

        rcost = aligator.CostStack(space, nu)

        # 1. EE pose cost (reads q_base + q_arm from x)
        ee_cost = EEPoseCostHybrid(
            space, nu, self.pin_robot,
            w["ee_pos"], w["ee_ori"],
            p_ee_ref, R_ee_ref
        )
        rcost.addCost("ee_pose", ee_cost)

        # 2. Base pose tracking (q_base = x[:4])
        q_base_ref = np.array([base_ref[0], base_ref[1], base_z_ref, base_ref[2]])  # [x, y, z, yaw]
        W_base = np.zeros(16)
        W_base[0] = w["base_xy"]
        W_base[1] = w["base_xy"]
        W_base[2] = w["base_z"]
        W_base[3] = w["base_yaw"]
        x_base_ref = np.zeros(16)
        x_base_ref[:4] = q_base_ref
        x_base_ref[4:10] = self.robot.q_nominal[4:10]  # arm nominal
        rcost.addCost("base_pose", aligator.QuadraticStateCost(space, nu, x_base_ref, np.diag(W_base)))

        # 3. Arm posture regularization (q_arm = x[4:10])
        W_arm_posture = np.zeros(16)
        W_arm_posture[4:10] = w["arm_posture"]
        x_arm_ref = np.zeros(16)
        x_arm_ref[4:10] = self.robot.q_nominal[4:10]
        rcost.addCost("arm_posture", aligator.QuadraticStateCost(space, nu, x_arm_ref, np.diag(W_arm_posture)))

        # 4. Arm velocity regularization (v_arm = x[10:16])
        W_v_arm = np.zeros(16)
        W_v_arm[10:16] = w["v_arm"]
        rcost.addCost("v_arm_reg", aligator.QuadraticStateCost(space, nu, np.zeros(16), np.diag(W_v_arm)))

        # 5. Base velocity regularization (v_base = u[:4])
        W_v_base = np.eye(nu) * 0.0
        W_v_base[:4, :4] = np.eye(4) * w["v_base"]
        rcost.addCost("v_base_reg", aligator.QuadraticControlCost(space, np.zeros(nu), W_v_base))

        # 6. Torque regularization (tau_arm = u[4:10])
        W_tau = np.zeros((nu, nu))
        W_tau[4:10, 4:10] = np.eye(6) * w["tau_arm"]
        rcost.addCost("tau_arm_reg", aligator.QuadraticControlCost(space, np.zeros(nu), W_tau))

        # 7. Torque smoothness (delta tau_arm)
        if u_prev is not None:
            W_dtau = np.zeros((nu, nu))
            W_dtau[4:10, 4:10] = np.eye(6) * w["dtau_arm"]
            rcost.addCost("dtau_arm_reg", aligator.QuadraticControlCost(space, u_prev, W_dtau))

        return rcost, ee_cost

    def _make_terminal_cost(
        self, p_ee_ref: np.ndarray, R_ee_ref: np.ndarray
    ) -> tuple:
        """Build terminal cost."""
        w = self.weights
        space = self._space
        nu = self._nu

        term_cost = aligator.CostStack(space, nu)

        # Terminal EE pose cost
        ee_cost = EEPoseCostHybrid(
            space, nu, self.pin_robot,
            w["terminal_ee_pos"], w["terminal_ee_ori"],
            p_ee_ref, R_ee_ref
        )
        term_cost.addCost("terminal_ee_pose", ee_cost)

        # Terminal arm posture
        W_arm = np.zeros(16)
        W_arm[4:10] = w["terminal_arm_posture"]
        x_arm_ref = np.zeros(16)
        x_arm_ref[4:10] = self.robot.q_nominal[4:10]
        term_cost.addCost("terminal_arm_posture", aligator.QuadraticStateCost(space, nu, x_arm_ref, np.diag(W_arm)))

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
        Build TrajOptProblem for hybrid MPC.

        Parameters
        ----------
        x0 : (16,) initial state [q_base(4), q_arm(6), v_arm(6)]
        ref_traj : dict with keys:
            "ee_pos"  : (N+1, 3)
            "ee_rot"  : (N+1, 3, 3)
            "base"    : (N+1, 3) [base_x, base_y, base_yaw]
            "base_z"  : (N+1,)
        u_prev : (10,) previous control [v_base(4), tau_arm(6)], or None

        Returns
        -------
        problem : aligator.TrajOptProblem
        ee_costs : list of EEPoseCostHybrid (for reference updates)
        """
        N = self.horizon
        ee_pos_ref = ref_traj["ee_pos"]
        ee_rot_ref = ref_traj["ee_rot"]
        base_ref = ref_traj["base"]
        base_z_ref = ref_traj["base_z"]

        # Terminal cost
        term_cost, term_ee_cost = self._make_terminal_cost(ee_pos_ref[N], ee_rot_ref[N])

        # Build problem
        problem = aligator.TrajOptProblem(x0, self._nu, self._space, term_cost)

        all_ee_costs = []

        for k in range(N):
            u_prev_k = u_prev if k == 0 else None
            rcost, ee_cost = self._make_running_cost(
                ee_pos_ref[k],
                ee_rot_ref[k],
                base_ref[k],
                float(base_z_ref[k]),
                u_prev_k,
            )
            all_ee_costs.append(ee_cost)
            stage = aligator.StageModel(rcost, self._dynamics)

            # Hard control box constraint
            ctrl_res = aligator.ControlErrorResidual(self._space.ndx, np.zeros(self._nu))
            box_cstr = aligator.constraints.BoxConstraint(self._u_min, self._u_max)
            stage.addConstraint(ctrl_res, box_cstr)

            # Optional hard state box constraint (16-dim)
            if self.use_hard_state_bounds:
                # State bounds: [q_base_min/max, q_arm_min/max, v_arm_min/max]
                # q bounds from robot.q_min/max
                # v_arm bounds: assume ±10 rad/s (typical UR5e joint speed)
                x_min = np.concatenate([self.robot.q_min, np.full(6, -10.0)])
                x_max = np.concatenate([self.robot.q_max, np.full(6, 10.0)])
                state_res = aligator.StateErrorResidual(self._space, self._nu, np.zeros(16))
                state_box = aligator.constraints.BoxConstraint(x_min, x_max)
                stage.addConstraint(state_res, state_box)

            problem.addStage(stage)

        all_ee_costs.append(term_ee_cost)
        return problem, all_ee_costs


# ------------------------------------------------------------------
# EE pose cost adapted for 16-dim hybrid state
# ------------------------------------------------------------------

class EEPoseCostHybrid(aligator.CostAbstract):
    """
    EE pose cost for hybrid state x = [q_base(4), q_arm(6), v_arm(6)].

    Extracts q(10) = [q_base, q_arm] from x and computes pose residual
    via pinocchio FK, same as Phase 2 EEPoseCost.
    """

    def __init__(
        self,
        space_or_dim,
        nu: int,
        pin_robot: PinocchioWheeledUR5eModel,
        w_position: float,
        w_orientation: float,
        p_ref: np.ndarray | None = None,
        R_ref: np.ndarray | None = None,
    ):
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
        else:
            space = space_or_dim
        super().__init__(space, nu)
        self._space_nx = space.nx
        self._pin_robot = pin_robot
        self._w_p = float(w_position)
        self._w_o = float(w_orientation)
        self._p_ref = np.array(p_ref) if p_ref is not None else np.zeros(3)
        self._R_ref = np.array(R_ref) if R_ref is not None else np.eye(3)

    def __reduce__(self):
        return (
            self.__class__,
            (
                self._space_nx,
                self.nu,
                self._pin_robot,
                self._w_p,
                self._w_o,
                self._p_ref.copy(),
                self._R_ref.copy(),
            ),
        )

    def set_reference(self, p_ref: np.ndarray, R_ref: np.ndarray) -> None:
        self._p_ref[:] = p_ref
        self._R_ref[:] = R_ref

    def set_position_reference(self, p_ref: np.ndarray) -> None:
        self._p_ref[:] = p_ref

    def evaluate(self, x, u, data) -> None:
        import pinocchio as pin
        # Extract q(10) = [q_base(4), q_arm(6)] from x(16)
        q = np.asarray(x[:10])
        p, R = self._pin_robot.fk_pose(q)
        e_p = p - self._p_ref
        e_o = pin.log3(self._R_ref.T @ R) if self._w_o > 0 else np.zeros(3)
        data.value = 0.5 * self._w_p * float(np.dot(e_p, e_p)) + \
                     0.5 * self._w_o * float(np.dot(e_o, e_o))

    def computeGradients(self, x, u, data) -> None:
        import pinocchio as pin
        q = np.asarray(x[:10])
        p, R = self._pin_robot.fk_pose(q)
        e_p = p - self._p_ref
        e_o = pin.log3(self._R_ref.T @ R) if self._w_o > 0 else np.zeros(3)

        J = self._pin_robot.frame_jacobian(q, local=False)  # (6, 10)
        J_p = J[:3, :]
        J_o = J[3:, :]
        J_o_local = R.T @ J_o if self._w_o > 0 else np.zeros((3, 10))

        # Gradient wrt q(10)
        grad_q = self._w_p * (J_p.T @ e_p) + self._w_o * (J_o_local.T @ e_o)

        # Extend to 16-dim: gradient wrt x = [q(10), v_arm(6)]
        # Only q part contributes, v_arm part is zero
        data.Lx[:10] = grad_q
        data.Lx[10:] = 0.0
        data.Lu[:] = 0.0

    def computeHessians(self, x, u, data) -> None:
        q = np.asarray(x[:10])
        _, R = self._pin_robot.fk_pose(q)
        J = self._pin_robot.frame_jacobian(q, local=False)
        J_p = J[:3, :]
        J_o = J[3:, :]
        J_o_local = R.T @ J_o if self._w_o > 0 else np.zeros((3, 10))

        # Gauss-Newton Hessian wrt q(10)
        H_q = self._w_p * (J_p.T @ J_p) + self._w_o * (J_o_local.T @ J_o_local)

        # Extend to 16×16: top-left 10×10 block, rest zeros
        data.Lxx[:10, :10] = H_q
        data.Lxx[10:, :] = 0.0
        data.Lxx[:, 10:] = 0.0
        data.Luu[:] = 0.0
        data.Lxu[:] = 0.0
