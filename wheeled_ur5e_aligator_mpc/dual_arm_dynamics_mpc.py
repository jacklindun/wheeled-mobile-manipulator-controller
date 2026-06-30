"""
Phase 6-v3: Dual-arm DYNAMICS MPC using ALIGATOR

Key differences from kinematic MPC:
- State: x = [q, v] ∈ R^32 (not just q)
- Control: u = τ ∈ R^16 (torque, not velocity)
- Dynamics: forward dynamics with Coriolis, gravity, inertia
"""

import sys
from pathlib import Path

import numpy as np

try:
    import aligator
    import pinocchio as pin
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [
        str(_repo_root / "build" / "bindings" / "python"),
    ]
    import aligator
    import pinocchio as pin

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel


# Default weights for dynamics MPC (EE-only, legacy)
DEFAULT_WEIGHTS_DYNAMICS = {
    "ee_left": 1000.0,
    "ee_right": 1000.0,
    "state_reg": 0.01,
    "control": 0.1,
    "control_rate": 0.01,
    "terminal_ee_left": 5000.0,
    "terminal_ee_right": 5000.0,
    "terminal_state": 10.0,
}

# IK-informed weights: joint reference dominates, EE refines, fixed base
DEFAULT_WEIGHTS_IK_INFORMED = {
    "ee_left": 200.0,
    "ee_right": 200.0,
    "q_track": 50.0,
    "v_track": 5.0,
    "base_q_track": 500.0,
    "base_v_track": 100.0,
    "state_reg": 0.0,
    "control": 0.01,
    "control_rate": 0.001,
    "terminal_ee_left": 500.0,
    "terminal_ee_right": 500.0,
    "terminal_q_track": 100.0,
    "terminal_v_track": 10.0,
    "terminal_state": 0.0,
}


class DualArmDynamicsMPC:
    """
    Dual-arm dynamics MPC using ALIGATOR.

    State: x = [q, v] ∈ R^32
      q: [base(4), left_arm(6), right_arm(6)] = 16
      v: [base(4), left_arm(6), right_arm(6)] = 16

    Control: u = τ ∈ R^16 (joint torques)

    Dynamics: M(q)v̇ + C(q,v) + g(q) = τ
    """

    def __init__(
        self,
        mjcf_path: str = 'assets/wheeled_dual_ur5e_v2.xml',
        horizon: int = 15,
        dt: float = 0.05,
        weights: dict | None = None,
        weight_preset: str = "legacy",
        fix_base: bool = False,
    ):
        self.horizon = horizon
        self.dt = dt
        preset = DEFAULT_WEIGHTS_IK_INFORMED if weight_preset == "ik_informed" else DEFAULT_WEIGHTS_DYNAMICS
        self.weights = {**preset, **(weights or {})}
        self.fix_base = fix_base

        # Load Pinocchio model
        self.pin_model = DualArmPinocchioModel(mjcf_path)
        self.model = self.pin_model.model
        self.data = self.pin_model.data

        # Dimensions
        self.nq = self.model.nq  # 16
        self.nv = self.model.nv  # 16
        self.nx = self.nq + self.nv  # 32
        self.nu = self.nv  # 16 (torque)

        # Control limits (torque limits in N·m, qpos order: x, y, yaw, z, arms...)
        # 放宽限制，留有余量
        self.u_min = np.array([
            -250, -250, -150, -1200,  # base: 增加25%余量
            -180, -180, -180, -35, -35, -35,  # left arm: 增加20%余量
            -180, -180, -180, -35, -35, -35,  # right arm
        ])
        self.u_max = np.array([
            250, 250, 150, 1200,  # base
            180, 180, 180, 35, 35, 35,  # left arm
            180, 180, 180, 35, 35, 35,  # right arm
        ])

        # ALIGATOR spaces
        self.space = aligator.manifolds.MultibodyPhaseSpace(self.model)

        # Nominal configuration (qpos order: x, y, yaw, z, arms...)
        self.q_nominal = np.array([
            0.0, 0.0, 0.0, 0.2,  # base: x, y, yaw, z (修正：z=0.2)
            -2.5434, -0.6884,  1.6850, 0.4209, -1.3484,  0.0000,  # left
             1.4529, -0.7472,  2.3605, 0.3727, -1.9646,  0.0000,  # right
        ])
        self.v_nominal = np.zeros(self.nv)
        self.x_nominal = np.concatenate([self.q_nominal, self.v_nominal])

        # Create dynamics model
        self.ode = aligator.dynamics.MultibodyFreeFwdDynamics(self.space)
        self.dyn_model = aligator.dynamics.IntegratorSemiImplEuler(self.ode, self.dt)

        # Persistent solver and warm start cache
        self.solver = None
        self.xs_warm = None  # (N+1, 32)
        self.us_warm = None  # (N, 16)

    def _state_weight_matrix(self, w_q: float, w_v: float, terminal: bool = False) -> np.ndarray:
        """Diagonal W for QuadraticStateCost; optional fixed-base boost."""
        w = self.weights
        W = np.zeros((self.nx, self.nx))
        W[: self.nq, : self.nq] = np.eye(self.nq) * w_q
        W[self.nq :, self.nq :] = np.eye(self.nv) * w_v
        if self.fix_base:
            bq = w.get("terminal_base_q_track" if terminal else "base_q_track", 500.0)
            bv = w.get("terminal_base_v_track" if terminal else "base_v_track", 100.0)
            W[:4, :4] = np.eye(4) * bq
            W[self.nq : self.nq + 4, self.nq : self.nq + 4] = np.eye(4) * bv
        return W

    def create_ee_tracking_cost(
        self,
        target_left: np.ndarray,
        target_right: np.ndarray,
        is_terminal: bool = False
    ):
        """
        Create end-effector tracking cost.

        Uses FrameTranslationResidual for efficient computation.
        """
        w = self.weights
        w_left = w["terminal_ee_left"] if is_terminal else w["ee_left"]
        w_right = w["terminal_ee_right"] if is_terminal else w["ee_right"]

        cost_stack = aligator.CostStack(self.space, self.nu)

        # Left EE tracking
        left_frame_id = self.pin_model.left_ee_frame_id
        res_left = aligator.FrameTranslationResidual(
            self.space.ndx,
            self.nu,
            self.model,
            target_left,
            left_frame_id
        )
        # 权重在这里设置，不在 addCost 中重复
        cost_left = aligator.QuadraticResidualCost(self.space, res_left, np.eye(3) * w_left)
        cost_stack.addCost("ee_left", cost_left)

        # Right EE tracking
        right_frame_id = self.pin_model.right_ee_frame_id
        res_right = aligator.FrameTranslationResidual(
            self.space.ndx,
            self.nu,
            self.model,
            target_right,
            right_frame_id
        )
        cost_right = aligator.QuadraticResidualCost(self.space, res_right, np.eye(3) * w_right)
        cost_stack.addCost("ee_right", cost_right)

        return cost_stack

    def create_running_cost(
        self,
        target_left: np.ndarray,
        target_right: np.ndarray,
        x_ref: np.ndarray | None = None,
        u_ref_k: np.ndarray | None = None,  # 单个时刻的控制
    ):
        """Create running cost for one stage."""
        w = self.weights

        cost_stack = self.create_ee_tracking_cost(target_left, target_right, is_terminal=False)

        if x_ref is None:
            x_ref = self.x_nominal

        w_q = w.get("q_track", 0.0) + w["state_reg"]
        w_v = w.get("v_track", 0.0) + w["state_reg"]
        if w_q > 0 or w_v > 0:
            W_state = self._state_weight_matrix(w_q, w_v, terminal=False)
            state_cost = aligator.QuadraticStateCost(self.space, self.nu, x_ref, W_state)
            cost_stack.addCost("state_track", state_cost)

        # Control regularization
        u_zero = np.zeros(self.nu)
        W_control = np.eye(self.nu) * w["control"]
        control_cost = aligator.QuadraticControlCost(self.space, u_zero, W_control)
        cost_stack.addCost("control", control_cost)

        # Control rate (if u_ref_k provided)
        if u_ref_k is not None and w["control_rate"] > 0:
            W_rate = np.eye(self.nu) * w["control_rate"]
            rate_cost = aligator.QuadraticControlCost(self.space, u_ref_k, W_rate)
            cost_stack.addCost("control_rate", rate_cost)

        return cost_stack

    def create_terminal_cost(
        self,
        target_left: np.ndarray,
        target_right: np.ndarray,
        x_ref: np.ndarray | None = None,
    ):
        """Create terminal cost."""
        w = self.weights

        cost_stack = self.create_ee_tracking_cost(target_left, target_right, is_terminal=True)

        if x_ref is None:
            x_ref = self.x_nominal

        w_q = w.get("terminal_q_track", 0.0) + w["terminal_state"]
        w_v = w.get("terminal_v_track", 0.0) + w["terminal_state"]
        if w_q > 0 or w_v > 0:
            W_terminal = self._state_weight_matrix(w_q, w_v, terminal=True)
            terminal_cost = aligator.QuadraticStateCost(self.space, self.nu, x_ref, W_terminal)
            cost_stack.addCost("terminal_state_track", terminal_cost)

        return cost_stack

    def build_problem(
        self,
        x0: np.ndarray,
        target_left_traj: np.ndarray,  # (N+1, 3)
        target_right_traj: np.ndarray, # (N+1, 3)
        u_prev: np.ndarray | None = None,
        x_ref_traj: np.ndarray | None = None,
    ):
        """
        Build ALIGATOR shooting problem.

        Parameters
        ----------
        x0 : (32,) array
            Initial state [q0, v0]
        target_left_traj : (N+1, 3) array
            Left EE target trajectory
        target_right_traj : (N+1, 3) array
            Right EE target trajectory
        u_prev : (N, 16) array, optional
            Previous control trajectory for warm start

        Returns
        -------
        problem : aligator.TrajOptProblem
        """
        N = self.horizon

        # Create stages
        stages = []

        for k in range(N):
            u_ref_k = u_prev[k] if u_prev is not None else None
            x_ref_k = x_ref_traj[k] if x_ref_traj is not None else None
            cost_k = self.create_running_cost(
                target_left_traj[k],
                target_right_traj[k],
                x_ref=x_ref_k,
                u_ref_k=u_ref_k,
            )

            # Create stage
            stage = aligator.StageModel(cost_k, self.dyn_model)

            # 暂时注释掉约束，测试是否影响收敛
            # # Add control (torque) constraints
            # u_constraint = aligator.constraints.BoxConstraint(self.u_min, self.u_max)
            # u_residual = aligator.ControlErrorResidual(self.space.ndx, self.nu)
            # stage.addConstraint(u_residual, u_constraint)

            stages.append(stage)

        # Terminal cost
        x_ref_N = x_ref_traj[N] if x_ref_traj is not None else None
        term_cost = self.create_terminal_cost(
            target_left_traj[N],
            target_right_traj[N],
            x_ref=x_ref_N,
        )

        # Build problem (terminal cost, not terminal constraint)
        problem = aligator.TrajOptProblem(x0, stages, term_cost)

        return problem

    def solve(
        self,
        x0: np.ndarray,
        target_left_traj: np.ndarray,
        target_right_traj: np.ndarray,
        u_init: np.ndarray | None = None,
        x_ref_traj: np.ndarray | None = None,
        max_iters: int = 10,
        verbose: bool = False,
    ):
        """
        Solve the OCP and return the trajectory.

        Returns
        -------
        xs : (N+1, 32) array
            State trajectory
        us : (N, 16) array
            Control trajectory (torques)
        results : solver results
            包含收敛信息等
        """
        # Build problem
        problem = self.build_problem(
            x0, target_left_traj, target_right_traj, u_init, x_ref_traj=x_ref_traj,
        )

        # Create or reuse solver
        if self.solver is None:
            tol = 1e-4
            mu_init = 1e-8
            verbose_level = aligator.VerboseLevel.VERBOSE if verbose else aligator.VerboseLevel.QUIET
            self.solver = aligator.SolverProxDDP(tol, mu_init, verbose=verbose_level)
            self.solver.max_iters = max_iters
            self.solver.rollout_type = aligator.ROLLOUT_LINEAR
        else:
            # 更新迭代次数（可能会变）
            self.solver.max_iters = max_iters

        # Warm start: prefer IK reference trajectory when available
        if x_ref_traj is not None and len(x_ref_traj) == self.horizon + 1:
            xs_init = [x_ref_traj[i].copy() for i in range(self.horizon + 1)]
            us_init = []
            for k in range(self.horizon):
                q_k = x_ref_traj[k, : self.nq]
                v_k = x_ref_traj[k, self.nq :]
                v_kp1 = x_ref_traj[k + 1, self.nq :]
                a_k = (v_kp1 - v_k) / self.dt
                us_init.append(pin.rnea(self.model, self.data, q_k, v_k, a_k))
        elif self.xs_warm is not None and self.us_warm is not None:
            xs_init = [self.xs_warm[i] for i in range(len(self.xs_warm))]
            us_init = [self.us_warm[i] for i in range(len(self.us_warm))]
        else:
            xs_init = []
            us_init = []

        # Solve
        self.solver.setup(problem)
        self.solver.run(problem, xs_init, us_init)

        # Extract results
        xs = np.array(self.solver.results.xs)
        us = np.array(self.solver.results.us)

        # Cache for next warm start
        self.xs_warm = xs.copy()
        self.us_warm = us.copy()

        return xs, us, self.solver.results


if __name__ == '__main__':
    """Test dual-arm dynamics MPC"""
    print("=" * 60)
    print("Phase 6-v3: Dual-Arm Dynamics MPC Test")
    print("=" * 60)

    # Create MPC
    mpc = DualArmDynamicsMPC(
        mjcf_path='assets/wheeled_dual_ur5e_v2.xml',
        horizon=15,
        dt=0.05
    )

    print(f"State dim: {mpc.nx} (q={mpc.nq}, v={mpc.nv})")
    print(f"Control dim: {mpc.nu}")
    print(f"Horizon: {mpc.horizon}")
    print(f"dt: {mpc.dt}")

    # Test problem building
    x0 = mpc.x_nominal.copy()
    target_left = np.array([0.6, 0.3, 0.8])
    target_right = np.array([0.6, -0.3, 0.8])

    target_left_traj = np.tile(target_left, (mpc.horizon + 1, 1))
    target_right_traj = np.tile(target_right, (mpc.horizon + 1, 1))

    print(f"\nBuilding problem...")
    problem = mpc.build_problem(x0, target_left_traj, target_right_traj)
    print(f"Problem created: {problem.num_steps} stages")

    print(f"\nSolving...")
    xs, us, results = mpc.solve(x0, target_left_traj, target_right_traj, max_iters=20, verbose=True)

    print(f"\nResults:")
    print(f"  Iterations: {results.num_iters}")
    print(f"  Final cost: {results.traj_cost:.6f}")
    print(f"  xs shape: {xs.shape}")
    print(f"  us shape: {us.shape}")
    print(f"  Torque range: [{us.min():.2f}, {us.max():.2f}] N·m")
    print(f"\n✓ Dynamics MPC module works!")
