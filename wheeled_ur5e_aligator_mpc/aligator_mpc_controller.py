"""
AligatorWholeBodyMPC: MPC controller using ALIGATOR SolverProxDDP.

MPC loop each cycle:
  1. Shift warm-start trajectory (xs, us) by one step
  2. Rebuild TrajOptProblem with updated initial state and reference
  3. Run SolverProxDDP for max_iters_per_mpc iterations
  4. Extract u0 = results.us[0] as control command
  5. Fallback to previous u0 on solver failure

ALIGATOR version: 0.19.0
Solver: aligator.SolverProxDDP
  - results.traj_cost  (not results.cost)
  - results.conv       (bool)
  - results.num_iters
  - np.array(results.xs) shape (N+1, nx)
  - np.array(results.us) shape (N, nu)
"""

import sys
import time
from pathlib import Path

import numpy as np

try:
    import aligator
    import aligator.dynamics
    import aligator.manifolds
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [
        str(_repo_root / "build" / "bindings" / "python"),
        str(_repo_root / "bindings" / "python"),
    ]
    import aligator
    import aligator.dynamics
    import aligator.manifolds

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder


class AligatorWholeBodyMPC:
    """
    Kinematic whole-body MPC for wheeled UR5e using ALIGATOR ProxDDP.

    Parameters
    ----------
    robot : WheeledUR5eModel
    horizon : int
        MPC prediction horizon (number of steps).
    dt : float
        MPC discretization timestep (s).
    weights : dict or None
        Cost weights (see KinematicWheeledUR5eProblemBuilder.DEFAULT_WEIGHTS).
    max_iters : int
        Maximum ProxDDP iterations per MPC cycle (real-time iteration mode).
    """

    def __init__(
        self,
        robot: WheeledUR5eModel,
        horizon: int = 20,
        dt: float = 0.05,
        weights: dict | None = None,
        max_iters: int = 10,
    ):
        self.robot = robot
        self.horizon = horizon
        self.dt = dt
        self.max_iters = max_iters

        self._builder = KinematicWheeledUR5eProblemBuilder(robot, horizon, dt, weights)

        # SolverProxDDP: tol=1e-4, mu_init=1e-4, Gauss-Newton Hessian, quiet.
        # mu_init is the initial augmented-Lagrangian penalty parameter. A large
        # value (e.g. 1e-2) leaves the constraint penalty soft, so primal_infeas
        # plateaus near mu and the KKT convergence test (tol=1e-4) is never met
        # within the real-time iteration cap — solves look "unconverged" despite
        # being usable. mu_init=1e-4 lets ProxDDP reach genuine convergence in
        # ~2-3 iterations, which both raises the success rate to ~100% and lowers
        # the per-step solve time (fewer effective iterations).
        self._solver = aligator.SolverProxDDP(
            1e-4,
            mu_init=1e-4,
            max_iters=max_iters,
        )

        # Warm-start trajectory buffers
        self._xs_warm: list[np.ndarray] | None = None
        self._us_warm: list[np.ndarray] | None = None

        # Last successful control
        self._u_last: np.ndarray = np.zeros(robot.nu)

    # ------------------------------------------------------------------
    # Warm-start management
    # ------------------------------------------------------------------

    def _init_warm_start(self, q0: np.ndarray) -> tuple[list, list]:
        """Initialize warm-start with zeros."""
        xs = [q0.copy() for _ in range(self.horizon + 1)]
        us = [np.zeros(self.robot.nu) for _ in range(self.horizon)]
        return xs, us

    def _shift_warm_start(
        self,
        xs_prev: list[np.ndarray],
        us_prev: list[np.ndarray],
    ) -> tuple[list, list]:
        """Shift trajectory by one step (MPC receding horizon)."""
        xs_new = xs_prev[1:] + [xs_prev[-1].copy()]
        us_new = us_prev[1:] + [us_prev[-1].copy()]
        return xs_new, us_new

    # ------------------------------------------------------------------
    # Main solve interface
    # ------------------------------------------------------------------

    def solve(
        self,
        q_current: np.ndarray,
        ref_traj: dict,
        u_prev: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, dict]:
        """
        Solve one MPC step.

        Parameters
        ----------
        q_current : (10,) current joint state
        ref_traj  : dict{"ee_pos":(N+1,3), "base":(N+1,3), "base_z":(N+1,)}
        u_prev    : (10,) previous applied control (for delta-u cost), or None

        Returns
        -------
        u0     : (10,) control command for this step
        q_pred : (N+1, 10) predicted state trajectory
        info   : dict with keys success, status, solve_time, cost, iter, fallback
        """
        t_start = time.perf_counter()
        fallback = False

        # Prepare warm start
        if self._xs_warm is None:
            xs_init, us_init = self._init_warm_start(q_current)
        else:
            xs_init, us_init = self._shift_warm_start(self._xs_warm, self._us_warm)
        # Fix initial state to current measurement
        xs_init[0] = q_current.copy()

        try:
            # Build OCP
            problem, _ = self._builder.build_problem(q_current, ref_traj, u_prev)

            # Setup and run solver
            self._solver.setup(problem)
            self._solver.run(problem, xs_init, us_init)

            res = self._solver.results
            success = bool(res.conv)
            status = "converged" if success else "max_iters"
            n_iters = int(res.num_iters)
            cost = float(res.traj_cost)

            # Extract optimal trajectory
            xs_opt = list(np.array(res.xs))
            us_opt = list(np.array(res.us))

            u0 = us_opt[0].copy()
            q_pred = np.array(res.xs)  # (N+1, 10)

            # Apply control bounds (safety clip)
            u0 = np.clip(u0, self.robot.u_min, self.robot.u_max)

            # Update warm start
            self._xs_warm = xs_opt
            self._us_warm = us_opt
            self._u_last = u0.copy()

        except Exception as e:
            # Fallback: use last successful control
            fallback = True
            success = False
            status = f"error: {e}"
            n_iters = 0
            cost = None
            u0 = self._u_last.copy()
            q_pred = np.tile(q_current, (self.horizon + 1, 1))

        solve_time = time.perf_counter() - t_start

        info = {
            "success": success,
            "status": status,
            "solve_time": solve_time,
            "cost": cost,
            "iter": n_iters,
            "fallback": fallback,
        }

        return u0, q_pred, info

    def reset(self) -> None:
        """Clear warm-start state (e.g., on reset of simulation)."""
        self._xs_warm = None
        self._us_warm = None
        self._u_last = np.zeros(self.robot.nu)

    def close(self) -> None:
        """Explicitly release ALIGATOR C++ objects in controlled order.

        This prevents a segfault during Python interpreter shutdown caused by
        non-deterministic destruction order of C++ extension objects (ALIGATOR,
        MuJoCo, Pinocchio).
        """
        self._xs_warm = None
        self._us_warm = None
        del self._solver
        self._builder.close()
        del self._builder
        del self.robot
