"""
Minimal hybrid kino-dynamic MPC demo (Phase 4).

Runs a short test (5 seconds) to verify:
- 16-dim state space works end-to-end
- Solver converges in closed loop
- MuJoCo hybrid environment is stable

Usage:
  python scripts/run_hybrid_demo.py [--render]
"""

import sys
import os
from pathlib import Path

import numpy as np

# Add repo root to path
_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(_REPO_ROOT / "build" / "bindings" / "python"),
    str(_REPO_ROOT / "study_example" / "wheeled_ur5e_aligator_mpc"),
]

import aligator

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


def run_hybrid_demo(duration: float = 5.0, render: bool = False):
    """
    Run a minimal hybrid MPC demo.

    Parameters
    ----------
    duration : float
        Simulation duration in seconds.
    render : bool
        Enable MuJoCo rendering.
    """
    print("[Hybrid Demo] Initializing...")

    # Models
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    # MuJoCo environment (hybrid mode)
    env = MujocoWheeledUR5eHybridEnv(render=render)

    # MPC problem builder
    horizon = 20
    dt = 0.05  # 20 Hz MPC
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        weights={"ee_pos": 100.0, "ee_ori": 0.0},  # position-only for first test
        use_hard_state_bounds=False,
    )

    # Reference: stationary at nominal EE position
    # For this minimal test, use a fixed reference (no moving target)
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)

    # Build static reference trajectory
    num_steps = int(duration / dt)
    ref_traj = {
        "ee_pos": np.tile(p_ee_nominal, (num_steps + horizon + 1, 1)),
        "ee_rot": np.tile(R_ee_nominal, (num_steps + horizon + 1, 1, 1)),
        "base": np.zeros((num_steps + horizon + 1, 3)),  # [x, y, yaw]
        "base_z": np.full(num_steps + horizon + 1, 0.2),
    }

    # Solver (more lenient for first hybrid test)
    solver = aligator.SolverProxDDP(1e-2, mu_init=1e-1, max_iters=50)

    # Initial state: [q_nominal(10), v_arm=0(6)]
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    # Set initial target marker to match EE position
    env.set_target_marker(ref_traj["ee_pos"][0])

    # Warm-start buffers
    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    # Control loop
    print(f"[Hybrid Demo] Running {duration:.1f}s @ {1.0/dt:.0f} Hz MPC...")
    num_steps = int(duration / dt)
    mujoco_substeps = int(dt / env.model.opt.timestep)

    success_count = 0
    solve_times = []

    for step in range(num_steps):
        t = step * dt

        # Current state from MuJoCo
        x_current = env.get_state()

        # Build OCP
        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        # Solve
        solver.setup(problem)
        import time
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start
        solve_times.append(t_solve)

        if solver.results.conv:
            success_count += 1

        # Extract control
        u0 = np.array(solver.results.us[0])

        # Apply to MuJoCo
        env.set_control(u0)
        env.set_target_marker(ref_traj["ee_pos"][step])
        env.step(mujoco_substeps)

        # Warm-start: shift + hold
        xs_sol = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_sol = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        xs_prev = xs_sol[1:] + [xs_sol[-1]]
        us_prev = us_sol[1:] + [us_sol[-1]]

        # Progress
        if (step + 1) % 20 == 0 or step == 0:
            ee_pos = env.get_ee_pos()
            ee_err = np.linalg.norm(ee_pos - ref_traj["ee_pos"][step]) * 100
            conv_str = "ok" if solver.results.conv else "FAIL"
            print(f"  t={t:5.1f}s  ee_err={ee_err:5.1f}cm  solve={t_solve*1000:5.1f}ms  {conv_str}")

    env.close()

    # Summary
    print("\n===== Hybrid Demo Summary =====")
    print(f"Duration:          {duration:.1f} s")
    print(f"MPC success rate:  {100.0 * success_count / num_steps:.1f} %")
    print(f"Avg solve time:    {1000 * np.mean(solve_times):.1f} ms")
    print(f"Max solve time:    {1000 * np.max(solve_times):.1f} ms")

    # Final EE error
    x_final = env.get_state()
    ee_final = pin_robot.fk_numpy(x_final[:10])
    ee_ref_final = ref_traj["ee_pos"][-1]
    ee_err_final = np.linalg.norm(ee_final - ee_ref_final) * 100
    print(f"Final EE error:    {ee_err_final:.2f} cm")

    return success_count == num_steps


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Hybrid kino-dynamic MPC demo")
    parser.add_argument("--render", action="store_true", help="Enable MuJoCo rendering")
    parser.add_argument("--duration", type=float, default=5.0, help="Duration in seconds")
    args = parser.parse_args()

    success = run_hybrid_demo(duration=args.duration, render=args.render)
    sys.exit(0 if success else 1)
