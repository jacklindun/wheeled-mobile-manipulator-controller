#!/usr/bin/env python3
"""
Phase 4 Hybrid MPC Tuning Script

Systematically tune solver parameters and weights to improve convergence.
"""

import sys
from pathlib import Path

# Ensure repo path is on sys.path when running without pip install
_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

# Add aligator build path (compiled from source, not pip-installed)
_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]

import time
import numpy as np
import aligator

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder


def run_test(duration=5.0, render=False, horizon=20, dt=0.05, max_iters=50,
             mu_init=1e-1, tol=1e-2, weights=None):
    """
    Run hybrid MPC test with given parameters.

    Returns
    -------
    dict with keys: success_rate, avg_solve_time, ee_rms_error, converged_count
    """
    print(f"\n{'='*60}")
    print(f"Testing: horizon={horizon}, max_iters={max_iters}, mu_init={mu_init:.1e}, tol={tol:.1e}")
    if weights:
        print(f"Weights: {weights}")
    print(f"{'='*60}")

    # Models
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    # MuJoCo environment (hybrid mode - xml path is hardcoded inside)
    env = MujocoWheeledUR5eHybridEnv(render=render)

    # MPC problem builder
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        weights=weights or {},
        use_hard_state_bounds=False,
    )

    # Reference: stationary at nominal EE position
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)

    # Build static reference trajectory
    num_steps = int(duration / dt)
    ref_traj = {
        "ee_pos": np.tile(p_ee_nominal, (num_steps + horizon + 1, 1)),
        "ee_rot": np.tile(R_ee_nominal, (num_steps + horizon + 1, 1, 1)),
        "base": np.zeros((num_steps + horizon + 1, 3)),
        "base_z": np.full(num_steps + horizon + 1, 0.2),
    }

    # Solver
    solver = aligator.SolverProxDDP(tol, mu_init=mu_init, max_iters=max_iters)

    # Initial state: [q_nominal(10), v_arm=0(6)]
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    # Warm-start buffers
    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    # Control loop
    mujoco_substeps = int(dt / env.model.opt.timestep)

    converged_count = 0
    solve_times = []
    ee_errors = []

    for step in range(num_steps):
        t = step * dt

        # Current state from MuJoCo
        x_current = env.get_state()

        # Build OCP
        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        # Solve
        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start
        solve_times.append(t_solve)

        if solver.results.conv:
            converged_count += 1

        # Extract control
        u0 = np.array(solver.results.us[0])

        # Apply to MuJoCo
        env.set_control(u0)
        env.set_target_marker(ref_traj["ee_pos"][step])
        env.step(mujoco_substeps)

        # Track error
        ee_pos = env.get_ee_pos()
        ee_err = np.linalg.norm(ee_pos - ref_traj["ee_pos"][step])
        ee_errors.append(ee_err)

        # Warm-start: shift + hold
        xs_sol = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_sol = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        xs_prev = xs_sol[1:] + [xs_sol[-1]]
        us_prev = us_sol[1:] + [us_sol[-1]]

        # Progress
        if (step + 1) % 20 == 0 or step == 0:
            conv_str = "✓" if solver.results.conv else "✗"
            print(f"  t={t:5.1f}s  ee_err={ee_err*100:5.1f}cm  solve={t_solve*1000:5.1f}ms  {conv_str}")

    env.close()

    # Results
    success_rate = (converged_count / num_steps) * 100
    avg_solve_time = np.mean(solve_times) * 1000  # ms
    ee_rms_error = np.sqrt(np.mean(np.array(ee_errors)**2)) * 100  # cm

    print(f"\n--- Results ---")
    print(f"Success rate:     {success_rate:.1f}% ({converged_count}/{num_steps})")
    print(f"Avg solve time:   {avg_solve_time:.1f} ms")
    print(f"EE RMS error:     {ee_rms_error:.2f} cm")

    return {
        "success_rate": success_rate,
        "avg_solve_time": avg_solve_time,
        "ee_rms_error": ee_rms_error,
        "converged_count": converged_count,
    }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", action="store_true", help="Show MuJoCo viewer")
    parser.add_argument("--duration", type=float, default=5.0, help="Test duration (s)")
    parser.add_argument("--sweep", action="store_true", help="Run parameter sweep")
    args = parser.parse_args()

    if args.sweep:
        print("\n" + "="*60)
        print("PHASE 4 HYBRID MPC PARAMETER SWEEP")
        print("="*60)

        results = []

        # Baseline (current settings from PROGRESS.md)
        print("\n### Baseline ###")
        res = run_test(duration=args.duration, render=False, horizon=20, dt=0.05,
                      max_iters=50, mu_init=1e-1, tol=1e-2)
        results.append(("Baseline", res))

        # Test 1: Smaller mu_init (tighter convergence)
        print("\n### Test 1: mu_init = 1e-4 ###")
        res = run_test(duration=args.duration, render=False, horizon=20, dt=0.05,
                      max_iters=50, mu_init=1e-4, tol=1e-2)
        results.append(("mu_init=1e-4", res))

        # Test 2: Increase tau regularization
        print("\n### Test 2: Higher tau weights ###")
        weights = {"tau_arm": 1.0, "dtau_arm": 1.0}
        res = run_test(duration=args.duration, render=False, horizon=20, dt=0.05,
                      max_iters=50, mu_init=1e-1, tol=1e-2, weights=weights)
        results.append(("tau_weight=1.0", res))

        # Test 3: Smaller horizon
        print("\n### Test 3: horizon = 10 ###")
        res = run_test(duration=args.duration, render=False, horizon=10, dt=0.05,
                      max_iters=50, mu_init=1e-1, tol=1e-2)
        results.append(("horizon=10", res))

        # Test 4: Tighter tolerance
        print("\n### Test 4: tol = 1e-4 ###")
        res = run_test(duration=args.duration, render=False, horizon=20, dt=0.05,
                      max_iters=50, mu_init=1e-1, tol=1e-4)
        results.append(("tol=1e-4", res))

        # Summary
        print("\n" + "="*60)
        print("SUMMARY")
        print("="*60)
        print(f"{'Config':<20} {'Success%':<12} {'Solve(ms)':<12} {'EE RMS(cm)':<12}")
        print("-"*60)
        for name, res in results:
            print(f"{name:<20} {res['success_rate']:>10.1f}% {res['avg_solve_time']:>10.1f}ms {res['ee_rms_error']:>10.2f}cm")

    else:
        # Single test
        run_test(duration=args.duration, render=args.render)
