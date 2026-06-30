#!/usr/bin/env python3
"""
Phase 4 Hybrid MPC Tuning - Round 2

Based on Round 1 findings:
- horizon=10 showed improvement (1% success, 37ms solve time)
- tau_weight=1.0 was too high (system unstable)
- mu_init and tol adjustments had no effect

Round 2 strategy:
- Test smaller horizons (5, 8)
- Test more iterations (100, 200)
- Test modest tau weight increases (0.01, 0.1)
- Test combinations of best settings
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

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
    """Run hybrid MPC test with given parameters."""

    print(f"\n{'='*60}")
    print(f"Testing: horizon={horizon}, max_iters={max_iters}, mu_init={mu_init:.1e}, tol={tol:.1e}")
    if weights:
        print(f"Weights: {weights}")
    print(f"{'='*60}")

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        weights=weights or {},
        use_hard_state_bounds=False,
    )

    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)
    num_steps = int(duration / dt)
    ref_traj = {
        "ee_pos": np.tile(p_ee_nominal, (num_steps + horizon + 1, 1)),
        "ee_rot": np.tile(R_ee_nominal, (num_steps + horizon + 1, 1, 1)),
        "base": np.zeros((num_steps + horizon + 1, 3)),
        "base_z": np.full(num_steps + horizon + 1, 0.2),
    }

    solver = aligator.SolverProxDDP(tol, mu_init=mu_init, max_iters=max_iters)

    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    converged_count = 0
    solve_times = []
    ee_errors = []
    iter_counts = []

    for step in range(num_steps):
        t = step * dt
        x_current = env.get_state()

        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start
        solve_times.append(t_solve)
        iter_counts.append(solver.results.num_iters)

        if solver.results.conv:
            converged_count += 1

        u0 = np.array(solver.results.us[0])
        env.set_control(u0)
        env.set_target_marker(ref_traj["ee_pos"][step])
        env.step(mujoco_substeps)

        ee_pos = env.get_ee_pos()
        ee_err = np.linalg.norm(ee_pos - ref_traj["ee_pos"][step])
        ee_errors.append(ee_err)

        xs_sol = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_sol = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        xs_prev = xs_sol[1:] + [xs_sol[-1]]
        us_prev = us_sol[1:] + [us_sol[-1]]

        if (step + 1) % 20 == 0 or step == 0:
            conv_str = "✓" if solver.results.conv else "✗"
            print(f"  t={t:5.1f}s  ee_err={ee_err*100:5.1f}cm  solve={t_solve*1000:5.1f}ms  iter={solver.results.num_iters:2d}  {conv_str}")

    env.close()

    success_rate = (converged_count / num_steps) * 100
    avg_solve_time = np.mean(solve_times) * 1000
    ee_rms_error = np.sqrt(np.mean(np.array(ee_errors)**2)) * 100
    avg_iters = np.mean(iter_counts)

    print(f"\n--- Results ---")
    print(f"Success rate:     {success_rate:.1f}% ({converged_count}/{num_steps})")
    print(f"Avg solve time:   {avg_solve_time:.1f} ms")
    print(f"Avg iterations:   {avg_iters:.1f}")
    print(f"EE RMS error:     {ee_rms_error:.2f} cm")

    return {
        "success_rate": success_rate,
        "avg_solve_time": avg_solve_time,
        "ee_rms_error": ee_rms_error,
        "converged_count": converged_count,
        "avg_iters": avg_iters,
    }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    print("\n" + "="*60)
    print("PHASE 4 HYBRID MPC TUNING - ROUND 2")
    print("="*60)
    print("\nBased on Round 1: horizon=10 showed 1% success, 37ms solve time")
    print("Testing: smaller horizons, more iterations, modest tau weights")

    results = []

    # Test 1: horizon=8
    print("\n### Test 1: horizon=8 ###")
    res = run_test(duration=args.duration, render=False, horizon=8, dt=0.05,
                  max_iters=50, mu_init=1e-1, tol=1e-2)
    results.append(("horizon=8", res))

    # Test 2: horizon=5
    print("\n### Test 2: horizon=5 ###")
    res = run_test(duration=args.duration, render=False, horizon=5, dt=0.05,
                  max_iters=50, mu_init=1e-1, tol=1e-2)
    results.append(("horizon=5", res))

    # Test 3: horizon=10 + max_iters=100
    print("\n### Test 3: horizon=10 + max_iters=100 ###")
    res = run_test(duration=args.duration, render=False, horizon=10, dt=0.05,
                  max_iters=100, mu_init=1e-1, tol=1e-2)
    results.append(("h=10,iter=100", res))

    # Test 4: horizon=10 + max_iters=200
    print("\n### Test 4: horizon=10 + max_iters=200 ###")
    res = run_test(duration=args.duration, render=False, horizon=10, dt=0.05,
                  max_iters=200, mu_init=1e-1, tol=1e-2)
    results.append(("h=10,iter=200", res))

    # Test 5: horizon=10 + tau_weight=0.01
    print("\n### Test 5: horizon=10 + tau_weight=0.01 ###")
    weights = {"tau_arm": 0.01, "dtau_arm": 0.01}
    res = run_test(duration=args.duration, render=False, horizon=10, dt=0.05,
                  max_iters=50, mu_init=1e-1, tol=1e-2, weights=weights)
    results.append(("h=10,tau=0.01", res))

    # Test 6: horizon=10 + tau_weight=0.1
    print("\n### Test 6: horizon=10 + tau_weight=0.1 ###")
    weights = {"tau_arm": 0.1, "dtau_arm": 0.1}
    res = run_test(duration=args.duration, render=False, horizon=10, dt=0.05,
                  max_iters=50, mu_init=1e-1, tol=1e-2, weights=weights)
    results.append(("h=10,tau=0.1", res))

    # Test 7: Best combination - horizon=5 + max_iters=100
    print("\n### Test 7: horizon=5 + max_iters=100 (aggressive) ###")
    res = run_test(duration=args.duration, render=False, horizon=5, dt=0.05,
                  max_iters=100, mu_init=1e-1, tol=1e-2)
    results.append(("h=5,iter=100", res))

    # Summary
    print("\n" + "="*60)
    print("ROUND 2 SUMMARY")
    print("="*60)
    print(f"{'Config':<20} {'Success%':<12} {'Solve(ms)':<12} {'Iters':<8} {'EE RMS(cm)':<12}")
    print("-"*70)
    for name, res in results:
        print(f"{name:<20} {res['success_rate']:>10.1f}% {res['avg_solve_time']:>10.1f}ms {res['avg_iters']:>6.1f}  {res['ee_rms_error']:>10.2f}cm")

    # Find best
    best = max(results, key=lambda x: x[1]['success_rate'])
    print("\n" + "="*60)
    print(f"BEST CONFIG: {best[0]}")
    print(f"  Success rate: {best[1]['success_rate']:.1f}%")
    print(f"  Solve time:   {best[1]['avg_solve_time']:.1f} ms")
    print(f"  EE RMS error: {best[1]['ee_rms_error']:.2f} cm")
    print("="*60)
