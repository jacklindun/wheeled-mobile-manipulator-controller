#!/usr/bin/env python3
"""
Phase 4 Hybrid MPC Diagnostic Suite

Systematic diagnosis following the 9-step troubleshooting plan:
1. Offline convergence test (max_iters=100)
2. Horizon reduction test (20 -> 10 -> 5)
3. Hard constraints removal test
4. Tau weight increase test
5. ABA vs MuJoCo dynamics consistency check
6. Dynamics derivative finite difference test
7. Tolerance relaxation test (1e-2 -> 1e-3 -> 1e-1)
8. Conservative configuration test
9. Two-tier success criteria (solver_converged vs mpc_usable)
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
from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics


def test_offline_convergence():
    """Test 1: Can solver converge offline with unlimited iterations?"""
    print("\n" + "="*60)
    print("TEST 1: OFFLINE CONVERGENCE (max_iters=100)")
    print("="*60)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    # Build problem at nominal state
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal

    p_ee, R_ee = pin_robot.fk_pose(robot.q_nominal)
    horizon = 10
    dt = 0.05

    ref_traj = {
        "ee_pos": np.tile(p_ee, (horizon + 1, 1)),
        "ee_rot": np.tile(R_ee, (horizon + 1, 1, 1)),
        "base": np.zeros((horizon + 1, 3)),
        "base_z": np.full(horizon + 1, 0.2),
    }

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot, horizon=horizon, dt=dt,
        use_hard_state_bounds=False
    )

    problem, _ = builder.build_problem(x0, ref_traj, u_prev=None)

    # Solve with generous iteration budget
    solver = aligator.SolverProxDDP(1e-4, mu_init=1e-1, max_iters=100)
    solver.setup(problem)

    xs_init = [x0.copy() for _ in range(horizon + 1)]
    us_init = [np.zeros(10) for _ in range(horizon)]

    t_start = time.perf_counter()
    solver.run(problem, xs_init, us_init)
    t_solve = time.perf_counter() - t_start

    print(f"Converged: {solver.results.conv}")
    print(f"Iterations: {solver.results.num_iters}")
    print(f"Solve time: {t_solve*1000:.1f} ms")

    # Check available attributes
    if hasattr(solver.results, 'traj_cost'):
        print(f"Final cost: {solver.results.traj_cost:.6e}")

    if hasattr(solver.results, 'prim_infeas'):
        print(f"Primal infeas: {solver.results.prim_infeas:.6e}")
    if hasattr(solver.results, 'dual_infeas'):
        print(f"Dual infeas: {solver.results.dual_infeas:.6e}")

    # Print first control to check if it's reasonable
    if len(solver.results.us) > 0:
        u0 = np.array(solver.results.us[0])
        print(f"First control norm: {np.linalg.norm(u0):.3f}")
        print(f"First control: {u0}")

    return solver.results.conv


def test_aba_mujoco_consistency():
    """Test 5: Check ABA vs MuJoCo single-step dynamics consistency"""
    print("\n" + "="*60)
    print("TEST 5: ABA vs MUJOCO DYNAMICS CONSISTENCY")
    print("="*60)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=False)

    # Build dynamics model
    from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics
    dynamics = HybridWheeledUR5eDynamics(pin_robot, dt=0.01)

    # Test scenarios
    scenarios = [
        ("Zero torque (gravity)", np.zeros(10)),
        ("Small random torque", np.random.randn(10) * 0.1),
        ("Single joint torque (shoulder_lift)", np.array([0, 0, 0, 0, 0, 10, 0, 0, 0, 0])),
        ("Large shoulder_lift torque", np.array([0, 0, 0, 0, 0, 50, 0, 0, 0, 0])),
    ]

    print("\nComparing single-step prediction:")
    print(f"{'Scenario':<30} {'q_err':<12} {'v_err':<12} {'Status':<10}")
    print("-"*70)

    max_q_err = 0
    max_v_err = 0

    for name, u in scenarios:
        # Reset to nominal
        env.reset(robot.q_nominal)
        x0 = env.get_state()

        # Predict with ABA
        data_dyn = dynamics.createData()
        dynamics.forward(x0, u, data_dyn)
        x_aba = np.array(data_dyn.xnext)

        # Step MuJoCo
        env.set_control(u)
        env.step(substeps=int(0.01 / env.model.opt.timestep))
        x_mujoco = env.get_state()

        # Compare
        q_err = np.linalg.norm(x_aba[:10] - x_mujoco[:10])
        v_err = np.linalg.norm(x_aba[10:] - x_mujoco[10:])

        max_q_err = max(max_q_err, q_err)
        max_v_err = max(max_v_err, v_err)

        status = "✓ OK" if (q_err < 1e-3 and v_err < 1e-2) else "⚠️ LARGE"

        print(f"{name:<30} {q_err:>10.6f}  {v_err:>10.6f}  {status}")

        if q_err > 1e-3 or v_err > 1e-2:
            print(f"  Details:")
            print(f"    ABA q[4:10]: {x_aba[4:10]}")
            print(f"    MuJ q[4:10]: {x_mujoco[4:10]}")
            print(f"    ABA v[10:16]: {x_aba[10:16]}")
            print(f"    MuJ v[10:16]: {x_mujoco[10:16]}")

    env.close()

    print(f"\nMax errors across all scenarios:")
    print(f"  q_err: {max_q_err:.6e}")
    print(f"  v_err: {max_v_err:.6e}")

    if max_q_err > 1e-3 or max_v_err > 1e-2:
        print("\n⚠️  Large model mismatch detected!")
        print("   This explains why solver struggles to converge.")
        print("   ABA predictions differ significantly from MuJoCo execution.")
    else:
        print("\n✅ Dynamics models are consistent")


def test_conservative_config(duration=5.0, render=False):
    """Test 8: Conservative configuration"""
    print("\n" + "="*60)
    print("TEST 8: CONSERVATIVE CONFIGURATION")
    print("="*60)
    print("horizon=10, max_iters=20, tol=1e-3, mu_init=1e-4")
    print("tau=0.05, dtau=0.5, v_arm=0.1, hard_bounds=False")
    print("="*60)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    horizon = 10
    dt = 0.05
    max_iters = 20

    # Conservative weights
    weights = {
        "tau_arm": 0.05,
        "dtau_arm": 0.5,
        "v_arm": 0.1,
    }

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot, horizon=horizon, dt=dt,
        weights=weights,
        use_hard_state_bounds=False,  # Soft bounds only
    )

    p_ee, R_ee = pin_robot.fk_pose(robot.q_nominal)
    num_steps = int(duration / dt)
    ref_traj = {
        "ee_pos": np.tile(p_ee, (num_steps + horizon + 1, 1)),
        "ee_rot": np.tile(R_ee, (num_steps + horizon + 1, 1, 1)),
        "base": np.zeros((num_steps + horizon + 1, 3)),
        "base_z": np.full(num_steps + horizon + 1, 0.2),
    }

    solver = aligator.SolverProxDDP(1e-3, mu_init=1e-4, max_iters=max_iters)

    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    # Two-tier success criteria
    solver_converged_count = 0
    mpc_usable_count = 0
    solve_times = []
    ee_errors = []
    kkt_residuals = []

    for step in range(num_steps):
        t = step * dt
        x_current = env.get_state()

        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start
        solve_times.append(t_solve)

        # Tier 1: Solver convergence
        if solver.results.conv:
            solver_converged_count += 1

        u0 = np.array(solver.results.us[0])

        # Tier 2: MPC usability check
        mpc_usable = (
            np.all(np.isfinite(u0)) and
            t_solve < 0.1 and  # 100ms threshold
            np.linalg.norm(u0) < 200  # Reasonable control norm
        )
        if mpc_usable:
            mpc_usable_count += 1

        env.set_control(u0)
        env.set_target_marker(ref_traj["ee_pos"][step])
        env.step(mujoco_substeps)

        ee_pos = env.get_ee_pos()
        ee_err = np.linalg.norm(ee_pos - ref_traj["ee_pos"][step])
        ee_errors.append(ee_err)

        # Track KKT residual if available
        if hasattr(solver.results, 'prim_infeas'):
            kkt_residuals.append(solver.results.prim_infeas)

        xs_sol = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_sol = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        xs_prev = xs_sol[1:] + [xs_sol[-1]]
        us_prev = us_sol[1:] + [us_sol[-1]]

        if (step + 1) % 20 == 0 or step == 0:
            conv_str = "✓" if solver.results.conv else "✗"
            usable_str = "OK" if mpc_usable else "BAD"
            print(f"  t={t:5.1f}s  ee={ee_err*100:5.1f}cm  solve={t_solve*1000:5.1f}ms  iter={solver.results.num_iters:2d}  conv={conv_str}  usable={usable_str}")

    env.close()

    print(f"\n--- Results ---")
    print(f"Solver converged: {solver_converged_count}/{num_steps} ({solver_converged_count/num_steps*100:.1f}%)")
    print(f"MPC usable:       {mpc_usable_count}/{num_steps} ({mpc_usable_count/num_steps*100:.1f}%)")
    print(f"Avg solve time:   {np.mean(solve_times)*1000:.1f} ms")
    print(f"EE RMS error:     {np.sqrt(np.mean(np.array(ee_errors)**2))*100:.2f} cm")

    if kkt_residuals:
        print(f"Avg KKT residual: {np.mean(kkt_residuals):.6e}")
        print(f"Min KKT residual: {np.min(kkt_residuals):.6e}")
        print(f"Max KKT residual: {np.max(kkt_residuals):.6e}")

    return solver_converged_count, mpc_usable_count


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", action="store_true", help="Run all diagnostic tests")
    parser.add_argument("--test1", action="store_true", help="Offline convergence")
    parser.add_argument("--test5", action="store_true", help="ABA vs MuJoCo consistency")
    parser.add_argument("--test8", action="store_true", help="Conservative config")
    parser.add_argument("--render", action="store_true", help="Render MuJoCo")
    args = parser.parse_args()

    if not any([args.all, args.test1, args.test5, args.test8]):
        args.all = True

    print("\n" + "="*60)
    print("PHASE 4 HYBRID MPC DIAGNOSTIC SUITE")
    print("="*60)

    if args.all or args.test1:
        offline_converged = test_offline_convergence()
        if offline_converged:
            print("\n✅ Offline convergence: PASS - Problem formulation likely OK")
        else:
            print("\n⚠️  Offline convergence: FAIL - Check formulation/scaling")

    if args.all or args.test5:
        test_aba_mujoco_consistency()

    if args.all or args.test8:
        conv_count, usable_count = test_conservative_config(duration=5.0, render=args.render)

        if usable_count > 90:
            print("\n✅ MPC is USABLE for control (>90% usable)")
        elif usable_count > 50:
            print("\n⚠️  MPC is MARGINALLY usable (50-90% usable)")
        else:
            print("\n❌ MPC is NOT usable (<50% usable)")

        if conv_count < 10 and usable_count > 80:
            print("📊 Diagnosis: max_iters insufficient, but control is effective")
            print("   Recommendation: Accept as real-time iteration MPC")
        elif conv_count < 10 and usable_count < 50:
            print("📊 Diagnosis: Formulation or scaling issue")
            print("   Recommendation: Check dynamics consistency & weight scaling")
