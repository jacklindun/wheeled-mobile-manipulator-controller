#!/usr/bin/env python3
"""
Phase 4 修复版本测试脚本

修复方案: 减小MPC时间步长
- dt: 0.05s → 0.01s (减少5倍)
- horizon: 10 → 5 (保持预测视野0.05s)
- max_iters: 50 → 100

目标: 减少积分器不匹配导致的累积误差
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


def generate_circle_trajectory_fixed(t_steps, dt, horizon, pin_robot, robot, radius=0.1, period=10.0):
    """生成圆形轨迹 (适配新的dt)"""
    num_waypoints = t_steps + horizon + 1

    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)
    center_y = p_ee_nominal[1]
    center_z = 0.5
    omega = 2 * np.pi / period

    ee_pos = np.zeros((num_waypoints, 3))
    ee_rot = np.zeros((num_waypoints, 3, 3))

    for i in range(num_waypoints):
        t = i * dt
        angle = omega * t
        ee_pos[i, 0] = p_ee_nominal[0]
        ee_pos[i, 1] = center_y + radius * np.cos(angle)
        ee_pos[i, 2] = center_z + radius * np.sin(angle)
        ee_rot[i] = R_ee_nominal

    return {
        "ee_pos": ee_pos,
        "ee_rot": ee_rot,
        "base": np.zeros((num_waypoints, 3)),
        "base_z": np.full(num_waypoints, 0.2),
    }


def test_phase4_fixed(duration=20.0, radius=0.1, period=10.0, render=False):
    """
    测试Phase 4修复版本

    修复参数:
    - dt = 0.01s (原0.05s)
    - horizon = 5 (原10)
    - max_iters = 100 (原50)
    """
    print("\n" + "="*70)
    print("Phase 4 修复版本测试")
    print("="*70)
    print("\n配置：")
    print(f"  duration = {duration} s")
    print(f"  dt = 0.01 s (修复: 原0.05s)")
    print(f"  horizon = 5 (修复: 原10)")
    print(f"  max_iters = 100 (修复: 原50)")
    print(f"  预测视野 = {5*0.01} s")
    print("\n修复目标：")
    print("  - 减少积分器累积误差 (0.034 → 0.007)")
    print("  - 提高MPC收敛率 (0% → 20-40%)")
    print("  - 改善跟踪误差 (2.5-5.0cm → 1.5-3.0cm)")
    print("="*70 + "\n")

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    # 修复后的参数
    horizon = 5
    dt = 0.01
    max_iters = 100

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        use_hard_state_bounds=False,
    )

    num_steps = int(duration / dt)
    ref_traj = generate_circle_trajectory_fixed(num_steps, dt, horizon, pin_robot, robot,
                                                radius=radius, period=period)

    solver = aligator.SolverProxDDP(1e-2, mu_init=1e-1, max_iters=max_iters)

    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)
    env.set_target_marker(ref_traj["ee_pos"][0])

    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    # 指标
    converged_count = 0
    solve_times = []
    ee_errors = []
    tau_maxs = []

    print("运行中...\n")
    print(f"{'时间':<8} {'EE误差':<10} {'求解时间':<12} {'扭矩峰值':<12} {'收敛':<8}")
    print("-"*70)

    for step in range(num_steps):
        t = step * dt
        x_current = env.get_state()

        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start

        solve_times.append(t_solve)

        if solver.results.conv:
            converged_count += 1

        u0 = np.array(solver.results.us[0])
        tau_arm = u0[4:10]
        tau_max = np.max(np.abs(tau_arm))
        tau_maxs.append(tau_max)

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

        if (step + 1) % int(1.0 / dt) == 0 or step == 0:
            conv_str = "✓" if solver.results.conv else "✗"
            print(f"{t:>6.1f}s  {ee_err*100:>8.2f}cm  {t_solve*1000:>10.1f}ms  {tau_max:>10.2f}Nm  {conv_str}")

    env.close()

    # 总结
    print("\n" + "="*70)
    print("Phase 4 修复版本测试结果")
    print("="*70)

    success_rate = (converged_count / num_steps) * 100
    ee_rms = np.sqrt(np.mean(np.array(ee_errors)**2)) * 100
    avg_solve = np.mean(solve_times) * 1000
    peak_tau = np.max(tau_maxs)

    print(f"\n求解器统计：")
    print(f"  收敛率:       {success_rate:>6.1f}% ({converged_count}/{num_steps})")
    print(f"  平均求解时间: {avg_solve:>6.1f} ms")
    print(f"  MPC调用次数:  {num_steps} (原版400次)")

    print(f"\n控制质量：")
    print(f"  EE RMS 误差:  {ee_rms:>6.2f} cm")
    print(f"  EE Max 误差:  {np.max(ee_errors)*100:>6.2f} cm")

    print(f"\n扭矩统计：")
    print(f"  最大峰值:     {peak_tau:>6.2f} Nm")

    # 对比原版
    print(f"\n" + "="*70)
    print("与原版Phase 4对比")
    print("="*70)

    print(f"\n原版Phase 4 (dt=0.05s, horizon=10):")
    print(f"  收敛率:   0%")
    print(f"  RMS误差:  2.5-5.0 cm")
    print(f"  MPC调用:  400次/20s")

    print(f"\n修复版Phase 4 (dt=0.01s, horizon=5):")
    print(f"  收敛率:   {success_rate:.1f}%")
    print(f"  RMS误差:  {ee_rms:.2f} cm")
    print(f"  MPC调用:  {num_steps}次/20s")

    print(f"\n改善评估:")
    if success_rate > 10:
        print(f"  ✓ 收敛率显著提升 ({success_rate:.1f}% vs 0%)")
    else:
        print(f"  ✗ 收敛率改善有限 ({success_rate:.1f}%)")

    if ee_rms < 3.0:
        print(f"  ✓ 跟踪误差改善 ({ee_rms:.2f}cm vs 2.5-5.0cm)")
    elif ee_rms < 5.0:
        print(f"  ⚠ 跟踪误差略有改善 ({ee_rms:.2f}cm)")
    else:
        print(f"  ✗ 跟踪误差未改善 ({ee_rms:.2f}cm)")

    print(f"\n代价:")
    print(f"  ⚠ MPC调用次数增加{num_steps/400:.1f}倍")
    print(f"  ⚠ 总求解时间增加{(avg_solve*num_steps)/(75*400):.1f}倍")

    print("\n" + "="*70)

    return {
        'convergence_rate': success_rate,
        'ee_rms_cm': ee_rms,
        'ee_max_cm': np.max(ee_errors)*100,
        'avg_solve_ms': avg_solve,
        'mpc_calls': num_steps,
    }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0, help="Test duration (s)")
    parser.add_argument("--radius", type=float, default=0.1, help="Circle radius (m)")
    parser.add_argument("--period", type=float, default=10.0, help="Circle period (s)")
    parser.add_argument("--no-render", action="store_true", help="Disable rendering")
    args = parser.parse_args()

    result = test_phase4_fixed(
        duration=args.duration,
        radius=args.radius,
        period=args.period,
        render=not args.no_render
    )

    print(f"\n最终结果摘要:")
    print(f"  收敛率: {result['convergence_rate']:.1f}%")
    print(f"  RMS误差: {result['ee_rms_cm']:.2f} cm")
    print(f"  MPC调用: {result['mpc_calls']} 次")
