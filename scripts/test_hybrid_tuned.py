#!/usr/bin/env python3
"""
混合MPC调优版本 - 改善求解器收敛性

关键改进：
1. 降低 mu_init: 1e-1 → 1e-3（更强的约束惩罚）
2. 增加扭矩正则化: 0.001 → 0.01（防止过大扭矩）
3. 增加 EE 权重: 100 → 200（更强的跟踪）
4. 减小 horizon: 20 → 15（减少计算量，可能更快收敛）

Usage:
  python scripts/test_hybrid_tuned.py --scenario ee_circle --duration 30 [--render]
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
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


def test_tuned_hybrid(scenario: str, duration: float = 30.0, render: bool = False):
    """调优后的混合MPC测试"""

    print("\n" + "="*80)
    print(f"混合MPC调优测试 - 场景: {scenario}")
    print("="*80)
    print(f"\n配置:")
    print(f"  场景:         {scenario}")
    print(f"  时长:         {duration} s")
    print(f"  Horizon:      15 (↓ from 20)")
    print(f"  MPC dt:       0.05 s")
    print(f"  Max iters:    50")
    print(f"  mu_init:      1e-3 (↓ from 1e-1, 更强约束)")
    print(f"  tolerance:    1e-3 (↓ from 1e-2, 更严格)")
    print("\n权重调整:")
    print(f"  ee_pos:       200.0 (↑ from 100.0)")
    print(f"  tau_arm:      0.01  (↑ from 0.001)")
    print(f"  dtau_arm:     0.05  (↑ from 0.01)")
    print("="*80 + "\n")

    # 初始化模型
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    # MPC 参数（调优）
    horizon = 15  # 减小horizon，可能更快收敛
    dt = 0.05
    max_iters = 50

    # 调优后的权重
    tuned_weights = {
        "ee_pos": 200.0,        # ↑ 增强跟踪
        "ee_ori": 0.0,
        "terminal_ee_pos": 400.0,  # ↑ 终端权重翻倍
        "base_xy": 50.0,        # ↓ 降低基座惩罚，让它更自由
        "base_yaw": 5.0,        # ↓
        "base_z": 50.0,         # ↓
        "arm_posture": 0.3,     # ↓ 降低姿态惩罚
        "v_arm": 0.01,
        "tau_arm": 0.01,        # ↑ 增强扭矩正则化
        "dtau_arm": 0.05,       # ↑ 增强平滑性
        "v_base": 0.01,
    }

    # 构建问题
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        weights=tuned_weights,
        use_hard_state_bounds=False,
    )

    # 生成参考轨迹
    num_steps = int(duration / dt)
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)

    ref_gen = ReferenceGenerator(
        scenario=scenario,
        ee_start=p_ee_nominal,
        ee_start_rot=R_ee_nominal
    )

    ref_traj = ref_gen.get_reference(t=0.0, horizon=num_steps + horizon, dt=dt)

    # 求解器（调优参数）
    solver = aligator.SolverProxDDP(
        tol=1e-3,           # ↓ 更严格的收敛条件
        mu_init=1e-3,       # ↓ 更强的约束惩罚（关键！）
        max_iters=max_iters
    )

    # 初始状态
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)
    env.set_target_marker(ref_traj["ee_pos"][0])

    # Warm-start
    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    # 指标收集
    converged_count = 0
    solve_times = []
    ee_errors = []
    tau_norms = []
    iters_used = []

    print("运行中...")
    print(f"{'时间':<8} {'EE误差':<12} {'求解时间':<14} {'迭代数':<10} {'收敛':<8}")
    print("-"*80)

    # 控制循环
    for step in range(num_steps):
        t = step * dt
        x_current = env.get_state()

        ref_window = {
            "ee_pos": ref_traj["ee_pos"][step:step+horizon+1],
            "ee_rot": ref_traj["ee_rot"][step:step+horizon+1],
            "base": ref_traj["base"][step:step+horizon+1],
            "base_z": ref_traj["base_z"][step:step+horizon+1],
        }

        problem, _ = builder.build_problem(x_current, ref_window, u_prev=us_prev[0] if step > 0 else None)

        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start

        solve_times.append(t_solve)
        iters_used.append(solver.results.num_iters)

        if solver.results.conv:
            converged_count += 1

        u0 = np.array(solver.results.us[0])
        tau_arm = u0[4:10]
        tau_norm = np.linalg.norm(tau_arm)
        tau_norms.append(tau_norm)

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
            print(f"{t:>6.1f}s  {ee_err*100:>10.2f}cm  {t_solve*1000:>12.1f}ms  "
                  f"{solver.results.num_iters:>8d}    {conv_str}")

    env.close()

    # 统计
    ee_errors_arr = np.array(ee_errors)
    solve_times_arr = np.array(solve_times)
    iters_arr = np.array(iters_used)

    success_rate = (converged_count / num_steps) * 100
    ee_rms = np.sqrt(np.mean(ee_errors_arr**2)) * 100
    ee_max = np.max(ee_errors_arr) * 100
    ee_mean = np.mean(ee_errors_arr) * 100
    avg_solve = np.mean(solve_times_arr) * 1000
    max_solve = np.max(solve_times_arr) * 1000
    avg_iters = np.mean(iters_arr)
    avg_tau = np.mean(tau_norms)
    max_tau = np.max(tau_norms)

    print("\n" + "="*80)
    print("调优测试总结")
    print("="*80)

    print(f"\n求解器性能:")
    print(f"  收敛率:         {success_rate:>6.1f}% ({converged_count}/{num_steps})")
    print(f"  平均迭代数:     {avg_iters:>6.1f}")
    print(f"  平均求解时间:   {avg_solve:>6.1f} ms")
    print(f"  最大求解时间:   {max_solve:>6.1f} ms")

    print(f"\n跟踪精度:")
    print(f"  EE RMS 误差:    {ee_rms:>6.2f} cm")
    print(f"  EE 平均误差:    {ee_mean:>6.2f} cm")
    print(f"  EE 最大误差:    {ee_max:>6.2f} cm")

    print(f"\n控制力矩:")
    print(f"  平均范数:       {avg_tau:>6.2f} Nm")
    print(f"  最大范数:       {max_tau:>6.2f} Nm")

    # 对比基线
    print(f"\n📊 与基线对比 (mu_init=1e-1):")
    baseline_conv = 0.0
    baseline_rms = 2.6
    print(f"  收敛率:  {baseline_conv:.1f}% → {success_rate:.1f}% "
          f"({'↑' if success_rate > baseline_conv else '→'} {success_rate - baseline_conv:+.1f}%)")
    print(f"  EE RMS:  {baseline_rms:.2f}cm → {ee_rms:.2f}cm "
          f"({'↓' if ee_rms < baseline_rms else '↑'} {abs(ee_rms - baseline_rms):.2f}cm)")

    print("\n" + "="*80 + "\n")

    return {
        "scenario": scenario,
        "success_rate": success_rate,
        "ee_rms_cm": ee_rms,
        "avg_solve_ms": avg_solve,
        "avg_iters": avg_iters,
    }


def main():
    import argparse
    parser = argparse.ArgumentParser(description="混合MPC调优测试")
    parser.add_argument("--scenario", choices=["ee_circle", "ee_line", "base_and_ee", "base_z_test"],
                        default="ee_circle", help="测试场景")
    parser.add_argument("--duration", type=float, default=30.0, help="测试时长(秒)")
    parser.add_argument("--render", action="store_true", help="启用MuJoCo渲染")
    args = parser.parse_args()

    result = test_tuned_hybrid(args.scenario, duration=args.duration, render=args.render)

    # 评估
    if result["success_rate"] > 50 and result["ee_rms_cm"] < 2.0:
        print("✓✓✓ 调优成功！收敛率和精度都有显著改善！")
    elif result["success_rate"] > 20:
        print("✓✓ 有改善，但仍需进一步调优")
    else:
        print("✗ 调优效果不明显，需要尝试其他策略")


if __name__ == "__main__":
    main()
