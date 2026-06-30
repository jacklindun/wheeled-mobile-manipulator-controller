#!/usr/bin/env python3
"""
混合动力学MPC四场景验证脚本

测试 Phase 4 混合MPC在四种场景下的跟踪精度：
- ee_circle: EE画圆，基座不动
- ee_line: EE直线移动
- base_and_ee: 基座前进，EE保持世界位置
- base_z_test: 基座升降，EE保持世界位置

Usage:
  python scripts/test_hybrid_scenarios.py --scenario ee_circle --duration 30 [--render]
  python scripts/test_hybrid_scenarios.py --all --duration 30  # 测试所有场景
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


def test_hybrid_scenario(scenario: str, duration: float = 30.0, render: bool = False):
    """
    测试混合MPC在指定场景下的表现

    Parameters
    ----------
    scenario : str
        场景名称: "ee_circle", "ee_line", "base_and_ee", "base_z_test"
    duration : float
        测试时长 (秒)
    render : bool
        是否渲染MuJoCo可视化

    Returns
    -------
    dict
        包含性能指标的字典
    """

    print("\n" + "="*80)
    print(f"混合动力学 MPC 测试 - 场景: {scenario}")
    print("="*80)
    print(f"\n配置:")
    print(f"  场景:         {scenario}")
    print(f"  时长:         {duration} s")
    print(f"  Horizon:      20")
    print(f"  MPC dt:       0.05 s")
    print(f"  Max iters:    50")
    print(f"  mu_init:      1e-1")
    print("="*80 + "\n")

    # 初始化模型
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    # MPC 参数
    horizon = 20
    dt = 0.05
    max_iters = 50

    # 构建问题
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        weights={
            "ee_pos": 100.0,
            "ee_ori": 0.0,  # 暂时只跟踪位置
        },
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

    # 预生成完整参考轨迹
    ref_traj = ref_gen.get_reference(t=0.0, horizon=num_steps + horizon, dt=dt)

    # 求解器
    solver = aligator.SolverProxDDP(1e-2, mu_init=1e-1, max_iters=max_iters)

    # 初始状态: [q_nominal(10), v_arm=0(6)]
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    # 设置初始目标标记位置（让黄球和红球重合）
    env.set_target_marker(ref_traj["ee_pos"][0])

    # Warm-start 缓冲
    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    # 指标收集
    converged_count = 0
    solve_times = []
    ee_errors = []
    tau_norms = []

    print("运行中...")
    print(f"{'时间':<8} {'EE误差':<12} {'求解时间':<14} {'扭矩范数':<12} {'收敛':<8}")
    print("-"*80)

    # 控制循环
    for step in range(num_steps):
        t = step * dt
        x_current = env.get_state()

        # 提取当前时刻的参考轨迹窗口
        ref_window = {
            "ee_pos": ref_traj["ee_pos"][step:step+horizon+1],
            "ee_rot": ref_traj["ee_rot"][step:step+horizon+1],
            "base": ref_traj["base"][step:step+horizon+1],
            "base_z": ref_traj["base_z"][step:step+horizon+1],
        }

        # 构建OCP
        problem, _ = builder.build_problem(
            x_current,
            ref_window,
            u_prev=us_prev[0] if step > 0 else None
        )

        # 求解
        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start

        solve_times.append(t_solve)

        if solver.results.conv:
            converged_count += 1

        # 提取控制
        u0 = np.array(solver.results.us[0])
        tau_arm = u0[4:10]
        tau_norm = np.linalg.norm(tau_arm)
        tau_norms.append(tau_norm)

        # 应用控制
        env.set_control(u0)
        env.set_target_marker(ref_traj["ee_pos"][step])
        env.step(mujoco_substeps)

        # 计算跟踪误差
        ee_pos = env.get_ee_pos()
        ee_err = np.linalg.norm(ee_pos - ref_traj["ee_pos"][step])
        ee_errors.append(ee_err)

        # Warm-start: shift + hold
        xs_sol = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_sol = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        xs_prev = xs_sol[1:] + [xs_sol[-1]]
        us_prev = us_sol[1:] + [us_sol[-1]]

        # 每秒输出一次
        if (step + 1) % 20 == 0 or step == 0:
            conv_str = "✓" if solver.results.conv else "✗"
            print(f"{t:>6.1f}s  {ee_err*100:>10.2f}cm  {t_solve*1000:>12.1f}ms  "
                  f"{tau_norm:>10.2f}Nm  {conv_str}")

    env.close()

    # 计算统计指标
    ee_errors_arr = np.array(ee_errors)
    solve_times_arr = np.array(solve_times)
    tau_norms_arr = np.array(tau_norms)

    success_rate = (converged_count / num_steps) * 100
    ee_rms = np.sqrt(np.mean(ee_errors_arr**2)) * 100  # cm
    ee_max = np.max(ee_errors_arr) * 100  # cm
    ee_mean = np.mean(ee_errors_arr) * 100  # cm
    avg_solve = np.mean(solve_times_arr) * 1000  # ms
    max_solve = np.max(solve_times_arr) * 1000  # ms
    avg_tau = np.mean(tau_norms_arr)  # Nm
    max_tau = np.max(tau_norms_arr)  # Nm

    # 输出总结
    print("\n" + "="*80)
    print("测试总结")
    print("="*80)

    print(f"\n求解器性能:")
    print(f"  收敛率:         {success_rate:>6.1f}% ({converged_count}/{num_steps})")
    print(f"  平均求解时间:   {avg_solve:>6.1f} ms")
    print(f"  最大求解时间:   {max_solve:>6.1f} ms")

    print(f"\n跟踪精度:")
    print(f"  EE RMS 误差:    {ee_rms:>6.2f} cm")
    print(f"  EE 平均误差:    {ee_mean:>6.2f} cm")
    print(f"  EE 最大误差:    {ee_max:>6.2f} cm")

    print(f"\n控制力矩:")
    print(f"  平均范数:       {avg_tau:>6.2f} Nm")
    print(f"  最大范数:       {max_tau:>6.2f} Nm")

    # 精度评估
    print(f"\n精度评估:")
    if ee_rms < 2.0:
        status = "优秀 ✓"
    elif ee_rms < 3.0:
        status = "良好 ○"
    elif ee_rms < 5.0:
        status = "合格 △"
    else:
        status = "需改进 ✗"
    print(f"  {status} (RMS < 2cm=优秀, <3cm=良好, <5cm=合格)")

    print("\n" + "="*80 + "\n")

    # 返回结果字典
    return {
        "scenario": scenario,
        "duration": duration,
        "num_steps": num_steps,
        "success_rate": success_rate,
        "converged_count": converged_count,
        "ee_rms_cm": ee_rms,
        "ee_mean_cm": ee_mean,
        "ee_max_cm": ee_max,
        "avg_solve_ms": avg_solve,
        "max_solve_ms": max_solve,
        "avg_tau_nm": avg_tau,
        "max_tau_nm": max_tau,
    }


def main():
    import argparse
    parser = argparse.ArgumentParser(description="混合MPC四场景验证")
    parser.add_argument(
        "--scenario",
        choices=["ee_circle", "ee_line", "base_and_ee", "base_z_test"],
        help="测试场景"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="测试所有四个场景"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="每个场景测试时长 (秒)"
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="启用MuJoCo渲染"
    )

    args = parser.parse_args()

    if not args.scenario and not args.all:
        parser.error("必须指定 --scenario 或 --all")

    scenarios = ["ee_circle", "ee_line", "base_and_ee", "base_z_test"] if args.all else [args.scenario]

    results = []
    for scenario in scenarios:
        result = test_hybrid_scenario(scenario, duration=args.duration, render=args.render)
        results.append(result)

        # 场景间短暂停顿
        if len(scenarios) > 1:
            time.sleep(1.0)

    # 如果测试多个场景，输出汇总表格
    if len(results) > 1:
        print("\n" + "="*100)
        print("所有场景汇总")
        print("="*100)
        print(f"\n{'场景':<15} {'收敛率':<10} {'EE RMS':<12} {'EE Max':<12} "
              f"{'平均求解':<12} {'最大扭矩':<12}")
        print("-"*100)

        for r in results:
            print(f"{r['scenario']:<15} {r['success_rate']:>6.1f}%   "
                  f"{r['ee_rms_cm']:>8.2f} cm  {r['ee_max_cm']:>8.2f} cm  "
                  f"{r['avg_solve_ms']:>8.1f} ms  {r['max_tau_nm']:>8.2f} Nm")

        print("\n" + "="*100)

        # 整体评估
        avg_rms = np.mean([r['ee_rms_cm'] for r in results])
        avg_success = np.mean([r['success_rate'] for r in results])

        print(f"\n整体平均:")
        print(f"  平均 EE RMS:   {avg_rms:.2f} cm")
        print(f"  平均收敛率:    {avg_success:.1f}%")

        if avg_rms < 3.0 and avg_success > 80:
            print(f"\n✓ 混合MPC整体表现良好！")
        elif avg_rms < 5.0:
            print(f"\n△ 混合MPC基本可用，但需进一步调优")
        else:
            print(f"\n✗ 混合MPC需要重要改进")

        print("\n" + "="*100 + "\n")


if __name__ == "__main__":
    main()
