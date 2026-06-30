#!/usr/bin/env python3
"""
Phase 4 扭矩尺度问题专项测试

基于诊断发现：离线测试显示 shoulder_lift 扭矩达到 50 Nm
这说明 tau_arm 权重太小（默认 0.001），导致优化器用激进扭矩追踪目标

测试策略：
1. 逐步增加 tau_arm 权重：0.001 -> 0.01 -> 0.1 -> 1.0
2. 观察：
   - 扭矩幅度是否下降
   - 求解器收敛性
   - EE 跟踪误差
   - 闭环稳定性
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


def test_offline_with_tau_weight(tau_weight, dtau_weight):
    """Test offline convergence with given tau weights"""

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

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

    weights = {
        "tau_arm": tau_weight,
        "dtau_arm": dtau_weight,
    }

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot, horizon=horizon, dt=dt,
        weights=weights,
        use_hard_state_bounds=False
    )

    problem, _ = builder.build_problem(x0, ref_traj, u_prev=None)

    solver = aligator.SolverProxDDP(1e-4, mu_init=1e-1, max_iters=100)
    solver.setup(problem)

    xs_init = [x0.copy() for _ in range(horizon + 1)]
    us_init = [np.zeros(10) for _ in range(horizon)]

    t_start = time.perf_counter()
    solver.run(problem, xs_init, us_init)
    t_solve = time.perf_counter() - t_start

    u0 = np.array(solver.results.us[0])
    tau_arm = u0[4:10]

    return {
        "converged": solver.results.conv,
        "iters": solver.results.num_iters,
        "solve_time": t_solve * 1000,
        "tau_norm": np.linalg.norm(tau_arm),
        "tau_max": np.max(np.abs(tau_arm)),
        "tau_values": tau_arm,
        "dual_infeas": solver.results.dual_infeas if hasattr(solver.results, 'dual_infeas') else None,
    }


def test_closedloop_with_tau_weight(tau_weight, dtau_weight, duration=5.0):
    """Test closed-loop control with given tau weights"""

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=False)

    horizon = 10
    dt = 0.05
    max_iters = 50

    weights = {
        "tau_arm": tau_weight,
        "dtau_arm": dtau_weight,
    }

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot, horizon=horizon, dt=dt,
        weights=weights,
        use_hard_state_bounds=False,
    )

    p_ee, R_ee = pin_robot.fk_pose(robot.q_nominal)
    num_steps = int(duration / dt)
    ref_traj = {
        "ee_pos": np.tile(p_ee, (num_steps + horizon + 1, 1)),
        "ee_rot": np.tile(R_ee, (num_steps + horizon + 1, 1, 1)),
        "base": np.zeros((num_steps + horizon + 1, 3)),
        "base_z": np.full(num_steps + horizon + 1, 0.2),
    }

    solver = aligator.SolverProxDDP(1e-3, mu_init=1e-1, max_iters=max_iters)

    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    converged_count = 0
    ee_errors = []
    tau_norms = []
    tau_maxs = []

    for step in range(num_steps):
        x_current = env.get_state()

        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        solver.setup(problem)
        solver.run(problem, xs_prev, us_prev)

        if solver.results.conv:
            converged_count += 1

        u0 = np.array(solver.results.us[0])
        tau_arm = u0[4:10]
        tau_norms.append(np.linalg.norm(tau_arm))
        tau_maxs.append(np.max(np.abs(tau_arm)))

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

    env.close()

    return {
        "converged_pct": converged_count / num_steps * 100,
        "ee_rms": np.sqrt(np.mean(np.array(ee_errors)**2)) * 100,
        "tau_norm_avg": np.mean(tau_norms),
        "tau_max_avg": np.mean(tau_maxs),
        "tau_max_peak": np.max(tau_maxs),
    }


if __name__ == "__main__":
    print("\n" + "="*70)
    print("PHASE 4 扭矩尺度问题专项测试")
    print("="*70)
    print("\n诊断发现：默认 tau_arm=0.001 导致扭矩达到 50 Nm")
    print("策略：逐步增加 tau_arm 权重，找到最佳平衡点\n")

    tau_weights = [
        (0.001, 0.01),   # 默认（已知问题）
        (0.01, 0.1),     # 10x 增加
        (0.05, 0.5),     # 50x 增加（保守配置）
        (0.1, 1.0),      # 100x 增加
        (0.5, 5.0),      # 500x 增加
        (1.0, 10.0),     # 1000x 增加
    ]

    print("="*70)
    print("第一部分：离线收敛测试（max_iters=100，固定状态）")
    print("="*70)
    print(f"{'tau_arm':<10} {'dtau_arm':<10} {'Conv':<6} {'Iters':<6} {'Time(ms)':<10} {'τ_norm':<10} {'τ_max':<10}")
    print("-"*70)

    offline_results = []
    for tau, dtau in tau_weights:
        res = test_offline_with_tau_weight(tau, dtau)
        offline_results.append((tau, dtau, res))

        conv_str = "✓" if res["converged"] else "✗"
        print(f"{tau:<10.3f} {dtau:<10.3f} {conv_str:<6} {res['iters']:<6} {res['solve_time']:<10.1f} {res['tau_norm']:<10.2f} {res['tau_max']:<10.2f}")

        if tau == 0.001:
            print(f"  ⚠️ 默认配置：τ_arm = {res['tau_values']}")

    print("\n" + "="*70)
    print("第二部分：闭环控制测试（5秒，实际控制）")
    print("="*70)
    print(f"{'tau_arm':<10} {'dtau_arm':<10} {'Conv%':<8} {'EE_RMS(cm)':<12} {'τ_avg':<10} {'τ_peak':<10}")
    print("-"*70)

    closedloop_results = []
    for tau, dtau in tau_weights:
        print(f"Testing tau={tau}, dtau={dtau}...", end=" ", flush=True)
        res = test_closedloop_with_tau_weight(tau, dtau, duration=5.0)
        closedloop_results.append((tau, dtau, res))
        print("Done")

        print(f"{tau:<10.3f} {dtau:<10.3f} {res['converged_pct']:<8.1f} {res['ee_rms']:<12.2f} {res['tau_norm_avg']:<10.2f} {res['tau_max_peak']:<10.2f}")

    print("\n" + "="*70)
    print("分析与推荐")
    print("="*70)

    # 找到最佳配置
    # 标准：收敛率 > 0% 且 EE误差 < 5cm 且 扭矩合理 < 30 Nm
    viable = [(tau, dtau, res) for tau, dtau, res in closedloop_results
              if res['ee_rms'] < 5.0 and res['tau_max_peak'] < 30.0]

    if viable:
        best = max(viable, key=lambda x: x[2]['converged_pct'])
        tau_best, dtau_best, res_best = best

        print(f"\n✅ 推荐配置：")
        print(f"   tau_arm  = {tau_best}")
        print(f"   dtau_arm = {dtau_best}")
        print(f"   收敛率   = {res_best['converged_pct']:.1f}%")
        print(f"   EE误差   = {res_best['ee_rms']:.2f} cm")
        print(f"   峰值扭矩 = {res_best['tau_max_peak']:.2f} Nm")
    else:
        print("\n⚠️  没有找到完全满足条件的配置")
        print("    可能需要进一步调整权重或放宽容差")

    # 检查默认配置问题
    default_res = closedloop_results[0][2]
    if default_res['tau_max_peak'] > 40:
        print(f"\n❌ 默认配置问题确认：峰值扭矩 {default_res['tau_max_peak']:.1f} Nm 过大")
        print(f"   建议至少使用 tau_arm=0.01 以上")
