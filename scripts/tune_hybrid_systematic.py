#!/usr/bin/env python3
"""
混合MPC系统性调参工具

按照优先级逐步调整参数，每一步对比基线性能。

调参策略：
1. 求解器参数（mu_init, tolerance）
2. EE跟踪权重
3. 扭矩正则化权重
4. Horizon长度
5. 组合优化

Usage:
  python scripts/tune_hybrid_systematic.py --phase 1 [--render]
  python scripts/tune_hybrid_systematic.py --phase all  # 运行所有阶段
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
from dataclasses import dataclass
from typing import Dict, List

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


@dataclass
class TestConfig:
    """测试配置"""
    name: str
    mu_init: float
    tolerance: float
    horizon: int
    weights: Dict[str, float]

    def __str__(self):
        return f"{self.name}: mu={self.mu_init:.0e}, tol={self.tolerance:.0e}, H={self.horizon}"


@dataclass
class TestResult:
    """测试结果"""
    config: TestConfig
    converged: int
    total: int
    conv_rate: float
    ee_rms: float
    ee_max: float
    avg_solve_ms: float
    avg_iters: float

    def summary(self):
        return (f"收敛率={self.conv_rate:5.1f}% RMS={self.ee_rms:5.2f}cm "
                f"Max={self.ee_max:5.2f}cm 求解={self.avg_solve_ms:5.1f}ms 迭代={self.avg_iters:4.1f}")


def run_test(config: TestConfig, duration: float = 20.0, render: bool = False) -> TestResult:
    """运行单次测试"""

    print(f"\n{'='*80}")
    print(f"测试: {config}")
    print(f"{'='*80}")

    # 初始化
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    dt = 0.05
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=config.horizon,
        dt=dt,
        weights=config.weights,
        use_hard_state_bounds=False,
    )

    # 参考轨迹
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=p_ee_nominal, ee_start_rot=R_ee_nominal)
    num_steps = int(duration / dt)
    ref_traj = ref_gen.get_reference(t=0.0, horizon=num_steps + config.horizon, dt=dt)

    # 求解器
    solver = aligator.SolverProxDDP(
        tol=config.tolerance,
        mu_init=config.mu_init,
        max_iters=50
    )

    # 初始化
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)
    env.set_target_marker(ref_traj["ee_pos"][0])

    xs_prev = [x0.copy() for _ in range(config.horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(config.horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    # 指标
    converged_count = 0
    solve_times = []
    ee_errors = []
    iters_used = []

    # 控制循环
    for step in range(num_steps):
        x_current = env.get_state()

        ref_window = {
            "ee_pos": ref_traj["ee_pos"][step:step+config.horizon+1],
            "ee_rot": ref_traj["ee_rot"][step:step+config.horizon+1],
            "base": ref_traj["base"][step:step+config.horizon+1],
            "base_z": ref_traj["base_z"][step:step+config.horizon+1],
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

        # 每秒输出
        if (step + 1) % 20 == 0:
            print(f"  t={step*dt:5.1f}s  err={ee_err*100:5.2f}cm  "
                  f"solve={t_solve*1000:5.1f}ms  iters={solver.results.num_iters:2d}  "
                  f"conv={'✓' if solver.results.conv else '✗'}")

    env.close()

    # 统计
    conv_rate = (converged_count / num_steps) * 100
    ee_rms = np.sqrt(np.mean(np.array(ee_errors)**2)) * 100
    ee_max = np.max(ee_errors) * 100
    avg_solve = np.mean(solve_times) * 1000
    avg_iters = np.mean(iters_used)

    result = TestResult(
        config=config,
        converged=converged_count,
        total=num_steps,
        conv_rate=conv_rate,
        ee_rms=ee_rms,
        ee_max=ee_max,
        avg_solve_ms=avg_solve,
        avg_iters=avg_iters,
    )

    print(f"\n结果: {result.summary()}")
    return result


# ============================================================================
# 调参阶段定义
# ============================================================================

def get_baseline_weights():
    """基线权重"""
    return {
        "ee_pos": 100.0,
        "ee_ori": 0.0,
        "terminal_ee_pos": 200.0,
        "base_xy": 60.0,
        "base_yaw": 10.0,
        "base_z": 60.0,
        "arm_posture": 0.5,
        "v_arm": 0.01,
        "tau_arm": 0.001,
        "dtau_arm": 0.01,
        "v_base": 0.01,
    }


def phase1_solver_params() -> List[TestConfig]:
    """阶段1：调整求解器参数（mu_init, tolerance）"""
    weights = get_baseline_weights()
    horizon = 20

    configs = []

    # 基线
    configs.append(TestConfig("Baseline", mu_init=1e-1, tolerance=1e-2, horizon=horizon, weights=weights))

    # 降低 mu_init（最关键！）
    configs.append(TestConfig("mu_1e-2", mu_init=1e-2, tolerance=1e-2, horizon=horizon, weights=weights))
    configs.append(TestConfig("mu_5e-2", mu_init=5e-2, tolerance=1e-2, horizon=horizon, weights=weights))
    configs.append(TestConfig("mu_2e-2", mu_init=2e-2, tolerance=1e-2, horizon=horizon, weights=weights))

    # 调整 tolerance
    configs.append(TestConfig("tol_1e-3", mu_init=1e-2, tolerance=1e-3, horizon=horizon, weights=weights))
    configs.append(TestConfig("tol_5e-3", mu_init=1e-2, tolerance=5e-3, horizon=horizon, weights=weights))

    return configs


def phase2_ee_weight(best_solver_config: TestConfig) -> List[TestConfig]:
    """阶段2：调整EE跟踪权重"""
    configs = []

    # 增强EE权重
    for ee_w in [150.0, 200.0, 300.0, 500.0]:
        w = best_solver_config.weights.copy()
        w["ee_pos"] = ee_w
        w["terminal_ee_pos"] = ee_w * 2
        configs.append(TestConfig(
            f"ee_w={ee_w}",
            mu_init=best_solver_config.mu_init,
            tolerance=best_solver_config.tolerance,
            horizon=best_solver_config.horizon,
            weights=w
        ))

    return configs


def phase3_tau_regularization(best_config: TestConfig) -> List[TestConfig]:
    """阶段3：调整扭矩正则化"""
    configs = []

    for tau_w in [0.005, 0.01, 0.02, 0.05]:
        w = best_config.weights.copy()
        w["tau_arm"] = tau_w
        w["dtau_arm"] = tau_w * 5  # 保持比例
        configs.append(TestConfig(
            f"tau_w={tau_w}",
            mu_init=best_config.mu_init,
            tolerance=best_config.tolerance,
            horizon=best_config.horizon,
            weights=w
        ))

    return configs


def phase4_horizon(best_config: TestConfig) -> List[TestConfig]:
    """阶段4：调整Horizon长度"""
    configs = []

    for H in [10, 15, 20, 25]:
        configs.append(TestConfig(
            f"H={H}",
            mu_init=best_config.mu_init,
            tolerance=best_config.tolerance,
            horizon=H,
            weights=best_config.weights.copy()
        ))

    return configs


def phase5_base_posture(best_config: TestConfig) -> List[TestConfig]:
    """阶段5：降低基座/姿态惩罚（让系统更自由）"""
    configs = []

    for scale in [0.5, 0.3, 0.1]:
        w = best_config.weights.copy()
        w["base_xy"] = 60.0 * scale
        w["base_yaw"] = 10.0 * scale
        w["base_z"] = 60.0 * scale
        w["arm_posture"] = 0.5 * scale
        configs.append(TestConfig(
            f"posture_scale={scale}",
            mu_init=best_config.mu_init,
            tolerance=best_config.tolerance,
            horizon=best_config.horizon,
            weights=w
        ))

    return configs


# ============================================================================
# 主流程
# ============================================================================

def run_phase(phase_num: int, render: bool = False) -> TestResult:
    """运行指定阶段"""
    duration = 20.0  # 每次测试20秒

    print(f"\n{'#'*80}")
    print(f"# 阶段 {phase_num}")
    print(f"{'#'*80}")

    if phase_num == 1:
        print("# 目标：找到最佳的 mu_init 和 tolerance")
        print("# 期望：收敛率从0%提升到>20%")
        configs = phase1_solver_params()

    elif phase_num == 2:
        print("# 目标：增强EE跟踪权重")
        print("# 期望：在收敛的基础上降低跟踪误差")
        # 假设阶段1找到了 mu_init=1e-2 最好
        best_solver = TestConfig("Phase1Best", mu_init=1e-2, tolerance=1e-2, horizon=20, weights=get_baseline_weights())
        configs = phase2_ee_weight(best_solver)

    elif phase_num == 3:
        print("# 目标：调整扭矩正则化，平衡跟踪与平滑")
        # 假设阶段2找到了 ee_pos=200 最好
        best_config = TestConfig("Phase2Best", mu_init=1e-2, tolerance=1e-2, horizon=20, weights=get_baseline_weights())
        best_config.weights["ee_pos"] = 200.0
        best_config.weights["terminal_ee_pos"] = 400.0
        configs = phase3_tau_regularization(best_config)

    elif phase_num == 4:
        print("# 目标：优化Horizon长度")
        best_config = TestConfig("Phase3Best", mu_init=1e-2, tolerance=1e-2, horizon=20, weights=get_baseline_weights())
        best_config.weights["ee_pos"] = 200.0
        best_config.weights["terminal_ee_pos"] = 400.0
        best_config.weights["tau_arm"] = 0.01
        best_config.weights["dtau_arm"] = 0.05
        configs = phase4_horizon(best_config)

    elif phase_num == 5:
        print("# 目标：降低基座/姿态约束，给系统更多自由度")
        best_config = TestConfig("Phase4Best", mu_init=1e-2, tolerance=1e-2, horizon=15, weights=get_baseline_weights())
        best_config.weights["ee_pos"] = 200.0
        best_config.weights["terminal_ee_pos"] = 400.0
        best_config.weights["tau_arm"] = 0.01
        best_config.weights["dtau_arm"] = 0.05
        configs = phase5_base_posture(best_config)

    else:
        raise ValueError(f"Unknown phase: {phase_num}")

    print(f"\n将测试 {len(configs)} 个配置，每个{duration}秒\n")

    # 运行所有配置
    results = []
    for i, cfg in enumerate(configs, 1):
        print(f"\n[{i}/{len(configs)}] ", end="")
        result = run_test(cfg, duration=duration, render=render)
        results.append(result)
        time.sleep(0.5)  # 短暂停顿

    # 汇总对比
    print(f"\n{'='*100}")
    print(f"阶段 {phase_num} 汇总")
    print(f"{'='*100}")
    print(f"{'配置':<25} {'收敛率':<10} {'EE RMS':<10} {'EE Max':<10} {'求解时间':<12} {'迭代数':<8}")
    print(f"{'-'*100}")

    best_result = None
    best_score = -1e9

    for r in results:
        print(f"{r.config.name:<25} {r.conv_rate:>6.1f}%   {r.ee_rms:>7.2f}cm  "
              f"{r.ee_max:>7.2f}cm  {r.avg_solve_ms:>8.1f}ms  {r.avg_iters:>6.1f}")

        # 评分：收敛率权重60%，RMS权重40%（RMS越小越好，取负）
        score = 0.6 * r.conv_rate - 0.4 * r.ee_rms
        if score > best_score:
            best_score = score
            best_result = r

    print(f"\n{'='*100}")
    print(f"✓ 最佳配置: {best_result.config.name}")
    print(f"  {best_result.summary()}")
    print(f"{'='*100}\n")

    return best_result


def main():
    import argparse
    parser = argparse.ArgumentParser(description="混合MPC系统性调参")
    parser.add_argument("--phase", choices=["1", "2", "3", "4", "5", "all"], required=True,
                        help="调参阶段（1=求解器, 2=EE权重, 3=扭矩, 4=Horizon, 5=姿态, all=全部）")
    parser.add_argument("--render", action="store_true", help="启用渲染")
    args = parser.parse_args()

    if args.phase == "all":
        phases = [1, 2, 3, 4, 5]
    else:
        phases = [int(args.phase)]

    print("\n" + "="*100)
    print("混合MPC系统性调参")
    print("="*100)
    print(f"\n将运行阶段: {phases}")
    print(f"渲染: {'开启' if args.render else '关闭'}")
    print("\n调参策略:")
    print("  阶段1: 求解器参数 (mu_init, tolerance) - 最关键！")
    print("  阶段2: EE跟踪权重 - 在收敛基础上提升精度")
    print("  阶段3: 扭矩正则化 - 平衡跟踪与平滑")
    print("  阶段4: Horizon长度 - 优化计算效率")
    print("  阶段5: 基座/姿态约束 - 增加系统自由度")
    print()

    best_results = {}
    for phase in phases:
        best_result = run_phase(phase, render=args.render)
        best_results[phase] = best_result

    # 最终总结
    print("\n" + "="*100)
    print("所有阶段最佳配置汇总")
    print("="*100)
    for phase, result in best_results.items():
        print(f"\n阶段{phase}: {result.config.name}")
        print(f"  配置: {result.config}")
        print(f"  结果: {result.summary()}")


if __name__ == "__main__":
    main()
