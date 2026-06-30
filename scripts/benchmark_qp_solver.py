#!/usr/bin/env python3
"""
QP求解器性能对比测试

对比scipy vs ProxQP的性能差异
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import numpy as np
import time

from wheeled_ur5e_aligator_mpc.wbc_controller import WholeBodyController, WBCWeights
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheelParameters


def benchmark_qp_solver(num_runs=100):
    """
    基准测试QP求解器性能

    Parameters
    ----------
    num_runs : int
        测试次数
    """
    print("="*80)
    print("QP求解器性能对比测试")
    print("="*80)

    # 初始化
    pin_robot = PinocchioWheeledUR5eModel()
    wheel_params = WheelParameters()
    weights = WBCWeights()

    wbc = WholeBodyController(pin_robot, wheel_params, weights)

    # 测试状态
    x_test = np.zeros(23)
    x_test[2] = 0.2  # base_z
    x_test[6:12] = [3.14159265, 1.04719755, -1.57079633, 0.52359878, 0., 0.]

    # 测试加速度
    a_des = np.random.randn(11) * 0.1

    # 检测当前使用的求解器
    from wheeled_ur5e_aligator_mpc.wbc_controller import QP_SOLVER
    print(f"\n当前QP求解器: {QP_SOLVER}")
    print(f"测试次数: {num_runs}")
    print("\n" + "-"*80)

    # 预热
    print("预热中...")
    for _ in range(5):
        wbc.compute_control(x_test, a_des)

    # 基准测试
    print("基准测试中...\n")
    solve_times = []
    dynamics_residuals = []

    for i in range(num_runs):
        t_start = time.perf_counter()
        τ_opt, info = wbc.compute_control(x_test, a_des)
        t_end = time.perf_counter()

        solve_times.append((t_end - t_start) * 1000)  # ms
        dynamics_residuals.append(info["dynamics_residual"])

        if (i + 1) % 20 == 0:
            print(f"  完成: {i+1}/{num_runs}")

    solve_times = np.array(solve_times)
    dynamics_residuals = np.array(dynamics_residuals)

    # 统计
    print("\n" + "="*80)
    print("测试结果")
    print("="*80)

    print(f"\n求解时间 (ms):")
    print(f"  平均:     {np.mean(solve_times):8.3f} ms")
    print(f"  中位数:   {np.median(solve_times):8.3f} ms")
    print(f"  最小:     {np.min(solve_times):8.3f} ms")
    print(f"  最大:     {np.max(solve_times):8.3f} ms")
    print(f"  标准差:   {np.std(solve_times):8.3f} ms")
    print(f"  95%分位: {np.percentile(solve_times, 95):8.3f} ms")

    print(f"\n动力学残差:")
    print(f"  平均:     {np.mean(dynamics_residuals):12.6e}")
    print(f"  中位数:   {np.median(dynamics_residuals):12.6e}")
    print(f"  最小:     {np.min(dynamics_residuals):12.6e}")
    print(f"  最大:     {np.max(dynamics_residuals):12.6e}")

    # 控制频率分析
    avg_time_ms = np.mean(solve_times)
    max_freq = 1000.0 / avg_time_ms
    print(f"\n可达到的最大控制频率:")
    print(f"  平均时间 {avg_time_ms:.2f} ms → 最大频率 {max_freq:.1f} Hz")

    if avg_time_ms < 1.0:
        print(f"  ✓ 优秀! 可以达到 >500 Hz 控制")
    elif avg_time_ms < 5.0:
        print(f"  ✓ 良好! 可以达到 >200 Hz 控制")
    elif avg_time_ms < 10.0:
        print(f"  ⚠ 可接受，可以达到 100 Hz 控制")
    else:
        print(f"  ✗ 较慢，建议优化或使用更快的求解器")

    # 性能等级
    print(f"\n性能评级:")
    if QP_SOLVER == "proxsuite":
        if avg_time_ms < 2.0 and np.mean(dynamics_residuals) < 1.0:
            print(f"  ⭐⭐⭐⭐⭐ 优秀")
        elif avg_time_ms < 5.0:
            print(f"  ⭐⭐⭐⭐ 良好")
        else:
            print(f"  ⭐⭐⭐ 中等")
    elif QP_SOLVER == "scipy":
        if avg_time_ms < 10.0:
            print(f"  ⭐⭐ 基本可用（建议升级到ProxQP）")
        else:
            print(f"  ⭐ 需要优化")
    else:
        print(f"  未评级")

    print("\n" + "="*80)

    return {
        "solver": QP_SOLVER,
        "avg_time_ms": avg_time_ms,
        "max_time_ms": np.max(solve_times),
        "avg_residual": np.mean(dynamics_residuals),
    }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="QP求解器性能测试")
    parser.add_argument("--runs", type=int, default=100, help="测试次数")
    args = parser.parse_args()

    results = benchmark_qp_solver(num_runs=args.runs)

    print(f"\n推荐:")
    if results["solver"] == "scipy":
        print(f"  当前使用scipy，性能一般")
        print(f"  建议安装ProxQP以获得 6-10× 性能提升:")
        print(f"    pixi add proxsuite")
    else:
        print(f"  当前使用{results['solver']}，性能良好")
