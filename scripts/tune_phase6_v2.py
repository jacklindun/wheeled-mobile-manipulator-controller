#!/usr/bin/env python3
"""
Phase 6-v2 参数调优脚本

系统地测试不同参数组合，找到最优配置
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import numpy as np
import time

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)

print("="*80)
print("Phase 6-v2 参数调优")
print("="*80)

# 测试配置
TEST_CONFIGS = [
    # Config 1: Baseline (当前配置)
    {
        "name": "Baseline",
        "mpc_weights": None,  # Use defaults
        "pd_gains": {
            "Kp_base_xy": 50.0, "Kd_base_xy": 10.0,
            "Kp_base_z": 500.0, "Kd_base_z": 100.0,
            "Kp_arm": 500.0, "Kd_arm": 50.0
        },
        "horizon": 15,
    },
    # Config 2: Higher EE weight
    {
        "name": "High_EE_Weight",
        "mpc_weights": {"ee_pos": 200.0, "terminal_ee_pos": 400.0},
        "pd_gains": {
            "Kp_base_xy": 50.0, "Kd_base_xy": 10.0,
            "Kp_base_z": 500.0, "Kd_base_z": 100.0,
            "Kp_arm": 500.0, "Kd_arm": 50.0
        },
        "horizon": 15,
    },
    # Config 3: Higher PD gains
    {
        "name": "High_PD_Gains",
        "mpc_weights": None,
        "pd_gains": {
            "Kp_base_xy": 100.0, "Kd_base_xy": 20.0,
            "Kp_base_z": 1000.0, "Kd_base_z": 200.0,
            "Kp_arm": 1000.0, "Kd_arm": 100.0
        },
        "horizon": 15,
    },
    # Config 4: Longer horizon
    {
        "name": "Long_Horizon",
        "mpc_weights": None,
        "pd_gains": {
            "Kp_base_xy": 50.0, "Kd_base_xy": 10.0,
            "Kp_base_z": 500.0, "Kd_base_z": 100.0,
            "Kp_arm": 500.0, "Kd_arm": 50.0
        },
        "horizon": 20,
    },
    # Config 5: Combined best (high EE weight + higher PD)
    {
        "name": "Combined_Best",
        "mpc_weights": {"ee_pos": 200.0, "terminal_ee_pos": 400.0},
        "pd_gains": {
            "Kp_base_xy": 100.0, "Kd_base_xy": 20.0,
            "Kp_base_z": 1000.0, "Kd_base_z": 200.0,
            "Kp_arm": 800.0, "Kd_arm": 80.0
        },
        "horizon": 15,
    },
]

def run_test(config, duration=10.0):
    """运行一次测试，返回性能指标"""

    robot = WheeledUR5eModel()
    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    mpc_dt = 0.05
    control_dt = 0.002

    env = MujocoWheeledUR5eEnv(
        xml_path=str(xml_path),
        render=False,
        sim_dt=control_dt,
        control_dt=control_dt
    )

    mpc = AligatorWholeBodyMPC(
        robot,
        horizon=config["horizon"],
        dt=mpc_dt,
        max_iters=10,
        weights=config["mpc_weights"]
    )

    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)

    pd_gains = FeedforwardPDGains(**config["pd_gains"])
    pd_controller = FeedforwardPDController(pd_gains)

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)
    low_level = LowLevelController(robot, dt=control_dt)

    env.reset(q0=robot.q_nominal)

    # Data logging
    ee_errors = []
    converged_count = 0
    total_mpc_calls = 0
    solve_times = []

    u_prev = np.zeros(robot.nu)
    last_mpc_time = -np.inf

    n_steps = int(duration / control_dt)

    for step in range(n_steps):
        t = step * control_dt
        q_current = env.get_q()

        # MPC update (20Hz)
        if t - last_mpc_time >= mpc_dt - 1e-9:
            ref_traj = ref_gen.get_reference(t=t, horizon=mpc.horizon, dt=mpc.dt)

            t_start = time.perf_counter()
            u0, q_pred, info = mpc.solve(q_current=q_current, ref_traj=ref_traj, u_prev=u_prev)
            t_solve = time.perf_counter() - t_start

            xs_mpc = [q_pred[i] for i in range(len(q_pred))]
            us_mpc = [u0 for _ in range(len(q_pred)-1)]
            ts_mpc = np.arange(len(q_pred)) * mpc_dt

            trajectory = {'xs': np.array(xs_mpc), 'us': np.array(us_mpc), 'ts': ts_mpc}
            interpolator.update_trajectory(trajectory, t)

            if info['success']:
                converged_count += 1
            total_mpc_calls += 1
            solve_times.append(t_solve)

            u_prev = u0
            last_mpc_time = t

        # 插值 + PD控制 (500Hz)
        x_des, u_feedforward = interpolator.interpolate(t)

        if x_des is not None:
            q_des = x_des
            v_des = np.zeros(robot.nu)
            v_current = np.zeros(robot.nu)

            u_control, _ = pd_controller.compute_control(
                q_current, v_current, q_des, v_des, u_feedforward=u_feedforward
            )
        else:
            u_control = u_prev if u_prev is not None else np.zeros(robot.nu)

        q_target = low_level.compute_q_des(q_current, u_control)

        ref_traj_current = ref_gen.get_reference(t=t, horizon=1, dt=mpc_dt)
        env.set_target_marker(ref_traj_current["ee_pos"][0])
        env.step(q_target)

        # 测量误差
        ee_pos = env.get_ee_pos()
        ee_ref = ref_traj_current['ee_pos'][0]
        ee_error = np.linalg.norm(ee_pos - ee_ref)
        ee_errors.append(ee_error)

    env.close()

    # 计算指标
    ee_errors = np.array(ee_errors)
    rms_error = np.sqrt(np.mean(ee_errors**2)) * 100  # cm
    max_error = np.max(ee_errors) * 100  # cm
    convergence_rate = (converged_count / total_mpc_calls * 100) if total_mpc_calls > 0 else 0
    avg_solve_time = np.mean(solve_times) * 1000 if solve_times else 0  # ms

    return {
        "rms_error": rms_error,
        "max_error": max_error,
        "convergence_rate": convergence_rate,
        "avg_solve_time": avg_solve_time,
    }

# 运行所有测试
print("\n开始参数调优测试...\n")
results = []

for i, config in enumerate(TEST_CONFIGS):
    print(f"[{i+1}/{len(TEST_CONFIGS)}] 测试配置: {config['name']}")
    print(f"  参数:")
    print(f"    Horizon: {config['horizon']}")
    print(f"    MPC权重: {config['mpc_weights'] or 'Default'}")
    print(f"    Kp_arm: {config['pd_gains']['Kp_arm']}")
    print(f"    Kd_arm: {config['pd_gains']['Kd_arm']}")

    try:
        result = run_test(config, duration=10.0)
        results.append({"config": config, "result": result})

        print(f"  结果:")
        print(f"    RMS误差:  {result['rms_error']:.2f} cm")
        print(f"    最大误差: {result['max_error']:.2f} cm")
        print(f"    收敛率:   {result['convergence_rate']:.1f}%")
        print(f"    求解时间: {result['avg_solve_time']:.1f} ms")

        # 评分
        score = 0
        if result['rms_error'] <= 2.5:
            score += 3
        elif result['rms_error'] <= 4.0:
            score += 2
        elif result['rms_error'] <= 6.0:
            score += 1

        if result['convergence_rate'] >= 95:
            score += 2
        elif result['convergence_rate'] >= 80:
            score += 1

        print(f"    评分: {'⭐' * score}/⭐⭐⭐⭐⭐")

    except Exception as e:
        print(f"  ✗ 测试失败: {e}")
        results.append({"config": config, "result": None})

    print()

# 总结
print("="*80)
print("调优结果总结")
print("="*80)

print(f"\n{'配置':<20} | {'RMS(cm)':<10} | {'最大(cm)':<10} | {'收敛率':<10} | {'评分'}")
print("-" * 80)

for item in results:
    config = item["config"]
    result = item["result"]
    if result:
        # 评分
        score = 0
        if result['rms_error'] <= 2.5:
            score += 3
        elif result['rms_error'] <= 4.0:
            score += 2
        elif result['rms_error'] <= 6.0:
            score += 1
        if result['convergence_rate'] >= 95:
            score += 2
        elif result['convergence_rate'] >= 80:
            score += 1

        score_str = '⭐' * score
        print(f"{config['name']:<20} | {result['rms_error']:>8.2f}   | {result['max_error']:>8.2f}   | {result['convergence_rate']:>7.1f}%  | {score_str}")
    else:
        print(f"{config['name']:<20} | {'FAILED':<10} | {'FAILED':<10} | {'FAILED':<10} | ")

# 找出最佳配置
best = None
best_score = -1

for item in results:
    if item["result"]:
        result = item["result"]
        score = 0
        if result['rms_error'] <= 2.5:
            score += 3
        elif result['rms_error'] <= 4.0:
            score += 2
        elif result['rms_error'] <= 6.0:
            score += 1
        if result['convergence_rate'] >= 95:
            score += 2
        elif result['convergence_rate'] >= 80:
            score += 1

        if score > best_score:
            best_score = score
            best = item

if best:
    print(f"\n✅ 最佳配置: {best['config']['name']}")
    print(f"   RMS误差: {best['result']['rms_error']:.2f} cm")
    print(f"   收敛率:  {best['result']['convergence_rate']:.1f}%")
    print(f"\n推荐参数:")
    print(f"  horizon = {best['config']['horizon']}")
    print(f"  mpc_weights = {best['config']['mpc_weights']}")
    print(f"  pd_gains = {best['config']['pd_gains']}")
else:
    print("\n⚠️  所有配置均未通过测试")

print("\n" + "="*80)
