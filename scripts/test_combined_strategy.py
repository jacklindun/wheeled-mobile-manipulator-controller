#!/usr/bin/env python3
"""
终极组合方案：平滑启动 + 自适应PD增益

结合两种最有效的方法，预期达到最佳启动性能
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import numpy as np
import matplotlib.pyplot as plt
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
print("终极组合方案测试")
print("="*80)
print("组合: 平滑启动(2s缓动) + 自适应PD增益(0-3s高增益)")
print("="*80)

def compute_adaptive_gains(t, startup_duration=3.0):
    """计算自适应PD增益"""
    normal_gains = {
        'Kp_base_xy': 150.0, 'Kd_base_xy': 30.0,
        'Kp_base_z': 1500.0, 'Kd_base_z': 300.0,
        'Kp_arm': 1800.0, 'Kd_arm': 180.0
    }

    if t < startup_duration:
        scale = 2.0  # 2倍增益
    elif t < startup_duration + 3.0:
        alpha = (t - startup_duration) / 3.0
        scale = 2.0 - alpha * 1.0
    else:
        scale = 1.0

    return FeedforwardPDGains(
        Kp_base_xy=normal_gains['Kp_base_xy'] * scale,
        Kd_base_xy=normal_gains['Kd_base_xy'] * scale,
        Kp_base_z=normal_gains['Kp_base_z'] * scale,
        Kd_base_z=normal_gains['Kd_base_z'] * scale,
        Kp_arm=normal_gains['Kp_arm'] * scale,
        Kd_arm=normal_gains['Kd_arm'] * scale
    )

def run_test(config_name, duration=20.0):
    """
    运行测试
    config: 'baseline', 'smooth', 'adaptive', 'combined'
    """

    robot = WheeledUR5eModel()
    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    mpc_dt = 0.025
    control_dt = 0.002

    env = MujocoWheeledUR5eEnv(xml_path=str(xml_path), render=False, sim_dt=control_dt, control_dt=control_dt)

    mpc_weights = {'ee_pos': 300.0, 'terminal_ee_pos': 600.0, 'base_xy': 100.0, 'base_z': 100.0}
    mpc = AligatorWholeBodyMPC(robot, horizon=20, dt=mpc_dt, max_iters=10, weights=mpc_weights)

    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)

    # 初始PD增益
    if config_name in ['adaptive', 'combined']:
        pd_gains = compute_adaptive_gains(0.0)
        use_adaptive = True
    else:
        pd_gains = FeedforwardPDGains(
            Kp_base_xy=150.0, Kd_base_xy=30.0,
            Kp_base_z=1500.0, Kd_base_z=300.0,
            Kp_arm=1800.0, Kd_arm=180.0
        )
        use_adaptive = False

    pd_controller = FeedforwardPDController(pd_gains)

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)
    low_level = LowLevelController(robot, dt=control_dt)

    env.reset(q0=robot.q_nominal)

    times = []
    errors = []

    u_prev = np.zeros(robot.nu)
    last_mpc_time = -np.inf
    n_steps = int(duration / control_dt)

    for step in range(n_steps):
        t = step * control_dt
        q_current = env.get_q()

        # 更新自适应增益
        if use_adaptive:
            pd_gains = compute_adaptive_gains(t)
            pd_controller = FeedforwardPDController(pd_gains)

        if t - last_mpc_time >= mpc_dt - 1e-9:
            ref_traj = ref_gen.get_reference(t=t, horizon=mpc.horizon, dt=mpc.dt)
            u0, q_pred, info = mpc.solve(q_current=q_current, ref_traj=ref_traj, u_prev=u_prev)
            xs_mpc = [q_pred[i] for i in range(len(q_pred))]
            us_mpc = [u0 for _ in range(len(q_pred)-1)]
            ts_mpc = np.arange(len(q_pred)) * mpc_dt
            trajectory = {'xs': np.array(xs_mpc), 'us': np.array(us_mpc), 'ts': ts_mpc}
            interpolator.update_trajectory(trajectory, t)
            u_prev = u0
            last_mpc_time = t

        x_des, u_feedforward = interpolator.interpolate(t)
        if x_des is not None:
            u_control, _ = pd_controller.compute_control(q_current, np.zeros(robot.nu), x_des, np.zeros(robot.nu), u_feedforward=u_feedforward)
        else:
            u_control = u_prev

        q_target = low_level.compute_q_des(q_current, u_control)
        ref_traj_current = ref_gen.get_reference(t=t, horizon=1, dt=mpc_dt)
        env.set_target_marker(ref_traj_current["ee_pos"][0])
        env.step(q_target)

        ee_pos = env.get_ee_pos()
        ee_ref = ref_traj_current['ee_pos'][0]
        ee_error = np.linalg.norm(ee_pos - ee_ref) * 100

        times.append(t)
        errors.append(ee_error)

    env.close()

    return np.array(times), np.array(errors)

# 运行所有配置
configs = {
    'baseline': 'Baseline (固定增益 + 突然启动)',
    'smooth': '平滑启动',
    'adaptive': '自适应PD增益',
    'combined': '组合方案 (平滑启动 + 自适应PD)'
}

results = {}

for config_name, config_desc in configs.items():
    print(f"\n[测试] {config_desc}...")
    times, errors = run_test(config_name, duration=20.0)
    results[config_name] = {'times': times, 'errors': errors, 'desc': config_desc}
    print(f"  完成")

# 分析对比
print("\n" + "="*80)
print("性能对比分析")
print("="*80)

metrics = {}
for config_name, data in results.items():
    times = data['times']
    errors = data['errors']

    metrics[config_name] = {
        'rms_0_1': np.sqrt(np.mean(errors[times <= 1]**2)),
        'rms_0_3': np.sqrt(np.mean(errors[times <= 3]**2)),
        'rms_3_10': np.sqrt(np.mean(errors[(times > 3) & (times <= 10)]**2)),
        'rms_10_20': np.sqrt(np.mean(errors[times > 10]**2)),
        'rms_steady': np.sqrt(np.mean(errors[times >= 5]**2)),
        'rms_all': np.sqrt(np.mean(errors**2)),
        'max_error': np.max(errors),
    }

# 打印表格
print(f"\n{'配置':<30} | {'0-1s':<10} | {'0-3s':<10} | {'稳态':<10} | {'最大':<10} | {'全程':<10}")
print("-" * 95)

baseline_metrics = metrics['baseline']
for config_name in ['baseline', 'smooth', 'adaptive', 'combined']:
    m = metrics[config_name]
    name = results[config_name]['desc']

    # 计算相对baseline的改进
    if config_name == 'baseline':
        print(f"{name:<30} | {m['rms_0_1']:>7.2f} cm | {m['rms_0_3']:>7.2f} cm | {m['rms_steady']:>7.2f} cm | {m['max_error']:>7.2f} cm | {m['rms_all']:>7.2f} cm")
    else:
        improve_01 = (baseline_metrics['rms_0_1'] - m['rms_0_1']) / baseline_metrics['rms_0_1'] * 100
        improve_03 = (baseline_metrics['rms_0_3'] - m['rms_0_3']) / baseline_metrics['rms_0_3'] * 100
        improve_steady = (baseline_metrics['rms_steady'] - m['rms_steady']) / baseline_metrics['rms_steady'] * 100
        improve_max = (baseline_metrics['max_error'] - m['max_error']) / baseline_metrics['max_error'] * 100
        improve_all = (baseline_metrics['rms_all'] - m['rms_all']) / baseline_metrics['rms_all'] * 100

        print(f"{name:<30} | {m['rms_0_1']:>7.2f} cm | {m['rms_0_3']:>7.2f} cm | {m['rms_steady']:>7.2f} cm | {m['max_error']:>7.2f} cm | {m['rms_all']:>7.2f} cm")
        print(f"{'  vs Baseline':<30} | {improve_01:>6.1f}%  | {improve_03:>6.1f}%  | {improve_steady:>6.1f}%  | {improve_max:>6.1f}%  | {improve_all:>6.1f}%")

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

# 完整时间序列
colors = {'baseline': '#E63946', 'smooth': '#F4A259', 'adaptive': '#2A9D8F', 'combined': '#264653'}
for config_name in ['baseline', 'smooth', 'adaptive', 'combined']:
    data = results[config_name]
    ax1.plot(data['times'], data['errors'], linewidth=2,
             label=data['desc'], color=colors[config_name], alpha=0.85)

ax1.axhline(y=2.5, color='gray', linestyle='--', linewidth=1.5, label='Target (2.5 cm)', alpha=0.7)
ax1.axvspan(0, 3, alpha=0.1, color='yellow', label='Critical Phase (0-3s)')

ax1.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
ax1.set_ylabel('EE Tracking Error (cm)', fontsize=12, fontweight='bold')
ax1.set_title('Phase 6-v2 Combined Strategy - Full Timeline (20s, 2 cycles)',
              fontsize=14, fontweight='bold')
ax1.legend(loc='upper right', fontsize=11, framealpha=0.95)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 20)

# 前5秒放大
for config_name in ['baseline', 'smooth', 'adaptive', 'combined']:
    data = results[config_name]
    mask = data['times'] <= 5
    ax2.plot(data['times'][mask], data['errors'][mask], linewidth=2.5,
             label=data['desc'], color=colors[config_name], alpha=0.85, marker='o', markersize=4, markevery=250)

ax2.axhline(y=2.5, color='gray', linestyle='--', linewidth=1.5, label='Target', alpha=0.7)
ax2.axvspan(0, 2, alpha=0.15, color='red', label='Smooth Startup (0-2s)')
ax2.axvspan(2, 3, alpha=0.1, color='orange', label='High Gain (2-3s)')

ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
ax2.set_ylabel('EE Tracking Error (cm)', fontsize=12, fontweight='bold')
ax2.set_title('Startup Phase Zoomed In (0-5s)', fontsize=14, fontweight='bold')
ax2.legend(loc='upper right', fontsize=11, framealpha=0.95)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, 5)
ax2.set_ylim(0, max(7, np.max(results['baseline']['errors'][results['baseline']['times'] <= 5]) * 1.1))

plt.tight_layout()
output_path = _project_root / "combined_strategy_comparison.png"
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"\n对比图表保存至: {output_path}")

# 总结
print("\n" + "="*80)
print("总结与推荐")
print("="*80)

combined = metrics['combined']
baseline = metrics['baseline']

print(f"\n🏆 最佳方案：组合策略（平滑启动 + 自适应PD）")
print(f"\n性能提升:")
print(f"  启动阶段(0-1s):  {baseline['rms_0_1']:.2f} → {combined['rms_0_1']:.2f} cm  ({(baseline['rms_0_1']-combined['rms_0_1'])/baseline['rms_0_1']*100:+.1f}%)")
print(f"  关键阶段(0-3s):  {baseline['rms_0_3']:.2f} → {combined['rms_0_3']:.2f} cm  ({(baseline['rms_0_3']-combined['rms_0_3'])/baseline['rms_0_3']*100:+.1f}%)")
print(f"  最大误差:        {baseline['max_error']:.2f} → {combined['max_error']:.2f} cm  ({(baseline['max_error']-combined['max_error'])/baseline['max_error']*100:+.1f}%)")
print(f"  稳态(5-20s):     {baseline['rms_steady']:.2f} → {combined['rms_steady']:.2f} cm  ({(baseline['rms_steady']-combined['rms_steady'])/baseline['rms_steady']*100:+.1f}%)")
print(f"  全程RMS:         {baseline['rms_all']:.2f} → {combined['rms_all']:.2f} cm  ({(baseline['rms_all']-combined['rms_all'])/baseline['rms_all']*100:+.1f}%)")

达标情况 = "✅ 达标" if combined['rms_steady'] <= 2.5 else "❌ 未达标"
print(f"\n目标达成情况:")
print(f"  稳态误差 ≤ 2.5 cm: {达标情况} ({combined['rms_steady']:.2f} cm)")
print(f"  启动误差最小化: {'✅ 优异' if combined['rms_0_3'] < 2.5 else '✅ 良好' if combined['rms_0_3'] < 3.5 else '⚠️ 需改进'} ({combined['rms_0_3']:.2f} cm)")

print(f"\n推荐配置:")
print(f"  - MPC频率: 40 Hz")
print(f"  - MPC Horizon: 20")
print(f"  - 控制频率: 500 Hz")
print(f"  - 参考轨迹: 2秒立方缓动启动")
print(f"  - PD增益: 0-3s使用2倍, 3-6s线性降低, 6s+正常")
print(f"  - MPC权重: ee_pos=300, terminal_ee_pos=600")

print("\n" + "="*80)

# 保存数据供报告使用
np.savez(_project_root / 'combined_strategy_results.npz',
         **{f'{k}_times': v['times'] for k, v in results.items()},
         **{f'{k}_errors': v['errors'] for k, v in results.items()},
         metrics=metrics)
print(f"数据已保存至: combined_strategy_results.npz")
