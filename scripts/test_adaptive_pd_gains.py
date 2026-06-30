#!/usr/bin/env python3
"""
方法4: 启动阶段自适应PD增益

在启动阶段（0-3秒）使用更高的PD增益，
快速响应参考轨迹，然后逐渐降低到正常值
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
print("方法4: 启动阶段自适应PD增益")
print("="*80)

def compute_adaptive_gains(t, startup_duration=3.0):
    """
    计算自适应PD增益

    启动阶段(0-3s): 使用2倍增益快速响应
    过渡阶段(3-6s): 线性降低到正常增益
    稳态阶段(6s+): 正常增益
    """
    # 正常增益
    normal_gains = {
        'Kp_base_xy': 150.0, 'Kd_base_xy': 30.0,
        'Kp_base_z': 1500.0, 'Kd_base_z': 300.0,
        'Kp_arm': 1800.0, 'Kd_arm': 180.0
    }

    if t < startup_duration:
        # 启动阶段: 2倍增益
        scale = 2.0
    elif t < startup_duration + 3.0:
        # 过渡阶段: 线性降低 2.0 -> 1.0
        alpha = (t - startup_duration) / 3.0
        scale = 2.0 - alpha * 1.0
    else:
        # 稳态阶段: 正常增益
        scale = 1.0

    return FeedforwardPDGains(
        Kp_base_xy=normal_gains['Kp_base_xy'] * scale,
        Kd_base_xy=normal_gains['Kd_base_xy'] * scale,
        Kp_base_z=normal_gains['Kp_base_z'] * scale,
        Kd_base_z=normal_gains['Kd_base_z'] * scale,
        Kp_arm=normal_gains['Kp_arm'] * scale,
        Kd_arm=normal_gains['Kd_arm'] * scale
    )

def run_test_adaptive_pd(use_adaptive=True, duration=15.0):
    """运行测试，可选自适应PD增益"""

    robot = WheeledUR5eModel()
    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    mpc_dt = 0.025
    control_dt = 0.002

    env = MujocoWheeledUR5eEnv(xml_path=str(xml_path), render=False, sim_dt=control_dt, control_dt=control_dt)

    mpc_weights = {'ee_pos': 300.0, 'terminal_ee_pos': 600.0, 'base_xy': 100.0, 'base_z': 100.0}
    mpc = AligatorWholeBodyMPC(robot, horizon=20, dt=mpc_dt, max_iters=10, weights=mpc_weights)

    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)

    # 初始PD控制器
    if use_adaptive:
        pd_gains = compute_adaptive_gains(0.0)
        print(f"  启动增益: Kp_arm={pd_gains.Kp_arm[0]:.0f}")
    else:
        pd_gains = FeedforwardPDGains(
            Kp_base_xy=150.0, Kd_base_xy=30.0,
            Kp_base_z=1500.0, Kd_base_z=300.0,
            Kp_arm=1800.0, Kd_arm=180.0
        )
        print(f"  固定增益: Kp_arm={pd_gains.Kp_arm[0]:.0f}")

    pd_controller = FeedforwardPDController(pd_gains)

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)
    low_level = LowLevelController(robot, dt=control_dt)

    env.reset(q0=robot.q_nominal)

    times = []
    errors = []
    gains_history = []

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
            gains_history.append(pd_gains.Kp_arm[0])

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

    return np.array(times), np.array(errors), np.array(gains_history) if use_adaptive else None

# 测试1: 固定增益
print("\n[测试1] 固定PD增益...")
times_fixed, errors_fixed, _ = run_test_adaptive_pd(use_adaptive=False, duration=15.0)

# 测试2: 自适应增益
print("\n[测试2] 自适应PD增益...")
times_adaptive, errors_adaptive, gains = run_test_adaptive_pd(use_adaptive=True, duration=15.0)

# 对比分析
print("\n" + "="*80)
print("对比结果")
print("="*80)

print(f"\n{'阶段':<20} | {'固定增益':<15} | {'自适应增益':<15} | {'改进':<10}")
print("-" * 75)

# 0-3秒（高增益阶段）
rms_fixed_03 = np.sqrt(np.mean(errors_fixed[times_fixed <= 3]**2))
rms_adaptive_03 = np.sqrt(np.mean(errors_adaptive[times_adaptive <= 3]**2))
improve_03 = (rms_fixed_03 - rms_adaptive_03) / rms_fixed_03 * 100
print(f"{'0-3秒 (高增益)':<20} | {rms_fixed_03:>10.2f} cm | {rms_adaptive_03:>10.2f} cm | {improve_03:>7.1f}%")

# 0-1秒（关键启动）
rms_fixed_01 = np.sqrt(np.mean(errors_fixed[times_fixed <= 1]**2))
rms_adaptive_01 = np.sqrt(np.mean(errors_adaptive[times_adaptive <= 1]**2))
improve_01 = (rms_fixed_01 - rms_adaptive_01) / rms_fixed_01 * 100
print(f"{'0-1秒 (关键启动)':<20} | {rms_fixed_01:>10.2f} cm | {rms_adaptive_01:>10.2f} cm | {improve_01:>7.1f}%")

# 3-6秒（过渡阶段）
mask_fixed_36 = (times_fixed > 3) & (times_fixed <= 6)
mask_adaptive_36 = (times_adaptive > 3) & (times_adaptive <= 6)
rms_fixed_36 = np.sqrt(np.mean(errors_fixed[mask_fixed_36]**2))
rms_adaptive_36 = np.sqrt(np.mean(errors_adaptive[mask_adaptive_36]**2))
improve_36 = (rms_fixed_36 - rms_adaptive_36) / rms_fixed_36 * 100
print(f"{'3-6秒 (过渡)':<20} | {rms_fixed_36:>10.2f} cm | {rms_adaptive_36:>10.2f} cm | {improve_36:>7.1f}%")

# 6-15秒（稳态）
rms_fixed_615 = np.sqrt(np.mean(errors_fixed[times_fixed >= 6]**2))
rms_adaptive_615 = np.sqrt(np.mean(errors_adaptive[times_adaptive >= 6]**2))
improve_615 = (rms_fixed_615 - rms_adaptive_615) / rms_fixed_615 * 100
print(f"{'6-15秒 (稳态)':<20} | {rms_fixed_615:>10.2f} cm | {rms_adaptive_615:>10.2f} cm | {improve_615:>7.1f}%")

# 全程
rms_fixed_all = np.sqrt(np.mean(errors_fixed**2))
rms_adaptive_all = np.sqrt(np.mean(errors_adaptive**2))
improve_all = (rms_fixed_all - rms_adaptive_all) / rms_fixed_all * 100
print(f"{'全程 (0-15秒)':<20} | {rms_fixed_all:>10.2f} cm | {rms_adaptive_all:>10.2f} cm | {improve_all:>7.1f}%")

# 最大误差
max_fixed = np.max(errors_fixed)
max_adaptive = np.max(errors_adaptive)
improve_max = (max_fixed - max_adaptive) / max_fixed * 100
print(f"\n{'最大误差':<20} | {max_fixed:>10.2f} cm | {max_adaptive:>10.2f} cm | {improve_max:>7.1f}%")

# 绘图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14))

# 子图1: 误差对比
ax1.plot(times_fixed, errors_fixed, 'r-', linewidth=1.5, label='Fixed Gains', alpha=0.8)
ax1.plot(times_adaptive, errors_adaptive, 'b-', linewidth=1.5, label='Adaptive Gains', alpha=0.8)
ax1.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='Target (2.5 cm)', alpha=0.7)
ax1.axvspan(0, 3, alpha=0.15, color='red', label='High Gains (0-3s)')
ax1.axvspan(3, 6, alpha=0.1, color='orange', label='Transition (3-6s)')

ax1.set_xlabel('Time (s)', fontsize=11)
ax1.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax1.set_title('Phase 6-v2 Adaptive PD Gains - Error Comparison', fontsize=13, fontweight='bold')
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 15)

# 子图2: 增益变化
if gains is not None:
    times_gains = times_adaptive[:len(gains)]
    ax2.plot(times_gains, gains, 'g-', linewidth=2, label='Kp_arm')
    ax2.axhline(y=1800, color='blue', linestyle='--', linewidth=1, label='Normal Gain', alpha=0.7)
    ax2.axhline(y=3600, color='red', linestyle='--', linewidth=1, label='Startup Gain (2x)', alpha=0.7)
    ax2.axvspan(0, 3, alpha=0.15, color='red')
    ax2.axvspan(3, 6, alpha=0.1, color='orange')

    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Kp_arm', fontsize=11)
    ax2.set_title('Adaptive PD Gain Schedule', fontsize=13, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, 15)

# 子图3: 前5秒放大
ax3.plot(times_fixed[times_fixed <= 5], errors_fixed[times_fixed <= 5],
         'r-', linewidth=2, label='Fixed Gains', marker='o', markersize=3, alpha=0.8)
ax3.plot(times_adaptive[times_adaptive <= 5], errors_adaptive[times_adaptive <= 5],
         'b-', linewidth=2, label='Adaptive Gains', marker='s', markersize=3, alpha=0.8)
ax3.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='Target', alpha=0.7)
ax3.axvspan(0, 3, alpha=0.15, color='red', label='High Gains Phase')

ax3.set_xlabel('Time (s)', fontsize=11)
ax3.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax3.set_title('Startup Phase Zoomed In (0-5s)', fontsize=13, fontweight='bold')
ax3.legend(loc='upper right', fontsize=10)
ax3.grid(True, alpha=0.3)
ax3.set_xlim(0, 5)

plt.tight_layout()
output_path = _project_root / "adaptive_pd_gains_comparison.png"
plt.savefig(output_path, dpi=150)
print(f"\n对比图表保存至: {output_path}")

# 总结
print("\n" + "="*80)
print("总结")
print("="*80)

print(f"\n✅ 自适应PD增益策略效果:")
print(f"   - 关键启动(0-1s): 误差{'降低' if improve_01 > 0 else '增加'} {abs(improve_01):.1f}% ({rms_fixed_01:.2f} → {rms_adaptive_01:.2f} cm)")
print(f"   - 高增益阶段(0-3s): 误差{'降低' if improve_03 > 0 else '增加'} {abs(improve_03):.1f}% ({rms_fixed_03:.2f} → {rms_adaptive_03:.2f} cm)")
print(f"   - 最大误差{'降低' if improve_max > 0 else '增加'}:   {abs(improve_max):.1f}% ({max_fixed:.2f} → {max_adaptive:.2f} cm)")
print(f"   - 全程误差{'降低' if improve_all > 0 else '增加'}:   {abs(improve_all):.1f}% ({rms_fixed_all:.2f} → {rms_adaptive_all:.2f} cm)")

print(f"\n💡 自适应增益策略:")
print(f"   - 0-3s: 使用2倍增益 (Kp_arm = 3600)")
print(f"   - 3-6s: 线性降低到正常增益")
print(f"   - 6s+: 正常增益 (Kp_arm = 1800)")
print(f"   - 避免一直高增益导致的震荡和不稳定")

if improve_01 > 20:
    print(f"\n🎉 推荐采用自适应PD增益策略！")
elif improve_01 > 10:
    print(f"\n✅ 自适应增益有明显改善效果")
elif improve_01 > 0:
    print(f"\n✅ 自适应增益有一定改善效果")
else:
    print(f"\n⚠️  自适应增益效果不明显，可能高增益带来震荡")

print("\n" + "="*80)
