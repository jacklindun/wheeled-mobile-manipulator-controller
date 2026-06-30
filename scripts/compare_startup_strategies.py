#!/usr/bin/env python3
"""
对比测试：原始启动 vs 平滑启动

展示两种启动策略对瞬态误差的影响
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
print("Phase 6-v2 启动策略对比测试")
print("="*80)
print("对比: 原始启动(突然加速) vs 平滑启动(2秒缓动)")
print("="*80)

def run_test(use_smooth_startup, duration=15.0):
    """运行测试，返回时间和误差数据"""

    robot = WheeledUR5eModel()
    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    mpc_dt = 0.025
    control_dt = 0.002

    env = MujocoWheeledUR5eEnv(xml_path=str(xml_path), render=False, sim_dt=control_dt, control_dt=control_dt)

    mpc_weights = {'ee_pos': 300.0, 'terminal_ee_pos': 600.0, 'base_xy': 100.0, 'base_z': 100.0}
    mpc = AligatorWholeBodyMPC(robot, horizon=20, dt=mpc_dt, max_iters=10, weights=mpc_weights)

    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)
    pd_gains = FeedforwardPDGains(Kp_base_xy=150.0, Kd_base_xy=30.0, Kp_base_z=1500.0, Kd_base_z=300.0, Kp_arm=1800.0, Kd_arm=180.0)
    pd_controller = FeedforwardPDController(pd_gains)

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)
    low_level = LowLevelController(robot, dt=control_dt)

    env.reset(q0=robot.q_nominal)

    # Data logging
    times = []
    errors = []

    u_prev = np.zeros(robot.nu)
    last_mpc_time = -np.inf
    n_steps = int(duration / control_dt)

    for step in range(n_steps):
        t = step * control_dt
        q_current = env.get_q()

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

# 测试1: 原始启动
print("\n[测试1] 原始启动（突然加速）...")
print("  正在运行...")

# 临时保存当前的_ee_circle实现
from wheeled_ur5e_aligator_mpc import reference
original_ee_circle = reference.ReferenceGenerator._ee_circle

# 创建原始版本（突然启动）
@staticmethod
def _ee_circle_original(ts, ee_pos, base, base_z, ee_start):
    """原始版本 - 突然启动"""
    radius = 0.10
    period = 10.0
    cy = float(ee_start[1]) - radius
    cz = float(ee_start[2])
    cx = float(ee_start[0])
    for i, t in enumerate(ts):
        theta = 2 * np.pi * t / period
        ee_pos[i] = [cx, cy + radius * np.cos(theta), cz + radius * np.sin(theta)]
    base[:, :] = [0.0, 0.0, 0.0]
    base_z[:] = 0.2

# 使用原始版本
reference.ReferenceGenerator._ee_circle = _ee_circle_original
times_original, errors_original = run_test(use_smooth_startup=False, duration=15.0)

# 测试2: 平滑启动
print("\n[测试2] 平滑启动（2秒缓动）...")
print("  正在运行...")

# 创建平滑启动版本
@staticmethod
def _ee_circle_smooth(ts, ee_pos, base, base_z, ee_start):
    """平滑启动版本 - 2秒缓动"""
    radius = 0.10
    period = 10.0
    startup_time = 2.0
    cy = float(ee_start[1]) - radius
    cz = float(ee_start[2])
    cx = float(ee_start[0])
    for i, t in enumerate(ts):
        if t < startup_time:
            s = (t / startup_time) ** 3
            effective_t = s * t
        else:
            effective_t = t - startup_time * (1 - 1/3)
        theta = 2 * np.pi * effective_t / period
        ee_pos[i] = [cx, cy + radius * np.cos(theta), cz + radius * np.sin(theta)]
    base[:, :] = [0.0, 0.0, 0.0]
    base_z[:] = 0.2

reference.ReferenceGenerator._ee_circle = _ee_circle_smooth
times_smooth, errors_smooth = run_test(use_smooth_startup=True, duration=15.0)

# 分析对比
print("\n" + "="*80)
print("对比结果")
print("="*80)

print(f"\n{'阶段':<20} | {'原始启动':<15} | {'平滑启动':<15} | {'改进':<10}")
print("-" * 70)

# 0-2秒（启动阶段）
rms_orig_02 = np.sqrt(np.mean(errors_original[times_original <= 2]**2))
rms_smooth_02 = np.sqrt(np.mean(errors_smooth[times_smooth <= 2]**2))
improve_02 = (rms_orig_02 - rms_smooth_02) / rms_orig_02 * 100
print(f"{'0-2秒 (启动)':<20} | {rms_orig_02:>10.2f} cm | {rms_smooth_02:>10.2f} cm | {improve_02:>7.1f}%")

# 2-5秒（过渡阶段）
mask_orig_25 = (times_original > 2) & (times_original <= 5)
mask_smooth_25 = (times_smooth > 2) & (times_smooth <= 5)
rms_orig_25 = np.sqrt(np.mean(errors_original[mask_orig_25]**2))
rms_smooth_25 = np.sqrt(np.mean(errors_smooth[mask_smooth_25]**2))
improve_25 = (rms_orig_25 - rms_smooth_25) / rms_orig_25 * 100
print(f"{'2-5秒 (过渡)':<20} | {rms_orig_25:>10.2f} cm | {rms_smooth_25:>10.2f} cm | {improve_25:>7.1f}%")

# 5-15秒（稳态阶段）
rms_orig_515 = np.sqrt(np.mean(errors_original[times_original >= 5]**2))
rms_smooth_515 = np.sqrt(np.mean(errors_smooth[times_smooth >= 5]**2))
improve_515 = (rms_orig_515 - rms_smooth_515) / rms_orig_515 * 100
print(f"{'5-15秒 (稳态)':<20} | {rms_orig_515:>10.2f} cm | {rms_smooth_515:>10.2f} cm | {improve_515:>7.1f}%")

# 全程
rms_orig_all = np.sqrt(np.mean(errors_original**2))
rms_smooth_all = np.sqrt(np.mean(errors_smooth**2))
improve_all = (rms_orig_all - rms_smooth_all) / rms_orig_all * 100
print(f"{'全程 (0-15秒)':<20} | {rms_orig_all:>10.2f} cm | {rms_smooth_all:>10.2f} cm | {improve_all:>7.1f}%")

# 最大误差
max_orig = np.max(errors_original)
max_smooth = np.max(errors_smooth)
improve_max = (max_orig - max_smooth) / max_orig * 100
print(f"\n{'最大误差':<20} | {max_orig:>10.2f} cm | {max_smooth:>10.2f} cm | {improve_max:>7.1f}%")

# 绘制对比图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# 完整时间序列
ax1.plot(times_original, errors_original, 'b-', linewidth=1.5, label='原始启动（突然加速）', alpha=0.8)
ax1.plot(times_smooth, errors_smooth, 'g-', linewidth=1.5, label='平滑启动（2秒缓动）', alpha=0.8)
ax1.axhline(y=2.5, color='r', linestyle='--', linewidth=1, label='目标 (2.5 cm)', alpha=0.7)
ax1.axvspan(0, 2, alpha=0.15, color='yellow', label='启动阶段(0-2s)')
ax1.axvspan(2, 5, alpha=0.1, color='orange', label='过渡阶段(2-5s)')

ax1.set_xlabel('时间 (s)', fontsize=11)
ax1.set_ylabel('EE跟踪误差 (cm)', fontsize=11)
ax1.set_title('Phase 6-v2 启动策略对比 - 完整时间序列', fontsize=13, fontweight='bold')
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 15)

# 放大前5秒
ax2.plot(times_original[times_original <= 5], errors_original[times_original <= 5],
         'b-', linewidth=2, label='原始启动', marker='o', markersize=3, alpha=0.8)
ax2.plot(times_smooth[times_smooth <= 5], errors_smooth[times_smooth <= 5],
         'g-', linewidth=2, label='平滑启动', marker='s', markersize=3, alpha=0.8)
ax2.axhline(y=2.5, color='r', linestyle='--', linewidth=1, label='目标 (2.5 cm)', alpha=0.7)
ax2.axvspan(0, 2, alpha=0.15, color='yellow', label='启动阶段')

ax2.set_xlabel('时间 (s)', fontsize=11)
ax2.set_ylabel('EE跟踪误差 (cm)', fontsize=11)
ax2.set_title('启动阶段放大视图 (0-5秒)', fontsize=13, fontweight='bold')
ax2.legend(loc='upper right', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, 5)

plt.tight_layout()
output_path = _project_root / "startup_comparison.png"
plt.savefig(output_path, dpi=150)
print(f"\n对比图表保存至: {output_path}")

# 总结
print("\n" + "="*80)
print("总结")
print("="*80)

print(f"\n✅ 平滑启动策略效果:")
print(f"   - 启动阶段(0-2s): 误差降低 {improve_02:.1f}% ({rms_orig_02:.2f} → {rms_smooth_02:.2f} cm)")
print(f"   - 最大误差降低:   {improve_max:.1f}% ({max_orig:.2f} → {max_smooth:.2f} cm)")
print(f"   - 全程误差降低:   {improve_all:.1f}% ({rms_orig_all:.2f} → {rms_smooth_all:.2f} cm)")

print(f"\n💡 关键改进:")
print(f"   - 使用立方缓动函数(cubic ease-in)在前2秒平滑加速")
print(f"   - 避免参考轨迹突然启动导致的大追踪误差")
print(f"   - 稳态性能基本不变({rms_orig_515:.2f} vs {rms_smooth_515:.2f} cm)")

if improve_02 > 10:
    print(f"\n🎉 推荐采用平滑启动策略！")
else:
    print(f"\n⚠️  平滑启动改进有限，原始策略也可接受")

print("\n" + "="*80)
