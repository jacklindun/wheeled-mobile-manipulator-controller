#!/usr/bin/env python3
"""
方法2: MPC预热 - 提前规划减小启动误差

在t=0时运行多次MPC迭代，让机器人提前"看到"未来轨迹
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
print("方法2: MPC预热减小启动误差")
print("="*80)

def run_test_with_warmup(use_warmup=True, duration=15.0):
    """运行测试，可选MPC预热"""

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

    # === MPC预热 ===
    if use_warmup:
        print("  执行MPC预热...")
        q_current = robot.q_nominal.copy()

        # 预热：在t=0提前看未来轨迹，进行5次MPC求解
        # 这样warm start会包含完整的未来轨迹信息
        for i in range(5):
            ref_traj = ref_gen.get_reference(t=0.0, horizon=mpc.horizon, dt=mpc.dt)
            u0, q_pred, info = mpc.solve(q_current=q_current, ref_traj=ref_traj, u_prev=np.zeros(robot.nu))
            # 不执行，只是让MPC建立warm start
        print(f"  ✓ 预热完成，warm start已建立")

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

# 测试1: 无预热
print("\n[测试1] 无MPC预热...")
times_no_warmup, errors_no_warmup = run_test_with_warmup(use_warmup=False, duration=15.0)

# 测试2: 有预热
print("\n[测试2] 有MPC预热...")
times_warmup, errors_warmup = run_test_with_warmup(use_warmup=True, duration=15.0)

# 对比分析
print("\n" + "="*80)
print("对比结果")
print("="*80)

print(f"\n{'阶段':<20} | {'无预热':<15} | {'有预热':<15} | {'改进':<10}")
print("-" * 70)

# 0-2秒
rms_no_02 = np.sqrt(np.mean(errors_no_warmup[times_no_warmup <= 2]**2))
rms_warmup_02 = np.sqrt(np.mean(errors_warmup[times_warmup <= 2]**2))
improve_02 = (rms_no_02 - rms_warmup_02) / rms_no_02 * 100
print(f"{'0-2秒 (启动)':<20} | {rms_no_02:>10.2f} cm | {rms_warmup_02:>10.2f} cm | {improve_02:>7.1f}%")

# 2-5秒
mask_no_25 = (times_no_warmup > 2) & (times_no_warmup <= 5)
mask_warmup_25 = (times_warmup > 2) & (times_warmup <= 5)
rms_no_25 = np.sqrt(np.mean(errors_no_warmup[mask_no_25]**2))
rms_warmup_25 = np.sqrt(np.mean(errors_warmup[mask_warmup_25]**2))
improve_25 = (rms_no_25 - rms_warmup_25) / rms_no_25 * 100
print(f"{'2-5秒 (过渡)':<20} | {rms_no_25:>10.2f} cm | {rms_warmup_25:>10.2f} cm | {improve_25:>7.1f}%")

# 5-15秒
rms_no_515 = np.sqrt(np.mean(errors_no_warmup[times_no_warmup >= 5]**2))
rms_warmup_515 = np.sqrt(np.mean(errors_warmup[times_warmup >= 5]**2))
improve_515 = (rms_no_515 - rms_warmup_515) / rms_no_515 * 100
print(f"{'5-15秒 (稳态)':<20} | {rms_no_515:>10.2f} cm | {rms_warmup_515:>10.2f} cm | {improve_515:>7.1f}%")

# 全程
rms_no_all = np.sqrt(np.mean(errors_no_warmup**2))
rms_warmup_all = np.sqrt(np.mean(errors_warmup**2))
improve_all = (rms_no_all - rms_warmup_all) / rms_no_all * 100
print(f"{'全程 (0-15秒)':<20} | {rms_no_all:>10.2f} cm | {rms_warmup_all:>10.2f} cm | {improve_all:>7.1f}%")

# 最大误差
max_no = np.max(errors_no_warmup)
max_warmup = np.max(errors_warmup)
improve_max = (max_no - max_warmup) / max_no * 100
print(f"\n{'最大误差':<20} | {max_no:>10.2f} cm | {max_warmup:>10.2f} cm | {improve_max:>7.1f}%")

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# 完整时间序列
ax1.plot(times_no_warmup, errors_no_warmup, 'r-', linewidth=1.5, label='无MPC预热', alpha=0.8)
ax1.plot(times_warmup, errors_warmup, 'b-', linewidth=1.5, label='有MPC预热', alpha=0.8)
ax1.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='目标 (2.5 cm)', alpha=0.7)
ax1.axvspan(0, 2, alpha=0.15, color='yellow', label='启动阶段(0-2s)')

ax1.set_xlabel('Time (s)', fontsize=11)
ax1.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax1.set_title('Phase 6-v2 MPC预热效果对比 - 完整时间序列', fontsize=13, fontweight='bold')
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 15)

# 放大前5秒
ax2.plot(times_no_warmup[times_no_warmup <= 5], errors_no_warmup[times_no_warmup <= 5],
         'r-', linewidth=2, label='无MPC预热', marker='o', markersize=3, alpha=0.8)
ax2.plot(times_warmup[times_warmup <= 5], errors_warmup[times_warmup <= 5],
         'b-', linewidth=2, label='有MPC预热', marker='s', markersize=3, alpha=0.8)
ax2.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='目标 (2.5 cm)', alpha=0.7)
ax2.axvspan(0, 2, alpha=0.15, color='yellow', label='启动阶段')

ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax2.set_title('启动阶段放大视图 (0-5秒)', fontsize=13, fontweight='bold')
ax2.legend(loc='upper right', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, 5)

plt.tight_layout()
output_path = _project_root / "mpc_warmup_comparison.png"
plt.savefig(output_path, dpi=150)
print(f"\n对比图表保存至: {output_path}")

# 总结
print("\n" + "="*80)
print("总结")
print("="*80)

print(f"\n✅ MPC预热策略效果:")
print(f"   - 启动阶段(0-2s): 误差{'降低' if improve_02 > 0 else '增加'} {abs(improve_02):.1f}% ({rms_no_02:.2f} → {rms_warmup_02:.2f} cm)")
print(f"   - 最大误差{'降低' if improve_max > 0 else '增加'}:   {abs(improve_max):.1f}% ({max_no:.2f} → {max_warmup:.2f} cm)")
print(f"   - 全程误差{'降低' if improve_all > 0 else '增加'}:   {abs(improve_all):.1f}% ({rms_no_all:.2f} → {rms_warmup_all:.2f} cm)")

print(f"\n💡 MPC预热原理:")
print(f"   - 在t=0时运行5次MPC迭代，建立完整的warm start")
print(f"   - MPC提前'看到'未来轨迹，规划好初始动作")
print(f"   - 减少从静止状态突然启动的滞后")

if improve_02 > 10:
    print(f"\n🎉 推荐采用MPC预热策略！")
elif improve_02 > 0:
    print(f"\n✅ MPC预热有一定改善效果")
else:
    print(f"\n⚠️  MPC预热效果不明显，可能需要其他方法")

print("\n" + "="*80)
