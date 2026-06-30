#!/usr/bin/env python3
"""
方法5: 前馈加速度补偿

基于圆形轨迹的已知动力学，计算并施加前馈加速度补偿，
帮助机器人更快地跟上参考轨迹
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
print("方法5: 前馈加速度补偿")
print("="*80)

def compute_reference_acceleration(t, ee_start, period=10.0, radius=0.10):
    """
    计算圆形轨迹的加速度

    位置: y = cy + r*cos(ω*t), z = cz + r*sin(ω*t), ω = 2π/T
    速度: vy = -r*ω*sin(ω*t), vz = r*ω*cos(ω*t)
    加速度: ay = -r*ω²*cos(ω*t), az = -r*ω²*sin(ω*t)

    加速度是向心加速度，指向圆心
    """
    omega = 2 * np.pi / period
    theta = omega * t

    # 加速度（世界坐标系）
    ay = -radius * omega**2 * np.cos(theta)
    az = -radius * omega**2 * np.sin(theta)

    return np.array([0.0, ay, az])

def compute_feedforward_compensation(robot, q, ee_acc_ref):
    """
    计算前馈补偿

    简化方法：使用末端雅可比将加速度映射到关节空间
    u_ff = J^† * a_ref

    其中J^†是伪逆
    """
    # 获取末端雅可比（简化：只考虑位置部分）
    J = robot.finite_difference_jacobian_fk(q)  # (3, 10)

    # 伪逆
    J_pinv = np.linalg.pinv(J)  # (10, 3)

    # 前馈补偿 = 雅可比伪逆 * 参考加速度
    u_ff = J_pinv @ ee_acc_ref

    return u_ff

def run_test_feedforward(use_feedforward=True, duration=15.0):
    """运行测试，可选前馈补偿"""

    robot = WheeledUR5eModel()
    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    mpc_dt = 0.025
    control_dt = 0.002

    env = MujocoWheeledUR5eEnv(xml_path=str(xml_path), render=False, sim_dt=control_dt, control_dt=control_dt)

    mpc_weights = {'ee_pos': 300.0, 'terminal_ee_pos': 600.0, 'base_xy': 100.0, 'base_z': 100.0}
    mpc = AligatorWholeBodyMPC(robot, horizon=20, dt=mpc_dt, max_iters=10, weights=mpc_weights)

    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)
    pd_gains = FeedforwardPDGains(
        Kp_base_xy=150.0, Kd_base_xy=30.0,
        Kp_base_z=1500.0, Kd_base_z=300.0,
        Kp_arm=1800.0, Kd_arm=180.0
    )
    pd_controller = FeedforwardPDController(pd_gains)

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)
    low_level = LowLevelController(robot, dt=control_dt)

    env.reset(q0=robot.q_nominal)

    times = []
    errors = []
    ff_magnitude = []

    u_prev = np.zeros(robot.nu)
    last_mpc_time = -np.inf
    n_steps = int(duration / control_dt)

    # 前馈补偿缩放因子（避免过度补偿）
    ff_scale = 0.3 if use_feedforward else 0.0

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
            # 基础PD控制
            u_control, _ = pd_controller.compute_control(
                q_current, np.zeros(robot.nu), x_des, np.zeros(robot.nu),
                u_feedforward=u_feedforward
            )

            # 添加前馈加速度补偿
            if use_feedforward:
                ee_acc_ref = compute_reference_acceleration(t, ee_start)
                u_ff = compute_feedforward_compensation(robot, q_current, ee_acc_ref)

                # 只在启动阶段（前3秒）使用前馈，逐渐减弱
                if t < 3.0:
                    alpha = 1.0
                elif t < 6.0:
                    alpha = 1.0 - (t - 3.0) / 3.0  # 线性降低
                else:
                    alpha = 0.0

                u_control = u_control + ff_scale * alpha * u_ff
                ff_magnitude.append(np.linalg.norm(u_ff))
            else:
                ff_magnitude.append(0.0)
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

    return np.array(times), np.array(errors), np.array(ff_magnitude)

# 测试1: 无前馈
print("\n[测试1] 无前馈补偿...")
times_no_ff, errors_no_ff, _ = run_test_feedforward(use_feedforward=False, duration=15.0)

# 测试2: 有前馈
print("\n[测试2] 有前馈补偿...")
times_ff, errors_ff, ff_mag = run_test_feedforward(use_feedforward=True, duration=15.0)

# 对比分析
print("\n" + "="*80)
print("对比结果")
print("="*80)

print(f"\n{'阶段':<20} | {'无前馈':<15} | {'有前馈':<15} | {'改进':<10}")
print("-" * 70)

# 0-3秒（前馈活跃）
rms_no_ff_03 = np.sqrt(np.mean(errors_no_ff[times_no_ff <= 3]**2))
rms_ff_03 = np.sqrt(np.mean(errors_ff[times_ff <= 3]**2))
improve_03 = (rms_no_ff_03 - rms_ff_03) / rms_no_ff_03 * 100
print(f"{'0-3秒 (前馈活跃)':<20} | {rms_no_ff_03:>10.2f} cm | {rms_ff_03:>10.2f} cm | {improve_03:>7.1f}%")

# 0-1秒
rms_no_ff_01 = np.sqrt(np.mean(errors_no_ff[times_no_ff <= 1]**2))
rms_ff_01 = np.sqrt(np.mean(errors_ff[times_ff <= 1]**2))
improve_01 = (rms_no_ff_01 - rms_ff_01) / rms_no_ff_01 * 100
print(f"{'0-1秒 (关键启动)':<20} | {rms_no_ff_01:>10.2f} cm | {rms_ff_01:>10.2f} cm | {improve_01:>7.1f}%")

# 3-6秒（过渡）
mask_no_ff_36 = (times_no_ff > 3) & (times_no_ff <= 6)
mask_ff_36 = (times_ff > 3) & (times_ff <= 6)
rms_no_ff_36 = np.sqrt(np.mean(errors_no_ff[mask_no_ff_36]**2))
rms_ff_36 = np.sqrt(np.mean(errors_ff[mask_ff_36]**2))
improve_36 = (rms_no_ff_36 - rms_ff_36) / rms_no_ff_36 * 100
print(f"{'3-6秒 (过渡)':<20} | {rms_no_ff_36:>10.2f} cm | {rms_ff_36:>10.2f} cm | {improve_36:>7.1f}%")

# 6-15秒（稳态）
rms_no_ff_615 = np.sqrt(np.mean(errors_no_ff[times_no_ff >= 6]**2))
rms_ff_615 = np.sqrt(np.mean(errors_ff[times_ff >= 6]**2))
improve_615 = (rms_no_ff_615 - rms_ff_615) / rms_no_ff_615 * 100
print(f"{'6-15秒 (稳态)':<20} | {rms_no_ff_615:>10.2f} cm | {rms_ff_615:>10.2f} cm | {improve_615:>7.1f}%")

# 全程
rms_no_ff_all = np.sqrt(np.mean(errors_no_ff**2))
rms_ff_all = np.sqrt(np.mean(errors_ff**2))
improve_all = (rms_no_ff_all - rms_ff_all) / rms_no_ff_all * 100
print(f"{'全程 (0-15秒)':<20} | {rms_no_ff_all:>10.2f} cm | {rms_ff_all:>10.2f} cm | {improve_all:>7.1f}%")

# 最大误差
max_no_ff = np.max(errors_no_ff)
max_ff = np.max(errors_ff)
improve_max = (max_no_ff - max_ff) / max_no_ff * 100
print(f"\n{'最大误差':<20} | {max_no_ff:>10.2f} cm | {max_ff:>10.2f} cm | {improve_max:>7.1f}%")

# 绘图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14))

# 子图1: 误差对比
ax1.plot(times_no_ff, errors_no_ff, 'r-', linewidth=1.5, label='No Feedforward', alpha=0.8)
ax1.plot(times_ff, errors_ff, 'b-', linewidth=1.5, label='With Feedforward', alpha=0.8)
ax1.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='Target (2.5 cm)', alpha=0.7)
ax1.axvspan(0, 3, alpha=0.15, color='cyan', label='Feedforward Active (0-3s)')
ax1.axvspan(3, 6, alpha=0.1, color='lightblue', label='Fade Out (3-6s)')

ax1.set_xlabel('Time (s)', fontsize=11)
ax1.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax1.set_title('Phase 6-v2 Feedforward Compensation - Error Comparison', fontsize=13, fontweight='bold')
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 15)

# 子图2: 前馈幅值
times_ff_plot = times_ff[:len(ff_mag)]
ax2.plot(times_ff_plot, ff_mag, 'g-', linewidth=2, label='Feedforward Magnitude')
ax2.axvspan(0, 3, alpha=0.15, color='cyan')
ax2.axvspan(3, 6, alpha=0.1, color='lightblue')

ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('||u_ff|| (rad/s)', fontsize=11)
ax2.set_title('Feedforward Compensation Magnitude', fontsize=13, fontweight='bold')
ax2.legend(loc='upper right', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, 15)

# 子图3: 前5秒放大
ax3.plot(times_no_ff[times_no_ff <= 5], errors_no_ff[times_no_ff <= 5],
         'r-', linewidth=2, label='No Feedforward', marker='o', markersize=3, alpha=0.8)
ax3.plot(times_ff[times_ff <= 5], errors_ff[times_ff <= 5],
         'b-', linewidth=2, label='With Feedforward', marker='s', markersize=3, alpha=0.8)
ax3.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='Target', alpha=0.7)
ax3.axvspan(0, 3, alpha=0.15, color='cyan', label='FF Active')

ax3.set_xlabel('Time (s)', fontsize=11)
ax3.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax3.set_title('Startup Phase Zoomed In (0-5s)', fontsize=13, fontweight='bold')
ax3.legend(loc='upper right', fontsize=10)
ax3.grid(True, alpha=0.3)
ax3.set_xlim(0, 5)

plt.tight_layout()
output_path = _project_root / "feedforward_compensation_comparison.png"
plt.savefig(output_path, dpi=150)
print(f"\n对比图表保存至: {output_path}")

# 总结
print("\n" + "="*80)
print("总结")
print("="*80)

print(f"\n✅ 前馈补偿策略效果:")
print(f"   - 关键启动(0-1s): 误差{'降低' if improve_01 > 0 else '增加'} {abs(improve_01):.1f}% ({rms_no_ff_01:.2f} → {rms_ff_01:.2f} cm)")
print(f"   - 活跃阶段(0-3s): 误差{'降低' if improve_03 > 0 else '增加'} {abs(improve_03):.1f}% ({rms_no_ff_03:.2f} → {rms_ff_03:.2f} cm)")
print(f"   - 最大误差{'降低' if improve_max > 0 else '增加'}:   {abs(improve_max):.1f}% ({max_no_ff:.2f} → {max_ff:.2f} cm)")
print(f"   - 全程误差{'降低' if improve_all > 0 else '增加'}:   {abs(improve_all):.1f}% ({rms_no_ff_all:.2f} → {rms_ff_all:.2f} cm)")

print(f"\n💡 前馈补偿原理:")
print(f"   - 基于圆形轨迹计算参考加速度（向心加速度）")
print(f"   - 通过雅可比伪逆映射到关节空间: u_ff = J^† * a_ref")
print(f"   - 0-3s: 全幅度前馈 (scale=0.3)")
print(f"   - 3-6s: 线性淡出")
print(f"   - 6s+: 关闭前馈")

if improve_01 > 20:
    print(f"\n🎉 推荐采用前馈补偿策略！")
elif improve_01 > 10:
    print(f"\n✅ 前馈补偿有明显改善效果")
elif improve_01 > 0:
    print(f"\n✅ 前馈补偿有一定改善效果")
else:
    print(f"\n⚠️  前馈补偿效果不明显或带来负面影响")

print("\n" + "="*80)
