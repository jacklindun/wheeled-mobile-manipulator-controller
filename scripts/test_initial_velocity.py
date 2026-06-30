#!/usr/bin/env python3
"""
方法3: 初始状态匹配 - 从运动状态启动

让机器人的初始速度匹配参考轨迹的初始速度，
而不是从静止状态突然启动
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
print("方法3: 初始速度匹配减小启动误差")
print("="*80)

def compute_initial_velocity(ee_start, period=10.0, radius=0.10):
    """
    计算圆形轨迹在t=0时的末端速度

    圆形轨迹:
    y = cy + r*cos(2π*t/T)
    z = cz + r*sin(2π*t/T)

    速度:
    vy = -r*(2π/T)*sin(2π*t/T)
    vz = r*(2π/T)*cos(2π*t/T)

    在t=0:
    vy = 0
    vz = r*2π/T
    """
    omega = 2 * np.pi / period
    vz = radius * omega  # m/s
    return np.array([0.0, 0.0, vz])

def run_test_with_initial_velocity(use_initial_velocity=True, duration=15.0):
    """运行测试，可选初始速度匹配"""

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

    # === 初始速度匹配 ===
    if use_initial_velocity:
        # 计算参考轨迹的初始速度
        ee_vel_ref = compute_initial_velocity(ee_start)
        print(f"  参考轨迹初始速度: vz = {ee_vel_ref[2]:.4f} m/s")

        # 给机器人一个初始速度（通过提前执行几步）
        # 这里简单模拟：让MPC输出一个非零的初始控制
        ref_traj = ref_gen.get_reference(t=0.0, horizon=mpc.horizon, dt=mpc.dt)
        u_init, _, _ = mpc.solve(q_current=robot.q_nominal, ref_traj=ref_traj, u_prev=None)

        # 在仿真中"预执行"几步，让机器人获得初速度
        for _ in range(5):  # 执行5步 = 0.01秒
            q_target = low_level.compute_q_des(env.get_q(), u_init)
            env.step(q_target)

        print(f"  ✓ 机器人已获得初始速度")
    else:
        u_init = np.zeros(robot.nu)

    # Data logging
    times = []
    errors = []

    u_prev = u_init.copy() if use_initial_velocity else np.zeros(robot.nu)
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

# 测试1: 从静止启动
print("\n[测试1] 从静止状态启动...")
times_static, errors_static = run_test_with_initial_velocity(use_initial_velocity=False, duration=15.0)

# 测试2: 带初始速度启动
print("\n[测试2] 带初始速度启动...")
times_moving, errors_moving = run_test_with_initial_velocity(use_initial_velocity=True, duration=15.0)

# 对比分析
print("\n" + "="*80)
print("对比结果")
print("="*80)

print(f"\n{'阶段':<20} | {'从静止启动':<15} | {'带初速度启动':<15} | {'改进':<10}")
print("-" * 75)

# 0-2秒
rms_static_02 = np.sqrt(np.mean(errors_static[times_static <= 2]**2))
rms_moving_02 = np.sqrt(np.mean(errors_moving[times_moving <= 2]**2))
improve_02 = (rms_static_02 - rms_moving_02) / rms_static_02 * 100
print(f"{'0-2秒 (启动)':<20} | {rms_static_02:>10.2f} cm | {rms_moving_02:>10.2f} cm | {improve_02:>7.1f}%")

# 2-5秒
mask_static_25 = (times_static > 2) & (times_static <= 5)
mask_moving_25 = (times_moving > 2) & (times_moving <= 5)
rms_static_25 = np.sqrt(np.mean(errors_static[mask_static_25]**2))
rms_moving_25 = np.sqrt(np.mean(errors_moving[mask_moving_25]**2))
improve_25 = (rms_static_25 - rms_moving_25) / rms_static_25 * 100
print(f"{'2-5秒 (过渡)':<20} | {rms_static_25:>10.2f} cm | {rms_moving_25:>10.2f} cm | {improve_25:>7.1f}%")

# 稳态
rms_static_515 = np.sqrt(np.mean(errors_static[times_static >= 5]**2))
rms_moving_515 = np.sqrt(np.mean(errors_moving[times_moving >= 5]**2))
improve_515 = (rms_static_515 - rms_moving_515) / rms_static_515 * 100
print(f"{'5-15秒 (稳态)':<20} | {rms_static_515:>10.2f} cm | {rms_moving_515:>10.2f} cm | {improve_515:>7.1f}%")

# 全程
rms_static_all = np.sqrt(np.mean(errors_static**2))
rms_moving_all = np.sqrt(np.mean(errors_moving**2))
improve_all = (rms_static_all - rms_moving_all) / rms_static_all * 100
print(f"{'全程 (0-15秒)':<20} | {rms_static_all:>10.2f} cm | {rms_moving_all:>10.2f} cm | {improve_all:>7.1f}%")

# 最大误差
max_static = np.max(errors_static)
max_moving = np.max(errors_moving)
improve_max = (max_static - max_moving) / max_static * 100
print(f"\n{'最大误差':<20} | {max_static:>10.2f} cm | {max_moving:>10.2f} cm | {improve_max:>7.1f}%")

# 前1秒的平均误差（最关键）
rms_static_01 = np.sqrt(np.mean(errors_static[times_static <= 1]**2))
rms_moving_01 = np.sqrt(np.mean(errors_moving[times_moving <= 1]**2))
improve_01 = (rms_static_01 - rms_moving_01) / rms_static_01 * 100
print(f"{'0-1秒 (关键启动)':<20} | {rms_static_01:>10.2f} cm | {rms_moving_01:>10.2f} cm | {improve_01:>7.1f}%")

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# 完整时间序列
ax1.plot(times_static, errors_static, 'r-', linewidth=1.5, label='From Static Start', alpha=0.8)
ax1.plot(times_moving, errors_moving, 'b-', linewidth=1.5, label='With Initial Velocity', alpha=0.8)
ax1.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='Target (2.5 cm)', alpha=0.7)
ax1.axvspan(0, 2, alpha=0.15, color='yellow', label='Startup Phase(0-2s)')

ax1.set_xlabel('Time (s)', fontsize=11)
ax1.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax1.set_title('Phase 6-v2 Initial Velocity Matching - Full Timeline', fontsize=13, fontweight='bold')
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 15)

# 放大前5秒
ax2.plot(times_static[times_static <= 5], errors_static[times_static <= 5],
         'r-', linewidth=2, label='From Static Start', marker='o', markersize=3, alpha=0.8)
ax2.plot(times_moving[times_moving <= 5], errors_moving[times_moving <= 5],
         'b-', linewidth=2, label='With Initial Velocity', marker='s', markersize=3, alpha=0.8)
ax2.axhline(y=2.5, color='gray', linestyle='--', linewidth=1, label='Target (2.5 cm)', alpha=0.7)
ax2.axvspan(0, 1, alpha=0.2, color='red', label='Critical (0-1s)')

ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('EE Tracking Error (cm)', fontsize=11)
ax2.set_title('Startup Phase Zoomed In (0-5s)', fontsize=13, fontweight='bold')
ax2.legend(loc='upper right', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, 5)

plt.tight_layout()
output_path = _project_root / "initial_velocity_comparison.png"
plt.savefig(output_path, dpi=150)
print(f"\n对比图表保存至: {output_path}")

# 总结
print("\n" + "="*80)
print("总结")
print("="*80)

print(f"\n✅ 初始速度匹配策略效果:")
print(f"   - 关键启动(0-1s): 误差{'降低' if improve_01 > 0 else '增加'} {abs(improve_01):.1f}% ({rms_static_01:.2f} → {rms_moving_01:.2f} cm)")
print(f"   - 启动阶段(0-2s): 误差{'降低' if improve_02 > 0 else '增加'} {abs(improve_02):.1f}% ({rms_static_02:.2f} → {rms_moving_02:.2f} cm)")
print(f"   - 最大误差{'降低' if improve_max > 0 else '增加'}:   {abs(improve_max):.1f}% ({max_static:.2f} → {max_moving:.2f} cm)")
print(f"   - 全程误差{'降低' if improve_all > 0 else '增加'}:   {abs(improve_all):.1f}% ({rms_static_all:.2f} → {rms_moving_all:.2f} cm)")

print(f"\n💡 初始速度匹配原理:")
print(f"   - 计算参考轨迹在t=0的速度 (vz ≈ 0.063 m/s)")
print(f"   - 让机器人在启动前获得相应的初速度")
print(f"   - 避免从静止突然加速到运动状态的冲击")

if improve_01 > 20:
    print(f"\n🎉 推荐采用初始速度匹配策略！显著改善启动阶段")
elif improve_01 > 10:
    print(f"\n✅ 初始速度匹配有明显改善效果")
elif improve_01 > 0:
    print(f"\n✅ 初始速度匹配有一定改善效果")
else:
    print(f"\n⚠️  初始速度匹配效果不明显")

print("\n" + "="*80)
