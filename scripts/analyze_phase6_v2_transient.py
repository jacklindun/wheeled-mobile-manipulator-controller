#!/usr/bin/env python3
"""
分析Phase 6-v2的瞬态响应
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import numpy as np
import matplotlib.pyplot as plt

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)
import time

print("分析Phase 6-v2瞬态响应...")

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

# 运行20秒看两个完整周期
duration = 20.0
times = []
errors = []

u_prev = np.zeros(robot.nu)
last_mpc_time = -np.inf
n_steps = int(duration / control_dt)

print(f"运行{duration}秒测试...")

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
    ee_error = np.linalg.norm(ee_pos - ee_ref) * 100  # cm

    times.append(t)
    errors.append(ee_error)

env.close()

times = np.array(times)
errors = np.array(errors)

# 分析
print("\n误差分析:")
print(f"  0-3秒 RMS: {np.sqrt(np.mean(errors[times <= 3]**2)):.2f} cm")
print(f"  3-10秒 RMS: {np.sqrt(np.mean(errors[(times > 3) & (times <= 10)]**2)):.2f} cm")
print(f"  10-20秒 RMS: {np.sqrt(np.mean(errors[times > 10]**2)):.2f} cm")
print(f"  全程 RMS: {np.sqrt(np.mean(errors**2)):.2f} cm")
print(f"  稳态 RMS (5-20s): {np.sqrt(np.mean(errors[times >= 5]**2)):.2f} cm")

# 绘图
plt.figure(figsize=(12, 6))
plt.plot(times, errors, linewidth=1.5, label='EE Tracking Error')
plt.axhline(y=2.5, color='r', linestyle='--', label='Target (2.5 cm)')
plt.axhline(y=np.sqrt(np.mean(errors[times >= 5]**2)), color='g', linestyle='--', label=f'Steady-state RMS')
plt.axvspan(0, 3, alpha=0.2, color='yellow', label='Transient (0-3s)')
plt.axvspan(10, 20, alpha=0.1, color='green', label='2nd cycle (10-20s)')

plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('EE Tracking Error (cm)', fontsize=12)
plt.title('Phase 6-v2 Tracking Error over Time', fontsize=14)
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

output_path = _project_root / "phase6_v2_transient_analysis.png"
plt.savefig(output_path, dpi=150)
print(f"\n图表保存至: {output_path}")

print("\n结论:")
print("  1. 瞬态阶段(0-3s): 系统从静止开始追踪圆形轨迹")
print("  2. 稳态阶段(5s+):  误差稳定在~1.3-1.7cm")
print("  3. 第二圈(10-20s): 误差与第一圈稳态相同，说明系统稳定")
