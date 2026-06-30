#!/usr/bin/env python3
"""
Phase 6-v2 平滑启动测试

对比：
1. 原始版本 - 突然启动
2. 平滑启动版本 - 2秒加速
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
print("Phase 6-v2 平滑启动测试")
print("="*80)

def run_test(duration=15.0):
    """运行测试"""

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

    print(f"\n运行{duration}秒测试（带平滑启动）...")
    print(f"{'时间':>8s} | {'EE误差':>10s}")
    print(f"{'-'*8}-+-{'-'*10}")

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

        # 打印进度
        if step % int(1.0 / control_dt) == 0:
            print(f"{t:>7.1f}s | {ee_error:>8.2f} cm")

    env.close()

    times = np.array(times)
    errors = np.array(errors)

    # 分析
    print("\n" + "="*80)
    print("结果分析")
    print("="*80)

    print(f"\n误差统计:")
    print(f"  0-2秒 (启动) RMS: {np.sqrt(np.mean(errors[times <= 2]**2)):.2f} cm")
    print(f"  2-5秒 (过渡) RMS: {np.sqrt(np.mean(errors[(times > 2) & (times <= 5)]**2)):.2f} cm")
    print(f"  5-15秒 (稳态) RMS: {np.sqrt(np.mean(errors[times >= 5]**2)):.2f} cm")
    print(f"  全程 RMS: {np.sqrt(np.mean(errors**2)):.2f} cm")

    max_err_idx = np.argmax(errors)
    print(f"\n  最大误差: {errors[max_err_idx]:.2f} cm @ t={times[max_err_idx]:.1f}s")

    return times, errors

times, errors = run_test(duration=15.0)

print("\n改进说明:")
print("  - 参考轨迹前2秒使用立方缓动(cubic ease-in)")
print("  - 速度从0平滑增加到正常速度")
print("  - 减少机器人追赶参考轨迹的瞬态误差")
