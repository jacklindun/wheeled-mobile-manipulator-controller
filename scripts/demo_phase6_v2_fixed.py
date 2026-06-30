#!/usr/bin/env python3
"""
Phase 6-v2 MuJoCo闭环仿真演示

使用成功的demo.py架构，添加Phase 6-v2的插值+前馈PD
"""

import sys
import gc
import time
from pathlib import Path
import numpy as np

# 使用与run_demo.py相同的路径设置
_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController

# Phase 6-v2组件
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)


def run_phase6_v2(
    xml_path: str,
    scenario: str = "ee_circle",
    duration: float = 20.0,
    render: bool = False,
    horizon: int = 15,
    mpc_dt: float = 0.05,
    sim_dt: float = 0.002,
):
    """运行Phase 6-v2: 运动学MPC + 插值 + 前馈PD"""

    print("="*80)
    print("Phase 6-v2 MuJoCo闭环仿真")
    print("="*80)
    print(f"\n配置:")
    print(f"  场景: {scenario}")
    print(f"  时长: {duration}s")
    print(f"  MPC: {horizon}步, {mpc_dt}s (20Hz)")
    print(f"  仿真: {sim_dt}s (500Hz)")
    print("="*80)

    # 创建组件
    robot = WheeledUR5eModel()
    env = MujocoWheeledUR5eEnv(xml_path=xml_path, render=render,
                               sim_dt=sim_dt, control_dt=mpc_dt)

    # MPC控制器（与Phase 1-3相同）
    mpc = AligatorWholeBodyMPC(robot, horizon=horizon, dt=mpc_dt, max_iters=10)

    # Phase 6-v2新增组件
    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=sim_dt)

    pd_gains = FeedforwardPDGains(
        Kp_base_xy=50.0, Kd_base_xy=10.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=500.0, Kd_arm=50.0
    )
    pd_controller = FeedforwardPDController(pd_gains)

    # 参考轨迹
    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario=scenario, ee_start=ee_start)

    # 低层控制
    low_level = LowLevelController(robot, dt=mpc_dt)

    print(f"\n✓ 组件创建完成")
    print(f"  Phase 6-v2插值: {interpolator.ratio}:1 (20Hz→500Hz)")
    print(f"  Phase 6-v2前馈PD: Kp={pd_gains.Kp_arm[0]:.0f}, Kd={pd_gains.Kd_arm[0]:.0f}")

    # 重置
    env.reset(q0=robot.q_nominal)
    u_prev = np.zeros(robot.nu)

    # 记录
    times = []
    ee_errors = []
    mpc_converged = []
    mpc_solve_times = []
    controls = []

    n_mpc_steps = int(duration / mpc_dt)
    mujoco_substeps = int(mpc_dt / sim_dt)

    print(f"\n开始仿真...")
    print(f"{'时间':<8} {'EE误差':<10} {'MPC时间':<12} {'收敛':<8} {'模式':<15}")
    print("-"*80)

    for step in range(n_mpc_steps):
        t = step * mpc_dt

        # 获取当前状态
        q = env.get_q()

        # 生成参考
        ref_traj = ref_gen.get_reference(t=t, horizon=horizon, dt=mpc_dt)

        # 求解MPC
        u0, q_pred, info = mpc.solve(q_current=q, ref_traj=ref_traj, u_prev=u_prev)

        # 更新插值器（Phase 6-v2）
        ts_mpc = np.arange(len(q_pred)) * mpc_dt
        trajectory = {
            'xs': q_pred,
            'us': np.tile(u0, (len(q_pred)-1, 1)),
            'ts': ts_mpc,
        }
        interpolator.update_trajectory(trajectory, t)

        # MuJoCo子步循环
        for substep in range(mujoco_substeps):
            t_sub = t + substep * sim_dt

            # 插值获取期望（Phase 6-v2）
            x_des, u_ff = interpolator.interpolate(t_sub)

            if x_des is not None:
                # 前馈PD控制（Phase 6-v2）
                q_current = env.get_q()
                v_current = np.zeros(robot.nu)
                q_des = x_des
                v_des = np.zeros(robot.nu)

                u_control, _ = pd_controller.compute_control(
                    q_current, v_current, q_des, v_des, u_feedforward=u_ff
                )
                control_mode = "插值+PD"
            else:
                # 回退到baseline方法
                u_control = u0
                q_des = low_level.compute_q_des(q, u0)
                control_mode = "Baseline"

            # 应用控制
            env.set_target_marker(ref_traj["ee_pos"][0])
            env.step(q_des)

            # 测量误差
            ee_pos = env.get_ee_pos()
            ref_traj_sub = ref_gen.get_reference(t_sub, 1, mpc_dt)
            ee_target = ref_traj_sub['ee_pos'][0]
            ee_error = np.linalg.norm(ee_pos - ee_target)

            # 记录
            times.append(t_sub)
            ee_errors.append(ee_error)
            controls.append(u_control.copy())

        # MPC统计
        mpc_converged.append(info['success'])
        mpc_solve_times.append(info['solve_time'])
        u_prev = u0

        # 打印
        if step % int(1.0 / mpc_dt) == 0:
            conv_str = "✓" if info['success'] else "✗"
            print(f"{t:>6.1f}s  {ee_error*100:>8.2f}cm  {info['solve_time']*1000:>10.1f}ms  {conv_str:>6}  {control_mode:<15}")

    # 统计
    print(f"\n" + "="*80)
    print("Phase 6-v2 仿真结果")
    print("="*80)

    ee_errors = np.array(ee_errors)
    controls = np.array(controls)

    ee_rms = np.sqrt(np.mean(ee_errors**2)) * 100
    ee_max = np.max(ee_errors) * 100
    convergence_rate = (sum(mpc_converged) / len(mpc_converged)) * 100
    avg_solve = np.mean(mpc_solve_times) * 1000

    print(f"\n跟踪误差:")
    print(f"  RMS误差:  {ee_rms:.2f} cm")
    print(f"  最大误差: {ee_max:.2f} cm")

    print(f"\nMPC性能:")
    print(f"  收敛率:       {convergence_rate:.1f}% ({sum(mpc_converged)}/{len(mpc_converged)})")
    print(f"  平均求解时间: {avg_solve:.1f} ms")

    control_diff = np.diff(controls[:, 0])
    control_smoothness = np.std(control_diff)

    print(f"\n控制质量:")
    print(f"  控制步数: {len(controls)}")
    print(f"  控制频率: {len(controls)/duration:.0f} Hz")
    print(f"  平滑度: {control_smoothness:.4f}")

    print(f"\nPhase 6-v2特性:")
    print(f"  ✓ 插值器: {len(mpc_converged)}次MPC → {len(controls)}个控制点")
    print(f"  ✓ 频率提升: {len(controls)/len(mpc_converged):.0f}倍 (500Hz vs 20Hz)")

    print(f"\n" + "="*80)

    # 清理
    env.close()
    mpc.close()
    del mpc
    del env
    del robot
    gc.collect()

    return {
        'ee_rms_cm': ee_rms,
        'ee_max_cm': ee_max,
        'convergence_rate': convergence_rate,
        'avg_solve_ms': avg_solve,
    }


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='ee_circle',
                       choices=['ee_circle', 'ee_line', 'base_and_ee', 'base_z_test'])
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--no-render', action='store_true')
    args = parser.parse_args()

    xml_path = str(_project_root / "assets" / "wheeled_ur5e.xml")

    result = run_phase6_v2(
        xml_path=xml_path,
        scenario=args.scenario,
        duration=args.duration,
        render=not args.no_render
    )

    print(f"\n✓ 演示完成")
    print(f"  RMS误差: {result['ee_rms_cm']:.2f} cm")
    print(f"  收敛率: {result['convergence_rate']:.1f}%")
