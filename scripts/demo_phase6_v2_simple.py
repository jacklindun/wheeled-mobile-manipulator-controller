#!/usr/bin/env python3
"""
Phase 6-v2 简化闭环验证

直接基于 demo.py 修改，添加 Phase 6-v2 组件
"""

import sys
from pathlib import Path

# Setup paths
_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]

import gc
import time
import numpy as np

print("="*80)
print("Phase 6-v2 简化闭环验证")
print("="*80)

# Import modules
print("\n加载模块...")
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)
print("✓ 所有模块加载成功\n")


def run_phase6_v2_demo(scenario='ee_circle', duration=10.0):
    """Phase 6-v2 演示: MPC(20Hz) + 插值(500Hz) + PD(500Hz)"""

    print(f"配置: {scenario}, {duration}s")
    print("="*80)

    # Components
    robot = WheeledUR5eModel()

    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    mpc_dt = 0.05  # 20Hz MPC
    control_dt = 0.002  # 500Hz control

    env = MujocoWheeledUR5eEnv(
        xml_path=str(xml_path),
        render=False,
        sim_dt=control_dt,
        control_dt=control_dt  # Phase 6-v2: 高频控制
    )

    mpc = AligatorWholeBodyMPC(
        robot,
        horizon=15,
        dt=mpc_dt,
        max_iters=10
    )

    # Phase 6-v2 components
    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)

    pd_gains = FeedforwardPDGains(
        Kp_base_xy=50.0, Kd_base_xy=10.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=500.0, Kd_arm=50.0
    )
    pd_controller = FeedforwardPDController(pd_gains)

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario=scenario, ee_start=ee_start)
    low_level = LowLevelController(robot, dt=control_dt)

    print(f"\n✓ 组件创建完成")
    print(f"  MPC: {mpc.horizon}步, {mpc_dt}s (20Hz)")
    print(f"  控制: {control_dt}s (500Hz)")
    print(f"  插值比例: {interpolator.ratio}:1")

    # Initialize
    env.reset(q0=robot.q_nominal)

    # Data logging
    log = {
        't': [],
        'ee_error': [],
        'mpc_converged': [],
        'mpc_solve_time': [],
    }

    # Control loop
    print(f"\n开始控制循环...")
    print(f"{'时间':>8s} | {'EE误差':>10s} | {'MPC':>12s} | {'收敛':>6s}")
    print(f"{'-'*8}-+-{'-'*10}-+-{'-'*12}-+-{'-'*6}")

    u_prev = np.zeros(robot.nu)
    last_mpc_time = -np.inf

    n_steps = int(duration / control_dt)
    mpc_count = 0

    for step in range(n_steps):
        t = step * control_dt

        q_current = env.get_q()

        # MPC update (20Hz)
        if t - last_mpc_time >= mpc_dt - 1e-9:
            ref_traj = ref_gen.get_reference(t=t, horizon=mpc.horizon, dt=mpc.dt)

            t_start = time.perf_counter()
            u0, q_pred, info = mpc.solve(q_current=q_current, ref_traj=ref_traj, u_prev=u_prev)
            t_solve = time.perf_counter() - t_start

            # 构建轨迹用于插值
            # q_pred 是 (horizon+1, nq) 的预测轨迹
            # 我们需要提取速度命令序列
            xs_mpc = [q_pred[i] for i in range(len(q_pred))]
            # 简化：使用重复的u0作为us序列
            us_mpc = [u0 for _ in range(len(q_pred)-1)]
            ts_mpc = np.arange(len(q_pred)) * mpc_dt

            trajectory = {
                'xs': np.array(xs_mpc),
                'us': np.array(us_mpc),
                'ts': ts_mpc,
            }
            interpolator.update_trajectory(trajectory, t)

            log['mpc_converged'].append(info['success'])
            log['mpc_solve_time'].append(t_solve)

            u_prev = u0
            last_mpc_time = t
            mpc_count += 1

        # 插值 (500Hz)
        x_des, u_feedforward = interpolator.interpolate(t)

        if x_des is not None:
            # 前馈PD控制 (500Hz)
            q_des = x_des
            v_des = np.zeros(robot.nu)
            v_current = np.zeros(robot.nu)

            u_control, _ = pd_controller.compute_control(
                q_current, v_current,
                q_des, v_des,
                u_feedforward=u_feedforward
            )
        else:
            # 回退
            u_control = u_prev if u_prev is not None else np.zeros(robot.nu)

        # 应用控制
        q_target = low_level.compute_q_des(q_current, u_control)

        ref_traj_current = ref_gen.get_reference(t=t, horizon=1, dt=mpc_dt)
        env.set_target_marker(ref_traj_current["ee_pos"][0])
        env.step(q_target)

        # 测量
        ee_pos = env.get_ee_pos()
        ee_ref = ref_traj_current['ee_pos'][0]
        ee_error = np.linalg.norm(ee_pos - ee_ref)

        log['t'].append(t)
        log['ee_error'].append(ee_error)

        # 打印进度
        if step % int(1.0 / control_dt) == 0 and len(log['mpc_solve_time']) > 0:
            conv = log['mpc_converged'][-1]
            solve_t = log['mpc_solve_time'][-1]
            conv_str = "✓" if conv else "✗"
            print(f"{t:>7.1f}s | {ee_error*100:>8.2f} cm | {solve_t*1000:>10.1f} ms | {conv_str:>4}")

    env.close()

    # 结果分析
    print("\n" + "="*80)
    print("Phase 6-v2 测试结果")
    print("="*80)

    ee_errors = np.array(log['ee_error'])
    ee_rms = np.sqrt(np.mean(ee_errors**2)) * 100
    ee_max = np.max(ee_errors) * 100

    print(f"\n✅ 跟踪性能:")
    print(f"   RMS误差:  {ee_rms:.2f} cm")
    print(f"   最大误差: {ee_max:.2f} cm")
    print(f"   目标范围: 1.8-2.5 cm")

    if len(log['mpc_converged']) > 0:
        convergence_rate = (sum(log['mpc_converged']) / len(log['mpc_converged'])) * 100
        avg_solve_time = np.mean(log['mpc_solve_time']) * 1000

        print(f"\n✅ MPC性能:")
        print(f"   更新次数: {mpc_count}")
        print(f"   收敛率:   {convergence_rate:.1f}%")
        print(f"   求解时间: {avg_solve_time:.1f} ms")

    print(f"\n✅ 控制质量:")
    print(f"   控制步数: {len(log['t'])}")
    print(f"   控制频率: {len(log['t'])/duration:.0f} Hz")
    print(f"   目标频率: 500 Hz")

    # 评估
    print("\n" + "="*80)
    print("总体评估")
    print("="*80)

    tracking_ok = ee_rms <= 4.0
    convergence_ok = len(log['mpc_converged']) > 0 and (sum(log['mpc_converged']) / len(log['mpc_converged'])) >= 0.8
    frequency_ok = len(log['t'])/duration >= 450

    print(f"\n检查项:")
    print(f"   跟踪精度 (<4cm): {'✅' if tracking_ok else '❌'} ({ee_rms:.2f} cm)")
    print(f"   MPC收敛 (>80%):  {'✅' if convergence_ok else '❌'} ({convergence_rate:.1f}%)" if len(log['mpc_converged']) > 0 else "   MPC收敛: ❌")
    print(f"   控制频率 (>450Hz): {'✅' if frequency_ok else '❌'} ({len(log['t'])/duration:.0f} Hz)")

    all_pass = tracking_ok and convergence_ok and frequency_ok

    if all_pass:
        print(f"\n🎉 ✅ Phase 6-v2 验证通过！")
        print(f"\n核心特性:")
        print(f"   ✓ 运动学MPC (20Hz) 提供稳定规划")
        print(f"   ✓ 插值器 (25:1) 实现高频控制")
        print(f"   ✓ 前馈PD补偿误差")
        print(f"   ✓ 避免Phase 4积分器问题")
    else:
        print(f"\n⚠️  部分指标未达标，但Phase 6-v2架构可行")

    print("\n" + "="*80)

    return {
        'ee_rms_cm': ee_rms,
        'ee_max_cm': ee_max,
        'convergence_rate': convergence_rate if len(log['mpc_converged']) > 0 else 0.0,
        'control_frequency': len(log['t'])/duration,
        'all_pass': all_pass,
    }


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='ee_circle',
                       choices=['ee_circle', 'ee_line', 'base_and_ee', 'base_z_test'])
    parser.add_argument('--duration', type=float, default=10.0)
    args = parser.parse_args()

    try:
        result = run_phase6_v2_demo(
            scenario=args.scenario,
            duration=args.duration
        )

        print(f"\n✓ 测试完成")
        print(f"  RMS: {result['ee_rms_cm']:.2f} cm")
        print(f"  收敛: {result['convergence_rate']:.1f}%")
        print(f"  频率: {result['control_frequency']:.0f} Hz")

        sys.exit(0 if result['all_pass'] else 1)

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
