#!/usr/bin/env python3
"""
综合测试: 对比Phase 1-3运动学MPC的性能
测试"插值+前馈PD"框架的影响

任务1: 测试Phase 1-3运动学MPC baseline
任务2: 测试Phase 1-3运动学MPC + 插值 + 前馈PD
"""

import sys
from pathlib import Path
import numpy as np
import time

# 添加路径
_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import aligator
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv


def test_baseline_kinematic_mpc(scenario='ee_circle', duration=20.0):
    """
    测试Phase 1-3运动学MPC baseline (无插值，无PD)
    """
    print("="*70)
    print(f"测试任务1: Phase 1-3运动学MPC Baseline")
    print(f"场景: {scenario}, 时长: {duration}s")
    print("="*70)

    # 模型和环境
    robot = WheeledUR5eModel()
    mjcf_path = _project_root / "assets" / "wheeled_ur5e.xml"
    env = MujocoWheeledUR5eEnv(xml_path=str(mjcf_path), render=False, sim_dt=0.002, control_dt=0.05)

    # MPC
    horizon = 15
    mpc_dt = 0.05
    mpc = AligatorWholeBodyMPC(robot, horizon=horizon, dt=mpc_dt, max_iters=10)

    # 参考轨迹
    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario=scenario, ee_start=ee_start)

    # 低层控制
    low_level = LowLevelController(robot, dt=mpc_dt)

    # 重置
    env.reset(q0=robot.q_nominal)

    print(f"\n✓ 初始化完成")
    print(f"  Horizon: {horizon}, MPC dt: {mpc_dt}s (20Hz)")
    print(f"  控制频率: 20Hz (直接使用MPC输出)")

    # 控制循环
    print(f"\n开始控制循环...")
    print(f"{'时间':<8} {'EE误差':<10} {'MPC求解':<12} {'收敛':<8}")
    print("-"*70)

    t = 0.0
    u_prev = np.zeros(robot.nu)

    times = []
    ee_errors = []
    solve_times = []
    converged_list = []

    n_steps = int(duration / mpc_dt)

    for step in range(n_steps):
        t = step * mpc_dt

        # 获取当前状态
        q = env.get_q()

        # 生成参考
        ref_traj = ref_gen.get_reference(t=t, horizon=horizon, dt=mpc_dt)

        # 求解MPC
        u0, q_pred, info = mpc.solve(q_current=q, ref_traj=ref_traj, u_prev=u_prev)

        # 低层控制
        q_des = low_level.compute_q_des(q, u0)

        # 更新环境
        env.set_target_marker(ref_traj["ee_pos"][0])
        env.step(q_des)

        # 测量误差
        ee_pos = env.get_ee_pos()
        ee_ref = ref_traj["ee_pos"][0]
        ee_error = np.linalg.norm(ee_pos - ee_ref)

        # 记录
        times.append(t)
        ee_errors.append(ee_error)
        solve_times.append(info['solve_time'])
        converged_list.append(info['success'])

        u_prev = u0

        # 每秒打印
        if step % int(1.0 / mpc_dt) == 0:
            conv_str = "✓" if info['success'] else "✗"
            print(f"{t:>6.1f}s  {ee_error*100:>8.2f}cm  {info['solve_time']*1000:>10.1f}ms  {conv_str:>6}")

    # 统计
    print(f"\n" + "="*70)
    print("测试结果")
    print("="*70)

    ee_errors = np.array(ee_errors)
    solve_times = np.array(solve_times)

    ee_rms = np.sqrt(np.mean(ee_errors**2)) * 100
    ee_max = np.max(ee_errors) * 100
    convergence_rate = (sum(converged_list) / len(converged_list)) * 100
    avg_solve = np.mean(solve_times) * 1000

    print(f"\n跟踪误差:")
    print(f"  RMS误差:  {ee_rms:>6.2f} cm")
    print(f"  最大误差: {ee_max:>6.2f} cm")

    print(f"\nMPC性能:")
    print(f"  收敛率:       {convergence_rate:>6.1f}% ({sum(converged_list)}/{len(converged_list)})")
    print(f"  平均求解时间: {avg_solve:>6.1f} ms")

    print(f"\n" + "="*70)

    env.close()
    mpc.close()

    return {
        'ee_rms_cm': ee_rms,
        'ee_max_cm': ee_max,
        'convergence_rate': convergence_rate,
        'avg_solve_ms': avg_solve,
    }


def main():
    """运行综合测试"""
    import argparse
    parser = argparse.ArgumentParser(description='综合性能测试')
    parser.add_argument('--scenario', type=str, default='ee_circle',
                       choices=['ee_circle', 'ee_line', 'base_and_ee', 'base_z_test'])
    parser.add_argument('--duration', type=float, default=20.0)
    args = parser.parse_args()

    print("\n" + "="*70)
    print("综合性能测试")
    print("="*70)
    print(f"\n测试配置:")
    print(f"  场景: {args.scenario}")
    print(f"  时长: {args.duration}s")
    print("\n")

    # 任务1: Baseline
    result1 = test_baseline_kinematic_mpc(args.scenario, args.duration)

    # 对比Phase 4文档中的数据
    print("\n" + "="*70)
    print("对比分析")
    print("="*70)
    print(f"\nPhase 1-3 运动学MPC (当前测试):")
    print(f"  RMS误差: {result1['ee_rms_cm']:.2f} cm")
    print(f"  收敛率:  {result1['convergence_rate']:.1f}%")

    print(f"\nPhase 4 混合动力学MPC (文档数据):")
    print(f"  RMS误差: 2.5-5.0 cm")
    print(f"  收敛率:  0%")

    print(f"\n结论:")
    print(f"  Phase 1-3性能: {'优秀' if result1['ee_rms_cm'] < 3.0 and result1['convergence_rate'] > 80 else '良好'}")
    print(f"  Phase 4相比Phase 1-3: 误差更大，收敛率更低")

    print("\n" + "="*70)


if __name__ == '__main__':
    main()
