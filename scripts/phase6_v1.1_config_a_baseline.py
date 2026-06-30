#!/usr/bin/env python3
"""
Phase 6-v1.1 配置A: Phase 4原版（Baseline）

测试Phase 4原始实现的性能作为对比基准
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
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridAligatorProblem
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator

print("="*80)
print("Phase 6-v1.1 配置A: Phase 4原版（Baseline）")
print("="*80)
print("架构: Hybrid Dynamics MPC (20Hz) → Torque直接执行 (20Hz)")
print("="*80)

def run_phase4_original(duration=10.0):
    robot = WheeledUR5eModel()
    xml_path = _project_root / "assets" / "wheeled_ur5e_hybrid.xml"

    # Phase 4原始配置
    mpc_dt = 0.05  # 20 Hz
    control_dt = 0.05  # 20 Hz (直接执行)

    env = MujocoWheeledUR5eHybridEnv(
        xml_path=str(xml_path),
        render=False,
        sim_dt=0.002,
        control_dt=control_dt
    )

    # MPC配置
    mpc = HybridAligatorProblem(
        robot=robot,
        horizon=15,
        dt=mpc_dt,
        max_iters=20
    )

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)

    env.reset(q0=robot.q_nominal)

    # 数据记录
    times = []
    errors = []
    convergence_count = 0
    total_mpc_calls = 0

    x_prev = np.concatenate([robot.q_nominal, np.zeros(robot.nv)])
    u_prev = np.zeros(robot.nu)

    n_steps = int(duration / control_dt)

    print(f"\n运行{duration}秒测试...")
    print(f"{'时间':>8s} | {'EE误差':>10s} | {'MPC求解':>12s} | {'收敛':>6s}")
    print("-" * 60)

    for step in range(n_steps):
        t = step * control_dt

        # 获取当前状态
        q_current = env.get_q()
        v_current = np.zeros(robot.nv)  # 简化：假设速度为0
        x_current = np.concatenate([q_current, v_current])

        # MPC求解
        ref_traj = ref_gen.get_reference(t=t, horizon=mpc.horizon, dt=mpc.dt)

        solve_start = time.time()
        try:
            u_mpc, x_pred, info = mpc.solve(
                x_current=x_current,
                ref_traj=ref_traj,
                x_prev=x_prev,
                u_prev=u_prev
            )
            solve_time = (time.time() - solve_start) * 1000

            converged = info.get('converged', False)
            if converged:
                convergence_count += 1

            total_mpc_calls += 1

            # 直接使用MPC输出的torque
            env.set_torque(u_mpc)

            x_prev = x_pred[1] if len(x_pred) > 1 else x_current
            u_prev = u_mpc

        except Exception as e:
            print(f"  MPC失败 @ t={t:.2f}s: {e}")
            env.set_torque(np.zeros(robot.nu))
            solve_time = 0
            converged = False

        # 仿真步进
        env.step()

        # 计算误差
        ee_pos = env.get_ee_pos()
        ref_traj_current = ref_gen.get_reference(t=t, horizon=1, dt=mpc.dt)
        ee_ref = ref_traj_current['ee_pos'][0]
        ee_error = np.linalg.norm(ee_pos - ee_ref) * 100

        times.append(t)
        errors.append(ee_error)

        # 打印进度
        if step % 10 == 0:
            status = "✓" if converged else "✗"
            print(f"{t:>7.1f}s | {ee_error:>8.2f} cm | {solve_time:>9.1f} ms | {status:>6s}")

    env.close()

    # 统计
    times = np.array(times)
    errors = np.array(errors)
    rms_error = np.sqrt(np.mean(errors**2))
    convergence_rate = convergence_count / max(total_mpc_calls, 1) * 100

    print()
    print("="*80)
    print("Phase 4原版结果")
    print("="*80)
    print(f"RMS误差:    {rms_error:.2f} cm")
    print(f"最大误差:   {np.max(errors):.2f} cm")
    print(f"MPC收敛率: {convergence_rate:.1f}% ({convergence_count}/{total_mpc_calls})")
    print("="*80)

    return {
        'rms': rms_error,
        'max': np.max(errors),
        'convergence': convergence_rate,
        'times': times,
        'errors': errors
    }

if __name__ == "__main__":
    try:
        result = run_phase4_original(duration=10.0)
    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
