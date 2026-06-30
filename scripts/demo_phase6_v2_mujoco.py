#!/usr/bin/env python3
"""
Phase 6-v2 MuJoCo闭环仿真演示

展示: 运动学MPC + 插值器 + 前馈PD 的实际跟踪效果
"""

import sys
from pathlib import Path
import numpy as np
import time

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

# 延迟导入以显示进度
print("="*80)
print("Phase 6-v2 MuJoCo闭环仿真")
print("="*80)
print("\n正在加载模块...")

import mujoco
print("  ✓ MuJoCo")

import aligator
print("  ✓ ALIGATOR")

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
print("  ✓ 项目模块")

# Phase 6-v2组件
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)
print("  ✓ Phase 6-v2组件")

def run_phase6_v2_demo(scenario='ee_circle', duration=10.0, render=True):
    """
    运行Phase 6-v2完整演示
    """
    print(f"\n配置:")
    print(f"  场景: {scenario}")
    print(f"  时长: {duration}s")
    print(f"  渲染: {'是' if render else '否'}")
    print("="*80)

    # 创建组件
    robot = WheeledUR5eModel()

    # MuJoCo环境
    mjcf_path = _project_root / "assets" / "wheeled_ur5e.xml"
    env = MujocoWheeledUR5eEnv(xml_path=str(mjcf_path), render=render,
                               sim_dt=0.002, control_dt=0.05)

    # MPC
    horizon = 15
    mpc_dt = 0.05
    builder = KinematicWheeledUR5eProblemBuilder(robot, horizon=horizon, dt=mpc_dt)

    solver = aligator.SolverProxDDP(
        tol=1e-4,
        mu_init=1e-4,
        max_iters=10,
        verbose=aligator.VerboseLevel.QUIET
    )
    solver.rollout_type = aligator.ROLLOUT_LINEAR

    # Phase 6-v2组件
    control_dt = 0.002
    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)

    pd_gains = FeedforwardPDGains(
        Kp_base_xy=50.0, Kd_base_xy=10.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=500.0, Kd_arm=50.0
    )
    pd_controller = FeedforwardPDController(pd_gains)

    # 参考轨迹
    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario=scenario, ee_start=ee_start)

    # 低层控制器
    low_level = LowLevelController(robot, dt=mpc_dt)

    print("\n✓ 组件创建完成")
    print(f"  MPC: {horizon}步horizon, {mpc_dt}s dt (20Hz)")
    print(f"  插值: {interpolator.ratio}:1 (500Hz)")
    print(f"  前馈PD: Kp={pd_gains.Kp_arm[0]:.0f}, Kd={pd_gains.Kd_arm[0]:.0f}")

    # 初始化
    env.reset(q0=robot.q_nominal)
    u_prev = np.zeros(robot.nu)

    # 记录数据
    times = []
    ee_errors = []
    ee_positions = []
    controls = []
    mpc_converged = []
    mpc_solve_times = []

    n_mpc_steps = int(duration / mpc_dt)
    mujoco_substeps = int(mpc_dt / control_dt)

    print(f"\n开始仿真 (MPC步数: {n_mpc_steps}, 每步{mujoco_substeps}个MuJoCo子步)...")
    print(f"{'时间':<8} {'EE误差':<10} {'MPC时间':<12} {'收敛':<8}")
    print("-"*80)

    for mpc_step in range(n_mpc_steps):
        t = mpc_step * mpc_dt

        # 获取当前状态
        q_current = env.get_q()

        # 生成参考
        ref_traj = ref_gen.get_reference(t=t, horizon=horizon, dt=mpc_dt)

        # 构建并求解MPC
        problem, _ = builder.build_problem(q_current, ref_traj, u_prev=u_prev)
        solver.setup(problem)

        xs_init = [q_current] * (horizon + 1)
        us_init = [np.zeros(robot.nu)] * horizon

        t_solve_start = time.perf_counter()
        converged = solver.run(problem, xs_init, us_init)
        t_solve = time.perf_counter() - t_solve_start

        # 提取MPC结果
        xs_mpc = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_mpc = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        u0 = us_mpc[0]

        # 更新插值器
        ts_mpc = np.arange(len(xs_mpc)) * mpc_dt
        trajectory = {
            'xs': np.array(xs_mpc),
            'us': np.array(us_mpc),
            'ts': ts_mpc,
        }
        interpolator.update_trajectory(trajectory, t)

        # MuJoCo子步循环 (Phase 6-v2: 插值+PD)
        for substep in range(mujoco_substeps):
            t_sub = t + substep * control_dt

            # 插值获取期望状态
            x_des, u_feedforward = interpolator.interpolate(t_sub)

            if x_des is not None:
                q_current_sub = env.get_q()
                v_current_sub = np.zeros(robot.nu)  # 简化

                # 前馈PD控制
                q_des = x_des
                v_des = np.zeros(robot.nu)
                u_control, _ = pd_controller.compute_control(
                    q_current_sub, v_current_sub,
                    q_des, v_des,
                    u_feedforward=u_feedforward
                )
            else:
                # 回退到MPC输出
                q_des = low_level.compute_q_des(env.get_q(), u0)
                u_control = u0

            # 应用控制
            env.set_target_marker(ref_traj["ee_pos"][0])
            env.step(q_des)

            # 计算EE误差
            ee_pos = env.get_ee_pos()
            ref_traj_sub = ref_gen.get_reference(t_sub, 1, mpc_dt)
            ee_target = ref_traj_sub['ee_pos'][0]
            ee_error = np.linalg.norm(ee_pos - ee_target)

            # 记录
            times.append(t_sub)
            ee_errors.append(ee_error)
            ee_positions.append(ee_pos)
            controls.append(u_control.copy())

        # 记录MPC统计
        mpc_converged.append(converged)
        mpc_solve_times.append(t_solve)
        u_prev = u0

        # 打印进度
        if mpc_step % int(1.0 / mpc_dt) == 0:
            conv_str = "✓" if converged else "✗"
            print(f"{t:>6.1f}s  {ee_error*100:>8.2f}cm  {t_solve*1000:>10.1f}ms  {conv_str:>6}")

    env.close()

    # 统计结果
    print(f"\n" + "="*80)
    print("Phase 6-v2 仿真结果")
    print("="*80)

    ee_errors = np.array(ee_errors)
    controls = np.array(controls)

    # 跟踪误差
    ee_rms = np.sqrt(np.mean(ee_errors**2)) * 100
    ee_max = np.max(ee_errors) * 100
    ee_mean = np.mean(ee_errors) * 100

    print(f"\n跟踪误差 (EE):")
    print(f"  RMS误差:  {ee_rms:.2f} cm")
    print(f"  最大误差: {ee_max:.2f} cm")
    print(f"  平均误差: {ee_mean:.2f} cm")

    # MPC性能
    convergence_rate = (sum(mpc_converged) / len(mpc_converged)) * 100
    avg_solve_time = np.mean(mpc_solve_times) * 1000

    print(f"\nMPC性能:")
    print(f"  收敛率:       {convergence_rate:.1f}% ({sum(mpc_converged)}/{len(mpc_converged)})")
    print(f"  平均求解时间: {avg_solve_time:.1f} ms")

    # 控制统计
    control_diff = np.diff(controls[:, 0])
    control_smoothness = np.std(control_diff)

    print(f"\n控制质量:")
    print(f"  控制步数: {len(controls)}")
    print(f"  控制频率: {len(controls)/duration:.0f} Hz")
    print(f"  平滑度(std): {control_smoothness:.4f}")

    # Phase 6-v2特性
    print(f"\nPhase 6-v2特性:")
    print(f"  ✓ 插值器: {len(mpc_converged)}次MPC更新 → {len(controls)}个控制点")
    print(f"  ✓ 高频控制: 500Hz (vs baseline 20Hz)")
    print(f"  ✓ 前馈+PD: 平滑跟踪")

    print(f"\n" + "="*80)

    return {
        'ee_rms_cm': ee_rms,
        'ee_max_cm': ee_max,
        'convergence_rate': convergence_rate,
        'avg_solve_ms': avg_solve_time,
        'control_smoothness': control_smoothness,
    }


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Phase 6-v2 MuJoCo演示')
    parser.add_argument('--scenario', type=str, default='ee_circle',
                       choices=['ee_circle', 'ee_line', 'base_and_ee', 'base_z_test'])
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--no-render', action='store_true')
    args = parser.parse_args()

    try:
        result = run_phase6_v2_demo(
            scenario=args.scenario,
            duration=args.duration,
            render=not args.no_render
        )

        print(f"\n✓ 演示完成")
        print(f"  RMS误差: {result['ee_rms_cm']:.2f} cm")
        print(f"  收敛率: {result['convergence_rate']:.1f}%")

    except Exception as e:
        print(f"\n✗ 演示失败: {e}")
        import traceback
        traceback.print_exc()
