#!/usr/bin/env python3
"""
Phase 6 简化测试: 插值+前馈PD方案性能测试

测试Phase 6 (运动学MPC + 插值 + 前馈PD) 的跟踪误差和收敛率
"""

import sys
from pathlib import Path
import numpy as np
import time
import mujoco

# 添加路径
_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import aligator
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, FeedforwardPDGains


def test_phase6_performance(scenario='ee_circle', duration=20.0):
    """
    测试Phase 6完整方案

    架构: 运动学MPC (20Hz) → 插值器 (500Hz) → 前馈PD (500Hz) → MuJoCo
    """
    print("="*70)
    print(f"Phase 6 性能测试: 插值+前馈PD方案")
    print(f"场景: {scenario}, 时长: {duration}s")
    print("="*70)

    # 1. 加载MuJoCo模型
    mjcf_path = _project_root / "assets" / "wheeled_ur5e.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    # 设置初始状态
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.2  # base_z
    data.qpos[6] = np.pi  # shoulder_pan
    data.qpos[7] = np.pi / 3  # shoulder_lift
    data.qpos[8] = -np.pi / 2  # elbow

    mujoco.mj_forward(model, data)

    print(f"\n✓ MuJoCo模型加载完成")
    print(f"  nq = {model.nq}, nu = {model.nu}")

    # 2. 创建运动学MPC (Phase 1-3)
    robot = WheeledUR5eModel()
    horizon = 15
    mpc_dt = 0.05
    builder = KinematicWheeledUR5eProblemBuilder(robot, horizon=horizon, dt=mpc_dt)

    # 创建求解器
    solver = aligator.SolverProxDDP(
        tol=1e-4,
        mu_init=1e-4,
        max_iters=10,
        verbose=aligator.VerboseLevel.QUIET
    )
    solver.rollout_type = aligator.ROLLOUT_LINEAR

    print(f"\n✓ MPC控制器创建完成")
    print(f"  Horizon: {horizon}")
    print(f"  MPC dt: {mpc_dt}s (20Hz)")

    # 3. 创建插值器
    control_dt = model.opt.timestep
    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)

    print(f"\n✓ 插值器创建完成")
    print(f"  控制频率: {1/control_dt:.0f} Hz")
    print(f"  插值比例: {interpolator.ratio}:1")

    # 4. 创建前馈PD控制器
    pd_gains = FeedforwardPDGains(
        Kp_base_xy=50.0, Kd_base_xy=10.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=500.0, Kd_arm=50.0
    )
    pd_controller = FeedforwardPDController(pd_gains)

    print(f"\n✓ 前馈PD控制器创建完成")
    print(f"  基座XY: Kp={pd_gains.Kp_base[0]:.0f}, Kd={pd_gains.Kd_base[0]:.0f}")
    print(f"  机械臂: Kp={pd_gains.Kp_arm[0]:.0f}, Kd={pd_gains.Kd_arm[0]:.0f}")

    # 5. 创建参考轨迹生成器
    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario=scenario, ee_start=ee_start)

    print(f"\n✓ 参考轨迹生成器创建完成")
    print(f"  场景: {scenario}")
    print(f"  EE起点: {ee_start}")

    # 6. 获取关节和执行器索引
    joint_ids = []
    for name in robot.q_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joint_ids.append(jid)

    actuator_ids = []
    for name in robot.u_names:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        actuator_ids.append(aid)

    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")

    # 7. 控制循环
    print(f"\n开始控制循环...")
    print(f"{'时间':<8} {'EE误差':<10} {'MPC求解':<12} {'收敛':<8} {'控制频率':<12}")
    print("-"*70)

    t = 0.0
    sim_dt = control_dt
    n_steps = int(duration / sim_dt)
    mujoco_substeps = 1

    # 初始化
    q_current = np.array([data.qpos[jid] for jid in joint_ids])
    v_current = np.zeros(10)

    # Warm start
    xs_prev = None
    us_prev = None
    last_mpc_time = -np.inf

    # 记录数据
    times = []
    ee_errors = []
    ee_positions = []
    controls = []
    mpc_converged = []
    mpc_solve_times = []

    for step in range(n_steps):
        t = step * sim_dt

        # 获取当前状态
        q_current = np.array([data.qpos[jid] for jid in joint_ids])
        x_current = q_current

        # 更新MPC (20Hz)
        if t - last_mpc_time >= mpc_dt:
            # 生成参考轨迹
            ref_traj = ref_gen.get_reference(t, horizon, mpc_dt)

            # 构建问题
            problem, _ = builder.build_problem(x_current, ref_traj)
            solver.setup(problem)

            # Warm start
            if xs_prev is None:
                xs_init = [x_current] * (horizon + 1)
                us_init = [np.zeros(10)] * horizon
            else:
                xs_init = list(xs_prev[1:]) + [xs_prev[-1]]
                us_init = list(us_prev[1:]) + [us_prev[-1]]

            # 求解
            t_solve_start = time.perf_counter()
            converged = solver.run(problem, xs_init, us_init)
            t_solve = time.perf_counter() - t_solve_start

            # 提取结果
            xs_prev = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
            us_prev = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]

            # 更新插值器
            ts = np.arange(len(xs_prev)) * mpc_dt
            trajectory = {
                'xs': np.array(xs_prev),
                'us': np.array(us_prev),
                'ts': ts,
            }
            interpolator.update_trajectory(trajectory, t)

            # 记录
            mpc_converged.append(converged)
            mpc_solve_times.append(t_solve)
            last_mpc_time = t

        # 插值获取期望
        x_des, u_feedforward = interpolator.interpolate(t)

        if x_des is not None:
            # 前馈PD控制
            q_des = x_des
            v_des = np.zeros(10)
            u_control, _ = pd_controller.compute_control(
                q_current, v_current, q_des, v_des, u_feedforward
            )
        else:
            u_control = np.zeros(10)

        # 应用控制
        for i, aid in enumerate(actuator_ids):
            data.ctrl[aid] = u_control[i]

        # 仿真步进
        for _ in range(mujoco_substeps):
            mujoco.mj_step(model, data)

        # 计算EE误差
        ee_pos = data.site_xpos[ee_site_id].copy()
        ref_traj_current = ref_gen.get_reference(t, 1, mpc_dt)
        ee_target = ref_traj_current['ee_pos'][0]
        ee_error = np.linalg.norm(ee_pos - ee_target)

        # 记录
        times.append(t)
        ee_errors.append(ee_error)
        ee_positions.append(ee_pos)
        controls.append(u_control.copy())

        # 每秒打印一次
        if step % int(1.0 / sim_dt) == 0:
            conv_str = "✓" if (len(mpc_converged) > 0 and mpc_converged[-1]) else "✗"
            solve_time_ms = mpc_solve_times[-1] * 1000 if len(mpc_solve_times) > 0 else 0
            print(f"{t:>6.1f}s  {ee_error*100:>8.2f}cm  {solve_time_ms:>10.1f}ms  {conv_str:>6}  {1/sim_dt:>10.0f}Hz")

    # 8. 统计结果
    print(f"\n" + "="*70)
    print("Phase 6 测试完成")
    print("="*70)

    ee_errors = np.array(ee_errors)
    controls = np.array(controls)

    # 跟踪误差统计
    ee_rms = np.sqrt(np.mean(ee_errors**2)) * 100
    ee_max = np.max(ee_errors) * 100
    ee_mean = np.mean(ee_errors) * 100

    print(f"\n跟踪误差 (EE):")
    print(f"  RMS误差:  {ee_rms:>6.2f} cm")
    print(f"  最大误差: {ee_max:>6.2f} cm")
    print(f"  平均误差: {ee_mean:>6.2f} cm")

    # MPC收敛率
    convergence_rate = (sum(mpc_converged) / len(mpc_converged)) * 100 if len(mpc_converged) > 0 else 0
    avg_solve_time = np.mean(mpc_solve_times) * 1000 if len(mpc_solve_times) > 0 else 0
    max_solve_time = np.max(mpc_solve_times) * 1000 if len(mpc_solve_times) > 0 else 0

    print(f"\nMPC性能:")
    print(f"  收敛率:       {convergence_rate:>6.1f}% ({sum(mpc_converged)}/{len(mpc_converged)})")
    print(f"  平均求解时间: {avg_solve_time:>6.1f} ms")
    print(f"  最大求解时间: {max_solve_time:>6.1f} ms")

    # 控制平滑度
    control_diff = np.diff(controls[:, 0])  # 只看第一个维度
    control_smoothness = np.std(control_diff)

    print(f"\n控制质量:")
    print(f"  控制步数:   {len(controls)}")
    print(f"  控制频率:   {len(controls)/duration:.0f} Hz")
    print(f"  控制范围:   [{controls.min():.3f}, {controls.max():.3f}]")
    print(f"  平滑度(std): {control_smoothness:.4f}")

    print(f"\n" + "="*70)

    return {
        'ee_rms_error_cm': ee_rms,
        'ee_max_error_cm': ee_max,
        'convergence_rate_pct': convergence_rate,
        'avg_solve_time_ms': avg_solve_time,
        'control_smoothness': control_smoothness,
    }


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Phase 6 性能测试')
    parser.add_argument('--scenario', type=str, default='ee_circle',
                       choices=['ee_circle', 'ee_line', 'base_and_ee', 'base_z_test'],
                       help='测试场景')
    parser.add_argument('--duration', type=float, default=20.0,
                       help='运行时长(秒)')
    args = parser.parse_args()

    result = test_phase6_performance(
        scenario=args.scenario,
        duration=args.duration
    )
