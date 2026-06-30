#!/usr/bin/env python3
"""
Phase 4 混合 MPC 圆形轨迹演示

展示混合动力学MPC跟踪圆形轨迹的效果
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]

import time
import numpy as np
import aligator

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder


def generate_circle_trajectory(t_steps, dt, horizon, pin_robot, robot, radius=0.1, height=0.5, period=10.0):
    """
    生成圆形轨迹参考

    Parameters
    ----------
    t_steps : int
        总步数
    dt : float
        时间步长
    horizon : int
        预测视野
    pin_robot : PinocchioWheeledUR5eModel
    robot : WheeledUR5eModel
    radius : float
        圆形半径 (m)
    height : float
        圆心高度 (m)
    period : float
        圆周期 (s)
    """
    num_waypoints = t_steps + horizon + 1

    # 圆形轨迹参数
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)
    center_y = p_ee_nominal[1]
    center_z = height

    omega = 2 * np.pi / period

    ee_pos = np.zeros((num_waypoints, 3))
    ee_rot = np.zeros((num_waypoints, 3, 3))

    for i in range(num_waypoints):
        t = i * dt
        angle = omega * t

        # 圆形轨迹
        ee_pos[i, 0] = p_ee_nominal[0]  # X保持不变
        ee_pos[i, 1] = center_y + radius * np.cos(angle)
        ee_pos[i, 2] = center_z + radius * np.sin(angle)

        # 姿态保持不变
        ee_rot[i] = R_ee_nominal

    return {
        "ee_pos": ee_pos,
        "ee_rot": ee_rot,
        "base": np.zeros((num_waypoints, 3)),
        "base_z": np.full(num_waypoints, 0.2),
    }


def demo_phase4_circle(duration=20.0, radius=0.1, period=10.0, render=True):
    """
    Run Phase 4 hybrid MPC with circular trajectory.
    """

    print("\n" + "="*70)
    print("PHASE 4 混合 MPC - 圆形轨迹演示")
    print("="*70)
    print("\n配置：")
    print(f"  duration = {duration} s")
    print(f"  radius = {radius} m")
    print(f"  period = {period} s")
    print("  horizon = 10")
    print("  max_iters = 50")
    print("  dt = 0.05 s")
    print("\n目标：末端执行器画圆（Y-Z平面）")
    print("="*70 + "\n")

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=render)

    horizon = 10
    dt = 0.05
    max_iters = 50

    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        use_hard_state_bounds=False,
    )

    num_steps = int(duration / dt)
    ref_traj = generate_circle_trajectory(num_steps, dt, horizon, pin_robot, robot,
                                          radius=radius, period=period)

    solver = aligator.SolverProxDDP(1e-2, mu_init=1e-1, max_iters=max_iters)

    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    env.reset(robot.q_nominal)

    # Set initial target marker to match EE position
    env.set_target_marker(ref_traj["ee_pos"][0])

    xs_prev = [x0.copy() for _ in range(horizon + 1)]
    us_prev = [np.zeros(10) for _ in range(horizon)]

    mujoco_substeps = int(dt / env.model.opt.timestep)

    # Metrics
    converged_count = 0
    solve_times = []
    ee_errors = []
    tau_maxs = []

    print("运行中...\n")
    print(f"{'时间':<8} {'EE误差':<10} {'求解时间':<12} {'扭矩峰值':<12} {'收敛':<8}")
    print("-"*70)

    for step in range(num_steps):
        t = step * dt
        x_current = env.get_state()

        problem, _ = builder.build_problem(x_current, ref_traj, u_prev=us_prev[0] if step > 0 else None)

        solver.setup(problem)
        t_start = time.perf_counter()
        solver.run(problem, xs_prev, us_prev)
        t_solve = time.perf_counter() - t_start

        solve_times.append(t_solve)

        if solver.results.conv:
            converged_count += 1

        u0 = np.array(solver.results.us[0])
        tau_arm = u0[4:10]
        tau_max = np.max(np.abs(tau_arm))
        tau_maxs.append(tau_max)

        env.set_control(u0)
        env.set_target_marker(ref_traj["ee_pos"][step])
        env.step(mujoco_substeps)

        ee_pos = env.get_ee_pos()
        ee_err = np.linalg.norm(ee_pos - ref_traj["ee_pos"][step])
        ee_errors.append(ee_err)

        xs_sol = [np.array(solver.results.xs[i]) for i in range(len(solver.results.xs))]
        us_sol = [np.array(solver.results.us[i]) for i in range(len(solver.results.us))]
        xs_prev = xs_sol[1:] + [xs_sol[-1]]
        us_prev = us_sol[1:] + [us_sol[-1]]

        if (step + 1) % 20 == 0 or step == 0:
            conv_str = "✓" if solver.results.conv else "✗"
            print(f"{t:>6.1f}s  {ee_err*100:>8.2f}cm  {t_solve*1000:>10.1f}ms  {tau_max:>10.2f}Nm  {conv_str}")

    env.close()

    # Summary
    print("\n" + "="*70)
    print("总结")
    print("="*70)

    success_rate = (converged_count / num_steps) * 100
    ee_rms = np.sqrt(np.mean(np.array(ee_errors)**2)) * 100
    avg_solve = np.mean(solve_times) * 1000
    peak_tau = np.max(tau_maxs)

    print(f"\n求解器统计：")
    print(f"  收敛率:       {success_rate:>6.1f}% ({converged_count}/{num_steps})")
    print(f"  平均求解时间: {avg_solve:>6.1f} ms")

    print(f"\n控制质量：")
    print(f"  EE RMS 误差:  {ee_rms:>6.2f} cm")
    print(f"  EE Max 误差:  {np.max(ee_errors)*100:>6.2f} cm")

    print(f"\n扭矩统计：")
    print(f"  最大峰值:     {peak_tau:>6.2f} Nm")

    print("\n" + "="*70)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0, help="Test duration (s)")
    parser.add_argument("--radius", type=float, default=0.1, help="Circle radius (m)")
    parser.add_argument("--period", type=float, default=10.0, help="Circle period (s)")
    parser.add_argument("--no-render", action="store_true", help="Disable rendering")
    args = parser.parse_args()

    demo_phase4_circle(duration=args.duration, radius=args.radius,
                       period=args.period, render=not args.no_render)
