"""
Dual-arm MPC demo: Independent circular trajectories.

Left arm traces a circle in the XZ plane.
Right arm traces a circle in the YZ plane.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco
import mujoco.viewer

try:
    import aligator
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]
    import aligator

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.dual_arm_aligator_problem import DualArmAligatorProblem


def generate_circle_trajectory(center, radius, plane, N, T):
    """
    Generate circular trajectory.

    Args:
        center: Circle center (3,)
        radius: Circle radius
        plane: 'xy', 'xz', or 'yz'
        N: Number of points
        T: Total time period

    Returns:
        traj: (N, 3) trajectory
    """
    t = np.linspace(0, T, N)
    theta = 2 * np.pi * t / T

    traj = np.zeros((N, 3))

    if plane == 'xy':
        traj[:, 0] = center[0] + radius * np.cos(theta)
        traj[:, 1] = center[1] + radius * np.sin(theta)
        traj[:, 2] = center[2]
    elif plane == 'xz':
        traj[:, 0] = center[0] + radius * np.cos(theta)
        traj[:, 1] = center[1]
        traj[:, 2] = center[2] + radius * np.sin(theta)
    elif plane == 'yz':
        traj[:, 0] = center[0]
        traj[:, 1] = center[1] + radius * np.cos(theta)
        traj[:, 2] = center[2] + radius * np.sin(theta)
    else:
        raise ValueError(f"Unknown plane: {plane}")

    return traj


def main():
    print("\n" + "="*60)
    print("双臂MPC演示：独立圆形轨迹跟踪")
    print("="*60 + "\n")

    # Models
    pin_model = DualArmPinocchioModel()
    mpc_builder = DualArmAligatorProblem(horizon=20, dt=0.05)

    mjcf_path = Path(__file__).resolve().parents[1] / "assets" / "wheeled_dual_ur5e_v2.xml"
    mj_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    mj_data = mujoco.MjData(mj_model)

    # Initial configuration
    q0 = pin_model.get_q_nominal()
    p_left_0, p_right_0 = pin_model.fk_left_ee(q0), pin_model.fk_right_ee(q0)

    print(f"初始位置:")
    print(f"  Left EE:  {p_left_0}")
    print(f"  Right EE: {p_right_0}\n")

    # Reference trajectories
    N_horizon = mpc_builder.horizon
    T_horizon = N_horizon * mpc_builder.dt

    # Left arm: circle in XZ plane (vertical circle)
    left_center = p_left_0 + np.array([0.0, 0.0, 0.1])
    p_left_ref = generate_circle_trajectory(left_center, 0.08, 'xz', N_horizon+1, T_horizon)

    # Right arm: circle in YZ plane (side circle)
    right_center = p_right_0 + np.array([0.0, 0.0, 0.1])
    p_right_ref = generate_circle_trajectory(right_center, 0.08, 'yz', N_horizon+1, T_horizon)

    print("轨迹参数:")
    print(f"  Horizon: {N_horizon} steps ({T_horizon:.2f}s)")
    print(f"  Left:  XZ平面圆 (中心={left_center}, 半径=0.08m)")
    print(f"  Right: YZ平面圆 (中心={right_center}, 半径=0.08m)\n")

    # Build MPC problem
    print("构建MPC问题...")
    problem = mpc_builder.build(q0, p_left_ref, p_right_ref)

    # Solver
    solver = aligator.SolverProxDDP(tol=1e-2, mu_init=1e-1, max_iters=50)
    solver.verbose = aligator.VerboseLevel.QUIET

    # Initial guess (stay at q0)
    xs_init = [q0.copy() for _ in range(N_horizon + 1)]
    us_init = [np.zeros(16) for _ in range(N_horizon)]

    print("求解初始MPC问题...")
    solver.setup(problem)
    conv = solver.run(problem, xs_init, us_init)

    if conv:
        print(f"✓ 收敛！迭代次数: {solver.results.num_iters}, Cost: {solver.results.traj_cost:.2f}\n")
    else:
        print(f"⚠ 未完全收敛，继续执行...\n")

    # Closed-loop simulation
    print("开始闭环MPC仿真...\n")
    print(f"{'Time(s)':>8} | {'Left Error(cm)':>14} | {'Right Error(cm)':>15} | {'Iters':>6} | {'Cost':>8}")
    print("-" * 75)

    q_current = q0.copy()
    u_prev = np.zeros(16)
    t_sim = 0.0
    dt_mpc = mpc_builder.dt
    max_time = 5.0

    left_errors = []
    right_errors = []

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        while viewer.is_running() and t_sim < max_time:
            # Shift reference trajectory (simple receding horizon)
            shift = min(1, int(t_sim / dt_mpc))
            p_left_current = p_left_ref[shift:shift + N_horizon + 1]
            p_right_current = p_right_ref[shift:shift + N_horizon + 1]

            # Pad if needed
            if len(p_left_current) < N_horizon + 1:
                p_left_current = np.vstack([p_left_current, np.tile(p_left_current[-1], (N_horizon + 1 - len(p_left_current), 1))])
                p_right_current = np.vstack([p_right_current, np.tile(p_right_current[-1], (N_horizon + 1 - len(p_right_current), 1))])

            # Update problem
            problem = mpc_builder.build(q_current, p_left_current, p_right_current, u_prev_traj=np.tile(u_prev, (N_horizon, 1)))

            # Solve MPC
            solver.setup(problem)
            conv = solver.run(problem, xs_init, us_init)

            # Extract control
            u_opt = solver.results.us[0]

            # Apply control (kinematic integration)
            q_next = q_current + dt_mpc * u_opt
            q_next[2] = np.arctan2(np.sin(q_next[2]), np.cos(q_next[2]))  # Wrap yaw

            # Compute tracking error
            p_left_actual = pin_model.fk_left_ee(q_next)
            p_right_actual = pin_model.fk_right_ee(q_next)
            err_left = np.linalg.norm(p_left_actual - p_left_current[0]) * 100  # cm
            err_right = np.linalg.norm(p_right_actual - p_right_current[0]) * 100  # cm

            left_errors.append(err_left)
            right_errors.append(err_right)

            # Logging
            if int(t_sim / dt_mpc) % 5 == 0:
                cost = solver.results.traj_cost
                iters = solver.results.num_iters
                print(f"{t_sim:8.2f} | {err_left:14.2f} | {err_right:15.2f} | {iters:6d} | {cost:8.1f}")

            # Update MuJoCo simulation (use actuators to drive the robot)
            # V2 model uses position actuators, send q_next as control target
            mj_data.ctrl[:] = q_next  # Position actuators track this target

            # Run multiple physics steps for smooth motion
            steps_per_mpc = int(dt_mpc / mj_model.opt.timestep)  # 0.05 / 0.002 = 25 steps
            for _ in range(steps_per_mpc):
                mujoco.mj_step(mj_model, mj_data)

            viewer.sync()

            # Prepare next iteration
            q_current = q_next
            u_prev = u_opt
            # Warmstart: shift trajectory
            xs_init = [solver.results.xs[i] for i in range(1, N_horizon+1)] + [solver.results.xs[-1]]
            us_init = [solver.results.us[i] for i in range(1, N_horizon)] + [solver.results.us[-1]]
            t_sim += dt_mpc

    # Statistics
    print("\n" + "="*60)
    print("统计结果:")
    print(f"  Left EE 平均误差:  {np.mean(left_errors):.2f} cm")
    print(f"  Left EE 最大误差:  {np.max(left_errors):.2f} cm")
    print(f"  Right EE 平均误差: {np.mean(right_errors):.2f} cm")
    print(f"  Right EE 最大误差: {np.max(right_errors):.2f} cm")
    print(f"  总仿真时间: {t_sim:.2f} s")
    print("="*60 + "\n")

    print("✓ 演示完成！")


if __name__ == "__main__":
    main()
