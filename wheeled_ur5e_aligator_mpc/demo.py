"""
Main MPC demo loop for wheeled UR5e.
Used by scripts/run_demo.py.
"""

import gc
import time
from pathlib import Path

import numpy as np

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
from wheeled_ur5e_aligator_mpc.logger import MPCLogger


def run_demo(
    *,
    xml_path: str,
    scenario: str = "ee_circle",
    duration: float = 30.0,
    render: bool = False,
    horizon: int = 20,
    mpc_dt: float = 0.05,
    sim_dt: float = 0.002,
    log_dir: str = "logs",
    aligator_max_iters: int = 10,
    weights: dict | None = None,
) -> None:
    """Run the full wheeled UR5e MPC demo."""

    print(f"[Demo] Initializing wheeled UR5e MPC demo")
    print(f"  Scenario: {scenario}")
    print(f"  Duration: {duration} s | Horizon: {horizon} | MPC dt: {mpc_dt} s")
    print(f"  Render: {render} | ALIGATOR max_iters: {aligator_max_iters}")

    robot = WheeledUR5eModel()
    env = MujocoWheeledUR5eEnv(
        xml_path=xml_path, render=render, sim_dt=sim_dt, control_dt=mpc_dt
    )
    mpc = AligatorWholeBodyMPC(robot, horizon=horizon, dt=mpc_dt, weights=weights,
                                max_iters=aligator_max_iters)

    # Compute circle/line starting point from actual FK at nominal posture
    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario=scenario, ee_start=ee_start)
    low_level = LowLevelController(robot, dt=mpc_dt)
    logger = MPCLogger(log_dir=log_dir)

    # Reset to nominal posture
    env.reset(q0=robot.q_nominal)

    u_prev = np.zeros(robot.nu)
    t = 0.0
    step = 0

    print("[Demo] Starting control loop...")
    loop_start = time.perf_counter()

    while t < duration:
        # Check if viewer was closed by user
        if not env.is_viewer_running():
            print("[Demo] Viewer closed. Stopping.")
            break

        # Read current state
        q = env.get_q()

        # Get reference trajectory
        ref_traj = ref_gen.get_reference(t=t, horizon=mpc.horizon, dt=mpc.dt)

        # Solve MPC
        u0, q_pred, info = mpc.solve(q_current=q, ref_traj=ref_traj, u_prev=u_prev)

        # Low-level: integrate velocity to position target
        q_des = low_level.compute_q_des(q, u0)

        # Update target marker to current reference
        env.set_target_marker(ref_traj["ee_pos"][0])

        # Step simulation
        env.step(q_des)

        # Post-step measurements
        ee_pos = env.get_ee_pos()
        ee_ref = ref_traj["ee_pos"][0]

        # Log
        logger.log(
            t=t,
            q=q,
            u=u0,
            q_des=q_des,
            ee_pos=ee_pos,
            ee_ref=ee_ref,
            base=q[:3],
            base_ref=ref_traj["base"][0],
            base_z=float(q[2]),
            base_z_ref=float(ref_traj["base_z"][0]),
            solve_time=info["solve_time"],
            solver_status=info["status"],
            mpc_success=info["success"],
            fallback=info["fallback"],
            aligator_iter=info["iter"],
            aligator_cost=info["cost"],
        )

        u_prev = u0
        t += mpc_dt
        step += 1

        # Print progress every ~5 s
        if step % max(1, int(5.0 / mpc_dt)) == 0:
            ee_err = float(np.linalg.norm(ee_pos - ee_ref))
            print(
                f"  t={t:5.1f}s  ee_err={ee_err*100:5.1f}cm  "
                f"solve={info['solve_time']*1e3:5.1f}ms  "
                f"{'FALLBACK' if info['fallback'] else 'ok'}"
            )

    loop_elapsed = time.perf_counter() - loop_start
    print(f"[Demo] Loop finished. {step} steps in {loop_elapsed:.1f}s")

    # Save logs and generate plots
    npz_path = logger.save("latest.npz")
    plot_paths = logger.plot("latest")

    # Print summary before cleanup (still have access to all objects)
    stats = logger.summary()
    print()
    print("===== Demo Summary =====")
    print(f"Scenario:               {scenario}")
    print(f"Duration:               {t:.1f} s")
    print(f"Solver:                 ALIGATOR / SolverProxDDP")
    print(f"ALIGATOR max iters/MPC: {aligator_max_iters}")
    print(f"MPC success rate:       {stats['mpc_success_rate']:.1f} %")
    print(f"Fallback rate:          {stats['fallback_rate']:.1f} %")
    print(f"Average solve time:     {stats['avg_solve_time_ms']:.1f} ms")
    print(f"Max solve time:         {stats['max_solve_time_ms']:.1f} ms")
    print(f"End-effector RMS error: {stats['ee_rms_error_m']*100:.2f} cm")
    print(f"End-effector max error: {stats['ee_max_error_m']*100:.2f} cm")
    print(f"Joint limit violation:  {stats['joint_limit_violation']}")
    print(f"Logs saved to:          {npz_path}")
    print(f"Figures saved to:")
    for p in plot_paths:
        print(f"  {p}")

    # --- Explicit cleanup to avoid segfault on interpreter shutdown ---
    # C++ objects from ALIGATOR, MuJoCo, and Pinocchio must be destroyed in a
    # controlled order before Python's GC tears them down non-deterministically.
    # Without this, the process often segfaults after main() returns (all work
    # is already done — logs, plots, summary — but the exit code is non-zero).
    env.close()
    mpc.close()
    del mpc
    del env
    del robot
    gc.collect()
