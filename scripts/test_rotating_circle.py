#!/usr/bin/env python3
"""
Test dynamic orientation tracking (Phase 2).

Creates a reference trajectory where the EE rotates while moving in a circle.
This shows the difference between position-only and 6D pose control.
"""

import sys
from pathlib import Path

_repo = Path(__file__).resolve().parents[3]
sys.path[:0] = [
    str(_repo / "build" / "bindings" / "python"),
    str(_repo / "study_example" / "wheeled_ur5e_aligator_mpc"),
]

import gc
import numpy as np
import pinocchio as pin

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env import MujocoWheeledUR5eEnv
from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.low_level_control import LowLevelController
from wheeled_ur5e_aligator_mpc.logger import MPCLogger


def generate_rotating_circle_reference(t, horizon, dt, robot, pin_robot, duration=10.0, rotate_ee=False):
    """
    Generate a rolling-window circle trajectory where EE rotates around Z axis.

    Position: circle in Y-Z plane (if rotate_ee=False) or stationary (if rotate_ee=True)
    Orientation: optionally rotates around Z axis (yaw changes)

    Parameters
    ----------
    t : float
        Current time (s)
    horizon : int
        MPC prediction horizon
    dt : float
        MPC timestep
    duration : float
        Period for one complete circle/rotation
    rotate_ee : bool
        If True, EE orientation rotates while position stays fixed.
        If False, EE traces a circle with constant orientation.
    """
    # Get nominal EE pose
    p_start, R_start = pin_robot.fk_pose(robot.q_nominal)

    # Generate rolling-window trajectory
    ts = t + np.arange(horizon + 1) * dt
    ee_pos = np.zeros((horizon + 1, 3))
    ee_rot = np.zeros((horizon + 1, 3, 3))

    if rotate_ee:
        # Mode: stationary position, rotating orientation
        omega_rot = 2 * np.pi / duration

        for i, ti in enumerate(ts):
            # Position stays at nominal
            ee_pos[i] = p_start

            # Rotate around Z axis
            yaw = omega_rot * ti
            R_z = pin.utils.rpyToMatrix(0, 0, yaw)
            ee_rot[i] = R_z @ R_start
    else:
        # Mode: circle position, constant orientation
        radius = 0.1  # 10 cm
        omega_circle = 2 * np.pi / duration

        # Center placed so circle starts at p_start
        center = p_start.copy()
        center[1] -= radius  # offset in Y so at theta=0, ee is at p_start

        for i, ti in enumerate(ts):
            theta = omega_circle * ti

            # Circle in Y-Z plane
            ee_pos[i] = center + np.array([
                0,
                radius * np.cos(theta),
                radius * np.sin(theta),
            ])

            # Constant orientation
            ee_rot[i] = R_start

    # Base stays at origin
    base = np.zeros((horizon + 1, 3))
    base_z = np.full(horizon + 1, 0.2)

    return {
        "ee_pos": ee_pos,
        "ee_rot": ee_rot,
        "base": base,
        "base_z": base_z,
    }


def run_test(duration=10.0, ee_ori_weight=0.0, rotate_ee=False):
    """
    Run demo with rotating reference.

    Parameters
    ----------
    duration : float
        Test duration (s)
    ee_ori_weight : float
        0.0 = position-only, 50.0 = 6D pose tracking
    rotate_ee : bool
        If True, EE orientation rotates during circle motion
    """
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    # XML path for kinematic MPC
    xml_path = str(Path(__file__).resolve().parents[1] / "assets" / "wheeled_ur5e.xml")
    env = MujocoWheeledUR5eEnv(xml_path, render=True)

    dt = 0.05
    horizon = 20

    # Different periods for different modes
    if rotate_ee:
        period = 30.0  # Slower rotation for orientation tracking (30s for 360°)
    else:
        period = 10.0  # Normal circle period

    # Build MPC with custom weights
    if rotate_ee:
        # Higher orientation weights for rotation tracking
        weights = {
            "ee_ori": ee_ori_weight * 2,  # 100 instead of 50
            "terminal_ee_ori": ee_ori_weight * 4,  # 200
            "ee_pos": 200.0,  # Higher position weight to keep stationary
        }
    else:
        weights = {"ee_ori": ee_ori_weight, "terminal_ee_ori": ee_ori_weight * 2}
    mpc = AligatorWholeBodyMPC(robot, horizon=horizon, dt=dt, weights=weights)
    low_level = LowLevelController(robot, dt)
    logger = MPCLogger()

    # Reset
    env.reset(robot.q_nominal)

    print(f"\n{'='*60}")
    if rotate_ee:
        print(f"ROTATING ORIENTATION TEST")
    else:
        print(f"CIRCLE POSITION TEST")
    print(f"{'='*60}")
    print(f"EE ori weight: {ee_ori_weight}")
    if rotate_ee:
        print(f"Mode: Stationary position, rotating orientation (360°)")
    else:
        print(f"Mode: Circle trajectory (10cm radius), constant orientation")
    print(f"Duration: {duration} s")
    print(f"Period: {period} s")
    print(f"{'='*60}\n")

    # Control loop
    num_steps = int(duration / dt)
    u_prev = np.zeros(robot.nu)

    for step in range(num_steps):
        t = step * dt

        # Read current state
        q = env.get_q()

        # Generate rolling-window reference
        ref_traj = generate_rotating_circle_reference(t, horizon, dt, robot, pin_robot, period, rotate_ee)

        # MPC solve
        u0, q_pred, info = mpc.solve(q_current=q, ref_traj=ref_traj, u_prev=u_prev)

        # Low-level: integrate velocity to position target
        q_des = low_level.compute_q_des(q, u0)

        # Update target marker (show current reference, not future)
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

        # Update u_prev for next iteration
        u_prev = u0

        if (step + 1) % 20 == 0:
            ee_err = np.linalg.norm(ee_pos - ee_ref) * 100
            status = "ok" if info["success"] else "FAIL"
            print(f"  t={t:5.1f}s  ee_err={ee_err:5.1f}cm  {status}")

    # Print summary
    logger.summary()

    # Explicit cleanup to avoid segfault on interpreter shutdown
    env.close()
    mpc.close()
    del mpc
    del env
    del robot
    gc.collect()

    return logger


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--with-orientation", action="store_true",
                        help="Enable orientation tracking (default: position-only)")
    parser.add_argument("--rotate-ee", action="store_true",
                        help="EE rotates during circle motion (very challenging)")
    args = parser.parse_args()

    weight = 50.0 if args.with_orientation else 0.0
    run_test(args.duration, weight, args.rotate_ee)
