#!/usr/bin/env python3
"""Minimal MPC feasibility tests before full circular tracking."""

import sys

sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import mujoco
import numpy as np

from wheeled_ur5e_aligator_mpc.coordinate_mapping import DUAL_ARM_Q_NOMINAL, q_to_ctrl
from wheeled_ur5e_aligator_mpc.dual_arm_dynamics_mpc import DualArmDynamicsMPC
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.phase6_v3_common import (
    CONTROL_DT,
    INTERPOLATION_RATIO,
    MJCF_PIN,
    MJCF_TORQUE,
    circle_trajectory,
    ee_tracking_errors,
)


def rollout_mujoco(mj_model, mj_data, q0, v0, tau_q, n_steps):
    mj_data.qpos[:16] = q0
    mj_data.qvel[:16] = v0
    mujoco.mj_forward(mj_model, mj_data)
    for _ in range(n_steps):
        mj_data.ctrl[:16] = q_to_ctrl(tau_q)
        mujoco.mj_step(mj_model, mj_data)
    return mj_data.qpos[:16].copy(), mj_data.qvel[:16].copy()


def run_case(
    name: str,
    horizon: int,
    target_mode: str,
    pin_model: DualArmPinocchioModel,
    mj_model,
    mj_data,
    max_iters: int = 50,
):
    mpc = DualArmDynamicsMPC(mjcf_path=MJCF_PIN, horizon=horizon, dt=0.05)
    q0 = DUAL_ARM_Q_NOMINAL.copy()
    v0 = np.zeros(16)
    x0 = np.concatenate([q0, v0])

    if target_mode == "fk":
        tl = pin_model.fk_left_ee(q0)
        tr = pin_model.fk_right_ee(q0)
    elif target_mode == "circle_t0":
        tl, tr = circle_trajectory(0.0)
    elif target_mode == "line":
        tl, tr = circle_trajectory(0.0)
        tl = tl + np.array([0.02, 0.0, 0.0])
        tr = tr + np.array([0.02, 0.0, 0.0])
    else:
        raise ValueError(target_mode)

    tl_traj = np.tile(tl, (horizon + 1, 1))
    tr_traj = np.tile(tr, (horizon + 1, 1))

    xs, us, res = mpc.solve(x0, tl_traj, tr_traj, max_iters=max_iters)

    q_pred = xs[-1, :16]
    ee_l_pred, ee_r_pred = ee_tracking_errors(pin_model, q_pred, tl, tr)

    q_mj, _ = rollout_mujoco(mj_model, mj_data, q0, v0, us[0], INTERPOLATION_RATIO)
    ee_l_act, ee_r_act = ee_tracking_errors(pin_model, q_mj, tl, tr)

    print(
        f"{name:32s} | conv={str(res.conv):5s} iters={res.num_iters:2d} cost={res.traj_cost:8.1f} "
        f"| pred EE L/R={ee_l_pred*100:5.2f}/{ee_r_pred*100:5.2f}cm "
        f"| act EE L/R={ee_l_act*100:5.2f}/{ee_r_act*100:5.2f}cm"
    )
    return res.conv


def main():
    print("=" * 90)
    print("Phase 6-v3 MPC Feasibility Tests")
    print("=" * 90)

    pin_model = DualArmPinocchioModel(MJCF_PIN)
    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)

    cases = [
        ("static_fk_h1", 1, "fk"),
        ("static_fk_h3", 3, "fk"),
        ("static_circle_h1", 1, "circle_t0"),
        ("static_circle_h3", 3, "circle_t0"),
        ("static_circle_h10", 10, "circle_t0"),
        ("slow_line_h5", 5, "line"),
    ]

    results = {}
    for name, h, mode in cases:
        results[name] = run_case(name, h, mode, pin_model, mj_model, mj_data)

    print("\n" + "=" * 90)
    print("Summary:")
    conv_rate = sum(results.values()) / len(results)
    print(f"  Convergence rate: {sum(results.values())}/{len(results)} ({100*conv_rate:.0f}%)")
    if not results.get("static_circle_h1", False):
        print("  WARNING: horizon-1 circle target does not converge — pause full tracking.")
    elif not results.get("static_circle_h3", False):
        print("  WARNING: horizon-3 fails — keep MPC research-only.")
    else:
        print("  Simplified static tasks partially converge; full h=10 still likely infeasible.")
    print("=" * 90)


if __name__ == "__main__":
    main()