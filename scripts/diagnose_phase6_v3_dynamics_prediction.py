#!/usr/bin/env python3
"""Compare Pinocchio/ALIGATOR forward dynamics vs MuJoCo for Phase 6-v3."""

import sys

sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import mujoco
import numpy as np
import pinocchio as pin

from wheeled_ur5e_aligator_mpc.coordinate_mapping import DUAL_ARM_Q_NOMINAL, q_to_ctrl
from wheeled_ur5e_aligator_mpc.dual_arm_dynamics_mpc import DualArmDynamicsMPC
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.phase6_v3_common import (
    CONTROL_DT,
    INTERPOLATION_RATIO,
    MJCF_PIN,
    MJCF_TORQUE,
    circle_trajectory,
    generate_ee_reference_trajectory,
)


def pinocchio_step(model, data, q, v, tau, dt):
    pin.computeAllTerms(model, data, q, v)
    a = np.linalg.solve(data.M, tau - data.nle)
    v_next = v + dt * a
    q_next = pin.integrate(model, q, dt * v_next)
    return q_next, v_next


def mujoco_step(mj_model, mj_data, q, v, tau_q, dt):
    mj_data.qpos[:16] = q
    mj_data.qvel[:16] = v
    mujoco.mj_forward(mj_model, mj_data)
    mj_data.ctrl[:16] = q_to_ctrl(tau_q)
    steps = max(1, int(round(dt / mj_model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(mj_model, mj_data)
    return mj_data.qpos[:16].copy(), mj_data.qvel[:16].copy()


def report_step(label, q_err, v_err, pin_model, q_pin, q_mj, target_left, target_right):
    ee_l_pin = np.linalg.norm(pin_model.fk_left_ee(q_pin) - target_left)
    ee_r_pin = np.linalg.norm(pin_model.fk_right_ee(q_pin) - target_right)
    ee_l_mj = np.linalg.norm(pin_model.fk_left_ee(q_mj) - target_left)
    ee_r_mj = np.linalg.norm(pin_model.fk_right_ee(q_mj) - target_right)
    worst_q = int(np.argmax(np.abs(q_pin - q_mj)))
    print(
        f"{label:28s} | q_err={q_err:8.4e} v_err={v_err:8.4e} "
        f"| EE_L pin/mj={ee_l_pin*100:5.2f}/{ee_l_mj*100:5.2f}cm "
        f"EE_R pin/mj={ee_r_pin*100:5.2f}/{ee_r_mj*100:5.2f}cm "
        f"| worst_dof={worst_q} dq={(q_pin-q_mj)[worst_q]:+.4e}"
    )


def test_static_gravity(pin_model, mj_model, mj_data):
    print("\n--- Test 1: Static gravity (tau=0) ---")
    q = DUAL_ARM_Q_NOMINAL.copy()
    v = np.zeros(16)
    for dt in [CONTROL_DT, 0.05]:
        q_pin, v_pin = pinocchio_step(pin_model.model, pin_model.data, q, v, np.zeros(16), dt)
        q_mj, v_mj = mujoco_step(mj_model, mj_data, q, v, np.zeros(16), dt)
        report_step(f"dt={dt:.3f}s tau=0", np.linalg.norm(q_pin - q_mj), np.linalg.norm(v_pin - v_mj),
                    pin_model, q_pin, q_mj, np.zeros(3), np.zeros(3))


def test_random_torque(pin_model, mj_model, mj_data, n_samples=5, seed=0):
    print("\n--- Test 2: Random bounded torque ---")
    rng = np.random.default_rng(seed)
    q = DUAL_ARM_Q_NOMINAL.copy()
    v = np.zeros(16)
    for i in range(n_samples):
        tau = rng.uniform(-20, 20, 16)
        for dt in [CONTROL_DT, 0.05]:
            q_pin, v_pin = pinocchio_step(pin_model.model, pin_model.data, q, v, tau, dt)
            q_mj, v_mj = mujoco_step(mj_model, mj_data, q, v, tau, dt)
            report_step(f"sample{i} dt={dt:.3f}", np.linalg.norm(q_pin - q_mj), np.linalg.norm(v_pin - v_mj),
                        pin_model, q_pin, q_mj, np.zeros(3), np.zeros(3))


def test_mpc_rollout(pin_model, mj_model, mj_data):
    print("\n--- Test 3: MPC first-control rollout (one MPC step) ---")
    mpc = DualArmDynamicsMPC(mjcf_path=MJCF_PIN, horizon=10, dt=0.05)
    q0 = DUAL_ARM_Q_NOMINAL.copy()
    v0 = np.zeros(16)
    x0 = np.concatenate([q0, v0])

    target_left, target_right = circle_trajectory(0.0)
    tl_traj, tr_traj = generate_ee_reference_trajectory(0.0, mpc.horizon, mpc.dt)

    xs, us, results = mpc.solve(x0, tl_traj, tr_traj, max_iters=50)
    tau0 = us[0]

    q_mj, v_mj = q0.copy(), v0.copy()
    for _ in range(INTERPOLATION_RATIO):
        q_mj, v_mj = mujoco_step(mj_model, mj_data, q_mj, v_mj, tau0, CONTROL_DT)

    q_pred, v_pred = xs[1, :16], xs[1, 16:]
    report_step(
        f"MPC conv={results.conv}",
        np.linalg.norm(q_pred - q_mj),
        np.linalg.norm(v_pred - v_mj),
        pin_model, q_pred, q_mj, target_left, target_right,
    )


def main():
    print("=" * 80)
    print("Phase 6-v3 Dynamics Prediction Diagnostic")
    print("=" * 80)

    pin_model = DualArmPinocchioModel(MJCF_PIN)
    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)

    test_static_gravity(pin_model, mj_model, mj_data)
    test_random_torque(pin_model, mj_model, mj_data)
    test_mpc_rollout(pin_model, mj_model, mj_data)

    print("\n" + "=" * 80)
    print("Interpretation:")
    print("  - If single-step q/v errors are O(1e-4..1e-2), model mismatch is minor.")
    print("  - If MPC rollout diverges while conv=False, problem is bad MPC solution.")
    print("=" * 80)


if __name__ == "__main__":
    main()