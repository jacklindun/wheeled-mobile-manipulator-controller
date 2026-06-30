#!/usr/bin/env python3
"""Phase 6-v3 Step 1 headless benchmark: IK → interpolate → gravity FF + PD → MuJoCo."""

import sys
import time

sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import mujoco
import numpy as np

from wheeled_ur5e_aligator_mpc.coordinate_mapping import DUAL_ARM_Q_NOMINAL, q_to_ctrl
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.phase6_v3_common import (
    CONTROL_DT,
    INTERPOLATION_RATIO,
    MJCF_PIN,
    MJCF_TORQUE,
    FixedBaseIKPlanner,
    JointInterpolator,
    circle_trajectory,
    compute_gravity_torque,
    ee_tracking_errors,
    make_pd_controller,
)


def main(duration: float = 10.0, omega: float = 0.5, radius: float = 0.08, use_gravity_ff: bool = True):
    print("=" * 60)
    print("Phase 6-v3 Step 1: Headless Torque PD Benchmark")
    print(f"  gravity feedforward: {use_gravity_ff}")
    print("=" * 60)

    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)
    pin_model = DualArmPinocchioModel(MJCF_PIN)

    ik = FixedBaseIKPlanner(pin_model)
    interpolator = JointInterpolator()
    pd = make_pd_controller()

    mj_data.qpos[:16] = DUAL_ARM_Q_NOMINAL
    mj_data.qvel[:16] = 0.0
    mujoco.mj_forward(mj_model, mj_data)
    interpolator.set_segment(DUAL_ARM_Q_NOMINAL, DUAL_ARM_Q_NOMINAL)

    sim_time = 0.0
    mpc_counter = 0
    errors_left, errors_right = [], []
    ik_residuals = []
    saturation_count = 0
    total_steps = 0
    t_wall_start = time.time()

    while sim_time < duration:
        target_left, target_right = circle_trajectory(sim_time, omega=omega, radius=radius)

        if mpc_counter % INTERPOLATION_RATIO == 0:
            q_ik = ik.solve_ik_fixed_base(target_left, target_right)
            q_current = mj_data.qpos[:16].copy()
            interpolator.set_segment(q_current, q_ik)
            el, er = ee_tracking_errors(pin_model, q_ik, target_left, target_right)
            ik_residuals.append((el + er) / 2)

        step_in_mpc = mpc_counter % INTERPOLATION_RATIO
        q_des, v_des = interpolator.interpolate(step_in_mpc)

        q_current = mj_data.qpos[:16].copy()
        v_current = mj_data.qvel[:16].copy()
        tau_ff = compute_gravity_torque(pin_model, q_des) if use_gravity_ff else None
        tau_control, _ = pd.compute_control(
            q_current, v_current, q_des, v_des, u_feedforward=tau_ff,
        )

        if pd.u_max is not None:
            saturated = np.any(np.isclose(tau_control, pd.u_max) | np.isclose(tau_control, pd.u_min))
            saturation_count += int(saturated)

        mj_data.ctrl[:16] = q_to_ctrl(tau_control)
        mujoco.mj_step(mj_model, mj_data)

        el, er = ee_tracking_errors(pin_model, q_current, target_left, target_right)
        errors_left.append(el)
        errors_right.append(er)

        sim_time += CONTROL_DT
        mpc_counter += 1
        total_steps += 1

    wall_time = time.time() - t_wall_start
    rms_left = np.sqrt(np.mean(np.array(errors_left) ** 2))
    rms_right = np.sqrt(np.mean(np.array(errors_right) ** 2))
    rms_avg = (rms_left + rms_right) / 2

    print()
    print("=" * 60)
    print("Results")
    print("=" * 60)
    print(f"Left  RMS: {rms_left * 100:6.2f} cm  (max: {np.max(errors_left) * 100:6.2f} cm)")
    print(f"Right RMS: {rms_right * 100:6.2f} cm  (max: {np.max(errors_right) * 100:6.2f} cm)")
    print(f"Avg   RMS: {rms_avg * 100:6.2f} cm")
    print(f"IK residual (avg): {np.mean(ik_residuals) * 100:.3f} cm")
    print(f"Torque saturation rate: {saturation_count}/{total_steps} ({100 * saturation_count / total_steps:.1f}%)")
    print(f"Wall time: {wall_time:.2f} s  (simulated {duration:.1f} s)")
    print("=" * 60)

    return {"rms_avg": rms_avg, "saturation_rate": saturation_count / total_steps}


if __name__ == "__main__":
    main()