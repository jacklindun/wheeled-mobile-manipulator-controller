#!/usr/bin/env python3
"""Phase 6-v3 Step 2: IK-informed dynamics MPC + gravity PD (headless)."""

import sys
import time

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
    FixedBaseIKPlanner,
    JointInterpolator,
    MpcSegmentInterpolator,
    circle_trajectory,
    compute_gravity_torque,
    generate_ee_reference_trajectory,
    generate_ik_state_reference,
    is_mpc_solution_usable,
    make_pd_controller,
)


def main(duration: float = 10.0, horizon: int = 5):
    print("=" * 60)
    print("Phase 6-v3 Step 2: IK-informed MPC + PD (headless)")
    print(f"  horizon={horizon}, fix_base=True, weight_preset=ik_informed")
    print("=" * 60)

    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)
    pin_model = DualArmPinocchioModel(MJCF_PIN)
    ik_planner = FixedBaseIKPlanner(pin_model)

    mpc = DualArmDynamicsMPC(
        mjcf_path=MJCF_PIN,
        horizon=horizon,
        dt=0.05,
        weight_preset="ik_informed",
        fix_base=True,
    )

    mpc_dt = 0.05
    mpc_interpolator = MpcSegmentInterpolator(ratio=INTERPOLATION_RATIO)
    ik_interpolator = JointInterpolator()
    pd_controller = make_pd_controller()

    mj_data.qpos[:16] = DUAL_ARM_Q_NOMINAL
    mj_data.qvel[:16] = 0.0
    mujoco.mj_forward(mj_model, mj_data)

    sim_time = 0.0
    mpc_counter = 0
    errors_left, errors_right = [], []
    solve_times = []
    convergence_count = 0
    usable_count = 0
    fallback_count = 0
    total_mpc_calls = 0
    active_mode = "ik"

    print(f"Running {duration} s...")
    print()

    while sim_time < duration:
        target_left, target_right = circle_trajectory(sim_time)

        if mpc_counter % INTERPOLATION_RATIO == 0:
            q_current = mj_data.qpos[:16].copy()
            v_current = mj_data.qvel[:16].copy()
            x_current = np.concatenate([q_current, v_current])

            target_left_traj, target_right_traj = generate_ee_reference_trajectory(
                sim_time, mpc.horizon, mpc_dt,
            )
            x_ref_traj = generate_ik_state_reference(
                ik_planner, target_left_traj, target_right_traj, mpc_dt,
            )

            mpc_t_start = time.time()
            try:
                xs, us, results = mpc.solve(
                    x_current,
                    target_left_traj,
                    target_right_traj,
                    x_ref_traj=x_ref_traj,
                    max_iters=50,
                    verbose=False,
                )
                solve_time = time.time() - mpc_t_start
                solve_times.append(solve_time)
                total_mpc_calls += 1

                mpc_ok = results.conv or is_mpc_solution_usable(
                    pin_model, xs, target_left, target_right, stage=1,
                )
                if results.conv:
                    convergence_count += 1
                if mpc_ok:
                    usable_count += 1
                    mpc_interpolator.set_segment(xs[:2], us[:1])
                    active_mode = "mpc"
                else:
                    fallback_count += 1
                    q_ik = ik_planner.solve_ik_fixed_base(target_left, target_right)
                    ik_interpolator.set_segment(q_current, q_ik)
                    active_mode = "ik"

                if total_mpc_calls <= 3:
                    print(
                        f"[MPC] t={sim_time:.2f}s solve={solve_time*1000:.1f}ms "
                        f"conv={results.conv} iters={results.num_iters} "
                        f"cost={results.traj_cost:.1e} mode={active_mode}"
                    )
            except Exception as e:
                print(f"[MPC] failed: {e}")
                break

        step_in_mpc = mpc_counter % INTERPOLATION_RATIO
        if active_mode == "mpc":
            q_des, v_des, tau_ff = mpc_interpolator.interpolate(step_in_mpc)
        else:
            q_des, v_des = ik_interpolator.interpolate(step_in_mpc)
            tau_ff = compute_gravity_torque(pin_model, q_des)

        q_current = mj_data.qpos[:16].copy()
        v_current = mj_data.qvel[:16].copy()
        tau_control, _ = pd_controller.compute_control(
            q_current, v_current, q_des, v_des, u_feedforward=tau_ff,
        )
        mj_data.ctrl[:16] = q_to_ctrl(tau_control)
        mujoco.mj_step(mj_model, mj_data)

        errors_left.append(np.linalg.norm(pin_model.fk_left_ee(q_current) - target_left))
        errors_right.append(np.linalg.norm(pin_model.fk_right_ee(q_current) - target_right))

        sim_time += CONTROL_DT
        mpc_counter += 1

    rms_left = np.sqrt(np.mean(np.array(errors_left) ** 2))
    rms_right = np.sqrt(np.mean(np.array(errors_right) ** 2))
    rms_avg = (rms_left + rms_right) / 2

    print()
    print("=" * 60)
    print("Results")
    print("=" * 60)
    print(f"Left  RMS: {rms_left*100:6.2f} cm  (max: {np.max(errors_left)*100:6.2f} cm)")
    print(f"Right RMS: {rms_right*100:6.2f} cm  (max: {np.max(errors_right)*100:6.2f} cm)")
    print(f"Avg   RMS: {rms_avg*100:6.2f} cm")
    if solve_times:
        print(f"MPC solve: {np.mean(solve_times)*1000:.1f} ± {np.std(solve_times)*1000:.1f} ms")
        print(f"MPC strict convergence: {convergence_count}/{total_mpc_calls} ({100*convergence_count/total_mpc_calls:.1f}%)")
        print(f"MPC usable (EE quality): {usable_count}/{total_mpc_calls} ({100*usable_count/total_mpc_calls:.1f}%)")
        print(f"IK fallback: {fallback_count}/{total_mpc_calls}")
    print("=" * 60)

    return {
        "rms_avg": rms_avg,
        "conv_rate": convergence_count / max(total_mpc_calls, 1),
        "usable_rate": usable_count / max(total_mpc_calls, 1),
    }


if __name__ == "__main__":
    main()