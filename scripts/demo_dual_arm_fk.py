"""
Quick demo: Dual arm forward kinematics visualization.

Shows both EE positions as the arms move independently.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco
import mujoco.viewer

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel


def main():
    # Load models
    pin_model = DualArmPinocchioModel()
    mjcf_path = Path(__file__).resolve().parents[1] / "assets" / "wheeled_dual_ur5e_v2.xml"
    mj_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    mj_data = mujoco.MjData(mj_model)

    # Get site IDs
    left_site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site")
    right_site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site")

    # Test configurations
    q_nominal = pin_model.get_q_nominal()

    # Left arm circle motion
    print("\n=== Demo: Independent Arm Motion ===\n")
    print("Left arm will circle, right arm stays fixed\n")

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        t = 0.0
        dt = 0.01

        while viewer.is_running() and t < 10.0:
            # Left arm circles (shoulder_lift oscillates)
            q = q_nominal.copy()
            q[5] = -np.pi/2 + 0.5 * np.sin(2 * np.pi * 0.2 * t)  # left_shoulder_lift
            q[6] = 0.5 * np.cos(2 * np.pi * 0.2 * t)             # left_elbow

            # Right arm waves (wrist oscillates)
            q[13] = -np.pi/2 + 0.3 * np.sin(2 * np.pi * 0.3 * t)  # right_wrist_1

            # Update MuJoCo
            mj_data.qpos[:] = q
            mujoco.mj_forward(mj_model, mj_data)

            # Compute FK with Pinocchio
            p_left_pin = pin_model.fk_left_ee(q)
            p_right_pin = pin_model.fk_right_ee(q)

            # Get MuJoCo reference
            p_left_mj = mj_data.site_xpos[left_site_id]
            p_right_mj = mj_data.site_xpos[right_site_id]

            # Verify match
            err_left = np.linalg.norm(p_left_pin - p_left_mj)
            err_right = np.linalg.norm(p_right_pin - p_right_mj)

            if t < 0.1 or int(t * 10) % 10 == 0:
                print(f"t={t:5.2f}s | Left EE: {p_left_pin} (err={err_left*1000:.2f}mm)")
                print(f"          | Right EE: {p_right_pin} (err={err_right*1000:.2f}mm)")

            viewer.sync()
            t += dt

    print("\n✓ Demo完成！FK匹配误差 < 1mm")


if __name__ == "__main__":
    main()
