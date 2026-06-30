#!/usr/bin/env python3
"""
Test ABA vs MuJoCo consistency with damping fix
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

import numpy as np
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics


def test_dynamics_consistency():
    """Compare ABA (with damping) vs MuJoCo single-step predictions"""

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=False)

    # Create dynamics model
    dynamics = HybridWheeledUR5eDynamics(pin_robot, dt=0.01)

    print("Dynamics model damping:", dynamics._damping)
    print()

    scenarios = [
        ("Zero torque (gravity)", np.zeros(10)),
        ("Small torque", np.array([0, 0, 0, 0, 0, 5, 0, 0, 0, 0])),
        ("Large torque", np.array([0, 0, 0, 0, 0, 50, 0, 0, 0, 0])),
    ]

    print(f"{'Scenario':<25} {'q_err':<12} {'v_err':<12} {'Status'}")
    print("-" * 60)

    for name, u in scenarios:
        # Reset
        env.reset(robot.q_nominal)
        x0 = env.get_state()

        # ABA prediction
        data_dyn = dynamics.createData()
        dynamics.forward(x0, u, data_dyn)
        x_aba = np.array(data_dyn.xnext)

        # MuJoCo execution
        env.set_control(u)
        env.step(substeps=int(0.01 / env.model.opt.timestep))
        x_mujoco = env.get_state()

        # Compare
        q_err = np.linalg.norm(x_aba[:10] - x_mujoco[:10])
        v_err = np.linalg.norm(x_aba[10:] - x_mujoco[10:])

        status = "✓ GOOD" if (q_err < 1e-3 and v_err < 1e-2) else "⚠️ BAD"

        print(f"{name:<25} {q_err:>10.6f}  {v_err:>10.6f}  {status}")

        if v_err > 0.05:
            print(f"  ABA v_arm: {x_aba[10:16]}")
            print(f"  MuJ v_arm: {x_mujoco[10:16]}")
            print(f"  Difference: {x_aba[10:16] - x_mujoco[10:16]}")

    env.close()


if __name__ == "__main__":
    test_dynamics_consistency()
