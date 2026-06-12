#!/usr/bin/env python
"""
Minimal single-step MPC test without MuJoCo.
Builds a horizon=5 OCP and runs one solve, prints results.
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
import aligator

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


def main():
    print(f"ALIGATOR version: {aligator.__version__}")
    robot = WheeledUR5eModel()
    mpc = AligatorWholeBodyMPC(robot, horizon=5, dt=0.05, max_iters=20)
    q0 = robot.q_nominal.copy()
    ref_gen = ReferenceGenerator(scenario="ee_circle", ee_start=robot.fk_numpy(q0))
    ref_traj = ref_gen.get_reference(t=0.0, horizon=mpc.horizon, dt=mpc.dt)

    print(f"\nSolving MPC: horizon={mpc.horizon}, nx=10, nu=10 ...")
    u0, q_pred, info = mpc.solve(q_current=q0, ref_traj=ref_traj)

    print(f"\n--- Results ---")
    print(f"  success:    {info['success']}")
    print(f"  status:     {info['status']}")
    print(f"  iter:       {info['iter']}")
    print(f"  cost:       {info['cost']}")
    print(f"  solve_time: {info['solve_time']*1e3:.2f} ms")
    print(f"  fallback:   {info['fallback']}")
    print(f"  u0 shape:   {u0.shape}  (expected (10,))")
    print(f"  u0:         {u0}")
    print(f"  q_pred shape: {q_pred.shape}  (expected (6, 10))")

    assert u0.shape == (10,), f"u0 shape error: {u0.shape}"
    assert q_pred.shape == (mpc.horizon + 1, 10), f"q_pred shape error: {q_pred.shape}"

    ee_ref = ref_traj["ee_pos"][0]
    ee_pred = robot.fk_numpy(q_pred[-1])
    ee_err = np.linalg.norm(ee_pred - ee_ref)
    print(f"\nEE ref:  {ee_ref}")
    print(f"EE pred (final): {ee_pred}")
    print(f"EE error at horizon end: {ee_err*100:.2f} cm")

    print("\nSingle-step MPC test PASSED.")


if __name__ == "__main__":
    main()
