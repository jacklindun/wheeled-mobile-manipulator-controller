"""Tests for ALIGATOR OCP construction and solver initialization."""

import sys
from pathlib import Path
import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))
_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]


def test_aligator_import():
    import aligator
    assert hasattr(aligator, "__version__")
    print(f"\nALIGATOR version: {aligator.__version__}")


def test_aligator_version():
    import aligator
    parts = [int(x) for x in aligator.__version__.split(".")[:2]]
    # Require >= 0.15
    assert parts[0] > 0 or parts[1] >= 15


def test_vector_space():
    import aligator
    space = aligator.manifolds.VectorSpace(10)
    assert space.nx == 10
    assert space.ndx == 10


def test_minimal_problem():
    """Build and solve a horizon=5 OCP without MuJoCo."""
    import aligator
    import aligator.dynamics
    import aligator.manifolds
    from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
    from wheeled_ur5e_aligator_mpc.aligator_problem import (
        KinematicWheeledUR5eProblemBuilder,
    )
    from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator

    robot = WheeledUR5eModel()
    builder = KinematicWheeledUR5eProblemBuilder(robot, horizon=5, dt=0.05)
    ref_gen = ReferenceGenerator("ee_circle")
    ref_traj = ref_gen.get_reference(t=0.0, horizon=5, dt=0.05)

    x0 = robot.q_nominal.copy()
    problem, ee_costs = builder.build_problem(x0, ref_traj)

    assert problem is not None
    assert len(ee_costs) == 6  # 5 running + 1 terminal


def test_solver_init():
    import aligator
    import aligator.manifolds
    from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
    from wheeled_ur5e_aligator_mpc.aligator_problem import (
        KinematicWheeledUR5eProblemBuilder,
    )
    from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator

    robot = WheeledUR5eModel()
    builder = KinematicWheeledUR5eProblemBuilder(robot, horizon=5, dt=0.05)
    ref_gen = ReferenceGenerator("ee_circle")
    ref_traj = ref_gen.get_reference(t=0.0, horizon=5, dt=0.05)
    x0 = robot.q_nominal.copy()
    problem, _ = builder.build_problem(x0, ref_traj)

    solver = aligator.SolverProxDDP(1e-4, mu_init=1e-2, max_iters=5)
    solver.setup(problem)
    # No assertion needed – just check it doesn't throw
    assert solver is not None
