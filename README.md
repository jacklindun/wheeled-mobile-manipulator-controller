# Wheeled UR5e — Whole-Body MPC with ALIGATOR

10-DoF wheeled mobile manipulator control using kinematic and dynamic MPC approaches.

## Quick Start (Phase 6-v2 Optimized) 🚀

**Best performance: 1.75 cm RMS tracking error, 100% convergence**

```bash
cd wheeled_ur5e_aligator_mpc
export PYTHONPATH=.:../../build/bindings/python
pixi run -e all python scripts/demo_phase6_v2_optimized.py --render
```

Performance:
- ✅ **Steady-state**: 1.50 cm RMS
- ✅ **Startup**: 2.20 cm RMS (0-3s)
- ✅ **Overall**: 1.75 cm RMS
- ✅ **Convergence**: 99.7%
- ✅ **Control freq**: 500 Hz

---

## System Description

| Layer | DOFs | Joints |
|-------|------|--------|
| Virtual mobile base | 4 | base_x (slide), base_y (slide), base_z (slide), base_yaw (hinge) |
| UR5e arm | 6 | shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 |
| **Total** | **10** | |

The MPC runs at 20 Hz (dt=0.05 s), outputs joint velocity commands, which are
integrated to position targets and sent to MuJoCo position actuators.

## Requirements

- ALIGATOR 0.19+ — built from source in this repo (`build/bindings/python`), not pip-installed
- Pinocchio 3.9+ — ALIGATOR's Python bindings link against it, so it must be importable
- MuJoCo 3.x (`mujoco` pypi package)
- NumPy, SciPy, Matplotlib

> **Environment note:** ALIGATOR is compiled with `BUILD_WITH_PINOCCHIO_SUPPORT=ON`, so
> importing it requires `pinocchio`. The default pixi environment does **not** include
> pinocchio — use the `all` environment (or `pin4`), which provides pinocchio,
> example-robot-data, mujoco, numpy, scipy, and matplotlib. All commands below use
> `pixi run -e all`. The demo/test scripts add the ALIGATOR build directory to
> `sys.path` automatically, so no `pip install` of aligator is needed.

## Installation

```bash
# From the aligator repo root — build ALIGATOR if not already built
pixi run -e all build

# mujoco is already declared as a pypi dependency of the env; if missing:
pixi add --pypi mujoco

# Verify the toolchain (run from repo root)
pixi run -e all python -c "import mujoco; print('mujoco', mujoco.__version__)"
PYTHONPATH=build/bindings/python pixi run -e all \
    python -c "import aligator; print('aligator', aligator.__version__)"
```

## Running

All scripts must be run from the `study_example/wheeled_ur5e_aligator_mpc/` directory
using the `all` pixi environment. The scripts inject the ALIGATOR build path themselves,
so you only need the environment active.

```bash
cd study_example/wheeled_ur5e_aligator_mpc

# Quick solver smoke-test (no MuJoCo required)
pixi run -e all python scripts/run_mpc_single_step.py

# Full demo — headless, 30 s
pixi run -e all python scripts/run_demo.py --scenario ee_circle --duration 30

# Full demo — with MuJoCo viewer
pixi run -e all python scripts/run_demo.py --scenario ee_circle --duration 30 --render

# Re-plot last saved log
pixi run -e all python scripts/plot_log.py
```

Alternatively, activate the environment once and drop the `pixi run -e all` prefix:

```bash
eval "$(pixi shell-hook -e all)"
cd study_example/wheeled_ur5e_aligator_mpc
python scripts/run_demo.py --scenario ee_circle --duration 30
```

### Scenarios

| Scenario | Description |
|----------|-------------|
| `ee_circle` | EE traces a 10 cm radius circle in the Y-Z plane, base stationary |
| `ee_line` | EE moves 20 cm in Y, 10 cm in Z over 8 s, base stationary |
| `base_and_ee` | Base drives 0.8 m forward over 20 s, EE holds world position |
| `base_z_test` | Base lift oscillates ±12 cm, EE holds world position |

### Demo options

```
--scenario    {ee_circle,ee_line,base_and_ee,base_z_test}  (default: ee_circle)
--duration    seconds  (default: 30)
--render      open MuJoCo interactive viewer
--horizon     MPC horizon N  (default: 20)
--mpc_dt      MPC timestep s  (default: 0.05)
--max_iters   ALIGATOR iterations per step  (default: 10)
```

## Tests

```bash
cd study_example/wheeled_ur5e_aligator_mpc
pixi run -e all python -m pytest tests/ -v
```

27 tests across 4 modules:

| Module | Tests | Coverage |
|--------|-------|----------|
| `test_robot_model.py` | 7 | FK correctness, dynamics, linearization |
| `test_aligator_problem.py` | 5 | OCP build, cost, dynamics Jacobians |
| `test_mpc_single_step.py` | 5 | Full solve, warm start, output shapes |
| `test_mujoco_load.py` | 10 | MJCF load, joint/actuator/site mapping |

## Performance (measured on a typical laptop)

| Scenario | EE RMS error | EE max error | Success rate | Avg solve time |
|----------|-------------|--------------|--------------|----------------|
| ee_circle (30 s) | 2.6 cm | 3.6 cm | 100 % | ~39 ms |
| base_z_test (30 s) | 2.6 cm | 3.5 cm | 100 % | ~44 ms |

The solver uses ProxDDP in real-time-iteration mode with `mu_init=1e-4` and a 10-iteration
cap. With warm starting it reaches genuine KKT convergence (`tol=1e-4`) in ~2-3 iterations
per step, so success rate is ~100% and the average solve time stays under the 50 ms control
budget. (An earlier `mu_init=1e-2` left the augmented-Lagrangian penalty too soft: solves
plateaued at `primal_infeas≈1e-2`, never tripped the convergence test, and burned all 10
iterations — ~130 ms/step and a misleading ~5% reported success rate despite correct tracking.)

## Architecture

```
scripts/run_demo.py
  └── wheeled_ur5e_aligator_mpc/demo.py          # main control loop
        ├── robot_model.py                        # FK, dynamics, linearization
        ├── reference.py                          # reference trajectory generators
        ├── aligator_problem.py                   # ALIGATOR OCP builder (costs + dynamics)
        ├── aligator_mpc_controller.py            # SolverProxDDP wrapper + warm start
        ├── mujoco_env.py                         # MuJoCo simulation interface
        ├── low_level_control.py                  # velocity → position integration
        └── logger.py                             # data logging + matplotlib plots
```

## Key Implementation Notes

**Forward Kinematics**: `fk_numpy` follows the MJCF body tree directly (not DH
convention), so Python FK matches `mujoco.data.site_xpos["ee_site"]` to sub-mm
accuracy. The chain is:

```
world → base (trans+Rz(yaw))
     → shoulder_link  (+z 0.27, Rz(pan))
     → upper_arm      (+y 0.1358, Ry(lift))
     → forearm        (+x −0.425, Ry(elbow))
     → wrist_1        (+x −0.3922 +z 0.1333, Ry(w1))
     → wrist_2        (+y −0.0997, Rz(w2))
     → wrist_3        (+z 0.0996, Ry(w3))
     → ee_site        (+y −0.0996)
```

**Nominal posture**: `shoulder_pan = π` points the arm in the +X world direction.
`FK(q_nominal) ≈ [0.619, 0.064, 0.857]` m.

**Simulation stability**: The MJCF uses `implicitfast` integrator (not RK4) with
`kp=35000` on the vertical lift actuator. RK4 becomes unstable with the high stiffness
needed to hold ~55 kg arm weight against gravity.

**Reference continuity**: Every scenario computes its starting point from
`FK(q_nominal)` so the reference at t=0 matches the robot's initial posture,
giving zero initial tracking error.

## Available Phases

### Phase 1-3: Kinematic MPC (Baseline)
- **Status**: ✅ Production-ready
- **Performance**: 1.83 cm RMS, 100% convergence
- **Frequency**: 20 Hz MPC
- **Use case**: Stable baseline for single-arm

```bash
pixi run -e all python scripts/run_demo.py --scenario ee_circle
```

### Phase 6-v2: Optimized Kinematic MPC (Recommended) ⭐
- **Status**: ✅ **Best single-arm performance**
- **Performance**: 1.75 cm RMS, 99.7% convergence
- **Frequency**: 40 Hz MPC + 500 Hz control
- **Features**: 
  - Smooth startup (2s cubic ease-in)
  - Adaptive PD gains (startup boost)
  - Optimized weights
- **Report**: See [PHASE6_V2_PERFORMANCE_REPORT.md](PHASE6_V2_PERFORMANCE_REPORT.md)

```bash
pixi run -e all python scripts/demo_phase6_v2_optimized.py --render
```

### Phase 6-v3: Dual-Arm Control
- **Status**: ✅ Tested (Step 1: IK + gravity feedforward)
- **Performance**: 2.28 cm RMS (dual-arm circle tracking)
- **System**: 16-DOF (4 base + 6 left arm + 6 right arm)
- **Actuators**: Torque control
- **Guide**: See [docs/phase6/V3.md](docs/phase6/V3.md)

```bash
pixi run -e all python scripts/test_phase6_v3_step1_simple.py
```

---

## Performance Comparison

| Phase | Approach | DOF | RMS Error | Freq | Status |
|-------|----------|-----|-----------|------|--------|
| 1-3 | Kinematic MPC | 10 | 1.83 cm | 20 Hz | ✅ Baseline |
| 4 | Hybrid Dynamics | 10 | 2.5-5 cm | 20 Hz | ❌ Failed |
| **6-v2** | **Optimized Kinematic** | **10** | **1.75 cm** | **40 Hz** | ✅ **Best** |
| 6-v3 | Dual-arm Torque | 16 | 2.28 cm | 20 Hz | ✅ Dual-arm |

---

## Limitations

- Kinematic (velocity-level) MPC — no torque/dynamics model
- No orientation control for the end-effector
- State/control constraints are box constraints via ALIGATOR; joint limits use soft
  penalty rather than hard constraints
- MPC runs ~2.5× slower than real-time at horizon=20 on a single CPU core; reducing
  to horizon=10 brings it close to real-time
