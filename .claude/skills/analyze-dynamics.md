# Analyze Dynamics

Diagnose dynamics model issues including integrator mismatch, FK accuracy, and Jacobian correctness.

## Usage

```
/analyze-dynamics [aspect]
```

## Arguments

- `aspect` (optional): What to analyze
  - `all` (default): Complete dynamics analysis
  - `fk`: Forward kinematics accuracy vs MuJoCo
  - `jacobian`: Jacobian finite-difference verification
  - `integrator`: Single-step and multi-step integration error (Phase 4 issue)
  - `mismatch`: Diagnose ALIGATOR vs MuJoCo prediction mismatch

## What it does

Performs comprehensive diagnostics of the dynamics model:

### FK Analysis
- Compares Python FK with MuJoCo site positions
- Tests multiple joint configurations
- Reports position error (should be <1mm)

### Jacobian Analysis
- Verifies analytical Jacobian against finite differences
- Tests both position and orientation components
- Reports numerical accuracy (should be <1e-5)

### Integrator Analysis (Phase 4 specific)
- Single-step prediction: ALIGATOR forward vs MuJoCo execution
- Multi-step accumulation: 25-step error growth
- Identifies source of mismatch (integrator type, damping, etc.)

### Recommendations
Based on findings, suggests:
- Parameter corrections
- Model adjustments
- Alternative integration schemes
- References to relevant Phase documentation

## Examples

```
/analyze-dynamics
/analyze-dynamics fk
/analyze-dynamics integrator
```

## Instructions

cd to study_example/wheeled_ur5e_aligator_mpc

Based on aspect, run appropriate analysis:

### For FK
```python
# Compare FK with MuJoCo
import mujoco
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5e

model = mujoco.MjModel.from_xml_path("assets/wheeled_ur5e.xml")
data = mujoco.MjData(model)
pin_model = PinocchioWheeledUR5e()

q_test = pin_model.get_q_nominal()
# Set MuJoCo state and compare
```

### For Integrator
Run existing diagnostic scripts:
```bash
pixi run -e all python scripts/compare_dynamics.py
pixi run -e all python scripts/diagnose_phase4.py
```

### For Jacobian
Check test files:
```bash
pixi run -e all python -m pytest tests/test_pinocchio_model.py::test_jacobian_finite_diff -v
```

Summarize findings:
1. Error magnitudes
2. Error locations (which joints/coordinates)
3. Trends (linear growth, accumulation, etc.)
4. Root cause hypothesis
5. Recommended fixes from ROADMAP.md Phase 8
