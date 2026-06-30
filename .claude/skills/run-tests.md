# Run Tests

Run the project's test suite with detailed reporting and error analysis.

## Usage

```
/run-tests [module]
```

## Arguments

- `module` (optional): Specific test module to run. Options:
  - `all` (default): Run all tests
  - `robot_model`: Test robot kinematics and dynamics
  - `aligator`: Test ALIGATOR problem construction
  - `mpc`: Test MPC single-step solving
  - `mujoco`: Test MuJoCo model loading
  - `hybrid`: Test hybrid dynamics
  - `dual_arm`: Test dual-arm functionality
  - `wbc`: Test whole-body control

## What it does

1. Changes to the project directory
2. Strips ROS environment variables (required for pytest)
3. Runs pytest with the specified module
4. Provides a summary of:
   - Number of tests passed/failed
   - Test execution time
   - Detailed error messages for failures
   - Suggestions for fixing common issues

## Examples

```
/run-tests
/run-tests robot_model
/run-tests dual_arm
```

## Instructions

cd to study_example/wheeled_ur5e_aligator_mpc

Run the appropriate pytest command using pixi:
```bash
# Strip ROS env vars and run pytest
env -i PATH="$PATH" HOME="$HOME" USER="$USER" \
  pixi run -e all python -m pytest tests/[module] -v --tb=short
```

Parse the output and provide:
1. Summary statistics (X passed, Y failed, total time)
2. For each failed test:
   - Test name
   - Error type
   - Key error message
   - Suggestion for fixing

Common issues and fixes:
- Import errors → Check PYTHONPATH includes build/bindings/python
- FK mismatch → Verify q_nominal and MJCF model consistency
- Solver convergence → Adjust weights or horizon
- Dynamics mismatch → Known Phase 4 issue, see ROADMAP.md
