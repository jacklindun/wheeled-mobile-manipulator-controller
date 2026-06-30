# Run Demo

Run a MuJoCo simulation demo with MPC control and generate performance reports.

## Usage

```
/run-demo [scenario] [options]
```

## Arguments

- `scenario` (optional): Which demo scenario to run
  - `ee_circle` (default): End-effector traces a circle
  - `ee_line`: End-effector moves in a line
  - `base_and_ee`: Base moves while EE holds position
  - `base_z_test`: Test vertical lift motion
  - `dual_arm`: Dual-arm independent motion
  - `hybrid`: Hybrid kinodynamic MPC (Phase 4)

- `options`: Additional flags
  - `--render`: Open MuJoCo viewer (visual)
  - `--duration <seconds>`: Simulation duration (default: 30)
  - `--headless`: Run without visualization
  - `--analyze`: Generate detailed performance analysis

## What it does

1. Changes to the project directory
2. Runs the appropriate demo script with pixi
3. Waits for completion
4. Analyzes the generated logs:
   - End-effector tracking error (RMS, max)
   - MPC solve time statistics
   - Success rate
   - Control smoothness
5. Displays key plots if they were generated
6. Provides recommendations for parameter tuning

## Examples

```
/run-demo ee_circle --render
/run-demo base_z_test --duration 20 --headless
/run-demo dual_arm --analyze
```

## Instructions

cd to study_example/wheeled_ur5e_aligator_mpc

Determine which script to run based on scenario:
- `ee_circle`, `ee_line`, `base_and_ee`, `base_z_test` → scripts/run_demo.py
- `dual_arm` → scripts/demo_dual_arm_mpc.py
- `hybrid` → scripts/run_hybrid_demo.py

Run with pixi:
```bash
pixi run -e all python scripts/[script].py --scenario [scenario] [options]
```

After completion:
1. Read logs/latest.npz (or equivalent) to extract metrics
2. Check if PNG plots were generated
3. Provide summary:
   - Scenario and duration
   - Tracking performance (RMS/max error)
   - MPC performance (avg/max solve time, success rate)
   - Control quality (smoothness, saturation)
4. Suggest improvements if performance is suboptimal

Tuning suggestions:
- High tracking error → increase horizon, reduce trajectory speed, tune weights
- Slow solve time → reduce horizon, reduce max_iters
- Control chatter → increase w_du (control smoothness weight)
- MPC failures → adjust mu_init, increase tolerance
