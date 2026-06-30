# Benchmark

Run comprehensive performance benchmarks and generate comparison reports.

## Usage

```
/benchmark [target] [options]
```

## Arguments

- `target` (optional): What to benchmark
  - `all` (default): All scenarios and metrics
  - `mpc`: MPC solve time across scenarios
  - `tracking`: End-effector tracking accuracy
  - `scenarios`: Compare all demo scenarios
  - `phases`: Compare Phase 1-3 vs Phase 4 vs Phase 7
  - `qp`: QP solver performance (Phase 6 WBC)

- `options`:
  - `--quick`: Short duration for fast feedback (10s per scenario)
  - `--thorough`: Long runs for statistical significance (60s per scenario)
  - `--compare`: Load historical data and compare trends
  - `--export`: Export results to CSV/JSON

## What it does

Systematically benchmarks the system across multiple dimensions:

### MPC Benchmarks
- Solve time (mean, std, p95, p99, max)
- Convergence rate (% successful solves)
- Iterations per solve
- Warm-start effectiveness

### Tracking Benchmarks
- End-effector position error (RMS, max)
- Base position tracking
- Joint limit violations
- Trajectory smoothness

### Scenario Comparison
Runs all scenarios (ee_circle, base_z_test, etc.) and compares:
- Which scenarios are hardest to track?
- Which have longest solve times?
- Performance consistency

### Phase Comparison
- Phase 1-3 (kinematic): Baseline performance
- Phase 4 (hybrid): Dynamics mismatch impact
- Phase 7 (dual-arm): Scalability

### Results Format
Generates:
- Summary table (markdown)
- Performance plots (PNG)
- Raw data (NPZ/CSV)
- Recommendations for tuning

## Examples

```
/benchmark
/benchmark mpc --quick
/benchmark scenarios --thorough --export
/benchmark phases --compare
```

## Instructions

cd to study_example/wheeled_ur5e_aligator_mpc

### For MPC Benchmark
Run each scenario headless, collect timing data:
```bash
for scenario in ee_circle ee_line base_and_ee base_z_test; do
  pixi run -e all python scripts/run_demo.py \
    --scenario $scenario --duration [duration] --no-render
done
```

### For Tracking Benchmark
Extract from logs/latest.npz:
- ee_error RMS/max
- base_error
- control smoothness (|u[t] - u[t-1]|)

### For Phase Comparison
Run representative scenario from each phase:
- Phase 1-3: run_demo.py --scenario ee_circle
- Phase 4: run_hybrid_demo.py
- Phase 7: demo_dual_arm_mpc.py

### Analysis
1. Load all log files
2. Compute statistics
3. Generate comparison tables:
   ```
   | Scenario    | EE RMS (cm) | EE Max (cm) | Solve (ms) | Success % |
   |-------------|-------------|-------------|------------|-----------|
   | ee_circle   | 2.6         | 3.6         | 39         | 100       |
   | ...         |             |             |            |           |
   ```
4. Generate plots:
   - Solve time distribution (boxplot)
   - Tracking error over time (line plot)
   - Success rate bar chart

5. Provide tuning recommendations based on bottlenecks

### Historical Comparison (--compare)
If previous results exist, show trends:
- Performance regression/improvement
- Impact of recent changes
