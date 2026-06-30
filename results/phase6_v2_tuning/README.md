# Phase 6-v2 Tuning Results

This directory contains all analysis figures and data from the Phase 6-v2 parameter tuning process.

## Contents

### Performance Analysis
- `phase6_v2_transient_analysis.png` - Original transient response analysis
- `phase6_v2_smooth_startup_analysis.png` - Smooth startup performance

### Parameter Tuning
- `adaptive_pd_gains_comparison.png` - Adaptive PD gains vs fixed gains
- `mpc_warmup_comparison.png` - MPC warmup test results
- `initial_velocity_comparison.png` - Initial velocity matching test
- `feedforward_compensation_comparison.png` - Feedforward compensation test

### Startup Optimization
- `startup_comparison.png` - Smooth vs abrupt startup comparison
- `combined_strategy_comparison.png` - Final combined strategy (smooth + adaptive PD)
- `combined_strategy_results.npz` - Raw data for combined strategy

### Key Findings

**Best Configuration (Combined Strategy)**:
- Smooth startup (2s cubic ease-in)
- Adaptive PD gains (2× for 0-3s, ramp down 3-6s)
- 40 Hz MPC frequency
- Optimized weights (ee_pos=300, terminal_ee_pos=600)

**Performance**:
- Startup (0-1s): 4.25 → 2.62 cm (-38.5%)
- Peak error: 6.35 → 3.29 cm (-48.2%)
- Overall RMS: 2.22 → 1.76 cm (-20.7%)
- Steady-state: 1.69 cm (maintained)

See `PHASE6_V2_PERFORMANCE_REPORT.md` in the project root for complete details.
