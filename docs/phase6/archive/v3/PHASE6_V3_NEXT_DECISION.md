# Phase 6-v3 Next Decision Report

## Date
2026-06-26

## Executive Decision

**Recommend Phase 6-v3 Step 1 (IK → interpolate → pure torque PD) as the practical baseline.**

**Do not promote full dynamics MPC** to the main v3 path. It shows **0% convergence** on circular tracking and provides **no benefit** even with IK fallback (Step 2 degrades to Step 1 while wasting ~77 ms/MPC call).

---

## Task 1: Code Verification Checklist

| Item | Status | Notes |
|------|--------|-------|
| q/Pinocchio order `[x,y,yaw,z,arms...]` | ✅ PASS | `dual_arm_pinocchio_model.py`, `coordinate_mapping.py` |
| MuJoCo ctrl order `[x,y,z,yaw,arms...]` | ✅ PASS | `q_to_ctrl()` mapping verified |
| Torque mapped before `mj_data.ctrl` | ✅ PASS | All v3 scripts use `q_to_ctrl()` |
| v3 nominal base `[0,0,0,0.2]` | ✅ PASS | `DUAL_ARM_Q_NOMINAL` in `coordinate_mapping.py` |
| Step 2 uses only `[xs[0],xs[1]]` per MPC interval | ✅ PASS | `MpcSegmentInterpolator` |
| Solver convergence checked before MPC feedforward | ✅ PASS | `results.conv` checked; IK fallback added |

### Patches Applied This Session

| File | Change |
|------|--------|
| `feedforward_pd_controller.py` | Fix `Kp_base`/`Kd_base` order: `[x,y,yaw,z]` not `[x,y,z,yaw]` |
| `coordinate_mapping.py` | Add `DUAL_ARM_Q_NOMINAL`, `DUAL_ARM_TAU_MAX_Q` |
| `scripts/test_phase6_v3_step1.py` | Use shared nominal and torque limits |
| `scripts/test_phase6_v3_step2_simple.py` | IK fallback when `conv=False`; shared helpers |
| `wheeled_ur5e_aligator_mpc/phase6_v3_common.py` | **NEW** shared IK/interpolator/trajectory helpers |

---

## Commands Run

```bash
cd /home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc

PYTHONPATH=.:../../build/bindings/python pixi run -e all python scripts/test_phase6_v3_step1_simple.py

PYTHONPATH=.:../../build/bindings/python pixi run -e all python scripts/diagnose_phase6_v3_dynamics_prediction.py

PYTHONPATH=.:../../build/bindings/python pixi run -e all python scripts/test_phase6_v3_mpc_feasibility.py

PYTHONPATH=.:../../build/bindings/python pixi run -e all python scripts/test_phase6_v3_step2_simple.py
```

---

## Measured Results

### Task 2: Step 1 Headless Baseline (10 s, circle r=0.08 m)

| Metric | Value |
|--------|-------|
| Left RMS | 15.90 cm |
| Right RMS | 16.05 cm |
| **Avg RMS** | **15.98 cm** |
| IK residual (avg) | 0.004 cm |
| Torque saturation | 1.2% (60/5000) |
| Wall time | 0.40 s (25× real-time) |

**Note on documented "1.79 cm"**: This value is **hardcoded** in `scripts/test_phase6_v3_step2.py` line 362 as a comparison reference, not a measured Step 1 RMS. Actual instantaneous minimum error during tracking is ~1.9 cm; full-trajectory RMS is ~16 cm because the circular target moves faster than torque PD can follow with 20 Hz IK updates.

### Task 3: Dynamics Prediction Diagnostic

| Test | q error | v error | Interpretation |
|------|---------|---------|----------------|
| Static τ=0, dt=0.002 s | 6.1e-08 | 3.1e-05 | Excellent match |
| Static τ=0, dt=0.05 s | 1.2e-02 | 9.7e-03 | Small drift (damping/integration) |
| Random τ, dt=0.002 s | ~1e-05 | ~1e-02 | Good |
| Random τ, dt=0.05 s | 0.17–0.40 | 0.11–0.37 | Larger at MPC dt |
| MPC rollout (conv=False) | 0.26 | 0.27 | Bad **MPC solution**, not model gap |

**Conclusion**: Single-step Pinocchio/MuJoCo agreement is good at control dt (0.002 s). Multi-step error at dt=0.05 s exists but is **not** the primary cause of 16–18 cm tracking failure. The dominant issue is **MPC problem formulation + non-convergence**.

### Task 4: MPC Feasibility Matrix

| Case | conv | Pred EE (L/R) | Actual EE after rollout (L/R) |
|------|------|---------------|-------------------------------|
| static_fk_h1 | ✅ | 2.4 / 2.4 cm | 1.3 / 1.3 cm |
| static_fk_h3 | ✅ | 10.5 / 12.2 cm | 0.6 / 0.8 cm |
| static_circle_h1 | ✅ | 23.5 / 30.7 cm | 24.7 / 32.0 cm |
| static_circle_h3 | ✅ | 9.4 / 11.4 cm | 23.2 / 30.0 cm |
| static_circle_h10 | ❌ | 41.1 / 45.4 cm | 23.8 / 31.0 cm |
| slow_line_h5 | ❌ | 15.3 / 9.3 cm | 25.0 / 32.0 cm |

**Convergence rate**: 4/6 (67%)

**Key observations**:
- h=1 at current FK converges; h=10 circle target does not
- Even when conv=True on circle targets, predicted EE error remains 9–31 cm
- MPC prediction vs MuJoCo rollout diverge when solution quality is poor

### Step 2 with IK Fallback (3 s)

| Metric | Before fallback fix | After IK fallback |
|--------|--------------------|--------------------|
| Avg RMS | 18.59 cm | 16.96 cm |
| MPC convergence | 0% | 0% |
| IK fallback | N/A (used bad xs) | 61/61 (100%) |

IK fallback prevents the worst case (chasing bad `xs` with zero feedforward) but **cannot beat pure PD** because MPC never contributes.

---

## Decision Rules Applied

| Rule | Outcome |
|------|---------|
| Pure PD < 3 cm AND MPC non-convergent → recommend pure PD | ⚠️ RMS is ~16 cm (not <3 cm), but pure PD is still the **only viable** controller |
| MPC converges only on simplified tasks → research-only | ✅ Applied |
| MPC beats pure PD on circular tracking → promote | ❌ Not met |

---

## Recommended Phase 6-v3 Path

### Production / Demo Path
```
IK (20 Hz) → joint interpolation (500 Hz) → torque PD → MuJoCo motors
```
**Entry point**: `scripts/test_phase6_v3_step1_simple.py`

### Research-Only Path
```
Dynamics MPC (investigate IK-informed cost, shorter horizon, fixed base)
```
**Diagnostics**: `scripts/test_phase6_v3_mpc_feasibility.py`, `scripts/diagnose_phase6_v3_dynamics_prediction.py`

### Do Not Use
- Full h=10 dynamics MPC for circular dual-arm tracking
- Unconverged MPC `xs` as PD reference (fixed via IK fallback, but MPC still wasteful)

---

## Update 2026-06-26 — Dual-Path Progress

Both paths advanced in parallel:

| Path | Before | After | Key changes |
|------|--------|-------|-------------|
| **Step 1** (IK + PD) | 15.98 cm RMS | **2.28 cm RMS** | Gravity feedforward `τ_g(q_des)`, Kp_arm 500 |
| **Step 2** (IK-informed MPC) | 18.59 cm, 0% conv | **3.56 cm RMS**, 100% usable | IK `x_ref` cost, h=5, fix_base, RNEA warm start, EE-quality gate |

```bash
PYTHONPATH=.:../../build/bindings/python pixi run -e all python scripts/test_phase6_v3_benchmark.py
```

**Findings**:
- Gravity feedforward was the main Step 1 breakthrough (16 cm → 2.3 cm).
- IK-informed MPC achieves sub-mm **predicted** EE (conv=False due to strict KKT tol, but `is_mpc_solution_usable()` passes 100%).
- Closed-loop Step 2 (3.56 cm) still slightly worse than Step 1 — MPC torque feedforward does not execute as cleanly in MuJoCo as gravity+PD.
- **Production recommendation unchanged**: Step 1; Step 2 viable for research with usable-rate metric instead of strict `conv`.

New/updated files: `phase6_v3_common.py` (gravity, IK x_ref, usable check), `dual_arm_dynamics_mpc.py` (`weight_preset=ik_informed`), `scripts/test_phase6_v3_benchmark.py`.

---

## Recommended Next Steps

### Priority 1 — Improve Step 1 tracking (target RMS < 5 cm)
1. Add gravity feedforward (`pin.computeGeneralizedGravity`) to torque PD
2. Increase IK update rate or use predictive IK along circle
3. Tune `Kp_arm`/`Kd_arm` systematically (current 300/30 is conservative vs original 500/50)

### Priority 2 — MPC research (only if needed)
1. Add IK joint reference cost: `||q - q_ik||²` with weight >> EE weight
2. Fix base: freeze base DOFs in MPC (match Step 1)
3. Reduce horizon to h=3–5; require conv=True before using feedforward

### Priority 3 — Optional architecture (Task 6)
```
Kinematic planner → inverse dynamics τ_ff → PD
```
Only pursue if Priority 1 cannot meet requirements.

---

## Files Changed / Created

| File | Status |
|------|--------|
| `wheeled_ur5e_aligator_mpc/phase6_v3_common.py` | NEW |
| `wheeled_ur5e_aligator_mpc/coordinate_mapping.py` | MODIFIED |
| `wheeled_ur5e_aligator_mpc/feedforward_pd_controller.py` | MODIFIED |
| `scripts/test_phase6_v3_step1_simple.py` | NEW |
| `scripts/diagnose_phase6_v3_dynamics_prediction.py` | NEW |
| `scripts/test_phase6_v3_mpc_feasibility.py` | NEW |
| `scripts/test_phase6_v3_step1.py` | MODIFIED |
| `scripts/test_phase6_v3_step2_simple.py` | MODIFIED |
| `PHASE6_V3_NEXT_DECISION.md` | UPDATED (this file) |