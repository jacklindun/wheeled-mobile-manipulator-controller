# Phase 6-v3 Follow-up Task Plan

## Context

Current project path:

```bash
/home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc
```

Phase 6-v3 is the dual-arm torque-control track. The target architecture under investigation was:

```text
Full dynamics MPC at 20 Hz
  -> q/v/tau interpolation at 500 Hz
  -> feedforward PD torque control
  -> MuJoCo motor actuators
```

Read these documents before making changes:

- `PHASE6_V3_FINAL_DIAGNOSIS.md`
- `PHASE6_V3_STATUS.md`
- `PHASE6_V3_DESIGN.md`

Important current result:

- Phase 6-v3 Step 1, pure torque PD: about `1.79 cm` tracking error.
- Phase 6-v3 Step 2, dynamics MPC + PD after structural fixes: about `18.59 cm` tracking error and `0%` MPC convergence.
- The structural fixes already completed include coordinate mapping, base nominal order, interpolation first-segment logic, cost de-duplication, persistent solver warm start, fallback, and torque constraints.

Do not repeat those fixes unless local code inspection shows they were reverted.

## Goal

Produce a reliable Phase 6-v3 path for dual-arm torque control and a clear diagnosis of whether full dynamics MPC is viable in this codebase.

The short-term deliverable is a stable, documented pure torque PD demo/test based on Step 1.

The research deliverable is a minimal set of dynamics-MPC diagnostics that determine whether Pinocchio/ALIGATOR prediction matches MuJoCo closely enough to justify continuing full dynamics MPC work.

## Non-goals

- Do not spend time sweeping MPC weights before model prediction has been validated.
- Do not claim dynamics MPC performance from unconverged solver results.
- Do not put the current full dynamics MPC in the main recommended path unless it achieves convergence and beats pure PD.
- Do not rework unrelated Phase 1-5 code.

## Task 1: Verify Current Code State

Inspect these files:

- `wheeled_ur5e_aligator_mpc/coordinate_mapping.py`
- `wheeled_ur5e_aligator_mpc/dual_arm_dynamics_mpc.py`
- `wheeled_ur5e_aligator_mpc/feedforward_pd_controller.py`
- `scripts/test_phase6_v3_step1.py`
- `scripts/test_phase6_v3_step2_simple.py`

Confirm:

- q/Pinocchio order is `base_x, base_y, base_yaw, base_z, left_arm..., right_arm...`.
- MuJoCo ctrl order is `base_x, base_y, base_z, base_yaw, left_arm..., right_arm...`.
- torque output is mapped before writing `mj_data.ctrl`.
- v3 nominal base is `[0.0, 0.0, 0.0, 0.2]`.
- Step 2 interpolation executes only `[xs[0], xs[1]]` during one 50 ms MPC interval.
- solver convergence is checked before using MPC feedforward.

Acceptance:

- Add a short note to the final report listing pass/fail for each item.
- If any item fails, fix only that item and document the patch.

## Task 2: Stabilize Step 1 As The Practical Phase 6-v3 Baseline

Create or update a non-viewer benchmark script if needed, preferably:

```text
scripts/test_phase6_v3_step1_simple.py
```

It should run the pure torque PD pipeline without MuJoCo viewer:

```text
IK target generation
  -> q/v interpolation
  -> torque PD
  -> MuJoCo motor actuator simulation
```

Log:

- left/right RMS tracking error
- left/right max tracking error
- torque saturation rate
- collision count if collision checking is enabled
- runtime and simulated duration

Recommended command:

```bash
pixi run -e all python scripts/test_phase6_v3_step1_simple.py
```

Acceptance:

- Headless script completes without viewer timeout.
- Average RMS error is in the same range as the documented Step 1 result, ideally under `3 cm`.
- Output is deterministic enough for comparison across runs.
- If performance differs from `1.79 cm`, explain why.

## Task 3: Dynamics Model Prediction Diagnostic

Create a diagnostic script, preferably:

```text
scripts/diagnose_phase6_v3_dynamics_prediction.py
```

Purpose:

Determine whether ALIGATOR/Pinocchio forward dynamics predicts MuJoCo well enough for MPC.

Tests:

1. Static gravity test at nominal pose:
   - Apply zero torque.
   - Compare one-step acceleration or next state between Pinocchio/ALIGATOR and MuJoCo.

2. Random small torque one-step test:
   - Sample several bounded torque vectors.
   - Compare predicted `q_next, v_next` after `dt=0.002` and `dt=0.05`.

3. MPC first-control rollout test:
   - Use the first MPC output `u0`.
   - Roll out ALIGATOR/Pinocchio for one MPC step.
   - Apply the same torque in MuJoCo for equivalent simulation steps.
   - Compare state and EE position deltas.

Report:

- q error norm
- v error norm
- left/right EE prediction error
- worst offending DOFs

Acceptance:

- Produce a concise table.
- If one-step errors are already large, state that full dynamics MPC should be paused.
- If one-step errors are small but multi-step errors grow, identify integration mismatch as likely.

## Task 4: Minimal MPC Feasibility Tests

Before testing full dual-arm circular tracking again, build simpler MPC tests:

1. Static target, full dual arm, horizon 1.
2. Static target, full dual arm, horizon 3.
3. Static target, one arm only if easy to disable the other EE cost.
4. Slow line target, horizon 5.

For each test, record:

- solver convergence rate
- iterations
- trajectory cost
- KKT/convergence status if available
- predicted EE error
- actual MuJoCo EE error if rolled out

Acceptance:

- If static horizon-1 or horizon-3 does not converge, do not proceed to circular tracking.
- If MPC converges in prediction but fails in MuJoCo, focus on model mismatch.
- If MPC fails even in prediction, focus on cost/residual/problem formulation.

## Task 5: Decide The Recommended Phase 6-v3 Path

After Tasks 2-4, write a short decision report:

```text
PHASE6_V3_NEXT_DECISION.md
```

Use this decision rule:

- If pure PD remains under `3 cm` and dynamics MPC remains non-convergent, recommend pure torque PD as the Phase 6-v3 practical baseline.
- If dynamics MPC converges only on simplified static tasks, keep it as research-only.
- If dynamics MPC converges and beats pure PD on headless circular tracking, promote it back to the main v3 path.

The report must include:

- exact commands run
- measured results
- changed files
- recommended next step

## Optional Task 6: Alternative Architecture Prototype

Only do this after the diagnostics above.

Prototype:

```text
kinematic / IK trajectory planner
  -> inverse dynamics torque feedforward
  -> PD torque tracking
```

Reason:

This preserves torque control but avoids solving the full constrained dynamics MPC online.

Acceptance:

- Compare against pure PD.
- Keep it only if it improves tracking error or reduces torque saturation.

## Final Deliverables

At the end, provide:

- `PHASE6_V3_NEXT_DECISION.md`
- any new diagnostic scripts
- benchmark logs or printed summary values
- a short list of files changed

Do not delete or rewrite existing diagnosis documents.

