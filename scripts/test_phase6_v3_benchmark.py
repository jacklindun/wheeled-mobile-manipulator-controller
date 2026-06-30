#!/usr/bin/env python3
"""Compare Phase 6-v3 Step 1 vs improved Step 2 on the same trajectory."""

import sys

sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

from scripts.test_phase6_v3_step1_simple import main as run_step1
from scripts.test_phase6_v3_step2_simple import main as run_step2


def main(duration: float = 10.0):
    print("\n" + "#" * 60)
    print("# Phase 6-v3 Dual-Path Benchmark")
    print("#" * 60 + "\n")

    print(">>> Path 1: Step 1 (IK + gravity PD)")
    r1 = run_step1(duration=duration)

    print("\n>>> Path 2: Step 2 (IK-informed MPC + PD)")
    r2 = run_step2(duration=duration, horizon=5)

    print("\n" + "#" * 60)
    print("Comparison")
    print("#" * 60)
    print(f"  Step 1 RMS: {r1['rms_avg']*100:.2f} cm")
    print(f"  Step 2 RMS: {r2['rms_avg']*100:.2f} cm")
    print(f"  Step 2 MPC strict conv: {r2['conv_rate']*100:.1f}%")
    print(f"  Step 2 MPC usable (EE): {r2['usable_rate']*100:.1f}%")
    if r2["rms_avg"] < r1["rms_avg"]:
        print("  => Step 2 beats Step 1 on this run")
    else:
        print("  => Step 1 still better; MPC needs more tuning")
    print("#" * 60)


if __name__ == "__main__":
    main()