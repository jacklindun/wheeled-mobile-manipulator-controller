#!/usr/bin/env python3
"""
Test EE orientation control (Phase 2).

Runs the same demo as run_demo.py, but with orientation control enabled.
Compare the behavior:
  - Without orientation control (ee_ori=0): EE orientation drifts freely
  - With orientation control (ee_ori=50): EE orientation stays fixed at nominal

Usage:
  python scripts/test_orientation.py --scenario ee_circle --duration 10 --render
  python scripts/test_orientation.py --scenario base_and_ee --duration 20 --render
"""

import sys
import os
import argparse
from pathlib import Path

# Add repo paths
_repo = Path(__file__).resolve().parents[3]
sys.path[:0] = [
    str(_repo / "build" / "bindings" / "python"),
    str(_repo / "study_example" / "wheeled_ur5e_aligator_mpc"),
]

from wheeled_ur5e_aligator_mpc.demo import run_demo


def main():
    parser = argparse.ArgumentParser(
        description="Test EE orientation control (Phase 2)"
    )
    parser.add_argument(
        "--scenario",
        choices=["ee_circle", "ee_line", "base_and_ee", "base_z_test"],
        default="ee_circle",
        help="Reference trajectory scenario",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Simulation duration in seconds",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Enable MuJoCo rendering (interactive viewer)",
    )
    parser.add_argument(
        "--ee-ori-weight",
        type=float,
        default=50.0,
        help="EE orientation tracking weight (default: 50.0)",
    )
    parser.add_argument(
        "--terminal-ori-weight",
        type=float,
        default=100.0,
        help="Terminal EE orientation weight (default: 100.0)",
    )

    args = parser.parse_args()

    # Custom weights with orientation control enabled
    weights = {
        "ee_ori": args.ee_ori_weight,
        "terminal_ee_ori": args.terminal_ori_weight,
    }

    print("=" * 60)
    print("EE ORIENTATION CONTROL TEST (Phase 2)")
    print("=" * 60)
    print(f"Scenario:              {args.scenario}")
    print(f"Duration:              {args.duration:.1f} s")
    print(f"Render:                {args.render}")
    print(f"EE ori weight:         {args.ee_ori_weight}")
    print(f"Terminal ori weight:   {args.terminal_ori_weight}")
    print("=" * 60)
    print()

    # Default XML path (kinematic MPC)
    xml_path = str(Path(__file__).resolve().parents[1] / "assets" / "wheeled_ur5e.xml")

    run_demo(
        scenario=args.scenario,
        duration=args.duration,
        render=args.render,
        xml_path=xml_path,
        weights=weights,
    )

    print()
    print("TIP: Compare with baseline (no orientation control):")
    print(f"  python scripts/run_demo.py --scenario {args.scenario} --duration {args.duration} --render")
    print()


if __name__ == "__main__":
    main()
