#!/usr/bin/env python
"""
Run the wheeled UR5e ALIGATOR MPC demo.

Usage:
  python scripts/run_demo.py --scenario ee_circle --duration 30 --render
  python scripts/run_demo.py --scenario base_z_test --duration 30
  python scripts/run_demo.py --scenario base_and_ee --duration 30 --render
"""

import sys
import argparse
from pathlib import Path

# Ensure repo path is on sys.path when running without pip install
_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

# Add aligator build path (compiled from source, not pip-installed)
_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]


def parse_args():
    p = argparse.ArgumentParser(description="Wheeled UR5e ALIGATOR MPC Demo")
    p.add_argument("--scenario", default="ee_circle",
                   choices=["ee_circle", "ee_line", "base_and_ee", "base_z_test"],
                   help="Reference trajectory scenario")
    p.add_argument("--duration", type=float, default=30.0,
                   help="Simulation duration (s)")
    p.add_argument("--render", action="store_true",
                   help="Launch MuJoCo viewer")
    p.add_argument("--horizon", type=int, default=20,
                   help="MPC prediction horizon")
    p.add_argument("--mpc-dt", type=float, default=0.05,
                   help="MPC timestep (s)")
    p.add_argument("--sim-dt", type=float, default=0.002,
                   help="MuJoCo simulation timestep (s)")
    p.add_argument("--log-dir", default="logs",
                   help="Directory to save logs and figures")
    p.add_argument("--aligator-max-iters", type=int, default=10,
                   help="Max ALIGATOR ProxDDP iterations per MPC cycle")
    return p.parse_args()


def main():
    args = parse_args()

    xml_path = _project_root / "assets" / "wheeled_ur5e.xml"
    if not xml_path.exists():
        print(f"[ERROR] MJCF not found: {xml_path}")
        sys.exit(1)

    from wheeled_ur5e_aligator_mpc.demo import run_demo
    run_demo(
        xml_path=str(xml_path),
        scenario=args.scenario,
        duration=args.duration,
        render=args.render,
        horizon=args.horizon,
        mpc_dt=args.mpc_dt,
        sim_dt=args.sim_dt,
        log_dir=str(_project_root / args.log_dir),
        aligator_max_iters=args.aligator_max_iters,
    )


if __name__ == "__main__":
    main()
