#!/usr/bin/env python
"""Regenerate plots from a saved logs/latest.npz file."""

import sys
import argparse
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("npz_path", default=str(_project_root / "logs" / "latest.npz"),
                   nargs="?", help="Path to .npz log file")
    args = p.parse_args()

    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = dict(np.load(args.npz_path, allow_pickle=True))
    t = data["time"]
    ee_pos = data["ee_pos"]
    ee_ref = data["ee_ref"]
    ee_err = np.linalg.norm(ee_pos - ee_ref, axis=1)
    u = data["u"]
    solve_times = data["solve_time"] * 1e3
    fallback = data["fallback"].astype(bool)
    rms_err = float(np.sqrt(np.mean(ee_err ** 2)))
    max_err = float(np.max(ee_err))

    out_dir = Path(args.npz_path).parent

    # Tracking
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(f"EE Tracking  RMS={rms_err*100:.2f}cm  Max={max_err*100:.2f}cm")
    for i, lbl in enumerate(["x (m)", "y (m)", "z (m)"]):
        axes[i].plot(t, ee_pos[:, i], label="actual")
        axes[i].plot(t, ee_ref[:, i], "--", label="ref")
        axes[i].set_ylabel(lbl); axes[i].legend(fontsize=8); axes[i].grid(True, alpha=0.3)
    axes[3].plot(t, ee_err * 100, color="red")
    axes[3].set_ylabel("error (cm)"); axes[3].set_xlabel("time (s)"); axes[3].grid(True, alpha=0.3)
    p_track = out_dir / "latest_tracking.png"
    fig.savefig(str(p_track), dpi=100, bbox_inches="tight"); plt.close(fig)

    # Controls
    fig, axes = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
    fig.suptitle("Control Inputs")
    for i, lbl in enumerate(["base_vx", "base_vy", "base_vz", "omega"]):
        axes[i].plot(t, u[:, i]); axes[i].set_ylabel(lbl); axes[i].grid(True, alpha=0.3)
    for j, n in enumerate(["pan","lift","elbow","w1","w2","w3"]):
        axes[4].plot(t, u[:, 4+j], label=n)
    axes[4].set_ylabel("arm qd"); axes[4].legend(fontsize=7)
    axes[4].set_xlabel("time (s)"); axes[4].grid(True, alpha=0.3)
    p_ctrl = out_dir / "latest_controls.png"
    fig.savefig(str(p_ctrl), dpi=100, bbox_inches="tight"); plt.close(fig)

    # MPC timing
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, solve_times, color="blue")
    ax.axhline(np.mean(solve_times), color="green", linestyle="--",
               label=f"avg={np.mean(solve_times):.1f}ms")
    ax.axhline(np.max(solve_times), color="red", linestyle=":",
               label=f"max={np.max(solve_times):.1f}ms")
    if np.any(fallback):
        ax.scatter(t[fallback], solve_times[fallback], color="red", s=20, zorder=5,
                   label="fallback")
    ax.set_title("ALIGATOR ProxDDP Solve Time"); ax.legend(fontsize=8)
    ax.set_ylabel("ms"); ax.set_xlabel("time (s)"); ax.grid(True, alpha=0.3)
    p_time = out_dir / "latest_mpc_time.png"
    fig.savefig(str(p_time), dpi=100, bbox_inches="tight"); plt.close(fig)

    print(f"Saved: {p_track}")
    print(f"Saved: {p_ctrl}")
    print(f"Saved: {p_time}")


if __name__ == "__main__":
    main()
