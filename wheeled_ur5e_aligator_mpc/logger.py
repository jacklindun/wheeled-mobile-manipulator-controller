"""
Data logger and result plotter for wheeled UR5e MPC demo.

Logs per control cycle:
  time, q, u, q_des, ee_pos, ee_ref, ee_error,
  base, base_ref, base_z, base_z_ref,
  solve_time, solver_status, mpc_success, fallback,
  aligator_iter, aligator_cost

Saves to logs/latest.npz and generates 3 figures.
"""

import os
from pathlib import Path
from typing import Any

import numpy as np


class MPCLogger:
    """Accumulates per-step data and saves/plots results."""

    def __init__(self, log_dir: str = "logs"):
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._data: dict[str, list] = {
            "time": [],
            "q": [],
            "u": [],
            "q_des": [],
            "ee_pos": [],
            "ee_ref": [],
            "base": [],
            "base_ref": [],
            "base_z": [],
            "base_z_ref": [],
            "solve_time": [],
            "solver_status": [],
            "mpc_success": [],
            "fallback": [],
            "aligator_iter": [],
            "aligator_cost": [],
        }

    def log(
        self,
        *,
        t: float,
        q: np.ndarray,
        u: np.ndarray,
        q_des: np.ndarray,
        ee_pos: np.ndarray,
        ee_ref: np.ndarray,
        base: np.ndarray,
        base_ref: np.ndarray,
        base_z: float,
        base_z_ref: float,
        solve_time: float,
        solver_status: str,
        mpc_success: bool,
        fallback: bool,
        aligator_iter: int,
        aligator_cost: float | None,
    ) -> None:
        """Append one timestep of data."""
        self._data["time"].append(t)
        self._data["q"].append(np.array(q))
        self._data["u"].append(np.array(u))
        self._data["q_des"].append(np.array(q_des))
        self._data["ee_pos"].append(np.array(ee_pos))
        self._data["ee_ref"].append(np.array(ee_ref))
        ee_err = float(np.linalg.norm(ee_pos - ee_ref))
        self._data["base"].append(np.array(base))
        self._data["base_ref"].append(np.array(base_ref))
        self._data["base_z"].append(float(base_z))
        self._data["base_z_ref"].append(float(base_z_ref))
        self._data["solve_time"].append(float(solve_time))
        self._data["solver_status"].append(str(solver_status))
        self._data["mpc_success"].append(bool(mpc_success))
        self._data["fallback"].append(bool(fallback))
        self._data["aligator_iter"].append(int(aligator_iter))
        self._data["aligator_cost"].append(
            float(aligator_cost) if aligator_cost is not None else float("nan")
        )

    def save(self, filename: str = "latest.npz") -> Path:
        """Save data to npz file."""
        out_path = self._log_dir / filename
        save_dict: dict[str, Any] = {}
        for key, val in self._data.items():
            if key == "solver_status":
                save_dict[key] = np.array(val, dtype=object)
            elif key in ("mpc_success", "fallback"):
                save_dict[key] = np.array(val, dtype=bool)
            elif key in ("time", "base_z", "base_z_ref", "solve_time",
                         "aligator_iter", "aligator_cost"):
                save_dict[key] = np.array(val, dtype=float)
            else:
                save_dict[key] = np.array(val)
        np.savez(str(out_path), **save_dict)
        return out_path

    def plot(self, prefix: str = "latest") -> list[Path]:
        """Generate and save tracking, control, and MPC-time plots."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = np.array(self._data["time"])
        ee_pos = np.array(self._data["ee_pos"])
        ee_ref = np.array(self._data["ee_ref"])
        ee_err = np.linalg.norm(ee_pos - ee_ref, axis=1)
        u = np.array(self._data["u"])
        solve_times = np.array(self._data["solve_time"]) * 1e3  # ms
        mpc_success = np.array(self._data["mpc_success"])
        fallback = np.array(self._data["fallback"])

        rms_err = float(np.sqrt(np.mean(ee_err ** 2)))
        max_err = float(np.max(ee_err))

        saved = []

        # --- Tracking plot ---
        fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
        fig.suptitle(f"EE Tracking  RMS={rms_err*100:.2f} cm  Max={max_err*100:.2f} cm")
        labels = ["x (m)", "y (m)", "z (m)"]
        for i, ax in enumerate(axes[:3]):
            ax.plot(t, ee_pos[:, i], label="actual", linewidth=1.5)
            ax.plot(t, ee_ref[:, i], label="ref", linestyle="--", linewidth=1.5)
            ax.set_ylabel(labels[i])
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)
        axes[3].plot(t, ee_err * 100, color="red", linewidth=1.5)
        axes[3].set_ylabel("error (cm)")
        axes[3].set_xlabel("time (s)")
        axes[3].grid(True, alpha=0.3)
        tracking_path = self._log_dir / f"{prefix}_tracking.png"
        fig.savefig(str(tracking_path), dpi=100, bbox_inches="tight")
        plt.close(fig)
        saved.append(tracking_path)

        # --- Control plot ---
        fig, axes = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
        fig.suptitle("Control Inputs (velocity commands)")
        ctrl_labels = [
            "base_vx (m/s)", "base_vy (m/s)", "base_vz (m/s)", "base_omega (r/s)",
            "arm qd (r/s)"
        ]
        for i in range(4):
            axes[i].plot(t, u[:, i], linewidth=1.5)
            axes[i].set_ylabel(ctrl_labels[i])
            axes[i].grid(True, alpha=0.3)
        # All 6 arm joints on one plot
        arm_labels = ["pan", "lift", "elbow", "w1", "w2", "w3"]
        for j in range(6):
            axes[4].plot(t, u[:, 4 + j], label=arm_labels[j], linewidth=1.0)
        axes[4].set_ylabel(ctrl_labels[4])
        axes[4].legend(loc="upper right", fontsize=7)
        axes[4].set_xlabel("time (s)")
        axes[4].grid(True, alpha=0.3)
        controls_path = self._log_dir / f"{prefix}_controls.png"
        fig.savefig(str(controls_path), dpi=100, bbox_inches="tight")
        plt.close(fig)
        saved.append(controls_path)

        # --- MPC timing plot ---
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(t, solve_times, color="blue", linewidth=1.0, label="solve time")
        ax.axhline(np.mean(solve_times), color="green", linestyle="--",
                   label=f"avg={np.mean(solve_times):.1f} ms")
        ax.axhline(np.max(solve_times), color="red", linestyle=":",
                   label=f"max={np.max(solve_times):.1f} ms")
        # Mark fallback steps
        fb_idx = np.where(fallback)[0]
        if len(fb_idx):
            ax.scatter(t[fb_idx], solve_times[fb_idx], color="red", s=20, zorder=5,
                       label="fallback")
        ax.set_ylabel("solve time (ms)")
        ax.set_xlabel("time (s)")
        ax.set_title("ALIGATOR ProxDDP Solve Time per MPC Cycle")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        mpc_time_path = self._log_dir / f"{prefix}_mpc_time.png"
        fig.savefig(str(mpc_time_path), dpi=100, bbox_inches="tight")
        plt.close(fig)
        saved.append(mpc_time_path)

        return saved

    def summary(self) -> dict:
        """Compute summary statistics for terminal output."""
        t = np.array(self._data["time"])
        ee_pos = np.array(self._data["ee_pos"])
        ee_ref = np.array(self._data["ee_ref"])
        ee_err = np.linalg.norm(ee_pos - ee_ref, axis=1)
        solve_times = np.array(self._data["solve_time"]) * 1e3
        mpc_success = np.array(self._data["mpc_success"])
        fallback = np.array(self._data["fallback"])
        u = np.array(self._data["u"])

        n = len(t)
        q = np.array(self._data["q"])

        # Joint limit violation check
        from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
        robot = WheeledUR5eModel()
        limit_viol = np.any(q < robot.q_min - 0.01) or np.any(q > robot.q_max + 0.01)

        return {
            "n_steps": n,
            "mpc_success_rate": float(np.mean(mpc_success)) * 100,
            "fallback_rate": float(np.mean(fallback)) * 100,
            "avg_solve_time_ms": float(np.mean(solve_times)),
            "max_solve_time_ms": float(np.max(solve_times)),
            "ee_rms_error_m": float(np.sqrt(np.mean(ee_err ** 2))),
            "ee_max_error_m": float(np.max(ee_err)),
            "joint_limit_violation": bool(limit_viol),
        }
