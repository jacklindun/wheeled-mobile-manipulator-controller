"""
Reference trajectory generators for wheeled UR5e MPC scenarios.

Returns ref_traj dict:
  "ee_pos"  : (N+1, 3)    end-effector position reference
  "ee_rot"  : (N+1, 3, 3) end-effector rotation matrix reference
  "base"    : (N+1, 3)    columns: base_x, base_y, base_yaw
  "base_z"  : (N+1,)      base_z (lift height) reference

The ee_circle scenario places the circle so that at t=0 the reference
starts exactly at FK(q_nominal), i.e. the robot begins tracking from rest.

Default orientation (ee_rot): the FK(q_nominal) orientation, held constant
throughout the trajectory. Orientation tracking is only active if the MPC
cost has w_orientation > 0.
"""

import numpy as np


class ReferenceGenerator:
    """
    Generates MPC reference trajectories for different scenarios.

    Scenarios:
      "ee_circle"    : EE traces a circle in world frame, base stationary
      "ee_line"      : EE moves from A to B, base stationary
      "base_and_ee"  : base moves forward while EE holds a world target
      "base_z_test"  : tests the lift DOF with sinusoidal base_z motion

    Parameters
    ----------
    scenario : str
    ee_start : (3,) optional EE position at t=0.
        Used to compute the circle/line starting point so the robot
        begins with zero tracking error.  If None, uses the default
        nominal (FK at q_nominal).
    ee_start_rot : (3, 3) optional EE rotation matrix at t=0.
        Used as the reference orientation (held constant). If None, uses
        the FK(q_nominal) orientation.
    """

    # Default start = FK(q_nominal) with shoulder_pan=pi, lift=pi/3, elbow=-pi/2, w1=pi/6
    _DEFAULT_EE_START = np.array([0.61880516, 0.0635, 0.85700198])
    _DEFAULT_EE_ROT = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])  # R(q_nominal): tool pointing +X, flange upright

    def __init__(self, scenario: str = "ee_circle", ee_start=None, ee_start_rot=None):
        if scenario not in ("ee_circle", "ee_line", "base_and_ee", "base_z_test"):
            raise ValueError(f"Unknown scenario: {scenario}")
        self.scenario = scenario
        self.ee_start = (
            np.asarray(ee_start, dtype=float) if ee_start is not None
            else self._DEFAULT_EE_START.copy()
        )
        self.ee_start_rot = (
            np.asarray(ee_start_rot, dtype=float) if ee_start_rot is not None
            else self._DEFAULT_EE_ROT.copy()
        )

    def get_reference(
        self,
        t: float,
        horizon: int,
        dt: float,
    ) -> dict:
        """
        Return reference trajectory starting at time t.

        Parameters
        ----------
        t       : current time (s)
        horizon : MPC horizon N
        dt      : MPC timestep

        Returns
        -------
        dict with "ee_pos" (N+1,3), "ee_rot" (N+1,3,3), "base" (N+1,3), "base_z" (N+1,)
        """
        ts = t + np.arange(horizon + 1) * dt
        ee_pos = np.zeros((horizon + 1, 3))
        ee_rot = np.tile(self.ee_start_rot, (horizon + 1, 1, 1))  # constant orientation
        base   = np.zeros((horizon + 1, 3))  # base_x, base_y, base_yaw
        base_z = np.zeros(horizon + 1)

        if self.scenario == "ee_circle":
            self._ee_circle(ts, ee_pos, base, base_z, self.ee_start)
        elif self.scenario == "ee_line":
            self._ee_line(ts, ee_pos, base, base_z, self.ee_start)
        elif self.scenario == "base_and_ee":
            self._base_and_ee(ts, ee_pos, base, base_z, self.ee_start)
        elif self.scenario == "base_z_test":
            self._base_z_test(ts, ee_pos, base, base_z, self.ee_start)

        return {"ee_pos": ee_pos, "ee_rot": ee_rot, "base": base, "base_z": base_z}

    # ------------------------------------------------------------------
    # Scenario implementations
    # ------------------------------------------------------------------

    @staticmethod
    def _ee_circle(ts, ee_pos, base, base_z, ee_start):
        """
        EE traces a circle in the Y-Z plane.
        Circle starts at ee_start at t=0, so initial tracking error = 0.
        Center = ee_start - [0, radius, 0], so the circle goes right from start.
        """
        radius = 0.10
        period = 10.0
        # Center placed so that at theta=0: y = cy + r = ee_start_y, z = cz = ee_start_z
        cy = float(ee_start[1]) - radius
        cz = float(ee_start[2])
        cx = float(ee_start[0])
        for i, t in enumerate(ts):
            theta = 2 * np.pi * t / period
            ee_pos[i] = [
                cx,
                cy + radius * np.cos(theta),
                cz + radius * np.sin(theta),
            ]
        base[:, :] = [0.0, 0.0, 0.0]
        base_z[:] = 0.2

    @staticmethod
    def _ee_line(ts, ee_pos, base, base_z, ee_start):
        """EE moves from ee_start to ee_start+delta over 8 s, then holds."""
        delta = np.array([0.0, 0.20, 0.10])
        A = ee_start.copy()
        B = ee_start + delta
        duration = 8.0
        for i, t in enumerate(ts):
            s = np.clip(t / duration, 0.0, 1.0)
            s = s * s * (3.0 - 2.0 * s)
            ee_pos[i] = A + s * (B - A)
        base[:, :] = [0.0, 0.0, 0.0]
        base_z[:] = 0.2

    @staticmethod
    def _base_and_ee(ts, ee_pos, base, base_z, ee_start):
        """Base moves from x=0 to x=0.8 over 20s, EE holds world position."""
        total_duration = 20.0
        for i, t in enumerate(ts):
            s = np.clip(t / total_duration, 0.0, 1.0)
            s = s * s * (3.0 - 2.0 * s)
            base[i] = [s * 0.8, 0.0, 0.0]
            base_z[i] = 0.2 + 0.08 * np.sin(2 * np.pi * t / 10.0)
            ee_pos[i] = ee_start.copy()

    @staticmethod
    def _base_z_test(ts, ee_pos, base, base_z, ee_start):
        """Tests lift DOF: base_z oscillates 0.2±0.12 m, EE holds world position."""
        for i, t in enumerate(ts):
            base[i] = [0.0, 0.0, 0.0]
            base_z[i] = 0.2 + 0.12 * np.sin(2 * np.pi * t / 8.0)
            ee_pos[i] = ee_start.copy()


