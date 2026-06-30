"""
Hybrid kino-dynamic dynamics for wheeled UR5e (Phase 4).

State representation: x = [q_base(4), q_arm(6), v_arm(6)] = 16-dim
  - q_base: [base_x, base_y, base_z, base_yaw] (position-level)
  - q_arm:  [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
  - v_arm:  joint velocities of the arm

Control: u = [v_base(4), tau_arm(6)] = 10-dim
  - v_base: [vx_body, vy_body, vz, omega_yaw] (body-frame velocities)
  - tau_arm: joint torques

Dynamics:
  - Base: kinematic integration (same as Phase 1-3)
      q_base_next = integrate_base_kinematics(q_base, v_base, dt)
  - Arm: semi-implicit Euler with ABA
      a_arm = aba(q_arm, v_arm, tau_arm)  via reduced 6-DOF model
      v_arm_next = v_arm + dt * a_arm
      q_arm_next = q_arm + dt * v_arm_next

This "hybrid" approach keeps the base velocity-controlled (as mobile bases
usually are in hardware) while upgrading the arm to full rigid-body dynamics
with torque control.
"""

import sys
import os
from pathlib import Path

import numpy as np

try:
    import aligator
    import pinocchio as pin
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [
        str(_repo_root / "build" / "bindings" / "python"),
        str(_repo_root / "bindings" / "python"),
    ]
    import aligator
    import pinocchio as pin

from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel


class HybridWheeledUR5eDynamics(aligator.dynamics.ExplicitDynamicsModel):
    """
    Hybrid kino-dynamic dynamics: kinematic base + arm ABA.

    State: x = [q_base(4), q_arm(6), v_arm(6)] = 16
    Control: u = [v_base(4), tau_arm(6)] = 10

    Base kinematics matches robot_model.py's integrate_step() for the base
    part (body-frame velocity → world position/yaw).
    """

    def __init__(self, pin_robot: PinocchioWheeledUR5eModel, dt: float):
        """
        Parameters
        ----------
        pin_robot : PinocchioWheeledUR5eModel
            Provides the reduced 6-DOF arm model for ABA.
        dt : float
            Integration timestep (semi-implicit Euler).
        """
        nx = 16  # state dimension
        nu = 10  # control dimension
        ndx = 16  # tangent space dimension (Euclidean)
        space = aligator.manifolds.VectorSpace(nx)
        super().__init__(space, nu)

        self._pin_robot = pin_robot
        self._dt = dt

        # Reduced 6-DOF arm model for ABA
        self._arm_model = pin_robot.arm_model
        self._arm_data = pin_robot.arm_data

        # Joint damping from MuJoCo XML (ur5e_kinematics.xml)
        # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        self._damping = np.array([1.0, 1.0, 0.5, 0.1, 0.1, 0.1])

        # Joint armature (rotor inertia) from MuJoCo XML
        # This is CRITICAL: armature adds to the effective inertia matrix
        self._armature = np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])

    def __reduce__(self):
        """Support deepcopy (ALIGATOR internally deepcopies dynamics objects)."""
        return (
            self.__class__,
            (self._pin_robot, self._dt),
        )

    def forward(self, x, u, data) -> None:
        """
        Compute x_next = f(x, u).

        x = [q_base(4), q_arm(6), v_arm(6)]
        u = [v_base(4), tau_arm(6)]
        """
        q_base = np.asarray(x[:4])
        q_arm = np.asarray(x[4:10])
        v_arm = np.asarray(x[10:16])

        v_base = np.asarray(u[:4])
        tau_arm = np.asarray(u[4:10])

        dt = self._dt

        # ============================================================
        # 1. Base kinematics (body-frame velocity → world position)
        # ============================================================
        base_x, base_y, base_z, base_yaw = q_base
        vx_body, vy_body, vz, omega_yaw = v_base

        # Rotate body-frame velocity to world frame
        c = np.cos(base_yaw)
        s = np.sin(base_yaw)
        vx_world = c * vx_body - s * vy_body
        vy_world = s * vx_body + c * vy_body

        # Integrate
        q_base_next = np.array([
            base_x + dt * vx_world,
            base_y + dt * vy_world,
            base_z + dt * vz,
            base_yaw + dt * omega_yaw,
        ])

        # Wrap yaw to [-pi, pi]
        q_base_next[3] = np.arctan2(np.sin(q_base_next[3]), np.cos(q_base_next[3]))

        # ============================================================
        # 2. Arm dynamics: ABA + semi-implicit Euler
        # ============================================================
        # Apply joint damping (matches MuJoCo XML damping parameters)
        tau_damped = tau_arm - self._damping * v_arm

        # ABA with armature: The arm_model now includes armature in its
        # rotor inertia (set in pinocchio_model.py), so ABA automatically
        # accounts for it: a = (M + diag(armature))^{-1} (tau - C - g)
        # No manual correction needed!
        a_arm = pin.aba(self._arm_model, self._arm_data, q_arm, v_arm, tau_damped)

        # Semi-implicit Euler
        v_arm_next = v_arm + dt * a_arm
        q_arm_next = q_arm + dt * v_arm_next

        # ============================================================
        # Assemble x_next
        # ============================================================
        data.xnext[:4] = q_base_next
        data.xnext[4:10] = q_arm_next
        data.xnext[10:16] = v_arm_next

    def dForward(self, x, u, data) -> None:
        """
        Compute Jacobians Jx (16×16) and Ju (16×10).

        Structure:
          Jx = [ dq_base_next/dx,  dq_arm_next/dx,  dv_arm_next/dx  ]
          Ju = [ dq_base_next/du,  dq_arm_next/du,  dv_arm_next/du  ]

        Base block: analytical (matches robot_model.py linearization).
        Arm block: pinocchio computeABADerivatives().
        """
        q_base = np.asarray(x[:4])
        q_arm = np.asarray(x[4:10])
        v_arm = np.asarray(x[10:16])

        v_base = np.asarray(u[:4])
        tau_arm = np.asarray(u[4:10])

        dt = self._dt

        # Initialize Jacobians to zero
        Jx = data.Jx  # (16, 16)
        Ju = data.Ju  # (16, 10)
        Jx[:] = 0.0
        Ju[:] = 0.0

        # ============================================================
        # 1. Base kinematics Jacobians
        # ============================================================
        base_yaw = q_base[3]
        vx_body, vy_body = v_base[0], v_base[1]
        c = np.cos(base_yaw)
        s = np.sin(base_yaw)

        # dq_base_next / dq_base (4×4 block, top-left)
        Jx[0, 0] = 1.0  # base_x
        Jx[1, 1] = 1.0  # base_y
        Jx[2, 2] = 1.0  # base_z
        # base_x derivative wrt base_yaw
        Jx[0, 3] = dt * (-s * vx_body - c * vy_body)
        # base_y derivative wrt base_yaw
        Jx[1, 3] = dt * (c * vx_body - s * vy_body)
        # base_yaw wrapping: d/d(yaw) of arctan2 ≈ 1 near nominal
        Jx[3, 3] = 1.0

        # dq_base_next / du_base (4×4 block, top rows of Ju)
        Ju[0, 0] = dt * c   # base_x / vx_body
        Ju[0, 1] = -dt * s  # base_x / vy_body
        Ju[1, 0] = dt * s   # base_y / vx_body
        Ju[1, 1] = dt * c   # base_y / vy_body
        Ju[2, 2] = dt       # base_z / vz
        Ju[3, 3] = dt       # base_yaw / omega_yaw

        # ============================================================
        # 2. Arm dynamics Jacobians via pinocchio ABA derivatives
        # ============================================================
        # Apply joint damping (must match forward())
        tau_damped = tau_arm - self._damping * v_arm

        # Compute ABA derivatives. Since the arm_model now includes armature
        # in its rotor inertia, computeABADerivatives() automatically gives us
        # the correct derivatives for (M + A)^{-1} * f
        pin.computeABADerivatives(self._arm_model, self._arm_data, q_arm, v_arm, tau_damped)

        da_dq = self._arm_data.ddq_dq.copy()
        da_dv = self._arm_data.ddq_dv.copy()

        # da/dtau is just the inverse of the effective mass matrix
        # For efficiency, Pinocchio provides this via Minv after computeABADerivatives
        # But we need to handle damping: d(tau_damped)/dtau = I, d(tau_damped)/dv = -diag(damping)
        # So: da/dtau = da/d(tau_damped) = Minv (already includes armature effect)
        #     da/dv_total = da/dv + da/d(tau_damped) * d(tau_damped)/dv
        #                 = da/dv - Minv @ diag(damping)

        # Minv is stored in arm_data after computeABADerivatives
        # But for cleaner code, we'll compute it from CRBA
        pin.crba(self._arm_model, self._arm_data, q_arm)
        M_with_armature = self._arm_data.M.copy()  # Already includes armature!
        Minv = np.linalg.inv(M_with_armature)

        da_dtau = Minv

        # Account for damping in velocity derivatives
        da_dv_with_damping = da_dv - da_dtau @ np.diag(self._damping)

        # Semi-implicit Euler Jacobians:
        #   v_next = v + dt * a(q, v, tau)
        #   q_next = q + dt * v_next
        #
        # dv_next/dq = dt * da/dq
        # dv_next/dv = I + dt * da/dv
        # dv_next/dtau = dt * da/dtau
        #
        # dq_next/dq = I + dt * dv_next/dq = I + dt^2 * da/dq
        # dq_next/dv = dt * dv_next/dv = dt * (I + dt * da/dv)
        # dq_next/dtau = dt * dv_next/dtau = dt^2 * da/dtau

        I6 = np.eye(6)

        dv_next_dq = dt * da_dq                      # (6, 6)
        dv_next_dv = I6 + dt * da_dv_with_damping    # (6, 6) with damping
        dv_next_dtau = dt * da_dtau                  # (6, 6)

        dq_next_dq = I6 + dt * dv_next_dq            # (6, 6)
        dq_next_dv = dt * dv_next_dv                 # (6, 6)
        dq_next_dtau = dt * dv_next_dtau             # (6, 6)

        # Fill Jx blocks (state Jacobian)
        # Rows 4:10 = q_arm_next
        Jx[4:10, 4:10] = dq_next_dq          # dq_arm_next / dq_arm
        Jx[4:10, 10:16] = dq_next_dv         # dq_arm_next / dv_arm

        # Rows 10:16 = v_arm_next
        Jx[10:16, 4:10] = dv_next_dq         # dv_arm_next / dq_arm
        Jx[10:16, 10:16] = dv_next_dv        # dv_arm_next / dv_arm

        # Fill Ju blocks (control Jacobian)
        # Columns 4:10 = tau_arm
        Ju[4:10, 4:10] = dq_next_dtau        # dq_arm_next / dtau_arm
        Ju[10:16, 4:10] = dv_next_dtau       # dv_arm_next / dtau_arm
