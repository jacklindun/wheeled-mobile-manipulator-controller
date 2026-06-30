"""Shared helpers for Phase 6-v3 scripts and tests."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from wheeled_ur5e_aligator_mpc.coordinate_mapping import DUAL_ARM_Q_NOMINAL, DUAL_ARM_TAU_MAX_Q
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, FeedforwardPDGains

MJCF_TORQUE = "assets/wheeled_dual_ur5e_v2_torque.xml"
MJCF_PIN = "assets/wheeled_dual_ur5e_v2.xml"

MPC_DT = 0.05
CONTROL_DT = 0.002
INTERPOLATION_RATIO = int(MPC_DT / CONTROL_DT)


class FixedBaseIKPlanner:
    """Fixed-base dual-arm IK (base DOFs frozen)."""

    def __init__(self, pinocchio_model: DualArmPinocchioModel):
        self.model = pinocchio_model
        self.q_nominal = DUAL_ARM_Q_NOMINAL.copy()

    def solve_ik_fixed_base(
        self,
        target_left: np.ndarray,
        target_right: np.ndarray,
        max_iter: int = 100,
    ) -> np.ndarray:
        q_ik = self.q_nominal.copy()

        for _ in range(max_iter):
            error_left = target_left - self.model.fk_left_ee(q_ik)
            error_right = target_right - self.model.fk_right_ee(q_ik)
            total_error = np.linalg.norm(error_left) + np.linalg.norm(error_right)
            if total_error < 1e-4:
                break

            J_left = self.model.jacobian_left_ee(q_ik)[:3, :]
            J_left[:, 0:4] = 0
            J_right = self.model.jacobian_right_ee(q_ik)[:3, :]
            J_right[:, 0:4] = 0

            dq = np.linalg.lstsq(
                np.vstack([J_left, J_right]),
                np.concatenate([error_left, error_right]),
                rcond=None,
            )[0]
            dq[0:4] = 0
            q_ik += 0.3 * dq

        return q_ik


class JointInterpolator:
    """Linear q/v interpolation over one MPC interval."""

    def __init__(self, ratio: int = INTERPOLATION_RATIO, mpc_dt: float = MPC_DT, control_dt: float = CONTROL_DT):
        self.ratio = ratio
        self.segment_dt = ratio * control_dt
        self.q_start = None
        self.q_end = None

    def set_segment(self, q_start: np.ndarray, q_end: np.ndarray):
        self.q_start = q_start.copy()
        self.q_end = q_end.copy()

    def interpolate(self, step: int) -> tuple[np.ndarray, np.ndarray]:
        alpha = step / self.ratio
        q_des = (1 - alpha) * self.q_start + alpha * self.q_end
        v_des = (self.q_end - self.q_start) / self.segment_dt
        return q_des, v_des


class MpcSegmentInterpolator:
    """Interpolate q/v from MPC xs[0:2]; hold tau_ff = us[0]."""

    def __init__(self, ratio: int = INTERPOLATION_RATIO):
        self.ratio = ratio
        self.x0 = None
        self.x1 = None
        self.u0 = None

    def set_segment(self, xs: np.ndarray, us: np.ndarray):
        self.x0 = xs[0]
        self.x1 = xs[1] if len(xs) > 1 else xs[0]
        self.u0 = us[0]

    def interpolate(self, step: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        alpha = step / self.ratio
        x_interp = (1 - alpha) * self.x0 + alpha * self.x1
        return x_interp[:16], x_interp[16:], self.u0


def circle_trajectory(
    t: float,
    *,
    radius: float = 0.08,
    omega: float = 0.5,
    center_left: np.ndarray | None = None,
    center_right: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    if center_left is None:
        center_left = np.array([0.600, 0.300, 0.800])
    if center_right is None:
        center_right = np.array([0.600, -0.300, 0.800])

    angle = omega * t
    offset = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])
    return center_left + offset, center_right + offset


def compute_gravity_torque(pin_model: DualArmPinocchioModel, q: np.ndarray) -> np.ndarray:
    """Generalized gravity torque τ_g(q) for gravity feedforward."""
    pin.computeGeneralizedGravity(pin_model.model, pin_model.data, q)
    return pin_model.data.g.copy()


def generate_ik_state_reference(
    ik_planner: FixedBaseIKPlanner,
    target_left_traj: np.ndarray,
    target_right_traj: np.ndarray,
    dt: float,
) -> np.ndarray:
    """
    Build (N+1, 32) state reference [q_ik, v_ik] from EE target trajectories.
    Velocities from central finite differences on IK joint solutions.
    """
    n = len(target_left_traj)
    q_refs = np.zeros((n, 16))
    for k in range(n):
        q_refs[k] = ik_planner.solve_ik_fixed_base(target_left_traj[k], target_right_traj[k])

    v_refs = np.zeros((n, 16))
    for k in range(1, n - 1):
        v_refs[k] = (q_refs[k + 1] - q_refs[k - 1]) / (2.0 * dt)
    if n > 1:
        v_refs[0] = (q_refs[1] - q_refs[0]) / dt
        v_refs[-1] = (q_refs[-1] - q_refs[-2]) / dt

    return np.hstack([q_refs, v_refs])


def generate_ee_reference_trajectory(
    t_start: float,
    horizon: int,
    dt: float,
    omega: float = 0.5,
    radius: float = 0.08,
) -> tuple[np.ndarray, np.ndarray]:
    target_left_traj = np.zeros((horizon + 1, 3))
    target_right_traj = np.zeros((horizon + 1, 3))
    for k in range(horizon + 1):
        tl, tr = circle_trajectory(t_start + k * dt, omega=omega, radius=radius)
        target_left_traj[k] = tl
        target_right_traj[k] = tr
    return target_left_traj, target_right_traj


def make_pd_controller(
    *,
    Kp_arm: float = 500.0,
    Kd_arm: float = 50.0,
) -> FeedforwardPDController:
    gains = FeedforwardPDGains(
        Kp_base_xy=200.0, Kd_base_xy=50.0,
        Kp_base_z=1000.0, Kd_base_z=200.0,
        Kp_base_yaw=100.0, Kd_base_yaw=20.0,
        Kp_arm=Kp_arm, Kd_arm=Kd_arm,
    )
    controller = FeedforwardPDController(gains)
    controller.set_control_limits(-DUAL_ARM_TAU_MAX_Q, DUAL_ARM_TAU_MAX_Q)
    return controller


def is_mpc_solution_usable(
    pin_model: DualArmPinocchioModel,
    xs: np.ndarray,
    target_left: np.ndarray,
    target_right: np.ndarray,
    stage: int = 1,
    ee_threshold: float = 0.02,
) -> bool:
    """Accept MPC output when predicted EE error is small, even if solver conv=False."""
    el, er = ee_tracking_errors(pin_model, xs[stage, :16], target_left, target_right)
    return (el + er) < ee_threshold


def ee_tracking_errors(
    pin_model: DualArmPinocchioModel,
    q: np.ndarray,
    target_left: np.ndarray,
    target_right: np.ndarray,
) -> tuple[float, float]:
    el = np.linalg.norm(pin_model.fk_left_ee(q) - target_left)
    er = np.linalg.norm(pin_model.fk_right_ee(q) - target_right)
    return el, er