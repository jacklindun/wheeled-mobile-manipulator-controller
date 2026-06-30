"""
End-effector 6D pose cost for ALIGATOR: position + orientation (SO(3)).

This is a custom CostAbstract subclass that tracks both the EE position and
orientation. The residual is 6-dimensional:
  r = [p - p_ref; log3(R_ref^T @ R)]

where log3: SO(3) -> so(3) is the log map (Rodrigues rotation vector).

The Gauss-Newton Hessian approximation gives:
  Lxx ≈ w_p * J_p^T @ J_p  +  w_o * J_o^T @ J_o
where J_p (3×10) and J_o (3×10) are the position and angular parts of the
geometric Jacobian (LOCAL_WORLD_ALIGNED frame).
"""

import sys
import os
from pathlib import Path

import numpy as np

try:
    import aligator
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [
        str(_repo_root / "build" / "bindings" / "python"),
        str(_repo_root / "bindings" / "python"),
    ]
    import aligator

try:
    import pinocchio as pin
except ImportError:
    print("pinocchio not found. Ensure 'all' pixi environment is active.", file=sys.stderr)
    raise

from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel

# Global shared robot instance to avoid deepcopy of Pinocchio C++ objects
_SHARED_ROBOT_INSTANCE = None

def _get_shared_robot():
    """Get or create shared PinocchioWheeledUR5eModel instance."""
    global _SHARED_ROBOT_INSTANCE
    if _SHARED_ROBOT_INSTANCE is None:
        _SHARED_ROBOT_INSTANCE = PinocchioWheeledUR5eModel()
    return _SHARED_ROBOT_INSTANCE


class EEPoseCost(aligator.CostAbstract):
    """
    End-effector 6D pose tracking cost (position + orientation).

    cost = 0.5 * w_p * ||p - p_ref||^2  +  0.5 * w_o * ||log3(R_ref^T @ R)||^2

    Gradient (Gauss-Newton):
      Lx = w_p * J_p^T @ e_p  +  w_o * (R^T @ J_o)^T @ e_o
      Lu = 0
    where
      e_p = p - p_ref
      e_o = log3(R_ref^T @ R)
      J_p, J_o: geometric Jacobian (LOCAL_WORLD_ALIGNED frame), 3×10 each

    Hessian (Gauss-Newton approximation):
      Lxx = w_p * J_p^T @ J_p  +  w_o * J_o_local^T @ J_o_local
      Lxu = 0, Luu = 0
    where
      J_o_local = R^T @ J_o  (orient Jacobian in local EE frame)

    The frame residual is expressed in the local EE frame (error coordinates)
    for the orientation part, which makes the Gauss-Newton approximation
    locally quadratic near convergence.
    """

    def __init__(
        self,
        space_or_dim,
        nu: int,
        robot: PinocchioWheeledUR5eModel | None,
        w_position: float,
        w_orientation: float,
        p_ref: np.ndarray | None = None,
        R_ref: np.ndarray | None = None,
    ):
        """
        Parameters
        ----------
        space_or_dim : VectorSpace | int
            State manifold (VectorSpace(10) for the kinematic model).
        nu : int
            Control dimension (10).
        robot : PinocchioWheeledUR5eModel | None
            Pinocchio robot model for FK and Jacobian.
            If None, uses shared global instance (for deepcopy compatibility).
        w_position : float
            Position tracking weight.
        w_orientation : float
            Orientation tracking weight.
        p_ref : (3,) ndarray | None
            Reference position. Default: zeros.
        R_ref : (3,3) ndarray | None
            Reference rotation matrix. Default: identity.
        """
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
        else:
            space = space_or_dim
        super().__init__(space, nu)
        self._space_nx = space.nx
        # Use shared instance if robot is None (during deepcopy reconstruction)
        self._robot = robot if robot is not None else _get_shared_robot()
        self._w_p = float(w_position)
        self._w_o = float(w_orientation)
        self._p_ref = np.array(p_ref) if p_ref is not None else np.zeros(3)
        self._R_ref = np.array(R_ref) if R_ref is not None else np.eye(3)

    def __reduce__(self):
        """Support deepcopy (called by CostStack.addCost internally).

        Don't pass robot object directly to avoid deepcopy of Pinocchio C++ objects.
        Instead, use None as placeholder and reconstruct will use shared instance.
        """
        return (
            self.__class__,
            (
                self._space_nx,
                self.nu,
                None,  # robot will be reconstructed from shared instance
                self._w_p,
                self._w_o,
                self._p_ref.copy(),
                self._R_ref.copy(),
            ),
        )

    def set_reference(self, p_ref: np.ndarray, R_ref: np.ndarray) -> None:
        """Update target EE pose."""
        self._p_ref[:] = p_ref
        self._R_ref[:] = R_ref

    def set_position_reference(self, p_ref: np.ndarray) -> None:
        """Update target EE position only (keep orientation unchanged)."""
        self._p_ref[:] = p_ref

    def evaluate(self, x, u, data) -> None:
        q = np.asarray(x)
        p, R = self._robot.fk_pose(q)
        e_p = p - self._p_ref
        # Orientation error in local frame: log3(R_ref^T @ R)
        e_o = pin.log3(self._R_ref.T @ R)
        data.value = 0.5 * self._w_p * float(np.dot(e_p, e_p)) + \
                     0.5 * self._w_o * float(np.dot(e_o, e_o))

    def computeGradients(self, x, u, data) -> None:
        q = np.asarray(x)
        p, R = self._robot.fk_pose(q)
        e_p = p - self._p_ref
        e_o = pin.log3(self._R_ref.T @ R)

        # Geometric Jacobian (LOCAL_WORLD_ALIGNED): J = [J_p; J_o] (6×10)
        J = self._robot.frame_jacobian(q, local=False)  # world-aligned
        J_p = J[:3, :]  # position part
        J_o = J[3:, :]  # angular part (world-aligned axes at EE)

        # For orientation: express Jacobian in local EE frame (R^T @ J_o)
        # so the gradient is J_o_local^T @ e_o, and e_o = log3(R_ref^T @ R)
        # is already in the local frame coordinates.
        J_o_local = R.T @ J_o  # (3, 10)

        data.Lx[:] = self._w_p * (J_p.T @ e_p) + self._w_o * (J_o_local.T @ e_o)
        data.Lu[:] = 0.0

    def computeHessians(self, x, u, data) -> None:
        q = np.asarray(x)
        _, R = self._robot.fk_pose(q)
        J = self._robot.frame_jacobian(q, local=False)
        J_p = J[:3, :]
        J_o = J[3:, :]
        J_o_local = R.T @ J_o

        # Gauss-Newton: Lxx ≈ J^T @ W @ J (drop second-order terms)
        data.Lxx[:] = self._w_p * (J_p.T @ J_p) + self._w_o * (J_o_local.T @ J_o_local)
        data.Luu[:] = 0.0
        data.Lxu[:] = 0.0
