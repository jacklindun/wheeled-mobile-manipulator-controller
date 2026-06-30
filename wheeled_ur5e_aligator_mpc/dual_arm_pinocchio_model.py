"""
Pinocchio-based model backend for wheeled dual UR5e robot.

This module extends PinocchioWheeledUR5eModel to support dual arms:
  - 16 DOF: [base(4), left_arm(6), right_arm(6)]
  - Dual FK/Jacobian for independent left/right EE control
  - Foundation for dual-arm MPC and coordinated manipulation

Joint order (same as MuJoCo):
  [base_x, base_y, base_yaw, base_z,
   left_shoulder_pan, left_shoulder_lift, left_elbow, left_wrist_1, left_wrist_2, left_wrist_3,
   right_shoulder_pan, right_shoulder_lift, right_elbow, right_wrist_1, right_wrist_2, right_wrist_3]
"""

import sys
import os
from pathlib import Path
from typing import Tuple
import tempfile

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    print("pinocchio not found. Make sure you're in the 'all' pixi environment.", file=sys.stderr)
    raise

try:
    import mujoco
except ImportError:
    print("mujoco not found. Run 'pixi add --pypi mujoco' in the 'all' env.", file=sys.stderr)
    raise


class DualArmPinocchioModel:
    """
    Pinocchio-based robot model for the wheeled dual UR5e.

    Loads the dual-arm MJCF, compiles/flattens it via mujoco, strips the mocap target bodies,
    and builds the pinocchio model. Provides dual FK and Jacobians for both arms.

    Joint order:
      [base_x, base_y, base_yaw, base_z,
       left_arm(6), right_arm(6)]

    Total: 16 DOF
    """

    nq: int = 16
    nu: int = 16

    LEFT_EE_FRAME: str = "left_ee_site"
    RIGHT_EE_FRAME: str = "right_ee_site"

    # Joint names for indexing
    _BASE_JOINTS = ["base_x", "base_y", "base_yaw", "base_z"]
    _LEFT_ARM_JOINTS = [
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_elbow_joint",
        "left_wrist_1_joint",
        "left_wrist_2_joint",
        "left_wrist_3_joint",
    ]
    _RIGHT_ARM_JOINTS = [
        "right_shoulder_pan_joint",
        "right_shoulder_lift_joint",
        "right_elbow_joint",
        "right_wrist_1_joint",
        "right_wrist_2_joint",
        "right_wrist_3_joint",
    ]

    def __init__(self, mjcf_path: str | None = None):
        if mjcf_path is None:
            mjcf_path = str(
                Path(__file__).resolve().parents[1] / "assets" / "wheeled_dual_ur5e_v2_pin.xml"
            )
        self._mjcf_path = mjcf_path

        # Build the pinocchio-loadable flattened MJCF (cached next to the source).
        pin_mjcf = self._make_pinocchio_mjcf(mjcf_path)

        self.model = pin.buildModelFromMJCF(pin_mjcf)
        self.data = self.model.createData()

        if self.model.nq != self.nq:
            raise RuntimeError(
                f"Expected {self.nq} DOF in pinocchio model, got {self.model.nq}. "
                f"Check MJCF preprocessing at {pin_mjcf}."
            )

        # Get frame IDs for both EE sites
        self.left_ee_frame_id = self.model.getFrameId(self.LEFT_EE_FRAME)
        self.right_ee_frame_id = self.model.getFrameId(self.RIGHT_EE_FRAME)

        # Joint index ranges for easy slicing
        self.base_idx = slice(0, 4)        # [0:4]
        self.left_arm_idx = slice(4, 10)   # [4:10]
        self.right_arm_idx = slice(10, 16) # [10:16]

    # ------------------------------------------------------------------
    # MJCF preprocessing
    # ------------------------------------------------------------------

    @staticmethod
    def _make_pinocchio_mjcf(mjcf_path: str) -> str:
        """
        Compile the MJCF via mujoco (expands <include>), flatten it, and strip
        the target_body mocap elements so pinocchio's parser reaches the robot tree.

        Returns the path to the generated pinocchio-loadable XML, cached next to
        the source as <name>_pin.xml.
        """
        import re

        src = Path(mjcf_path)
        out_path = src.with_name(src.stem + "_pin.xml")

        # Compile via mujoco to expand includes and resolve defaults.
        m = mujoco.MjModel.from_xml_path(mjcf_path)
        flat_path = tempfile.mktemp(suffix=".xml")
        mujoco.mj_saveLastXML(flat_path, m)

        txt = Path(flat_path).read_text()
        os.unlink(flat_path)

        # Strip all mocap target_body blocks (left and right targets)
        # Pattern matches: <body name="left_target_body"...>...</body>
        txt = re.sub(
            r'\s*<body name="(left|right)_target_body".*?</body>',
            "",
            txt,
            flags=re.DOTALL
        )

        out_path.write_text(txt)
        return str(out_path)

    # ------------------------------------------------------------------
    # Forward kinematics - Left Arm
    # ------------------------------------------------------------------

    def fk_left_ee_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Forward kinematics of the left EE frame.

        Input:  q shape (16,)
        Output: (p, R) world position (3,) and rotation matrix (3,3)
        """
        q = np.asarray(q)
        if q.shape[0] != self.nq:
            raise ValueError(f"Expected q shape ({self.nq},), got {q.shape}")

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.left_ee_frame_id]
        return np.array(oMf.translation), np.array(oMf.rotation)

    def fk_left_ee(self, q: np.ndarray) -> np.ndarray:
        """Left EE world position only, shape (3,)."""
        p, _ = self.fk_left_ee_pose(q)
        return p

    # ------------------------------------------------------------------
    # Forward kinematics - Right Arm
    # ------------------------------------------------------------------

    def fk_right_ee_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Forward kinematics of the right EE frame.

        Input:  q shape (16,)
        Output: (p, R) world position (3,) and rotation matrix (3,3)
        """
        q = np.asarray(q)
        if q.shape[0] != self.nq:
            raise ValueError(f"Expected q shape ({self.nq},), got {q.shape}")

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.right_ee_frame_id]
        return np.array(oMf.translation), np.array(oMf.rotation)

    def fk_right_ee(self, q: np.ndarray) -> np.ndarray:
        """Right EE world position only, shape (3,)."""
        p, _ = self.fk_right_ee_pose(q)
        return p

    # ------------------------------------------------------------------
    # Jacobians - Left Arm
    # ------------------------------------------------------------------

    def jacobian_left_ee(self, q: np.ndarray, local: bool = False) -> np.ndarray:
        """
        Geometric Jacobian of the left EE frame, shape (6, 16).
        Rows: [linear (3); angular (3)].

        local=False -> LOCAL_WORLD_ALIGNED (world-aligned axes at the frame)
        local=True  -> LOCAL frame
        """
        q = np.asarray(q)
        if q.shape[0] != self.nq:
            raise ValueError(f"Expected q shape ({self.nq},), got {q.shape}")

        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        ref = pin.LOCAL if local else pin.LOCAL_WORLD_ALIGNED
        J = pin.getFrameJacobian(self.model, self.data, self.left_ee_frame_id, ref)
        return np.array(J)

    def position_jacobian_left_ee(self, q: np.ndarray) -> np.ndarray:
        """Linear (position) part of the left EE Jacobian, shape (3, 16)."""
        return self.jacobian_left_ee(q)[:3, :]

    # ------------------------------------------------------------------
    # Jacobians - Right Arm
    # ------------------------------------------------------------------

    def jacobian_right_ee(self, q: np.ndarray, local: bool = False) -> np.ndarray:
        """
        Geometric Jacobian of the right EE frame, shape (6, 16).
        Rows: [linear (3); angular (3)].

        local=False -> LOCAL_WORLD_ALIGNED (world-aligned axes at the frame)
        local=True  -> LOCAL frame
        """
        q = np.asarray(q)
        if q.shape[0] != self.nq:
            raise ValueError(f"Expected q shape ({self.nq},), got {q.shape}")

        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        ref = pin.LOCAL if local else pin.LOCAL_WORLD_ALIGNED
        J = pin.getFrameJacobian(self.model, self.data, self.right_ee_frame_id, ref)
        return np.array(J)

    def position_jacobian_right_ee(self, q: np.ndarray) -> np.ndarray:
        """Linear (position) part of the right EE Jacobian, shape (3, 16)."""
        return self.jacobian_right_ee(q)[:3, :]

    # ------------------------------------------------------------------
    # Utility methods
    # ------------------------------------------------------------------

    def get_q_nominal(self) -> np.ndarray:
        """
        Nominal configuration for both arms.

        Returns: q_nominal (16,)
        """
        # Base at origin, no rotation, height 0.2m
        q_base = np.array([0.0, 0.0, 0.0, 0.2])

        # Both arms in upright configuration (shoulder_pan = π)
        # This is the standard UR5e home position
        q_arm_nominal = np.array([np.pi, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])

        q_nominal = np.concatenate([
            q_base,
            q_arm_nominal,  # left arm
            q_arm_nominal,  # right arm
        ])

        return q_nominal

    def print_ee_positions(self, q: np.ndarray) -> None:
        """Debug utility: print both EE positions."""
        p_left = self.fk_left_ee(q)
        p_right = self.fk_right_ee(q)
        print(f"Left EE:  [{p_left[0]:+.4f}, {p_left[1]:+.4f}, {p_left[2]:+.4f}]")
        print(f"Right EE: [{p_right[0]:+.4f}, {p_right[1]:+.4f}, {p_right[2]:+.4f}]")
